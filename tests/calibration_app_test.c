/* Adapter + real driver integration under a synthetic HAL. Never opens hardware. */
#include "cybergear_calibration_app.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    FDCAN_TxHeaderTypeDef header;
    uint8_t data[8];
    bool queued;
} TestTx;

static TestTx frames[4096];
static unsigned int frame_count, consumed;
static uint32_t tick_ms, fifo_free;
static HAL_StatusTypeDef tx_status, protocol_status, counters_status, uart_status;
static bool bus_off, passive, accept_mode_read, accept_stop;
static uint32_t tx_errors, rx_errors;
static uint8_t reported_run_mode, simulated_mode;
static float simulated_position;
static FDCAN_HandleTypeDef can;
static UART_HandleTypeDef uart;
static char uart_capture[131072];
static size_t uart_count;
static const char *capture_path;

uint32_t HAL_GetTick(void) { return tick_ms; }
void HAL_Delay(uint32_t milliseconds) { tick_ms += milliseconds; }
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef *handle)
{ assert(handle == &can); return fifo_free; }
uint32_t HAL_FDCAN_GetError(const FDCAN_HandleTypeDef *handle)
{ return handle->ErrorCode; }
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(const FDCAN_HandleTypeDef *handle,
                                            FDCAN_ProtocolStatusTypeDef *status)
{
    assert(handle == &can);
    memset(status, 0, sizeof(*status));
    status->BusOff = bus_off;
    status->ErrorPassive = passive;
    return protocol_status;
}
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(const FDCAN_HandleTypeDef *handle,
                                           FDCAN_ErrorCountersTypeDef *counters)
{
    assert(handle == &can);
    memset(counters, 0, sizeof(*counters));
    counters->TxErrorCnt = tx_errors;
    counters->RxErrorCnt = rx_errors;
    return counters_status;
}
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *handle,
                                               FDCAN_TxHeaderTypeDef *header, uint8_t *data)
{
    assert(handle == &can);
    assert(frame_count < sizeof(frames) / sizeof(frames[0]));
    TestTx *frame = &frames[frame_count++];
    frame->header = *header;
    memcpy(frame->data, data, sizeof(frame->data));
    HAL_StatusTypeDef result = fifo_free == 0U ? HAL_BUSY : tx_status;
    frame->queued = result == HAL_OK;
    return result;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *handle, const uint8_t *data,
                                    uint16_t length, uint32_t timeout)
{
    assert(handle == &uart);
    (void)timeout;
    if (uart_status != HAL_OK) return uart_status;
    assert(uart_count + length < sizeof(uart_capture));
    memcpy(uart_capture + uart_count, data, length);
    uart_count += length;
    uart_capture[uart_count] = '\0';
    return HAL_OK;
}

static unsigned int kind(unsigned int index)
{ return (frames[index].header.Identifier >> 24) & 31U; }
static uint16_t parameter(unsigned int index)
{ return (uint16_t)(frames[index].data[0] | (uint16_t)frames[index].data[1] << 8); }
static float current(unsigned int index)
{
    float value;
    memcpy(&value, frames[index].data + 4, sizeof(value));
    return value;
}
static unsigned int count_kind(unsigned int type)
{
    unsigned int count = 0;
    for (unsigned int i = 0; i < frame_count; ++i)
        if (kind(i) == type && frames[i].queued) count++;
    return count;
}
static bool is_current(unsigned int index)
{ return kind(index) == CYBERGEAR_COMM_WRITE_PARAMETER && parameter(index) == CYBERGEAR_PARAM_IQ_REF; }
static unsigned int nonzero_currents(void)
{
    unsigned int count = 0;
    for (unsigned int i = 0; i < frame_count; ++i)
        if (is_current(i) && frames[i].queued && fabsf(current(i)) > 1e-7f) count++;
    return count;
}
static void only_stop_since(unsigned int first)
{
    for (unsigned int i = first; i < frame_count; ++i)
        assert(kind(i) == CYBERGEAR_COMM_STOP);
}
static FDCAN_RxHeaderTypeDef receive_header(uint32_t identifier)
{
    FDCAN_RxHeaderTypeDef header = {0};
    header.Identifier = identifier;
    header.IdType = FDCAN_EXTENDED_ID;
    header.RxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    return header;
}
static void put_u16(uint8_t *bytes, uint16_t value)
{ bytes[0] = (uint8_t)(value >> 8); bytes[1] = (uint8_t)value; }
static uint16_t encode(float value, float lower, float upper)
{ return (uint16_t)((value - lower) * 65535.0f / (upper - lower)); }
static void receive_feedback(CyberGearMotor *motor, float position, float speed,
                             float temperature, uint8_t fault)
{
    uint8_t data[8] = {0};
    put_u16(data, encode(position, -12.5f, 12.5f));
    put_u16(data + 2, encode(speed, -30.0f, 30.0f));
    put_u16(data + 4, 32767U);
    put_u16(data + 6, (uint16_t)(temperature * 10.0f));
    FDCAN_RxHeaderTypeDef header = receive_header(0x02007ffeU |
        (uint32_t)simulated_mode << 22 | (uint32_t)fault << 16);
    assert(cybergear_process_rx(motor, &header, data));
}
static void consume_commands(CyberGearMotor *motor)
{
    while (consumed < frame_count) {
        unsigned int index = consumed++;
        if (!frames[index].queued) continue;
        if (kind(index) == CYBERGEAR_COMM_STOP && accept_stop) simulated_mode = 0;
        if (kind(index) == CYBERGEAR_COMM_ENABLE) simulated_mode = 2;
        if (kind(index) == CYBERGEAR_COMM_READ_PARAMETER && accept_mode_read) {
            uint8_t data[8] = {0x05, 0x70, 0, 0, 0, 0, 0, 0};
            data[4] = reported_run_mode;
            FDCAN_RxHeaderTypeDef header = receive_header(0x11007ffeU);
            assert(cybergear_process_rx(motor, &header, data));
        }
    }
}
static void cycle_at(CgCalApp *app, float position, float speed, bool receive)
{
    tick_ms += CG_CAL_APP_PERIOD_MS;
    consume_commands(app->motor);
    if (receive) receive_feedback(app->motor, position, speed, 25.0f, 0U);
    cg_cal_app_tick(app, tick_ms);
}
static void cycle(CgCalApp *app)
{
    /* Deliberately controlled feedback trace for state/transport tests. Not a
     * physical motor model and never a recommended current/speed for the robot. */
    float speed = 0;
    if (app->state == CG_CAL_APP_TRIAL) {
        float direction = app->trial_config.pulse_current_a > 0 ? 1.0f : -1.0f;
        if (app->core.phase == CG_CAL_PHASE_PULSE)
            speed = 0.25f * (float)(tick_ms - app->core.phase_ms + CG_CAL_APP_PERIOD_MS) * 0.001f * direction;
        if (app->core.phase == CG_CAL_PHASE_BRAKE && tick_ms - app->core.phase_ms < 30U)
            speed = 0.04f * direction;
    }
    simulated_position += speed * 0.01f;
    cycle_at(app, simulated_position, speed, true);
}
static CgCalConfig fixture(void)
{
    CgCalConfig config;
    cg_cal_app_config_defaults(&config);
    assert(config.armed == (CG_CAL_ARMED != 0));
    cg_cal_config_defaults(&config); /* Synthetic fixture, not installed machine settings. */
    config.armed = true;
    config.pulse_current_a = 0.05f;
    config.brake_current_a = 0.1f;
    config.current_limit_a = 0.2f;
    config.temp_trip_c = 60.0f;
    config.speed_trip_rad_s = 1.0f;
    assert(cg_cal_config_valid(&config));
    return config;
}
static void initialize(CgCalApp *app, CyberGearMotor *motor)
{
    memset(&can, 0, sizeof(can));
    frame_count = consumed = 0;
    tick_ms = 100U;
    fifo_free = 3U;
    tx_status = protocol_status = counters_status = uart_status = HAL_OK;
    bus_off = passive = false;
    tx_errors = rx_errors = 0;
    accept_mode_read = accept_stop = true;
    reported_run_mode = CYBERGEAR_RUN_MODE_CURRENT;
    simulated_mode = 0;
    simulated_position = 0.5f;
    uart_count = 0;
    uart_capture[0] = '\0';
    assert(cybergear_init(motor, &can, 0x7fU, 0xfeU));
    assert(!motor->config.hardware_confirmed);
    assert(isnan(motor->config.controller.b0_initial));
    assert(!cybergear_config_valid(&motor->config));
    CgCalConfig config = fixture();
    assert(cg_cal_app_init(app, motor, &config));
    assert(motor->calibration_owned && motor->managed);
}
static void run_to(CgCalApp *app, CgCalAppState state, unsigned int max_cycles)
{
    while (max_cycles-- && app->state != state && app->state != CG_CAL_APP_FAULT) cycle(app);
    if (app->state != state)
        fprintf(stderr, "expected app=%u actual=%u core=%u fault=%u tick=%lu\n",
                (unsigned)state, (unsigned)app->state, (unsigned)app->core.phase,
                (unsigned)app->core.fault, (unsigned long)tick_ms);
    assert(app->state == state);
}
static void ready(CgCalApp *app, CyberGearMotor *motor)
{
    initialize(app, motor);
    assert(!cg_cal_app_request_trial(app, 1));
    run_to(app, CG_CAL_APP_READY, 40);
    assert(app->origin_captured);
    assert(count_kind(CYBERGEAR_COMM_STOP) > 0);
    assert(count_kind(CYBERGEAR_COMM_ENABLE) == 0);
}
static void start_trial(CgCalApp *app, int direction)
{
    assert(cg_cal_app_request_trial(app, direction));
    assert(!cg_cal_app_request_trial(app, direction));
    run_to(app, CG_CAL_APP_TRIAL, 100);
    assert(app->core.phase == CG_CAL_PHASE_BASELINE);
    assert(app->motor->mode_read_valid);
    assert(app->motor->mode_read_value == CYBERGEAR_RUN_MODE_CURRENT);
    assert(!app->motor->config.hardware_confirmed);
    assert(isnan(app->motor->config.controller.b0_initial));
}

static void successful_trial_and_origin_tests(void)
{
    CgCalApp app;
    CyberGearMotor motor;
    ready(&app, &motor);
    float origin = app.initial_position_rad;
    unsigned int before = frame_count;
    for (unsigned int i = 0; i < 50; ++i) cycle(&app);
    assert(app.state == CG_CAL_APP_READY);
    only_stop_since(before);
    assert(!cg_cal_app_request_trial(&app, 0));
    start_trial(&app, 1);
    assert(!cg_cal_app_dump(&app, &uart));
    assert(uart_count == 0);
    run_to(&app, CG_CAL_APP_DONE, 200);
    for (uint32_t i=1U;i<app.log_count;++i)
        assert(app.rows[i].rx_sequence != app.rows[i-1U].rx_sequence);
    assert(app.core.fault == CG_CAL_FAULT_NONE && app.stop_confirmed && app.reset_confirmed);
    assert(app.trial_id == 1 && app.dump_pending && app.log_count > 40);
    assert(app.initial_position_rad == origin && app.core.initial_position_rad == origin);
    assert(!cg_cal_app_request_trial(&app, -1)); /* first export must be consumed */
    unsigned int pulses = 0, brakes = 0, zero_after_brake = 0, enables = 0;
    bool saw_brake = false;
    for (unsigned int i = 0; i < frame_count; ++i) {
        if (kind(i) == CYBERGEAR_COMM_ENABLE) enables++;
        if (kind(i) == CYBERGEAR_COMM_SET_ZERO) assert(false);
        if (!is_current(i) || !frames[i].queued) continue;
        float value = current(i);
        assert(fabsf(value) <= 0.2f);
        if (value > 0) { assert(!saw_brake); assert(fabsf(value - 0.05f) < 1e-6f); pulses++; }
        if (value < 0) { assert(fabsf(value + 0.1f) < 1e-6f); brakes++; saw_brake = true; }
        if (value == 0 && saw_brake) zero_after_brake++;
    }
    assert(pulses == 20 && brakes >= 1 && zero_after_brake >= 10 && enables == 1);
    uart_status = HAL_ERROR;
    assert(!cg_cal_app_dump(&app, &uart) && app.dump_pending);
    uart_status = HAL_OK;
    assert(cg_cal_app_dump(&app, &uart));
    assert(!app.dump_pending);
    assert(strstr(uart_capture, "timestamp_ms,feedback_timestamp_ms,rx_sequence,trial_id,phase,position_rad,velocity_rad_s,command_current_a,rx_age_ms,fault,saturated,dropped,tx_failed,feedback_valid,initial_position_rad,temperature_c,app_state,motor_mode\r\n"));
    assert(strstr(uart_capture, ",BASELINE,") && strstr(uart_capture, ",PULSE,") &&
           strstr(uart_capture, ",BRAKE,") && strstr(uart_capture, ",COAST,") && strstr(uart_capture, ",DONE,"));
    unsigned int old_nonzero = nonzero_currents();
    before = frame_count;
    for (unsigned int i = 0; i < 50; ++i) cycle(&app);
    assert(app.state == CG_CAL_APP_DONE && app.trial_id == 1);
    only_stop_since(before);
    assert(nonzero_currents() == old_nonzero);
    simulated_position += 0.01f; /* manual change inside original window */
    start_trial(&app, -1);
    assert(app.trial_id == 2 && app.initial_position_rad == origin);
    assert(app.core.initial_position_rad == origin);
    assert(app.trial_config.pulse_current_a == -0.05f);
    run_to(&app, CG_CAL_APP_DONE, 200);
    assert(app.core.fault == CG_CAL_FAULT_NONE && app.initial_position_rad == origin);
    assert(count_kind(CYBERGEAR_COMM_ENABLE) == 2);
    assert(cg_cal_app_dump(&app, &uart));
    if (capture_path) {
        FILE *capture = fopen(capture_path, "wb");
        assert(capture);
        assert(fwrite(uart_capture, 1, uart_count, capture) == uart_count);
        assert(fclose(capture) == 0);
    }
}

static void ownership_tests(void)
{
    CgCalApp app;
    CyberGearMotor motor;
    ready(&app, &motor);
    unsigned int before = frame_count;
    assert(!cybergear_enable(&motor));
    assert(!cybergear_enable_calibration(&motor, &app.config));
    assert(!cybergear_set_zero(&motor));
    assert(!cybergear_clear_fault(&motor));
    assert(!cybergear_set_current(&motor, 0.1f));
    assert(!cybergear_set_run_mode(&motor, CYBERGEAR_RUN_MODE_SPEED));
    assert(!cybergear_control(&motor, 0, 0, 1, 1, 0));
    assert(!cybergear_begin_position_control(&motor, CYBERGEAR_RUN_MODE_CURRENT));
    assert(frame_count == before);
    assert(cybergear_stop(&motor));
    cycle(&app);
    assert(app.core.fault == CG_CAL_FAULT_REQUESTED_STOP);
    run_to(&app, CG_CAL_APP_FAULT, 150);
    only_stop_since(before);
    assert(!cg_cal_app_request_trial(&app, 1));
}

static void startup_failure_tests(void)
{
    CgCalApp app;
    CyberGearMotor motor;
    ready(&app, &motor);
    reported_run_mode = CYBERGEAR_RUN_MODE_SPEED;
    assert(cg_cal_app_request_trial(&app, 1));
    run_to(&app, CG_CAL_APP_FAULT, 150);
    assert(app.core.fault == CG_CAL_FAULT_MOTOR);
    assert(count_kind(CYBERGEAR_COMM_ENABLE) == 0 && nonzero_currents() == 0);
    ready(&app, &motor);
    accept_mode_read = false;
    assert(cg_cal_app_request_trial(&app, 1));
    run_to(&app, CG_CAL_APP_FAULT, 500);
    assert(app.core.fault == CG_CAL_FAULT_FEEDBACK);
    assert(count_kind(CYBERGEAR_COMM_ENABLE) == 0 && nonzero_currents() == 0);
    initialize(&app, &motor);
    for (unsigned int i = 0; i < 450 && app.state != CG_CAL_APP_FAULT; ++i)
        cycle_at(&app, simulated_position, 0, false);
    assert(app.state == CG_CAL_APP_FAULT && app.core.fault == CG_CAL_FAULT_FEEDBACK);
    assert(!app.origin_captured && !app.stop_confirmed);
    assert(count_kind(CYBERGEAR_COMM_ENABLE) == 0);
}

static void fault_tests(void)
{
    CgCalApp app;
    CyberGearMotor motor;
    for (unsigned int fault = 0; fault < 9; ++fault) {
        ready(&app, &motor);
        start_trial(&app, 1);
        unsigned int before = frame_count;
        switch (fault) {
        case 0: cg_cal_app_abort(&app); break;
        case 1: tx_status = HAL_ERROR; break;
        case 2: bus_off = true; break;
        case 3: passive = true; break;
        case 4: tx_errors = 1U; break;
        case 5: protocol_status = HAL_ERROR; break;
        case 6: counters_status = HAL_ERROR; break;
        case 7: fifo_free = 0; break;
        default: rx_errors = 1U; break;
        }
        cycle(&app);
        assert(app.state == CG_CAL_APP_STOPPING);
        if (fault == 0) assert(app.core.fault == CG_CAL_FAULT_REQUESTED_STOP);
        else if (fault == 1 || fault == 7) assert(app.core.fault == CG_CAL_FAULT_TX);
        else assert(app.core.fault == CG_CAL_FAULT_BUS);
        if (fault == 1 || fault == 7) {
            assert(motor.tx_failed > 0);
            assert(!frames[before].queued);
            assert(app.core.last_current_a == 0);
            before++; /* the one failing current attempt precedes STOP */
        }
        fifo_free = 3; tx_status = protocol_status = counters_status = HAL_OK;
        bus_off = passive = false; tx_errors = rx_errors = 0;
        run_to(&app, CG_CAL_APP_FAULT, 150);
        only_stop_since(before);
        assert(app.dump_pending);
        assert(!cg_cal_app_request_trial(&app, 1));
    }
}

static void boundary_and_liveness_tests(void)
{
    CgCalApp app;
    CyberGearMotor motor;
    for (int sign = -1; sign <= 1; sign += 2) {
        ready(&app, &motor);
        start_trial(&app, sign);
        unsigned int before = frame_count;
        cycle_at(&app, app.initial_position_rad + (float)sign * CG_CAL_ABSOLUTE_TRAVEL_RAD, 0, true);
        assert(app.state == CG_CAL_APP_STOPPING && app.core.fault == CG_CAL_FAULT_POSITION);
        run_to(&app, CG_CAL_APP_FAULT, 150);
        only_stop_since(before);
    }
    ready(&app, &motor);
    start_trial(&app, 1);
    unsigned int before = frame_count;
    for (unsigned int i = 0; i < 4; ++i) cycle_at(&app, simulated_position, 0, false);
    assert(app.state == CG_CAL_APP_STOPPING && app.core.fault == CG_CAL_FAULT_FEEDBACK);
    before = frame_count; /* only STOP retries may follow stale detection */
    run_to(&app, CG_CAL_APP_FAULT, 150);
    only_stop_since(before);
    ready(&app, &motor);
    start_trial(&app, 1);
    accept_stop = false;
    cg_cal_app_abort(&app);
    for (unsigned int i = 0; i < 150 && app.state != CG_CAL_APP_FAULT; ++i) cycle(&app);
    assert(app.state == CG_CAL_APP_FAULT && !app.stop_confirmed && !app.reset_confirmed);
    assert(app.core.fault != CG_CAL_FAULT_NONE);
    assert(count_kind(CYBERGEAR_COMM_STOP) > 10);
    assert(!cg_cal_app_request_trial(&app, -1));
    ready(&app, &motor);
    before = frame_count;
    tick_ms += app.config.max_step_ms + 1;
    receive_feedback(&motor, simulated_position, 0, 25, 0);
    cg_cal_app_tick(&app, tick_ms);
    assert(app.core.fault == CG_CAL_FAULT_TIMING && app.state == CG_CAL_APP_STOPPING);
    run_to(&app, CG_CAL_APP_FAULT, 150);
    only_stop_since(before);
}

static void measured_guard_and_capacity_tests(void)
{
    CgCalApp app;
    CyberGearMotor motor;
    for (unsigned int scenario = 0; scenario < 5; ++scenario) {
        ready(&app, &motor);
        start_trial(&app, 1);
        unsigned int before = frame_count;
        tick_ms += CG_CAL_APP_PERIOD_MS;
        consume_commands(&motor);
        float speed = scenario == 0 ? 1.1f : 0;
        float temperature = scenario == 1 ? 61.0f : 25.0f;
        uint8_t fault = scenario == 2 ? 1 : 0;
        if (scenario == 3) simulated_mode = 0;
        if (scenario == 4) app.log_count = CG_CAL_APP_LOG_CAPACITY - 1;
        receive_feedback(&motor, simulated_position, speed, temperature, fault);
        cg_cal_app_tick(&app, tick_ms);
        static const CgCalFault expected[] = {
            CG_CAL_FAULT_SPEED, CG_CAL_FAULT_TEMPERATURE, CG_CAL_FAULT_MOTOR,
            CG_CAL_FAULT_MOTOR, CG_CAL_FAULT_LOG_OVERFLOW
        };
        assert(app.state == CG_CAL_APP_STOPPING && app.core.fault == expected[scenario]);
        run_to(&app, CG_CAL_APP_FAULT, 150);
        only_stop_since(before);
        if (scenario == 4) {
            assert(app.dropped > 0 && app.log_count == CG_CAL_APP_LOG_CAPACITY);
            assert(app.rows[app.log_count - 1].fault == CG_CAL_FAULT_LOG_OVERFLOW);
        }
    }
    ready(&app, &motor);
    assert(cg_cal_app_request_trial(&app, 1));
    run_to(&app, CG_CAL_APP_WRITE_MODE, 30);
    unsigned int before = frame_count;
    cycle_at(&app, simulated_position + 0.002f, 0.04f, true);
    assert(app.state == CG_CAL_APP_STOPPING && app.core.fault == CG_CAL_FAULT_BASELINE_MOTION);
    run_to(&app, CG_CAL_APP_FAULT, 150);
    only_stop_since(before);
    assert(count_kind(CYBERGEAR_COMM_ENABLE) == 0);
    initialize(&app, &motor);
    assert(cybergear_init(&motor, &can, 0x7fU, 0xfeU));
    CgCalConfig invalid;
    cg_cal_app_config_defaults(&invalid);
    invalid.armed = false; /* Independent of the operator's installed config. */
    before = frame_count;
    assert(!cg_cal_app_init(&app, &motor, &invalid));
    assert(app.state == CG_CAL_APP_FAULT && !motor.calibration_owned);
    assert(frame_count == before && !cg_cal_app_request_trial(&app, 1));
}

static void startup_speed_noise_tests(void)
{
    CgCalApp app;
    CyberGearMotor motor;
    ready(&app, &motor);
    assert(cg_cal_app_request_trial(&app, 1));
    run_to(&app, CG_CAL_APP_WAIT_RUN, 50);
    cycle_at(&app, simulated_position, -0.1515217f, true);
    assert(app.state == CG_CAL_APP_WAIT_RUN && app.core.fault == CG_CAL_FAULT_NONE);
    assert(nonzero_currents() == 0);
    run_to(&app, CG_CAL_APP_TRIAL, 30);
    assert(app.core.phase == CG_CAL_PHASE_BASELINE && nonzero_currents() == 0);
    run_to(&app, CG_CAL_APP_DONE, 150);
    assert(app.core.fault == CG_CAL_FAULT_NONE && nonzero_currents() > 0);

    ready(&app, &motor);
    assert(cg_cal_app_request_trial(&app, 1));
    run_to(&app, CG_CAL_APP_WAIT_RUN, 50);
    for (unsigned i=0; i<350 && app.state==CG_CAL_APP_WAIT_RUN; ++i)
        cycle_at(&app, simulated_position, 0.15f, true);
    assert(app.state == CG_CAL_APP_STOPPING && app.core.fault == CG_CAL_FAULT_FEEDBACK);
    assert(nonzero_currents() == 0);
    run_to(&app, CG_CAL_APP_FAULT, 150);

    ready(&app, &motor);
    assert(cg_cal_app_request_trial(&app, 1));
    run_to(&app, CG_CAL_APP_WAIT_RUN, 50);
    cycle_at(&app, simulated_position + 0.002f, 0.0f, true);
    assert(app.state == CG_CAL_APP_STOPPING && app.core.fault == CG_CAL_FAULT_BASELINE_MOTION);
    assert(nonzero_currents() == 0);
}

int main(int argc, char **argv)
{
    assert(argc <= 2);
    capture_path = argc == 2 ? argv[1] : NULL;
    successful_trial_and_origin_tests();
    startup_speed_noise_tests();
    ownership_tests();
    startup_failure_tests();
    fault_tests();
    boundary_and_liveness_tests();
    measured_guard_and_capacity_tests();
    puts("Calibration adapter tests passed (synthetic HAL only)");
    return 0;
}
