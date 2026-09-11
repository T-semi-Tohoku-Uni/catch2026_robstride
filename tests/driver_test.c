#include "cybergear.h"
#include "cybergear_test_motion.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct { FDCAN_TxHeaderTypeDef header; uint8_t data[8]; bool queued; } Tx;
static Tx tx[4096];
static unsigned int tx_count;
static uint32_t tick_ms, fifo_free;
static HAL_StatusTypeDef send_status;
static bool bus_off;
static bool bus_passive, bus_warning;
static uint32_t bus_tx_errors, bus_rx_errors;
static HAL_StatusTypeDef bus_status_result, bus_counters_result;
static FDCAN_HandleTypeDef can;
static void (*irq_restore_hook)(void);
static CyberGearMotor *planning_race_motor;

void cybergear_test_restore_irq(uint32_t mask)
{
    (void)mask;
    void (*hook)(void) = irq_restore_hook;
    irq_restore_hook = NULL;
    if (hook != NULL) hook();
}

uint32_t HAL_GetTick(void) { return tick_ms; }
void HAL_Delay(uint32_t ms) { tick_ms += ms; }
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef *handle)
{ (void)handle; return fifo_free; }
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(const FDCAN_HandleTypeDef *handle,
    FDCAN_ProtocolStatusTypeDef *status)
{
    (void)handle;
    memset(status, 0, sizeof(*status));
    status->BusOff = bus_off;
    status->ErrorPassive = bus_passive;
    status->Warning = bus_warning;
    return bus_status_result;
}
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(const FDCAN_HandleTypeDef *handle,
    FDCAN_ErrorCountersTypeDef *counters)
{
    (void)handle;
    memset(counters, 0, sizeof(*counters));
    counters->TxErrorCnt = bus_tx_errors;
    counters->RxErrorCnt = bus_rx_errors;
    return bus_counters_result;
}
uint32_t HAL_FDCAN_GetError(const FDCAN_HandleTypeDef *handle)
{ return handle->ErrorCode; }
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *handle,
    FDCAN_TxHeaderTypeDef *header, uint8_t *data)
{
    assert(handle == &can);
    assert(tx_count < sizeof(tx) / sizeof(tx[0]));
    tx[tx_count].header = *header;
    memcpy(tx[tx_count].data, data, 8U);
    tx[tx_count++].queued = send_status == HAL_OK;
    return send_status;
}

static void clear_mock(uint32_t now)
{
    irq_restore_hook = NULL;
    memset(&can, 0, sizeof(can));
    tx_count = 0U;
    tick_ms = now;
    fifo_free = 3U;
    send_status = HAL_OK;
    bus_off = false;
    bus_passive = false;
    bus_warning = false;
    bus_tx_errors = 0U;
    bus_rx_errors = 0U;
    bus_status_result = HAL_OK;
    bus_counters_result = HAL_OK;
}

static unsigned int command_type(unsigned int index)
{ return (tx[index].header.Identifier >> 24) & 31U; }

static FDCAN_RxHeaderTypeDef rx_header(uint32_t id)
{
    FDCAN_RxHeaderTypeDef header = {0};
    header.Identifier = id;
    header.IdType = FDCAN_EXTENDED_ID;
    header.RxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    return header;
}

static uint16_t encode(float value, float lower, float upper)
{ return (uint16_t)((value - lower) * 65535.0f / (upper - lower)); }
static void be16(uint8_t *p, uint16_t value)
{ p[0] = (uint8_t)(value >> 8); p[1] = (uint8_t)value; }

static void feedback(CyberGearMotor *motor, uint8_t mode, float position,
    float velocity, float temperature, uint8_t fault)
{
    uint8_t data[8];
    be16(data, encode(position, -12.5f, 12.5f));
    be16(data + 2, encode(velocity, -30.0f, 30.0f));
    be16(data + 4, 32767U);
    be16(data + 6, (uint16_t)(temperature * 10.0f));
    const uint32_t id = 0x02007ffeU | ((uint32_t)mode << 22) | ((uint32_t)fault << 16);
    const FDCAN_RxHeaderTypeDef header = rx_header(id);
    assert(cybergear_process_rx(motor, &header, data));
}

/* Deliberately synthetic limits: these values are not hardware recommendations. */
static CyberGearConfig fixture(void)
{
    CyberGearConfig config;
    cybergear_config_defaults(&config);
    config.hardware_confirmed = true;
    config.trajectory.position_min_rad = -2.0f;
    config.trajectory.position_max_rad = 2.0f;
    config.hard_min_rad = -3.0f;
    config.hard_max_rad = 3.0f;
    config.speed_trip_rad_s = 5.0f;
    config.temperature_trip_c = 60.0f;
    config.controller.b0_initial = 10.0f;
    config.controller.b0_min = 5.0f;
    config.controller.b0_max = 20.0f;
    config.controller.current_limit_a = 2.0f;
    config.dynamics.fixed_b0 = 10.0f;
    config.dynamics.b0_min = 5.0f;
    config.dynamics.b0_max = 20.0f;
    config.dynamics.acceleration_current_a = 2.0f;
    config.dynamics.braking_current_a = 2.0f;
    config.dynamics.reserve_current_a = 1.0f;
    config.stall_current_a = 1.5f;
    config.operation_kp = 3.0f;
    config.operation_kd = 1.0f;
    config.operation_torque_limit_nm = 1.0f;
    assert(cybergear_config_valid(&config));
    return config;
}

static void configure(CyberGearMotor *motor)
{
    assert(cybergear_init(motor, &can, 0x7fU, 0xfeU));
    const CyberGearConfig config = fixture();
    assert(cybergear_configure(motor, &config));
}

static void mode_reply(CyberGearMotor *motor, uint8_t mode)
{
    uint8_t data[8] = {0x05, 0x70, 0, 0, 0, 0, 0, 0};
    data[4] = mode;
    const FDCAN_RxHeaderTypeDef header = rx_header(0x11007ffeU);
    assert(cybergear_process_rx(motor, &header, data));
}

static void start_configured(CyberGearMotor *motor, CyberGearRunMode mode)
{
    unsigned int consumed = tx_count;
    uint8_t run_state = 0U;
    assert(cybergear_begin_position_control(motor, mode));
    for (unsigned int step = 0; step < 200U && motor->state != CG_STATE_RUNNING; ++step) {
        while (consumed < tx_count) {
            unsigned int index = consumed++;
            if (!tx[index].queued) continue;
            if (command_type(index) == CYBERGEAR_COMM_STOP) run_state = 0U;
            if (command_type(index) == CYBERGEAR_COMM_ENABLE) run_state = 2U;
            if (command_type(index) == CYBERGEAR_COMM_READ_PARAMETER) mode_reply(motor, (uint8_t)mode);
        }
        tick_ms += motor->config.controller.period_ms;
        feedback(motor, run_state, 0.0f, 0.0f, 25.0f, 0U);
        (void)cybergear_control_position_adrc(motor, 0.0f);
        cybergear_service(motor);
    }
    assert(motor->state == CG_STATE_RUNNING);
    assert(motor->fault == CG_FAULT_NONE);
    assert(motor->mode_read_valid && motor->mode_read_value == (uint8_t)mode);
}

static void start(CyberGearMotor *motor, CyberGearRunMode mode)
{
    configure(motor);
    start_configured(motor, mode);
}

static void normal_tick(CyberGearMotor *motor, float target)
{
    tick_ms += motor->config.controller.period_ms;
    feedback(motor, 2U, 0.0f, 0.0f, 25.0f, 0U);
    (void)cybergear_control_position_adrc(motor, target);
    cybergear_service(motor);
}

static void assert_only_stop_after(unsigned int begin)
{
    for (unsigned int i = begin; i < tx_count; ++i)
        assert(command_type(i) == CYBERGEAR_COMM_STOP);
}

static void protocol_tests(void)
{
    CyberGearMotor motor;
    clear_mock(100U);
    assert(cybergear_init(&motor, &can, 0x7fU, 0xfeU));
    assert(cybergear_set_run_mode(&motor, CYBERGEAR_RUN_MODE_CURRENT));
    assert(tx[0].header.Identifier == 0x1200fe7fU);
    const uint8_t mode_data[8] = {0x05, 0x70, 0, 0, 3, 0, 0, 0};
    assert(memcmp(tx[0].data, mode_data, 8U) == 0);
    assert(cybergear_set_current(&motor, 1.0f));
    const uint8_t current_data[8] = {0x06, 0x70, 0, 0, 0, 0, 0x80, 0x3f};
    assert(memcmp(tx[1].data, current_data, 8U) == 0);
    assert(cybergear_stop(&motor));
    assert(tx[2].header.Identifier == 0x0400fe7fU);
    assert(cybergear_read_parameter(&motor, CYBERGEAR_PARAM_RUN_MODE));
    assert(tx[3].header.Identifier == 0x1100fe7fU);
    assert(tx[3].data[0] == 5U && tx[3].data[1] == 0x70U);
    const unsigned int before_invalid = tx_count;
    assert(!cybergear_set_current(&motor, NAN));
    assert(!cybergear_control(&motor, INFINITY, 0.0f, 0.0f, 0.0f, 0.0f));
    assert(tx_count == before_invalid);
    feedback(&motor, 2U, 1.0f, -2.0f, 34.5f, 0U);
    assert(fabsf(motor.feedback.position_rad - 1.0f) < 0.0005f);
    assert(fabsf(motor.feedback.velocity_rad_s + 2.0f) < 0.001f);
    assert(motor.feedback.temperature_c == 34.5f);
    const uint32_t sequence = motor.feedback.rx_sequence;
    feedback(&motor, 2U, 1.0f, -2.0f, 34.5f, 0U);
    assert(motor.feedback.rx_sequence == sequence + 1U); /* Same ms, separate frame. */
    const uint8_t bytes[8] = {0};
    FDCAN_RxHeaderTypeDef header = rx_header(0x02807ffeU);
    header.DataLength = FDCAN_DLC_BYTES_7;
    assert(!cybergear_process_rx(&motor, &header, bytes));
    header.DataLength = FDCAN_DLC_BYTES_12;
    assert(!cybergear_process_rx(&motor, &header, bytes));
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.IdType = FDCAN_STANDARD_ID;
    assert(!cybergear_process_rx(&motor, &header, bytes));
    header.IdType = FDCAN_EXTENDED_ID;
    header.RxFrameType = FDCAN_REMOTE_FRAME;
    assert(!cybergear_process_rx(&motor, &header, bytes));
    header = rx_header(0x02807effU);
    assert(!cybergear_process_rx(&motor, &header, bytes));
    header = rx_header(0x02807ffeU | 0x20000000U);
    assert(!cybergear_process_rx(&motor, &header, bytes));
    assert(motor.feedback.rx_sequence == sequence + 1U);
}

static void startup_and_fault_tests(void)
{
    CyberGearMotor motor;
    clear_mock(100U);
    assert(cybergear_init(&motor, &can, 0x7fU, 0xfeU));
    motor.config.hardware_confirmed = false; /* Explicit unresolved-machine fixture. */
    assert(!cybergear_config_valid(&motor.config));
    assert(!cybergear_begin_position_control(&motor, CYBERGEAR_RUN_MODE_CURRENT));
    assert(tx_count == 0U); /* An unresolved mechanical setup cannot enable. */

    clear_mock(1000U);
    start(&motor, CYBERGEAR_RUN_MODE_CURRENT);
    const unsigned int owned = tx_count;
    assert(!cybergear_enable(&motor));
    assert(!cybergear_set_run_mode(&motor, CYBERGEAR_RUN_MODE_SPEED));
    assert(!cybergear_set_current(&motor, 1.0f));
    assert(!cybergear_set_zero(&motor));
    assert(tx_count == owned);
    normal_tick(&motor, 0.2f);
    assert(motor.state == CG_STATE_RUNNING);
    normal_tick(&motor, NAN);
    assert(motor.fault == CG_FAULT_TARGET);
    const unsigned int after_fault = tx_count;
    for (unsigned int i = 0U; i < 100U; ++i) normal_tick(&motor, 0.0f);
    assert(motor.state == CG_STATE_FAULT);
    assert(motor.fault == CG_FAULT_TARGET);
    assert(motor.stop_attempts <= motor.config.stop_max_attempts);
    assert_only_stop_after(after_fault);
    assert(!cybergear_begin_position_control(&motor, CYBERGEAR_RUN_MODE_CURRENT));
    assert(!cybergear_reset_fault(&motor)); /* Run feedback is not Reset. */

    clear_mock(UINT32_MAX - 150U);
    start(&motor, CYBERGEAR_RUN_MODE_CURRENT);
    normal_tick(&motor, 0.0f);
    assert(motor.state == CG_STATE_RUNNING); /* startup and control survive tick wrap. */
    send_status = HAL_ERROR;
    const float queued = motor.controller.last_queued_current_a;
    normal_tick(&motor, 0.1f);
    assert(motor.fault == CG_FAULT_TX);
    assert(motor.controller.last_queued_current_a == queued);
    assert(!motor.controller.applied_current_valid);

    clear_mock(1000U);
    start(&motor, CYBERGEAR_RUN_MODE_CURRENT);
    bus_off = true;
    normal_tick(&motor, 0.0f);
    assert(motor.state == CG_STATE_RUNNING && motor.fault == CG_FAULT_NONE);

    clear_mock(1000U);
    start(&motor, CYBERGEAR_RUN_MODE_CURRENT);
    tick_ms += motor.config.controller.feedback_timeout_ms + 1U;
    (void)cybergear_control_position_adrc(&motor, 0.0f);
    assert(motor.fault != CG_FAULT_NONE);

    clear_mock(1000U);
    start(&motor, CYBERGEAR_RUN_MODE_CURRENT);
    const FDCAN_RxHeaderTypeDef header = rx_header(0x15007ffeU);
    const uint8_t fault_bytes[8] = {0, 0, 0, 0, 0, 0, 0, 1};
    assert(cybergear_process_rx(&motor, &header, fault_bytes));
    normal_tick(&motor, 0.0f);
    assert(motor.fault == CG_FAULT_MOTOR);
    assert(memcmp(motor.fault_payload, fault_bytes, 8U) == 0);

    clear_mock(1000U);
    start(&motor, CYBERGEAR_RUN_MODE_OPERATION);
    normal_tick(&motor, 0.1f);
    assert(command_type(tx_count - 1U) == CYBERGEAR_COMM_MOTION_CONTROL);
    assert(motor.state == CG_STATE_RUNNING);
    CyberGearLog log;
    assert(cybergear_pop_log(&motor, &log));
}

static void long_hold_wrap_test(void)
{
    CyberGearMotor motor;
    clear_mock(1000U);
    start(&motor, CYBERGEAR_RUN_MODE_CURRENT);
    assert(cg_trajectory_plan(&motor.trajectory, 0.01f, &motor.effective_limits));
    motor.requested_target_rad = 0.01f;
    /* The previous cycle is valid, but this completed trajectory started almost
     * one complete tick counter cycle ago. Reaching wrap must never replay it. */
    motor.trajectory_start_ms = tick_ms + 20U;
    normal_tick(&motor, 0.01f);
    assert(fabsf(motor.trajectory.point.q_rad - 0.01f) < 1e-6f);
    normal_tick(&motor, 0.01f);
    assert(fabsf(motor.trajectory.point.q_rad - 0.01f) < 1e-6f);
    assert(motor.state == CG_STATE_RUNNING);
}

static void timer_handoff_test(void)
{
    const uint32_t periods[] = {10U, 5U};
    for (unsigned int rate = 0U; rate < sizeof(periods) / sizeof(periods[0]); ++rate) {
        CyberGearMotor motor;
        clear_mock(1000U);
        assert(cybergear_init(&motor, &can, 0x7fU, 0xfeU));
        CyberGearConfig config = fixture();
        config.controller.period_ms = periods[rate];
        assert(cybergear_configure(&motor, &config));
        start_configured(&motor, CYBERGEAR_RUN_MODE_CURRENT);
        for (unsigned int step = 0U; step < 5U; ++step) normal_tick(&motor, 0.0f);
        assert(!motor.first_cyclic);
        const uint32_t last_control = motor.last_control_ms;
        const unsigned int before_handoff = tx_count;
        motor.first_cyclic = true;
        tick_ms++;
        feedback(&motor, 2U, 0.0f, 0.0f, 25.0f, 0U);
        assert(cybergear_control_position_adrc(&motor, 0.0f));
        assert(motor.first_cyclic && motor.last_control_ms == last_control);
        assert(tx_count == before_handoff);
        tick_ms += periods[rate];
        feedback(&motor, 2U, 0.0f, 0.0f, 25.0f, 0U);
        assert(cybergear_control_position_adrc(&motor, 0.0f));
        assert(!motor.first_cyclic && motor.last_control_ms == tick_ms);
        assert(motor.state == CG_STATE_RUNNING && motor.fault == CG_FAULT_NONE);
        assert(tx_count == before_handoff + 1U);
    }
}

static void wait_for_mode_request(CyberGearMotor *motor)
{
    configure(motor);
    assert(cybergear_begin_position_control(motor, CYBERGEAR_RUN_MODE_CURRENT));
    mode_reply(motor, 3U); /* An unsolicited answer cannot confirm the later request. */
    assert(!motor->mode_read_valid);
    for (unsigned int i = 0U; i < 100U && !motor->mode_read_pending; ++i) {
        tick_ms += motor->config.controller.period_ms;
        feedback(motor, 0U, 0.0f, 0.0f, 25.0f, 0U);
        (void)cybergear_control_position_adrc(motor, 0.0f);
    }
    assert(motor->state == CG_STATE_WAIT_MODE && motor->mode_read_pending);
}

static void wait_for_run_feedback(CyberGearMotor *motor)
{
    wait_for_mode_request(motor);
    mode_reply(motor, CYBERGEAR_RUN_MODE_CURRENT);
    for (unsigned int step = 0U; step < 6U && motor->state != CG_STATE_WAIT_RUN; ++step) {
        tick_ms += motor->config.controller.period_ms;
        feedback(motor, 0U, 0.0f, 0.0f, 25.0f, 0U);
        assert(cybergear_control_position_adrc(motor, 0.0f));
    }
    assert(motor->state == CG_STATE_WAIT_RUN);
}

void test_startup_handoff(void)
{
    const float velocities[] = {-0.04349f, 0.0371f};
    for (unsigned int sample = 0U; sample < sizeof(velocities) / sizeof(velocities[0]); ++sample) {
        CyberGearMotor motor;
        clear_mock(1000U);
        wait_for_run_feedback(&motor);
        tick_ms += motor.config.controller.period_ms;
        feedback(&motor, 2U, 0.01545f, velocities[sample], 25.0f, 0U);
        const unsigned int before = tx_count;
        assert(cybergear_control_position_adrc(&motor, 0.0f));
        assert(motor.state == CG_STATE_RUNNING && motor.fault == CG_FAULT_NONE);
        assert(motor.controller.position_rad == motor.feedback.position_rad);
        assert(motor.controller.velocity_rad_s == motor.feedback.velocity_rad_s);
        assert(motor.trajectory.point.q_rad == motor.feedback.position_rad);
        assert(motor.controller.applied_current_valid && motor.controller.last_queued_current_a == 0.0f);
        assert(tx_count == before);
        tick_ms += motor.config.controller.period_ms;
        feedback(&motor, 2U, 0.01545f, velocities[sample], 25.0f, 0U);
        assert(cybergear_control_position_adrc(&motor, motor.requested_target_rad));
        assert(tx_count == before + 1U);
        assert(command_type(before) == CYBERGEAR_COMM_WRITE_PARAMETER);
        assert(fabsf(motor.controller.last_queued_current_a) <=
            fmaxf(motor.config.controller.current_rise_a_s, motor.config.controller.current_fall_a_s) *
            (float)motor.config.controller.period_ms * 0.001f + 1e-6f);
    }
    for (unsigned int scenario = 0U; scenario < 3U; ++scenario) {
        CyberGearMotor motor;
        clear_mock(1000U);
        wait_for_run_feedback(&motor);
        tick_ms += motor.config.controller.period_ms;
        if (scenario != 0U) feedback(&motor, scenario == 1U ? 0U : 2U, 0.0f, 0.0f, 25.0f, 0U);
        if (scenario == 2U) motor.feedback.last_received_ms = tick_ms - motor.config.controller.feedback_timeout_ms - 1U;
        assert(cybergear_control_position_adrc(&motor, 0.0f));
        assert(motor.state == CG_STATE_WAIT_RUN);
        tick_ms = motor.startup_ms + motor.config.startup_timeout_ms + 1U;
        assert(!cybergear_control_position_adrc(&motor, 0.0f));
        assert(motor.fault == CG_FAULT_START_TIMEOUT);
    }
    CyberGearMotor motor;
    clear_mock(1000U);
    wait_for_run_feedback(&motor);
    tick_ms += motor.config.controller.period_ms;
    feedback(&motor, 2U, 0.0f, motor.config.speed_trip_rad_s + 0.1f, 25.0f, 0U);
    assert(!cybergear_control_position_adrc(&motor, 0.0f));
    assert(motor.fault == CG_FAULT_SPEED);
    puts("Startup: fresh Run feedback hands measured motion to ADRC; response and speed checks remain.");
}

static void rejected_startup_tests(void)
{
    CyberGearMotor motor;
    clear_mock(100U);
    wait_for_mode_request(&motor);
    mode_reply(&motor, CYBERGEAR_RUN_MODE_SPEED);
    normal_tick(&motor, 0.0f);
    assert(motor.fault == CG_FAULT_MODE);
    for (unsigned int i = 0U; i < tx_count; ++i)
        assert(command_type(i) != CYBERGEAR_COMM_ENABLE);

    clear_mock(100U);
    wait_for_mode_request(&motor);
    uint8_t malformed[8] = {5U, 0x70U, 1U, 0U, 3U, 0U, 0U, 0U};
    FDCAN_RxHeaderTypeDef header = rx_header(0x11007ffeU);
    assert(cybergear_process_rx(&motor, &header, malformed));
    assert(!motor.mode_read_valid);
    malformed[2] = 0U;
    header.Identifier = 0x11017ffeU; /* Nonzero readback error/reserved ID bits. */
    assert(cybergear_process_rx(&motor, &header, malformed));
    assert(!motor.mode_read_valid);
    for (unsigned int i = 0U; i < 400U && motor.fault == CG_FAULT_NONE; ++i) {
        tick_ms += motor.config.controller.period_ms;
        feedback(&motor, 2U, 0.0f, 0.0f, 25.0f, 0U);
        (void)cybergear_control_position_adrc(&motor, 0.0f);
    }
    assert(motor.fault == CG_FAULT_START_TIMEOUT);
    for (unsigned int i = 0U; i < tx_count; ++i)
        assert(command_type(i) != CYBERGEAR_COMM_ENABLE);
}

static void streaming_and_protection_tests(void)
{
    CyberGearMotor motor;
    clear_mock(1000U);
    start(&motor, CYBERGEAR_RUN_MODE_CURRENT);
    const uint32_t initial_generation = motor.plan_generation;
    const float initial_position = motor.trajectory.point.q_rad;
    for (unsigned int i = 0U; i < 30U; ++i) normal_tick(&motor, 0.03f + 0.0002f * (float)i);
    assert(motor.state == CG_STATE_RUNNING);
    assert(motor.plan_generation > initial_generation);
    assert(motor.trajectory.point.q_rad > initial_position + 0.0001f);

    clear_mock(1000U);
    start(&motor, CYBERGEAR_RUN_MODE_CURRENT);
    for (unsigned int i = 0U; i < 100U && motor.fault == CG_FAULT_NONE; ++i) {
        tick_ms += motor.config.controller.period_ms;
        feedback(&motor, 2U, 0.0f, 0.0f, 25.0f, 0U);
        (void)cybergear_control_position_adrc(&motor, 0.05f); /* main planner deliberately absent. */
    }
    assert(motor.fault == CG_FAULT_PLANNER);

    const CyberGearFault expected[] = {CG_FAULT_TEMPERATURE, CG_FAULT_SPEED,
        CG_FAULT_POSITION, CG_FAULT_MOTOR, CG_FAULT_TARGET};
    for (unsigned int i = 0U; i < sizeof(expected) / sizeof(expected[0]); ++i) {
        clear_mock(1000U);
        start(&motor, CYBERGEAR_RUN_MODE_CURRENT);
        tick_ms += motor.config.controller.period_ms;
        feedback(&motor, 2U, i == 2U ? 3.1f : 0.0f,
            i == 1U ? 5.1f : 0.0f, i == 0U ? 61.0f : 25.0f, i == 3U ? 1U : 0U);
        (void)cybergear_control_position_adrc(&motor, i == 4U ? 2.1f : 0.0f);
        assert(motor.fault == expected[i]);
    }

    clear_mock(1000U);
    start(&motor, CYBERGEAR_RUN_MODE_CURRENT);
    assert(cybergear_stop(&motor));
    for (unsigned int i = 0U; i < 50U; ++i) {
        tick_ms += motor.config.controller.period_ms;
        feedback(&motor, 0U, 0.0f, 0.0f, 25.0f, 0U);
        (void)cybergear_control_position_adrc(&motor, 0.0f);
    }
    assert(motor.state == CG_STATE_FAULT && motor.reset_confirmed && motor.stationary);
    assert(cybergear_reset_fault(&motor));
    assert(motor.state == CG_STATE_OFF && motor.fault == CG_FAULT_NONE);
    assert(cybergear_begin_position_control(&motor, CYBERGEAR_RUN_MODE_OPERATION));
    assert(motor.state == CG_STATE_WAIT_STOP); /* Explicit mode change repeats the handshake. */
}

static void unmanaged_fault_gate_test(void)
{
    const uint32_t fault_ids[] = {0x15007ffeU, 0x1500fe7fU};
    for (unsigned int direction = 0U; direction < 2U; ++direction) {
        CyberGearMotor motor;
        clear_mock(1000U);
        configure(&motor);
        assert(!motor.managed);
        assert(cybergear_enable(&motor));
        assert(cybergear_set_velocity(&motor, 0.1f));
        const uint8_t fault_bytes[8] = {0U, 0U, 0U, 1U, 0U, 0U, 0U, 0U};
        FDCAN_RxHeaderTypeDef header = rx_header(0x1500fe03U);
        assert(!cybergear_process_rx(&motor, &header, fault_bytes));
        assert(motor.motor_fault_sequence == 0U); /* Another motor never poisons this one. */
        header.Identifier = fault_ids[direction];
        assert(cybergear_process_rx(&motor, &header, fault_bytes));
        assert(motor.motor_fault_sequence != 0U);
        assert(memcmp(motor.fault_payload, fault_bytes, 8U) == 0);
        const unsigned int before = tx_count;
        assert(!cybergear_enable(&motor));
        assert(!cybergear_set_velocity(&motor, 0.1f));
        assert(!cybergear_set_run_mode(&motor, CYBERGEAR_RUN_MODE_SPEED));
        assert(!cybergear_set_current(&motor, 0.1f));
        assert(!cybergear_set_zero(&motor));
        assert(tx_count == before);
        assert(cybergear_stop(&motor));
        assert(tx_count == before + 1U);
        assert(command_type(tx_count - 1U) == CYBERGEAR_COMM_STOP);
        /* A subsequent normal fault report does not silently clear the latch. */
        const uint8_t clear_bytes[8] = {0};
        assert(cybergear_process_rx(&motor, &header, clear_bytes));
        assert(!cybergear_set_velocity(&motor, 0.1f));
    }
}

static void configured_motion_protection_test(void)
{
    const float current_limits[] = {6.0f, CG_TEST_CURRENT_LIMIT_A, 0.5f};
    for (unsigned int variant = 0U; variant < sizeof(current_limits) / sizeof(current_limits[0]); ++variant) {
        for (int direction = -1; direction <= 1; direction += 2) {
            CyberGearMotor motor;
            clear_mock(1000U);
            assert(cybergear_init(&motor, &can, 0x7fU, 0xfeU));
            if (variant == 2U) motor.config.controller.current_limit_a = current_limits[variant];
            if (variant != 0U) assert(cybergear_test_configure(&motor));
            assert(cybergear_config_valid(&motor.config));
            assert(motor.config.controller.current_limit_a == current_limits[variant]);
            assert(motor.config.dynamics.braking_current_a == current_limits[variant]);
            assert(motor.config.dynamics.acceleration_current_a == current_limits[variant]);
            assert(motor.config.dynamics.reserve_current_a == 0.5f * current_limits[variant]);
            start_configured(&motor, CYBERGEAR_RUN_MODE_CURRENT);
            CgTrajectory swing;
            CgTrajectoryPoint origin = {0};
            origin.q_rad = -CG_TEST_AMPLITUDE_RAD;
            assert(cg_trajectory_reset(&swing, &origin, &motor.effective_limits));
            assert(cg_trajectory_plan(&swing, CG_TEST_AMPLITUDE_RAD, &motor.effective_limits));
            assert(swing.duration_s + (double)CG_TEST_DWELL_MS / 1000.0 <
                   (double)CG_TEST_LEG_TIMEOUT_MS / 1000.0);
            assert(cybergear_controller_commit_queued(&motor.controller,
                current_limits[variant] * direction, tick_ms));
            assert(!cybergear_controller_commit_queued(&motor.controller,
                (current_limits[variant] + 0.01f) * direction, tick_ms));
            tick_ms += motor.config.controller.period_ms;
            feedback(&motor, 2U, -0.04f, 0.05f * direction, 27.0f, 0U);
            assert(fabsf(motor.feedback.velocity_rad_s) > motor.config.stationary_speed_rad_s);
            assert(cybergear_control_position_adrc(&motor, 0.0f));
            assert(motor.state == CG_STATE_RUNNING && motor.fault == CG_FAULT_NONE);
            assert(fabsf(motor.controller.last_queued_current_a) <= current_limits[variant]);

            const float boundary = direction > 0 ? motor.config.hard_max_rad : motor.config.hard_min_rad;
            motor.controller.last_measured_position_rad = boundary;
            tick_ms += motor.config.controller.period_ms;
            feedback(&motor, 2U, boundary, 0.05f * direction, 27.0f, 0U);
            assert(!cybergear_control_position_adrc(&motor, 0.0f));
            assert(motor.fault == CG_FAULT_POSITION);
        }
    }
}

static void start_saturation_fixture(CyberGearMotor *motor, bool amplitude, bool recovery)
{
    configure(motor);
    CyberGearConfig config = motor->config;
    config.recover_on_saturation = recovery;
    config.controller.bandwidth_rad_s = amplitude ? 100.0f : 10.0f;
    config.controller.damping_ratio = 0.01f;
    config.controller.observer_rad_s = 100.0f;
    config.controller.compensation_gain = 0.0f;
    config.controller.current_rise_a_s = amplitude ? 1000.0f : 0.1f;
    config.controller.current_fall_a_s = config.controller.current_rise_a_s;
    config.dynamics.current_slew_a_s = config.controller.current_rise_a_s;
    config.dynamics.reserve_slew_a_s = 0.5f * config.dynamics.current_slew_a_s;
    assert(cybergear_configure(motor, &config));
    start_configured(motor, CYBERGEAR_RUN_MODE_CURRENT);
}

static void saturation_tick(CyberGearMotor *motor, float position_rad)
{
    tick_ms += motor->config.controller.period_ms;
    feedback(motor, 2U, position_rad, 0.0f, 25.0f, 0U);
    (void)cybergear_control_position_adrc(motor, motor->requested_target_rad);
    cybergear_service(motor);
    assert(motor->latest_log.amp_limited == motor->output.amplitude_limited);
    assert(motor->latest_log.slew_limited == motor->output.slew_limited);
}

static void sustained_saturation_protection_test(void)
{
    for (unsigned int variant = 0U; variant < 3U; ++variant) {
        for (int direction = -1; direction <= 1; direction += 2) {
            CyberGearMotor motor;
            clear_mock(1000U);
            const bool amplitude = variant == 0U;
            start_saturation_fixture(&motor, amplitude, false);
            const uint32_t saturation_start_ms = tick_ms + motor.config.controller.period_ms;
            for (uint32_t step = 0U; step <= motor.config.saturation_timeout_ms /
                 motor.config.controller.period_ms; ++step) {
                const int feedback_direction = variant == 2U && step % 2U != 0U ? -direction : direction;
                const unsigned int before = tx_count;
                saturation_tick(&motor, 0.02f * (float)feedback_direction);
                assert(motor.output.amplitude_limited == amplitude);
                assert(motor.output.slew_limited != amplitude);
                assert(motor.saturation_active);
                assert(motor.saturation_since_ms == saturation_start_ms);
                assert(!motor.tracking_active && !motor.stall_active);
                if (tick_ms - saturation_start_ms < motor.config.saturation_timeout_ms) {
                    assert(motor.state == CG_STATE_RUNNING && motor.fault == CG_FAULT_NONE);
                } else {
                    assert(motor.state == CG_STATE_STOPPING && motor.fault == CG_FAULT_SATURATION);
                    assert_only_stop_after(before);
                }
            }
        }
    }
}

static void transient_saturation_reset_test(void)
{
    CyberGearMotor motor;
    clear_mock(1000U);
    start_saturation_fixture(&motor, false, false);
    for (unsigned int step = 0U; step < 10U; ++step) {
        saturation_tick(&motor, 0.02f);
        assert(motor.saturation_active && motor.output.slew_limited);
        assert(motor.fault == CG_FAULT_NONE);
    }
    const uint32_t first_saturation_ms = motor.saturation_since_ms;
    for (unsigned int step = 0U; step < 40U && motor.saturation_active; ++step)
        saturation_tick(&motor, 0.0f);
    assert(!motor.saturation_active && !motor.output.slew_limited && !motor.output.amplitude_limited);
    assert(motor.state == CG_STATE_RUNNING && motor.fault == CG_FAULT_NONE);
    const uint32_t restarted_ms = tick_ms + motor.config.controller.period_ms;
    for (uint32_t step = 0U; step <= motor.config.saturation_timeout_ms /
         motor.config.controller.period_ms; ++step) {
        saturation_tick(&motor, 0.02f);
        assert(motor.saturation_active && motor.output.slew_limited);
        assert(motor.saturation_since_ms == restarted_ms && restarted_ms > first_saturation_ms);
        if (tick_ms - restarted_ms < motor.config.saturation_timeout_ms)
            assert(motor.state == CG_STATE_RUNNING && motor.fault == CG_FAULT_NONE);
        else
            assert(motor.state == CG_STATE_STOPPING && motor.fault == CG_FAULT_SATURATION);
    }
}

static void sustained_saturation_recovery_test(void)
{
    for (unsigned int variant = 0U; variant < 2U; ++variant) {
        for (int direction = -1; direction <= 1; direction += 2) {
            CyberGearMotor motor;
            clear_mock(direction < 0 ? UINT32_MAX - 1500U : 1000U);
            start_saturation_fixture(&motor, variant == 0U, true);
            motor.dynamics.b0 = 12.0f;
            motor.controller.b0 = 12.0f;
            motor.prepared = motor.trajectory;
            assert(cg_trajectory_plan(&motor.prepared, 0.01f, &motor.effective_limits));
            motor.prepared_start_ms = tick_ms + 5000U;
            motor.prepared_ready = true;
            const CgTrajectory prepared = motor.prepared;
            const uint32_t prepared_start = motor.prepared_start_ms;
            const uint32_t trajectory_start = motor.trajectory_start_ms;
            const uint32_t controller_start = motor.controller.start_timestamp_ms;
            const uint32_t generation = motor.plan_generation;
            const float target = motor.requested_target_rad;
            const CyberGearConfig config = motor.config;
            const unsigned int first_command = tx_count;
            const uint32_t max_steps = 4U * config.saturation_timeout_ms / config.controller.period_ms;
            for (uint32_t step = 0U; step < max_steps && motor.recovery_count < 2U; ++step) {
                const float previous_current = motor.controller.last_queued_current_a;
                const uint32_t previous_count = motor.recovery_count;
                const int feedback_direction = variant != 0U && step % 2U != 0U ? -direction : direction;
                saturation_tick(&motor, 0.02f * (float)feedback_direction);
                assert(motor.state == CG_STATE_RUNNING && motor.fault == CG_FAULT_NONE);
                assert(motor.managed && !motor.first_cyclic);
                assert(motor.controller.start_timestamp_ms == controller_start);
                if (motor.recovery_count == 0U) {
                    assert(motor.trajectory_start_ms == trajectory_start);
                    assert(motor.plan_generation == generation);
                    assert(motor.prepared_ready && motor.prepared_start_ms == prepared_start);
                    assert(memcmp(&motor.prepared, &prepared, sizeof(prepared)) == 0);
                } else {
                    assert(motor.plan_generation >= generation + motor.recovery_count);
                    if (motor.recovery_zero_active) assert(!motor.prepared_ready);
                }
                assert(motor.requested_target_rad == target);
                assert(memcmp(&motor.config, &config, sizeof(config)) == 0);
                assert(motor.controller.applied_current_valid);
                assert(motor.controller.last_queued_timestamp_ms == tick_ms);
                assert(motor.controller.rx_sequence == motor.feedback.rx_sequence);
                const float change = motor.controller.last_queued_current_a - previous_current;
                const float period_s = (float)config.controller.period_ms * 0.001f;
                assert(fabsf(motor.controller.last_queued_current_a) <= config.controller.current_limit_a);
                if (motor.recovery_count == previous_count) {
                    assert(change <= config.controller.current_rise_a_s * period_s + 1e-5f);
                    assert(-change <= config.controller.current_fall_a_s * period_s + 1e-5f);
                }
                if (motor.recovery_zero_active) assert(motor.controller.last_queued_current_a == 0.0f);
                if (motor.recovery_count != previous_count) {
                    assert(motor.controller.b0 == config.controller.b0_initial);
                    assert(motor.dynamics.b0 == config.dynamics.fixed_b0);
                    assert(motor.controller.disturbance_rad_s2 == 0.0f);
                    assert(motor.controller.compensation_elapsed_ms == 0U);
                    assert(!motor.saturation_active);
                }
            }
            assert(motor.recovery_count == 2U);
            for (unsigned int command = first_command; command < tx_count; ++command) {
                assert(command_type(command) == CYBERGEAR_COMM_WRITE_PARAMETER);
                const uint16_t parameter = (uint16_t)(tx[command].data[0] |
                    ((uint16_t)tx[command].data[1] << 8));
                assert(parameter == CYBERGEAR_PARAM_IQ_REF);
            }
        }
    }
}

static void saturation_recovery_preserves_protection_test(void)
{
    for (unsigned int scenario = 0U; scenario < 2U; ++scenario) {
        CyberGearMotor motor;
        clear_mock(1000U);
        start_saturation_fixture(&motor, true, true);
        motor.config.tracking_error_rad = 0.001f;
        motor.config.recover_on_tracking = scenario != 0U;
        motor.tracking_active = true;
        motor.tracking_since_ms = scenario == 0U ? tick_ms - motor.config.tracking_timeout_ms : tick_ms;
        motor.saturation_active = true;
        motor.saturation_since_ms = tick_ms - motor.config.saturation_timeout_ms;
        motor.stall_active = scenario != 0U;
        motor.stall_since_ms = tick_ms - motor.config.stall_timeout_ms;
        motor.stall_start_rad = 0.02f;
        assert(cybergear_controller_commit_queued(&motor.controller, motor.config.controller.current_limit_a, tick_ms));
        const unsigned int before = tx_count;
        saturation_tick(&motor, 0.02f);
        assert(motor.state == CG_STATE_STOPPING);
        assert(motor.fault == (scenario == 0U ? CG_FAULT_TRACKING : CG_FAULT_STALL));
        assert(motor.recovery_count == 0U);
        assert_only_stop_after(before);
    }
}

static void recovery_zero_current_test(void)
{
    for (unsigned int trigger = 0U; trigger < 2U; ++trigger) {
        for (int direction = -1; direction <= 1; direction += 2) {
            CyberGearMotor motor;
            clear_mock(1000U);
            configure(&motor);
            CyberGearConfig config = motor.config;
            config.controller.period_ms = direction < 0 ? 5U : 10U;
            config.controller.current_rise_a_s = 2.0f;
            config.controller.current_fall_a_s = 3.0f;
            config.dynamics.current_slew_a_s = 2.0f;
            config.dynamics.reserve_slew_a_s = 1.0f;
            config.tracking_error_rad = trigger == 0U ? 0.001f : 1.0f;
            config.recovery_zero_ms = 203U;
            assert(cybergear_configure(&motor, &config));
            start_configured(&motor, CYBERGEAR_RUN_MODE_CURRENT);
            tick_ms = UINT32_MAX - 100U;
            motor.last_control_ms = tick_ms;
            motor.controller.last_control_timestamp_ms = tick_ms;
            motor.controller.observer_timestamp_ms = tick_ms;
            motor.controller.last_queued_timestamp_ms = tick_ms;
            feedback(&motor, 2U, 0.0f, 0.0f, 25.0f, 0U);
            motor.controller.rx_timestamp_ms = tick_ms;
            assert(cybergear_controller_commit_queued(&motor.controller, (float)direction, tick_ms));
            motor.controller.b0 = motor.dynamics.b0 = 12.0f;
            motor.controller.disturbance_rad_s2 = 1.0f;
            motor.controller.disturbance_current_a = 0.3f;
            motor.tracking_active = trigger == 0U;
            motor.tracking_since_ms = tick_ms - config.tracking_timeout_ms;
            motor.saturation_active = trigger == 1U;
            motor.saturation_since_ms = tick_ms - config.saturation_timeout_ms;
            const uint32_t original_start = motor.controller.start_timestamp_ms;
            const uint32_t original_trajectory_start = motor.trajectory_start_ms;
            const float target = motor.requested_target_rad;
            const unsigned int first_command = tx_count;
            saturation_tick(&motor, 0.02f * (float)direction);
            const uint32_t recovery_start = tick_ms;
            assert(motor.recovery_zero_active && motor.recovery_count == 1U);
            while (true) {
                assert(motor.state == CG_STATE_RUNNING && motor.fault == CG_FAULT_NONE);
                assert(motor.controller.last_queued_current_a == 0.0f);
                assert(motor.controller.applied_current_estimate_a == 0.0f && motor.controller.applied_current_valid);
                assert(motor.output.current_a == 0.0f && motor.controller.disturbance_current_a == 0.0f);
                assert(motor.controller.disturbance_rad_s2 == 0.0f);
                assert(motor.controller.b0 == config.controller.b0_initial && motor.dynamics.b0 == config.dynamics.fixed_b0);
                assert(motor.controller.compensation_elapsed_ms == 0U);
                assert(motor.recovery_count == 1U && motor.recovery_started_ms == recovery_start);
                assert(!motor.tracking_active && !motor.saturation_active && !motor.stall_active);
                assert(motor.controller.start_timestamp_ms == original_start);
                assert(motor.trajectory_start_ms == original_trajectory_start);
                assert(motor.requested_target_rad == target);
                assert(motor.controller.rx_sequence == motor.feedback.rx_sequence);
                float sent_current;
                memcpy(&sent_current, tx[tx_count - 1U].data + 4, sizeof(sent_current));
                assert(sent_current == 0.0f);
                saturation_tick(&motor, 0.02f * (float)direction);
                if (tick_ms - recovery_start >= config.recovery_zero_ms) break;
                assert(motor.recovery_zero_active);
            }
            assert(!motor.recovery_zero_active && motor.recovery_count == 1U);
            assert(motor.trajectory_start_ms == tick_ms);
            assert(motor.trajectory.point.q_rad == motor.feedback.position_rad);
            assert(motor.trajectory.point.v_rad_s == 0.0f && motor.trajectory.point.a_rad_s2 == 0.0f);
            assert(motor.requested_target_rad == target);
            assert(tick_ms - recovery_start < config.recovery_zero_ms + config.controller.period_ms);
            assert(fabsf(motor.controller.last_queued_current_a) <=
                config.controller.current_fall_a_s * (float)config.controller.period_ms * 0.001f + 1e-6f);
            for (unsigned int command = first_command; command < tx_count; ++command) {
                assert(command_type(command) == CYBERGEAR_COMM_WRITE_PARAMETER);
                assert(tx[command].data[0] == (CYBERGEAR_PARAM_IQ_REF & 0xffU));
                assert(tx[command].data[1] == (CYBERGEAR_PARAM_IQ_REF >> 8));
            }
        }
    }
}

static void recovery_replan_test(void)
{
    for (unsigned int scenario = 0U; scenario < 3U; ++scenario) {
        CyberGearMotor motor;
        clear_mock(1000U);
        configure(&motor);
        CyberGearConfig config = motor.config;
        config.controller.period_ms = scenario == 0U ? 5U : 10U;
        config.recovery_zero_ms = 703U;
        assert(cybergear_configure(&motor, &config));
        start_configured(&motor, CYBERGEAR_RUN_MODE_CURRENT);
        CgTrajectoryPoint old_reference = {.q_rad = 0.6f};
        assert(cg_trajectory_reset(&motor.trajectory, &old_reference, &motor.effective_limits));
        motor.requested_target_rad = 0.8f;
        motor.prepared = motor.trajectory;
        assert(cg_trajectory_plan(&motor.prepared, motor.requested_target_rad, &motor.effective_limits));
        motor.prepared_ready = true;
        motor.prepared_start_ms = tick_ms + 2U * config.controller.period_ms;
        motor.tracking_active = true;
        motor.tracking_since_ms = tick_ms - config.tracking_timeout_ms;
        const uint32_t generation = motor.plan_generation;
        saturation_tick(&motor, 0.02f);
        assert(motor.recovery_zero_active && !motor.prepared_ready);
        assert(motor.plan_generation == generation + 1U);
        const uint32_t recovery_start = tick_ms;
        motor.requested_target_rad = scenario == 0U ? 0.4f : 0.8f;
        while (tick_ms + config.controller.period_ms - recovery_start < config.recovery_zero_ms) {
            saturation_tick(&motor, 0.04f);
            assert(motor.state == CG_STATE_RUNNING && motor.recovery_zero_active);
            assert(!motor.prepared_ready && motor.plan_generation == generation + 1U);
            assert(motor.controller.last_queued_current_a == 0.0f);
        }
        tick_ms += config.controller.period_ms;
        feedback(&motor, 2U, 0.05f, 0.0f, 25.0f, 0U);
        if (scenario == 2U) motor.requested_target_rad = motor.feedback.position_rad;
        const CyberGearController before = motor.controller;
        CyberGearController expected = before;
        CyberGearControllerReference reference = {motor.feedback.position_rad, 0.0f, 0.0f};
        CyberGearControllerMeasurement measurement = {motor.feedback.position_rad,
            motor.feedback.rx_sequence, motor.feedback.last_received_ms, true};
        CyberGearControllerOutput output;
        assert(cybergear_controller_step(&expected, &reference, &measurement, tick_ms,
            config.controller.b0_initial, &output));
        assert(cybergear_control_position_adrc(&motor, motor.requested_target_rad));
        assert(!motor.recovery_zero_active && !motor.prepared_ready);
        assert(motor.plan_generation == generation + 2U);
        assert(motor.trajectory.point.q_rad == motor.feedback.position_rad);
        assert(motor.trajectory_start_ms == tick_ms && motor.requested_since_ms == tick_ms);
        assert(!motor.tracking_active);
        assert(motor.controller.position_rad == expected.position_rad);
        assert(motor.controller.velocity_rad_s == expected.velocity_rad_s);
        assert(motor.output.current_a == output.current_a);
        cybergear_service(&motor);
        if (scenario == 2U) {
            assert(!motor.prepared_ready && !motor.trajectory.active);
        } else {
            assert(motor.prepared_ready);
            assert(motor.prepared.initial.q_rad == motor.feedback.position_rad);
            assert(motor.prepared.initial.v_rad_s == 0.0f && motor.prepared.initial.a_rad_s2 == 0.0f);
            assert(motor.prepared.target_rad == motor.requested_target_rad);
            assert(motor.prepared_start_ms == tick_ms + config.planner_lead_ms);
            assert(cg_trajectory_validate(&motor.prepared, &motor.effective_limits));
        }
    }
}

static void interrupt_plan_with_recovery(void)
{
    CyberGearMotor *motor = planning_race_motor;
    tick_ms += motor->config.controller.period_ms;
    feedback(motor, 2U, 0.02f, 0.0f, 25.0f, 0U);
    assert(cybergear_control_position_adrc(motor, motor->requested_target_rad));
    assert(motor->recovery_zero_active);
}

static void recovery_replan_race_test(void)
{
    CyberGearMotor motor;
    clear_mock(1000U);
    start(&motor, CYBERGEAR_RUN_MODE_CURRENT);
    const CgTrajectoryPoint old_reference = {.q_rad = 0.6f};
    assert(cg_trajectory_reset(&motor.trajectory, &old_reference, &motor.effective_limits));
    motor.requested_target_rad = 0.8f;
    motor.tracking_active = true;
    motor.tracking_since_ms = tick_ms - motor.config.tracking_timeout_ms;
    const uint32_t generation = motor.plan_generation;
    planning_race_motor = &motor;
    irq_restore_hook = interrupt_plan_with_recovery;
    cybergear_service(&motor);
    assert(irq_restore_hook == NULL);
    assert(motor.recovery_zero_active && motor.plan_generation == generation + 1U);
    assert(!motor.prepared_ready && !motor.planning_fault);
    planning_race_motor = NULL;
}

static void recovery_replan_release_test(void)
{
    for (int direction = -1; direction <= 1; direction += 2) {
        CyberGearMotor motor;
        clear_mock(1000U);
        configure(&motor);
        CyberGearConfig config = motor.config;
        config.controller.period_ms = direction < 0 ? 5U : 10U;
        assert(cybergear_configure(&motor, &config));
        start_configured(&motor, CYBERGEAR_RUN_MODE_CURRENT);
        const uint32_t started = tick_ms;
        const float target = (float)direction * 0.8f;
        const float period_s = (float)config.controller.period_ms * 0.001f;
        float position = motor.feedback.position_rad;
        float velocity = 0.0f;
        bool released = false;
        bool reached = false;
        for (unsigned int step = 0U; step < 15000U / config.controller.period_ms; ++step) {
            if (motor.recovery_zero_active &&
                tick_ms - motor.recovery_started_ms >= config.recovery_zero_ms / 2U) released = true;
            if (released) {
                const float acceleration = config.controller.b0_initial * motor.controller.last_queued_current_a - 0.2f * velocity;
                position += period_s * velocity + 0.5f * period_s * period_s * acceleration;
                velocity += period_s * acceleration;
            }
            tick_ms += config.controller.period_ms;
            feedback(&motor, 2U, position, velocity, 25.0f, 0U);
            tx_count = 0U;
            assert(cybergear_control_position_adrc(&motor, target));
            cybergear_service(&motor);
            assert(motor.recovery_count <= 1U);
            assert(motor.controller.start_timestamp_ms == started);
            if (released && !motor.recovery_zero_active && !motor.trajectory.active && !motor.prepared_ready &&
                fabsf(target - position) < 0.005f && fabsf(velocity) < 0.03f) {
                reached = true;
                break;
            }
        }
        assert(released && reached && motor.recovery_count == 1U);
        printf("Recovery release: %u Hz target=%.2f reached after %lu ms without another recovery\n",
            1000U / config.controller.period_ms, (double)target, (unsigned long)(tick_ms - started));
    }
}

static void recovery_replan_failure_test(void)
{
    for (unsigned int scenario = 0U; scenario < 3U; ++scenario) {
        CyberGearMotor motor;
        clear_mock(1000U);
        start(&motor, CYBERGEAR_RUN_MODE_CURRENT);
        const CgTrajectoryPoint old_reference = {.q_rad = 0.6f};
        assert(cg_trajectory_reset(&motor.trajectory, &old_reference, &motor.effective_limits));
        motor.requested_target_rad = 0.8f;
        motor.tracking_active = true;
        motor.tracking_since_ms = tick_ms - motor.config.tracking_timeout_ms;
        saturation_tick(&motor, 0.0f);
        assert(motor.recovery_zero_active);
        if (scenario == 0U) {
            motor.effective_limits.position_max_rad = 0.01f;
            motor.effective_limits.position_min_rad = -0.01f;
        }
        if (scenario == 1U) motor.effective_limits.duration_max_s = motor.effective_limits.duration_min_s;
        while (motor.recovery_zero_active) {
            tick_ms += motor.config.controller.period_ms;
            feedback(&motor, 2U, 0.02f, 0.0f, 25.0f, 0U);
            (void)cybergear_control_position_adrc(&motor, motor.requested_target_rad);
        }
        if (scenario == 0U) {
            assert(motor.fault == CG_FAULT_POSITION);
        } else {
            assert(motor.fault == CG_FAULT_NONE);
            if (scenario == 1U) {
                cybergear_service(&motor);
                assert(motor.planning_fault);
            }
            for (unsigned int step = 0U; step <= motor.config.planner_timeout_ms / motor.config.controller.period_ms + 1U &&
                motor.state == CG_STATE_RUNNING; ++step) {
                tick_ms += motor.config.controller.period_ms;
                feedback(&motor, 2U, 0.02f, 0.0f, 25.0f, 0U);
                (void)cybergear_control_position_adrc(&motor, motor.requested_target_rad);
            }
            assert(motor.fault == CG_FAULT_PLANNER);
        }
        assert(motor.state == CG_STATE_STOPPING);
    }
}

static void recovery_zero_failure_test(void)
{
    for (unsigned int scenario = 0U; scenario < 3U; ++scenario) {
        CyberGearMotor motor;
        clear_mock(1000U);
        start_saturation_fixture(&motor, true, true);
        motor.config.tracking_error_rad = 0.001f;
        motor.tracking_active = true;
        motor.tracking_since_ms = tick_ms - motor.config.tracking_timeout_ms;
        assert(cybergear_controller_commit_queued(&motor.controller, 1.0f, tick_ms));
        if (scenario == 0U) send_status = HAL_ERROR;
        saturation_tick(&motor, 0.02f);
        if (scenario == 0U) {
            assert(motor.fault == CG_FAULT_TX);
            assert(motor.controller.last_queued_current_a == 1.0f && !motor.controller.applied_current_valid);
        } else {
            assert(motor.recovery_zero_active && motor.controller.last_queued_current_a == 0.0f);
            if (scenario == 1U) {
                tick_ms += motor.config.controller.feedback_timeout_ms + 1U;
                assert(!cybergear_control_position_adrc(&motor, motor.requested_target_rad));
                assert(motor.fault == CG_FAULT_FEEDBACK);
            } else {
                assert(cybergear_stop(&motor));
                assert(motor.fault == CG_FAULT_REQUESTED_STOP);
            }
        }
        assert(motor.state == CG_STATE_STOPPING);
        assert(!motor.recovery_zero_active);
    }
}

static void standalone_feedback_delay_test(void)
{
    for (unsigned int original_observer = 0U; original_observer < 2U; ++original_observer) {
        CyberGearMotor motor;
        clear_mock(1000U);
        assert(cybergear_init(&motor, &can, 0x7fU, 0xfeU));
        assert(cybergear_test_configure(&motor));
        if (original_observer != 0U) {
            CyberGearConfig config = motor.config;
            config.controller.observer_rad_s = 10.0f;
            config.recover_on_saturation = false;
            assert(cybergear_configure(&motor, &config));
        } else assert(motor.config.controller.observer_rad_s <= 6.0f);
        start_configured(&motor, CYBERGEAR_RUN_MODE_CURRENT);
        CyberGearTestMotion motion;
        cybergear_test_motion_init(&motion, tick_ms);
        const uint32_t started_ms = tick_ms;
        float position_rad = motor.feedback.position_rad;
        float velocity_rad_s = 0.0f;
        float position_history[256] = {0};
        float velocity_history[256] = {0};
        float current_history[256] = {0};
        float peak_current_a = 0.0f;
        unsigned int amplitude_clips = 0U;
        CyberGearTestResult result = CG_TEST_WAIT;
        while (tick_ms - started_ms < 8U * CG_TEST_LEG_TIMEOUT_MS &&
               motion.completed_legs < 8U && !motion.halted) {
            const uint32_t elapsed_ms = tick_ms - started_ms + 1U;
            const float delayed_current_a = elapsed_ms > 10U ?
                current_history[(elapsed_ms - 11U) % 256U] : 0.0f;
            const float acceleration_rad_s2 = delayed_current_a - 0.45f - 0.2f * velocity_rad_s;
            position_rad += 0.001f * velocity_rad_s + 0.0000005f * acceleration_rad_s2;
            velocity_rad_s += 0.001f * acceleration_rad_s2;
            position_history[elapsed_ms % 256U] = position_rad;
            velocity_history[elapsed_ms % 256U] = velocity_rad_s;
            tick_ms++;
            if (elapsed_ms % 11U == 0U) {
                const float noise_rad = 0.003f * sinf(0.037f * (float)elapsed_ms);
                feedback(&motor, 2U, position_history[(elapsed_ms - 10U) % 256U] + noise_rad,
                    velocity_history[(elapsed_ms - 10U) % 256U], 25.0f, 0U);
            }
            if (elapsed_ms % motor.config.controller.period_ms == 0U) {
                tx_count = 0U;
                const float previous_current_a = motor.controller.last_queued_current_a;
                (void)cybergear_control_position_adrc(&motor, motion.target_rad);
                cybergear_service(&motor);
                peak_current_a = fmaxf(peak_current_a, fabsf(motor.controller.last_queued_current_a));
                assert(peak_current_a <= CG_TEST_CURRENT_LIMIT_A);
                const float current_change_a = motor.controller.last_queued_current_a - previous_current_a;
                const float period_s = 0.001f * (float)motor.config.controller.period_ms;
                assert(current_change_a <= motor.config.controller.current_rise_a_s * period_s + 1e-5f);
                assert(-current_change_a <= motor.config.controller.current_fall_a_s * period_s + 1e-5f);
                amplitude_clips += motor.output.amplitude_limited ? 1U : 0U;
                const bool trajectory_done = !motor.trajectory.active && !motor.prepared_ready &&
                    fabsf(motor.trajectory.target_rad - motion.target_rad) <=
                        motor.effective_limits.target_tolerance_rad &&
                    fabsf(motor.requested_target_rad - motion.target_rad) <=
                        motor.effective_limits.target_tolerance_rad;
                const float arrival_error_rad = fabsf(motion.target_rad - position_rad);
                result = cybergear_test_motion_update(&motion, tick_ms,
                    motor.state == CG_STATE_RUNNING,
                    tick_ms - motor.feedback.last_received_ms <= motor.config.controller.feedback_timeout_ms,
                    trajectory_done, motor.feedback.position_rad, motor.controller.velocity_rad_s);
                if (original_observer == 0U && result == CG_TEST_NEW_TARGET) {
                    assert(arrival_error_rad <= CG_TEST_REACHED_RAD);
                    assert(fabsf(velocity_rad_s) <= 0.03f);
                }
            }
            current_history[elapsed_ms % 256U] = motor.controller.last_queued_current_a;
        }
        assert(amplitude_clips == 0U);
        if (original_observer != 0U) {
            assert(motion.halted && motion.completed_legs < 8U);
            assert(result == CG_TEST_STOP_FAULT && motor.fault == CG_FAULT_SATURATION);
        } else {
            assert(!motion.halted && motion.completed_legs == 8U);
            assert(motor.state == CG_STATE_RUNNING && motor.fault == CG_FAULT_NONE);
        }
        printf("Standalone delayed/noisy feedback: observer=%.1f rad/s, legs=%lu, fault=%u, peak current=%.3f A\n",
            (double)motor.config.controller.observer_rad_s, (unsigned long)motion.completed_legs,
            (unsigned int)motor.fault, (double)peak_current_a);
    }
}

static void standalone_loaded_motion_test(void)
{
    for (int direction = -1; direction <= 1; direction += 2) {
        CyberGearMotor motor;
        clear_mock(1000U);
        assert(cybergear_init(&motor, &can, 0x7fU, 0xfeU));
        assert(cybergear_test_configure(&motor));
        start_configured(&motor, CYBERGEAR_RUN_MODE_CURRENT);
        CyberGearTestMotion motion;
        cybergear_test_motion_init(&motion, tick_ms);
        const uint32_t started_ms = tick_ms;
        const float load_current_a = 0.94f * (float)direction;
        const float plant_b0_rad_s2_a = 1.0f;
        float position_rad = motor.feedback.position_rad;
        float velocity_rad_s = 0.0f;
        float peak_current_a = 0.0f;
        float maximum_arrival_error_rad = 0.0f;
        float maximum_arrival_speed_rad_s = 0.0f;
        CyberGearTestResult result = CG_TEST_WAIT;
        while (tick_ms - started_ms < 8U * CG_TEST_LEG_TIMEOUT_MS &&
               motion.completed_legs < 8U && !motion.halted) {
            const float acceleration_rad_s2 = plant_b0_rad_s2_a *
                (motor.controller.last_queued_current_a - load_current_a) - 0.2f * velocity_rad_s;
            position_rad += 0.001f * velocity_rad_s + 0.0000005f * acceleration_rad_s2;
            velocity_rad_s += 0.001f * acceleration_rad_s2;
            tick_ms++;
            if ((tick_ms - started_ms) % motor.config.controller.period_ms != 0U) continue;
            tx_count = 0U;
            feedback(&motor, 2U, position_rad, velocity_rad_s, 25.0f, 0U);
            const float previous_current_a = motor.controller.last_queued_current_a;
            (void)cybergear_control_position_adrc(&motor, motion.target_rad);
            cybergear_service(&motor);
            peak_current_a = fmaxf(peak_current_a, fabsf(motor.controller.last_queued_current_a));
            assert(peak_current_a <= CG_TEST_CURRENT_LIMIT_A);
            const float current_change_a = motor.controller.last_queued_current_a - previous_current_a;
            const float period_s = 0.001f * (float)motor.config.controller.period_ms;
            assert(current_change_a <= motor.config.controller.current_rise_a_s * period_s + 1e-5f);
            assert(-current_change_a <= motor.config.controller.current_fall_a_s * period_s + 1e-5f);
            assert(fabsf(motor.output.disturbance_current_a) <= motor.config.dynamics.reserve_current_a);
            const bool trajectory_done = !motor.trajectory.active && !motor.prepared_ready &&
                fabsf(motor.trajectory.target_rad - motion.target_rad) <=
                    motor.effective_limits.target_tolerance_rad &&
                fabsf(motor.requested_target_rad - motion.target_rad) <=
                    motor.effective_limits.target_tolerance_rad;
            const float arrival_error_rad = fabsf(motion.target_rad - position_rad);
            result = cybergear_test_motion_update(&motion, tick_ms,
                motor.state == CG_STATE_RUNNING, true, trajectory_done,
                motor.feedback.position_rad, motor.controller.velocity_rad_s);
            if (result == CG_TEST_NEW_TARGET) {
                maximum_arrival_error_rad = fmaxf(maximum_arrival_error_rad, arrival_error_rad);
                maximum_arrival_speed_rad_s = fmaxf(maximum_arrival_speed_rad_s, fabsf(velocity_rad_s));
            }
        }
        if (motion.halted || motion.completed_legs != 8U)
            fprintf(stderr, "Standalone load=%.2f A: legs=%lu result=%u fault=%u error=%.6f rad current=%.6f A compensation=%.6f A\n",
                (double)load_current_a, (unsigned long)motion.completed_legs,
                (unsigned int)result, (unsigned int)motor.fault,
                (double)(motion.target_rad - motor.feedback.position_rad),
                (double)motor.controller.last_queued_current_a,
                (double)motor.output.disturbance_current_a);
        assert(!motion.halted && motion.completed_legs == 8U);
        assert(motor.state == CG_STATE_RUNNING && motor.fault == CG_FAULT_NONE);
        assert(maximum_arrival_error_rad <= CG_TEST_REACHED_RAD);
        assert(maximum_arrival_speed_rad_s <= 0.03f);
        printf("Standalone synthetic load=%.2f A: 8 legs, maximum arrival error=%.6f rad, peak current=%.3f A\n",
            (double)load_current_a, (double)maximum_arrival_error_rad, (double)peak_current_a);
    }
}

static void reinitialize_stop_confirmation_test(void)
{
    CyberGearMotor uninitialized = {0};
    assert(!cybergear_request_reinitialize_stop(NULL));
    assert(!cybergear_request_reinitialize_stop(&uninitialized));
    for (unsigned int scenario = 0U; scenario < 3U; ++scenario) {
        CyberGearMotor motor;
        clear_mock(1000U);
        if (scenario == 0U) {
            configure(&motor);
            feedback(&motor, 2U, 0.1f, 0.2f, 25.0f, 0U);
        } else {
            start(&motor, CYBERGEAR_RUN_MODE_CURRENT);
            if (scenario == 2U) {
                normal_tick(&motor, NAN);
                tick_ms += motor.config.stop_timeout_ms + 1U;
                (void)cybergear_control_position_adrc(&motor, 0.0f);
                assert(motor.state == CG_STATE_FAULT);
                assert(!cybergear_reset_fault(&motor));
            }
        }
        const CyberGearFault expected_fault = scenario == 2U ? CG_FAULT_TARGET : CG_FAULT_REQUESTED_STOP;
        const CyberGearConfig previous_config = motor.config;
        const uint32_t previous_generation = motor.plan_generation;
        const unsigned int first_stop = tx_count;
        motor.prepared_ready = true;
        assert(cybergear_request_reinitialize_stop(&motor));
        assert(motor.state == CG_STATE_STOPPING && motor.managed);
        assert(motor.fault == expected_fault);
        assert(memcmp(&motor.config, &previous_config, sizeof(previous_config)) == 0);
        assert(motor.plan_generation == previous_generation + 1U);
        assert(!motor.prepared_ready && !motor.controller.applied_current_valid);
        assert(!motor.stop_queued && !motor.reset_confirmed && !motor.stationary);
        assert(tx_count == first_stop);
        assert(!cybergear_reset_fault(&motor));
        (void)cybergear_control_position_adrc(&motor, 0.0f);
        assert(motor.stop_queued && !motor.reset_confirmed);
        assert(!cybergear_reset_fault(&motor));
        tick_ms += motor.config.controller.period_ms;
        feedback(&motor, 0U, 0.1f, 0.2f, 25.0f, 0U);
        (void)cybergear_control_position_adrc(&motor, 0.0f);
        assert(motor.reset_confirmed && !motor.stationary);
        assert(!cybergear_reset_fault(&motor));
        for (unsigned int sample = 0U; sample < 30U && motor.state != CG_STATE_FAULT; ++sample) {
            tick_ms += motor.config.controller.period_ms;
            feedback(&motor, 0U, 0.1f, 0.0f, 25.0f, 0U);
            (void)cybergear_control_position_adrc(&motor, 0.0f);
            assert(motor.fault == expected_fault);
        }
        assert(motor.state == CG_STATE_FAULT && motor.stationary && motor.reset_confirmed);
        assert_only_stop_after(first_stop);
        assert(cybergear_reset_fault(&motor));
        assert(motor.state == CG_STATE_OFF && !motor.managed && motor.fault == CG_FAULT_NONE);
        assert(memcmp(&motor.config, &previous_config, sizeof(previous_config)) == 0);
        assert_only_stop_after(first_stop);
    }
}

static void reinitialize_unconfirmed_stop_test(void)
{
    CyberGearMotor motor;
    clear_mock(1000U);
    start(&motor, CYBERGEAR_RUN_MODE_CURRENT);
    feedback(&motor, 0U, 0.0f, 0.0f, 25.0f, 0U);
    const unsigned int first_stop = tx_count;
    assert(cybergear_request_reinitialize_stop(&motor));
    (void)cybergear_control_position_adrc(&motor, 0.0f);
    tick_ms += motor.config.controller.feedback_timeout_ms + 1U;
    (void)cybergear_control_position_adrc(&motor, 0.0f);
    assert(!motor.reset_confirmed && !motor.stationary);
    assert(!cybergear_reset_fault(&motor));
    tick_ms += motor.config.stop_timeout_ms;
    (void)cybergear_control_position_adrc(&motor, 0.0f);
    assert(motor.state == CG_STATE_FAULT);
    assert(!cybergear_reset_fault(&motor));

    assert(cybergear_request_reinitialize_stop(&motor));
    send_status = HAL_ERROR;
    for (unsigned int sample = 0U; sample < 30U; ++sample) {
        tick_ms += motor.config.controller.period_ms;
        feedback(&motor, 0U, 0.0f, 0.0f, 25.0f, 0U);
        (void)cybergear_control_position_adrc(&motor, 0.0f);
        assert(!motor.stop_queued && !motor.reset_confirmed);
        assert(!cybergear_reset_fault(&motor));
    }
    send_status = HAL_OK;
    assert(cybergear_request_reinitialize_stop(&motor));
    (void)cybergear_control_position_adrc(&motor, 0.0f);
    assert(motor.stop_queued && !motor.reset_confirmed && !motor.stationary);
    for (unsigned int sample = 0U; sample < 30U && motor.state != CG_STATE_FAULT; ++sample) {
        tick_ms += motor.config.controller.period_ms;
        feedback(&motor, 0U, 0.0f, 0.0f, 25.0f, 1U);
        (void)cybergear_control_position_adrc(&motor, 0.0f);
        assert(!cybergear_reset_fault(&motor));
    }
    assert(motor.state == CG_STATE_FAULT && motor.reset_confirmed && motor.stationary);
    bus_off = true;
    tick_ms += motor.config.controller.period_ms;
    feedback(&motor, 0U, 0.0f, 0.0f, 25.0f, 0U);
    (void)cybergear_control_position_adrc(&motor, 0.0f);
    assert(!cybergear_reset_fault(&motor));
    bus_off = false;
    tick_ms += motor.config.controller.period_ms;
    feedback(&motor, 0U, 0.0f, 0.0f, 25.0f, 0U);
    (void)cybergear_control_position_adrc(&motor, 0.0f);
    tick_ms += motor.config.controller.feedback_timeout_ms + 1U;
    assert(!cybergear_reset_fault(&motor));
    tick_ms += motor.config.controller.period_ms;
    feedback(&motor, 0U, 0.0f, 0.0f, 25.0f, 0U);
    (void)cybergear_control_position_adrc(&motor, 0.0f);
    assert(cybergear_reset_fault(&motor));
    assert_only_stop_after(first_stop);
}

static void configuration_boundaries_test(void)
{
    CyberGearConfig config = fixture();
    config.stationary_dwell_ms = UINT32_C(0x80000032);
    assert(!cybergear_config_valid(&config));

}

void test_bus_monitoring(void)
{
    for (unsigned int scenario = 0U; scenario < 6U; ++scenario) {
        CyberGearMotor motor;
        clear_mock(1000U);
        bus_tx_errors = 128U;
        bus_rx_errors = 96U;
        bus_warning = scenario == 1U;
        bus_passive = scenario == 2U;
        bus_off = scenario == 3U;
        bus_status_result = scenario == 4U ? HAL_ERROR : HAL_OK;
        bus_counters_result = scenario == 5U ? HAL_ERROR : HAL_OK;
        start(&motor, CYBERGEAR_RUN_MODE_CURRENT);
        const unsigned int before = tx_count;
        normal_tick(&motor, 0.0f);
        assert(motor.state == CG_STATE_RUNNING && motor.fault == CG_FAULT_NONE);
        assert(tx_count == before + 1U);
        assert(command_type(before) == CYBERGEAR_COMM_WRITE_PARAMETER);
        if (bus_counters_result == HAL_OK) assert(motor.tec == 128U && motor.rec == 96U);
    }
    CyberGearMotor motor;
    clear_mock(1000U);
    start(&motor, CYBERGEAR_RUN_MODE_CURRENT);
    bus_tx_errors = 8U;
    tick_ms += motor.config.controller.feedback_timeout_ms + 1U;
    assert(!cybergear_control_position_adrc(&motor, 0.0f));
    assert(motor.fault == CG_FAULT_FEEDBACK);
    puts("CAN bus: status monitoring does not stop startup/control; stale feedback still stops.");
}

void test_saturation(void)
{
    CyberGearConfig config;
    cybergear_config_defaults(&config);
    assert(config.recover_on_saturation == (CYBERGEAR_SATURATION_RECOVERY_ENABLED != 0));
    assert(config.recover_on_tracking == (CYBERGEAR_TRACKING_RECOVERY_ENABLED != 0));
    assert(config.recovery_zero_ms == CYBERGEAR_RECOVERY_ZERO_MS);
    config.recovery_zero_ms = 0U;
    assert(!cybergear_config_valid(&config));
    sustained_saturation_protection_test();
    transient_saturation_reset_test();
    sustained_saturation_recovery_test();
    saturation_recovery_preserves_protection_test();
    recovery_zero_current_test();
    recovery_replan_test();
    recovery_replan_race_test();
    recovery_replan_release_test();
    recovery_replan_failure_test();
    recovery_zero_failure_test();
    CyberGearMotor motor;
    clear_mock(1000U);
    start(&motor, CYBERGEAR_RUN_MODE_OPERATION);
    motor.config.tracking_error_rad = 0.001f;
    motor.tracking_active = true;
    motor.tracking_since_ms = tick_ms - motor.config.tracking_timeout_ms;
    saturation_tick(&motor, 0.02f);
    assert(motor.fault == CG_FAULT_TRACKING && motor.recovery_count == 0U);
    puts("Recovery: tracking/saturation, timed zero current, restart slew limits, and protection passed.");
}

void test_driver(void)
{
    reinitialize_stop_confirmation_test();
    reinitialize_unconfirmed_stop_test();
    configuration_boundaries_test();
    protocol_tests();
    startup_and_fault_tests();
    test_bus_monitoring();
    long_hold_wrap_test();
    timer_handoff_test();
    rejected_startup_tests();
    test_startup_handoff();
    streaming_and_protection_tests();
    unmanaged_fault_gate_test();
    configured_motion_protection_test();
    test_saturation();
    standalone_feedback_delay_test();
    standalone_loaded_motion_test();
    puts("Driver: protocol, ownership, startup, faults, TX failure, and wrap passed.");
}
