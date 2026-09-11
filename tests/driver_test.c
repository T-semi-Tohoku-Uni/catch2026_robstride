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
static FDCAN_HandleTypeDef can;

uint32_t HAL_GetTick(void) { return tick_ms; }
void HAL_Delay(uint32_t ms) { tick_ms += ms; }
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef *handle)
{ (void)handle; return fifo_free; }
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(const FDCAN_HandleTypeDef *handle,
    FDCAN_ProtocolStatusTypeDef *status)
{ (void)handle; memset(status, 0, sizeof(*status)); status->BusOff = bus_off; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(const FDCAN_HandleTypeDef *handle,
    FDCAN_ErrorCountersTypeDef *counters)
{ (void)handle; memset(counters, 0, sizeof(*counters)); return HAL_OK; }
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
    memset(&can, 0, sizeof(can));
    tx_count = 0U;
    tick_ms = now;
    fifo_free = 3U;
    send_status = HAL_OK;
    bus_off = false;
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
    assert(motor.fault == CG_FAULT_BUS);

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

static void start_saturation_fixture(CyberGearMotor *motor, bool amplitude)
{
    configure(motor);
    CyberGearConfig config = motor->config;
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
    (void)cybergear_control_position_adrc(motor, 0.0f);
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
            start_saturation_fixture(&motor, amplitude);
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
    start_saturation_fixture(&motor, false);
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

void test_driver(void)
{
    reinitialize_stop_confirmation_test();
    reinitialize_unconfirmed_stop_test();
    configuration_boundaries_test();
    protocol_tests();
    startup_and_fault_tests();
    long_hold_wrap_test();
    timer_handoff_test();
    rejected_startup_tests();
    streaming_and_protection_tests();
    unmanaged_fault_gate_test();
    configured_motion_protection_test();
    sustained_saturation_protection_test();
    transient_saturation_reset_test();
    standalone_feedback_delay_test();
    standalone_loaded_motion_test();
    puts("Driver: protocol, ownership, startup, faults, TX failure, and wrap passed.");
}
