#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../Core/Src/motor_app.c"

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "line %d: %s\n", __LINE__, #condition); exit(1); \
} } while (0)

/* Use the actual application, RobStride driver, and CAN feedback decoder.
   The fake HAL models independently powered motors and queue/delivery failures. */
FDCAN_HandleTypeDef hfdcan1 = {FDCAN1}, hfdcan3 = {FDCAN3};
TIM_HandleTypeDef htim6;

typedef struct {
    uint32_t available_after_ms;
    uint32_t stop_attempts;
    uint32_t stop_queued;
    uint32_t enable_queued;
    uint32_t enable_attempts;
    uint32_t mode_attempts;
    uint32_t current_attempts;
    uint32_t last_mode_reply_ms;
    uint32_t last_command;
    uint32_t terminal_stop_failures;
    float hold_target;
    uint8_t mode;
    uint8_t configured_parameters;
    bool contacted;
    bool enabled;
    bool active_reply_sent;
    bool ever_active_reply_sent;
    bool reply_pending;
} SimMotor;

static SimMotor sim_motors[256];
static uint32_t tick, elapsed_ms, interrupt_mask;
static uint32_t reset_count, error_count, cg_stop_count, cg_homing_stop_count;
static uint32_t cg_enable_count, timer_start_count;
static uint32_t timer_start_attempts;
static uint32_t cg_adrc_start_count;
static uint32_t cg_stop_failures;
static bool stopping, cg_enabled;
static const char *scenario;
static jmp_buf terminal_exit;
static bool rx_pending;
static FDCAN_RxHeaderTypeDef rx_header;
static uint8_t rx_bytes[64];
static uint8_t ids[ROBSTRIDE_MOTOR_COUNT] = {RIGHT_RS03_ID, LEFT_RS03_ID, EL05_ID};

static bool is_scenario(const char *name) { return strcmp(scenario, name) == 0; }

static void receive_feedback(uint8_t id, uint8_t mode, uint8_t fault)
{
    CHECK(!rx_pending);
    rx_header = (FDCAN_RxHeaderTypeDef){
        (2U << 24) | ((uint32_t)mode << 22) | ((uint32_t)fault << 16) |
            ((uint32_t)id << 8) | HOST_ID,
        FDCAN_DLC_BYTES_8, FDCAN_EXTENDED_ID, FDCAN_DATA_FRAME};
    memset(rx_bytes, 0, sizeof rx_bytes);
    uint16_t position_raw = (uint16_t)(0x8000U + (uint32_t)id * 100U);
    rx_bytes[0] = (uint8_t)(position_raw >> 8);
    rx_bytes[1] = (uint8_t)position_raw;
    rx_bytes[2] = 0x80U;
    rx_bytes[4] = 0x80U;
    rx_pending = true;
    motor_app_receive_feedback(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    CHECK(!rx_pending);
}

uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return interrupt_mask; }
void __disable_irq(void) { interrupt_mask = 1U; }
void __set_PRIMASK(uint32_t mask) { interrupt_mask = mask; }

void HAL_Delay(uint32_t milliseconds)
{
    CHECK(interrupt_mask == 0U);
    tick += milliseconds;
    elapsed_ms += milliseconds;
    CHECK(elapsed_ms <= 120000U); /* An accidental unbounded wait fails the test. */
    receive_feedback(CYBER_GEAR_ID, cg_enabled ? 2U : 0U, 0U);
    for (uint32_t i = 0; i < ROBSTRIDE_MOTOR_COUNT; ++i)
    {
        uint8_t id = ids[i];
        SimMotor *motor = &sim_motors[id];
        if (!motor->contacted || !motor->reply_pending || elapsed_ms < motor->available_after_ms) continue;
        motor->reply_pending = false;
        if (id == LEFT_RS03_ID && is_scenario("missing-feedback")) continue;
        if (id == LEFT_RS03_ID && is_scenario("persistent-stop") && !stopping) continue;
        if (id == RIGHT_RS03_ID && is_scenario("stale-ready") &&
            motor->ever_active_reply_sent && !stopping) continue;
        if (id == LEFT_RS03_ID && is_scenario("stale-feedback"))
        {
            /* A pre-Enable mode2 frame must not satisfy a later Enable attempt. */
            if (!motor->enabled)
                receive_feedback(id, motor->last_command == StopId ? 0U : 2U, 0U);
            continue;
        }
        uint8_t mode = motor->enabled ? 2U : 0U;
        uint8_t fault = 0U;
        if (id == LEFT_RS03_ID && is_scenario("wrong-mode")) mode = 1U;
        if (id == LEFT_RS03_ID && (is_scenario("fault") || is_scenario("stop-retry")))
        {
            fault = 1U;
        }
        if (id == LEFT_RS03_ID && is_scenario("temporary-fault") && elapsed_ms < 1000U) fault = 1U;
        if (id == RIGHT_RS03_ID && is_scenario("late-fault") && elapsed_ms >= 500U) fault = 1U;
        uint8_t reply_id = id;
        if (id == LEFT_RS03_ID && is_scenario("wrong-id")) reply_id = 6U;
        receive_feedback(reply_id, mode, fault);
        if (reply_id == id && mode == 2U && fault == 0U && motor->enabled)
        {
            motor->active_reply_sent = true;
            motor->ever_active_reply_sent = true;
            motor->last_mode_reply_ms = tick;
        }
    }
}

void NVIC_SystemReset(void) { ++reset_count; longjmp(terminal_exit, 1); }
void Error_Handler(void) { ++error_count; longjmp(terminal_exit, 1); }

HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *h, FDCAN_ProtocolStatusTypeDef *s)
{ (void)h; memset(s, 0, sizeof *s); return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *h, FDCAN_ErrorCountersTypeDef *c)
{ (void)h; memset(c, 0, sizeof *c); return HAL_OK; }
uint32_t HAL_FDCAN_GetError(FDCAN_HandleTypeDef *h) { (void)h; return 0U; }
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *h) { (void)h; return 3U; }
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *h, uint32_t fifo)
{ (void)fifo; return h == &hfdcan3 && rx_pending ? 1U : 0U; }
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h, uint32_t fifo,
    FDCAN_RxHeaderTypeDef *header, uint8_t *bytes)
{
    CHECK(h == &hfdcan3 && fifo == FDCAN_RX_FIFO0 && rx_pending);
    *header = rx_header;
    memcpy(bytes, rx_bytes, sizeof rx_bytes);
    rx_pending = false;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h,
    FDCAN_TxHeaderTypeDef *header, uint8_t *bytes)
{
    CHECK(h == &hfdcan3); /* Board/cyclic traffic must not start during startup. */
    uint8_t id = robstride_get_destination_id(header->Identifier);
    uint8_t type = robstride_get_communication_type(header->Identifier);
    SimMotor *motor = &sim_motors[id];
    uint16_t index = convert_u8_u16_le(bytes);
    CHECK(id == RIGHT_RS03_ID || id == LEFT_RS03_ID || id == EL05_ID);
    if (type == StopId)
    {
        ++motor->stop_attempts;
        if (is_scenario("persistent-stop") && stopping && id == RIGHT_RS03_ID)
        {
            ++motor->terminal_stop_failures;
            return HAL_ERROR;
        }
        if (is_scenario("stop-retry") && stopping && motor->terminal_stop_failures < 2U)
        {
            ++motor->terminal_stop_failures;
            return HAL_ERROR;
        }
        ++motor->stop_queued;
    }
    if (type == 18U && index == RUN_MODE)
    {
        ++motor->mode_attempts;
        if (is_scenario("transient-tx") && id == RIGHT_RS03_ID && motor->mode_attempts <= 2U)
            return HAL_ERROR;
        if (is_scenario("silent-config-loss") && id == RIGHT_RS03_ID && motor->mode_attempts == 1U)
            return HAL_OK;
    }
    if (type == 18U && index == LIMIT_CURRENT)
    {
        ++motor->current_attempts;
        if (is_scenario("silent-config-loss") && id == LEFT_RS03_ID && motor->current_attempts == 1U)
            return HAL_OK;
    }
    if (type == EnableId)
    {
        ++motor->enable_attempts;
        if (is_scenario("ignored-enable") && id == RIGHT_RS03_ID && motor->enable_attempts == 1U)
            return HAL_OK;
    }
    if (elapsed_ms < motor->available_after_ms) return HAL_OK; /* Bus ACK is not a device ACK. */
    motor->contacted = true;
    motor->reply_pending = true;
    motor->last_command = type;
    if (type == StopId)
    {
        motor->enabled = false;
        motor->configured_parameters = 0U;
        motor->active_reply_sent = false;
    }
    if (type == EnableId)
    {
        CHECK(motor->configured_parameters == 15U); /* Limits and hold precede Enable. */
        float stopped_position = -12.57f + (float)(0x8000U + (uint32_t)id * 100U) * 25.14f / 65535.0f;
        CHECK(fabsf(motor->hold_target - stopped_position) < 0.0001f);
        ++motor->enable_queued;
        motor->enabled = motor->mode == POSITION_PP;
        motor->active_reply_sent = false;
    }
    if (type == 18U)
    {
        if (index == RUN_MODE) motor->mode = bytes[4];
        if (index == PP_VELOCITY_MAX) motor->configured_parameters |= 1U;
        if (index == PP_ACCELERATION) motor->configured_parameters |= 2U;
        if (index == LIMIT_CURRENT) motor->configured_parameters |= 4U;
        if (index == POSITION_REF)
        {
            motor->configured_parameters |= 8U;
            motor->hold_target = convert_u8_f32_le(bytes + 4);
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *timer)
{
    CHECK(timer == &htim6);
    ++timer_start_attempts;
    for (uint32_t i = 0; i < ROBSTRIDE_MOTOR_COUNT; ++i)
    {
        SimMotor *motor = &sim_motors[ids[i]];
        CHECK(motor->enabled && motor->mode == POSITION_PP);
        CHECK(motor->configured_parameters == 15U);
        CHECK(motor->active_reply_sent);
        CHECK((uint32_t)(tick - motor->last_mode_reply_ms) < ROBSTRIDE_FEEDBACK_TIMEOUT_MS);
    }
    if (is_scenario("timer-start-failure")) return HAL_ERROR;
    ++timer_start_count;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *timer)
{ CHECK(timer == &htim6); stopping = true; return HAL_OK; }
HAL_StatusTypeDef inter_board_CAN_RxTxSettings_init(FDCAN_TxHeaderTypeDef *header)
{ memset(header, 0, sizeof *header); return HAL_OK; }
HAL_StatusTypeDef motor_CAN_RxTxSettings_init(FDCAN_TxHeaderTypeDef *header)
{ memset(header, 0, sizeof *header); return HAL_OK; }
void motor_CAN_txheader_init(FDCAN_TxHeaderTypeDef *header)
{ memset(header, 0, sizeof *header); }

bool cybergear_init(CyberGearMotor *motor, FDCAN_HandleTypeDef *h, uint8_t id, uint8_t host)
{ memset(motor, 0, sizeof *motor); motor->hfdcan = h; motor->motor_id = id; motor->master_id = host; return true; }
bool cybergear_stop(CyberGearMotor *motor)
{
    CHECK(motor == &cybergear_base);
    ++cg_stop_count;
    if (is_scenario("stop-retry") && stopping && cg_stop_failures < 2U)
    { ++cg_stop_failures; return false; }
    cg_enabled = false;
    motor->adrc.active = false;
    return true;
}
bool cybergear_enable(CyberGearMotor *motor)
{ CHECK(motor == &cybergear_base); ++cg_enable_count; cg_enabled = true; return true; }
bool cybergear_set_run_mode(CyberGearMotor *motor, CyberGearRunMode mode)
{ motor->run_mode = mode; return true; }
bool cybergear_start_position_adrc(CyberGearMotor *motor)
{
    ++cg_adrc_start_count;
    /* Keep the production routine's three blocking 10 ms configuration waits. */
    HAL_Delay(10U);
    HAL_Delay(10U);
    HAL_Delay(10U);
    motor->adrc.active = true;
    cg_enabled = true;
    if ((is_scenario("handoff-fault") && cg_adrc_start_count == 1U) ||
        is_scenario("persistent-handoff-fault"))
    {
        /* The earlier RobStride barrier passed, but startup must recheck this
           new fault before it starts the control timer. Subsequent STOP replies
           report the fault cleared, allowing the bounded startup to recover. */
        receive_feedback(RIGHT_RS03_ID, 2U, 1U);
        sim_motors[RIGHT_RS03_ID].active_reply_sent = false;
    }
    return !is_scenario("cg-adrc-failure");
}
bool cybergear_control_position_adrc(CyberGearMotor *motor, float target)
{ (void)motor; (void)target; CHECK(false); return false; }
bool cybergear_homing_run(const CyberGearHomingContext *context)
{
    CHECK(context->motor == &cybergear_base);
    HAL_Delay(100U);
    CHECK(cybergear_stop(context->motor));
    cg_homing_stop_count = cg_stop_count;
    return true;
}
bool cybergear_parse_feedback(CyberGearMotor *motor, uint32_t id, const uint8_t *bytes)
{
    (void)bytes;
    if (((id >> 8) & 255U) != motor->motor_id) return false;
    motor->feedback.online = true;
    motor->feedback.last_received_ms = tick;
    motor->feedback.mode = (uint8_t)((id >> 22) & 3U);
    motor->feedback.fault_flags = (uint8_t)((id >> 16) & 63U);
    return true;
}

int main(int argc, char **argv)
{
    CHECK(argc == 2);
    scenario = argv[1];
    bool expect_success = is_scenario("delayed-power") || is_scenario("transient-tx") ||
        is_scenario("tick-rollover") || is_scenario("silent-config-loss") ||
        is_scenario("ignored-enable") || is_scenario("temporary-fault") ||
        is_scenario("handoff-fault");
    if (is_scenario("delayed-power") || is_scenario("tick-rollover"))
    {
        sim_motors[LEFT_RS03_ID].available_after_ms = 500U;
        sim_motors[EL05_ID].available_after_ms = 1200U;
    }
    if (is_scenario("stale-ready") || is_scenario("late-fault"))
        sim_motors[EL05_ID].available_after_ms = 1200U;
    if (is_scenario("tick-rollover")) tick = UINT32_MAX - 200U;
    motor_init_requested = true;
    if (setjmp(terminal_exit) == 0) motor_app_start();
    CHECK(reset_count == 0U);
    CHECK(elapsed_ms <= ROBSTRIDE_INIT_TIMEOUT_MS + MOTOR_STOP_TIMEOUT_MS + 1000U);
    if (expect_success)
    {
        CHECK(error_count == 0U && timer_start_count == 1U && motors_running);
        if (is_scenario("transient-tx")) CHECK(sim_motors[RIGHT_RS03_ID].mode_attempts >= 3U);
        if (is_scenario("delayed-power") || is_scenario("tick-rollover")) CHECK(elapsed_ms >= 1200U);
        if (is_scenario("silent-config-loss"))
        {
            CHECK(sim_motors[RIGHT_RS03_ID].mode_attempts >= 2U);
            CHECK(sim_motors[LEFT_RS03_ID].current_attempts >= 2U);
        }
        if (is_scenario("ignored-enable"))
        {
            CHECK(sim_motors[RIGHT_RS03_ID].enable_attempts >= 2U);
            CHECK(sim_motors[RIGHT_RS03_ID].stop_queued >= 2U);
        }
        if (is_scenario("temporary-fault")) CHECK(elapsed_ms >= 1000U);
        if (is_scenario("handoff-fault")) CHECK(cg_adrc_start_count >= 2U);
    }
    else
    {
        CHECK(error_count == 1U && timer_start_count == 0U && !motors_running);
        if (is_scenario("persistent-handoff-fault")) CHECK(cg_adrc_start_count >= 2U);
        if (is_scenario("timer-start-failure")) CHECK(timer_start_attempts == 1U);
        CHECK(cg_stop_count > cg_homing_stop_count);
        CHECK(!cg_enabled);
        if (is_scenario("stop-retry")) CHECK(cg_stop_failures == 2U);
        for (uint32_t i = 0; i < ROBSTRIDE_MOTOR_COUNT; ++i)
        {
            SimMotor *motor = &sim_motors[ids[i]];
            if (is_scenario("persistent-stop") && ids[i] == RIGHT_RS03_ID)
            {
                CHECK(motor->terminal_stop_failures > 1U);
                continue; /* No code can deliver a STOP through persistent HAL errors. */
            }
            CHECK(motor->stop_queued > 0U && motor->last_command == StopId);
            CHECK(!motor->enabled);
            if (is_scenario("stop-retry")) CHECK(motor->terminal_stop_failures == 2U);
        }
    }
    printf("PASS %s elapsed=%lu ms\n", scenario, (unsigned long)elapsed_ms);
    return 0;
}
