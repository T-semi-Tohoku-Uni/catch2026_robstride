#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "robstride_startup.h"

typedef struct {
    uint32_t available_after_ms;
    uint32_t stop_attempts;
    uint32_t enable_attempts;
    uint32_t mode_attempts;
    uint32_t current_attempts;
    uint32_t hold_attempts;
    uint32_t reply_due_ms;
    uint32_t last_command;
    uint8_t configured_parameters;
    uint8_t run_mode;
    float hold_target;
    bool enabled;
    bool ever_running;
    bool reply_pending;
} SimMotor;

FDCAN_HandleTypeDef hfdcan3;
static RobstrideMotor motors[ROBSTRIDE_STARTUP_MAX_MOTORS];
static SimMotor simulated[ROBSTRIDE_STARTUP_MAX_MOTORS];
static const char *scenario;
static uint32_t tick_ms, elapsed_ms, interrupt_mask, sends;

static bool is_scenario(const char *name)
{
    return strcmp(scenario, name) == 0;
}

static uint16_t position_raw(uint32_t index)
{
    uint32_t raw = 0x8000U + index * 100U;
    if (is_scenario("hold-shift") && (simulated[index].configured_parameters & 1U) != 0U) raw += 300U;
    return (uint16_t)raw;
}

uint32_t HAL_GetTick(void) { return tick_ms; }
uint32_t __get_PRIMASK(void) { return interrupt_mask; }
void __disable_irq(void) { interrupt_mask = 1U; }
void __set_PRIMASK(uint32_t mask) { interrupt_mask = mask; }
void HAL_Delay(uint32_t milliseconds) { (void)milliseconds; assert(false); }

HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *can,
    FDCAN_TxHeaderTypeDef *header, uint8_t *data)
{
    assert(can == &hfdcan3 && interrupt_mask == 1U);
    ++sends;
    const uint8_t destination = robstride_get_destination_id(header->Identifier);
    assert(destination >= 3U && destination <= 5U);
    const uint32_t motor_index = destination - 3U;
    SimMotor *motor = &simulated[motor_index];
    const uint8_t type = robstride_get_communication_type(header->Identifier);
    const uint16_t parameter = convert_u8_u16_le(data);
    if (type == StopId) {
        assert(data[0] == 0U);
        ++motor->stop_attempts;
    }
    if (type == 18U && parameter == RUN_MODE) {
        ++motor->mode_attempts;
        if (is_scenario("transient-tx") && motor_index == 0U && motor->mode_attempts <= 2U) return HAL_BUSY;
        if (is_scenario("silent-config-loss") && motor_index == 0U && motor->mode_attempts == 1U) return HAL_OK;
    }
    if (type == 18U && parameter == LIMIT_CURRENT) {
        ++motor->current_attempts;
        if (is_scenario("silent-config-loss") && motor_index == 1U && motor->current_attempts == 1U) return HAL_OK;
    }
    if (type == 18U && parameter == POSITION_REF) {
        ++motor->hold_attempts;
        if (is_scenario("lost-hold") && motor_index == 0U && motor->hold_attempts == 1U) return HAL_OK;
    }
    if (type == EnableId) {
        ++motor->enable_attempts;
        assert(motor->configured_parameters == 15U);
        assert(motor->run_mode == POSITION_PP);
        const float expected_position = robstride_u16_to_float(position_raw(motor_index), -12.57f, 12.57f);
        assert(fabsf(motor->hold_target - expected_position) < 0.00001f);
        if (is_scenario("ignored-enable") && motor_index == 0U && motor->enable_attempts == 1U) return HAL_OK;
    }
    if (is_scenario("persistent-tx") && motor_index == 1U) return HAL_ERROR;
    if (elapsed_ms < motor->available_after_ms) return HAL_OK;
    motor->reply_pending = true;
    motor->reply_due_ms = elapsed_ms + 1U;
    motor->last_command = type;
    if (type == StopId) {
        motor->enabled = false;
        motor->configured_parameters = 0U;
    } else if (type == EnableId) {
        motor->enabled = true;
    } else if (type == 18U) {
        switch (parameter) {
        case RUN_MODE:
            motor->run_mode = data[4];
            break;
        case PP_VELOCITY_MAX:
            assert(convert_u8_f32_le(data + 4) == 10.0f);
            motor->configured_parameters |= 1U;
            break;
        case PP_ACCELERATION:
            assert(convert_u8_f32_le(data + 4) == 1.0f);
            motor->configured_parameters |= 2U;
            break;
        case LIMIT_CURRENT:
            assert(convert_u8_f32_le(data + 4) == 10.0f);
            motor->configured_parameters |= 4U;
            break;
        case POSITION_REF:
            motor->hold_target = convert_u8_f32_le(data + 4);
            motor->configured_parameters |= 8U;
            break;
        default:
            assert(false);
        }
    } else {
        assert(false);
    }
    return HAL_OK;
}

static void advance_time(void)
{
    assert(interrupt_mask == 0U);
    ++tick_ms;
    ++elapsed_ms;
    for (uint32_t index = 0U; index < ROBSTRIDE_STARTUP_MAX_MOTORS; ++index) {
        SimMotor *motor = &simulated[index];
        if (!motor->reply_pending || elapsed_ms < motor->reply_due_ms) continue;
        motor->reply_pending = false;
        if (index == 1U && (is_scenario("missing-feedback") || is_scenario("wrong-id"))) continue;
        if (index == 0U && is_scenario("stale-ready") && motor->ever_running) continue;
        if (index == 1U && is_scenario("stale-feedback") && motor->enabled) continue;
        uint8_t mode = motor->enabled ? 2U : 0U;
        uint8_t fault = 0U;
        if (index == 1U && is_scenario("wrong-mode")) mode = 1U;
        if (index == 1U && is_scenario("stale-feedback") && motor->last_command != StopId) mode = 2U;
        if (index == 1U && is_scenario("fault")) fault = 1U;
        if (index == 1U && is_scenario("temporary-fault") && elapsed_ms < 1000U) fault = 1U;
        if (index == 0U && is_scenario("late-fault") && elapsed_ms >= 700U) fault = 1U;
        const uint32_t can_id = (2U << 24) | ((uint32_t)mode << 22) |
            ((uint32_t)fault << 16) | ((index + 3U) << 8) | 0xfeU;
        const uint16_t position = position_raw(index);
        uint8_t data[8] = {(uint8_t)(position >> 8), (uint8_t)position, 0x80U, 0U, 0x80U, 0U, 0U, 0U};
        robstride_parse_feedback(can_id, data, &motors[index].feedback);
        if (index == 1U && is_scenario("invalid-position")) motors[index].feedback.position_rad = NAN;
        if (mode == 2U && fault == 0U) motor->ever_running = true;
    }
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    scenario = argv[1];
    const bool expect_success = is_scenario("normal") || is_scenario("delayed-power") ||
        is_scenario("transient-tx") || is_scenario("tick-rollover") ||
        is_scenario("silent-config-loss") || is_scenario("ignored-enable") ||
        is_scenario("temporary-fault") || is_scenario("lost-hold") ||
        is_scenario("hold-shift") || is_scenario("handoff-keepalive");
    RobstrideStartup startup;
    assert(!robstride_startup_init(NULL, motors, 3U));
    assert(!robstride_startup_init(&startup, NULL, 3U));
    assert(robstride_startup_failed(&startup));
    assert(!robstride_startup_init(&startup, motors, 0U));
    assert(!robstride_startup_init(&startup, motors, 4U));
    for (uint32_t index = 0U; index < ROBSTRIDE_STARTUP_MAX_MOTORS; ++index) {
        motors[index].motor_id = (uint8_t)(index + 3U);
        motors[index].host_id = 0xfeU;
    }
    if (is_scenario("delayed-power") || is_scenario("tick-rollover")) {
        simulated[1].available_after_ms = 500U;
        simulated[2].available_after_ms = 1200U;
    }
    if (is_scenario("stale-ready") || is_scenario("late-fault")) simulated[2].available_after_ms = 1200U;
    if (is_scenario("tick-rollover")) tick_ms = UINT32_MAX - 200U;
    assert(robstride_startup_init(&startup, motors, 3U));
    while (!robstride_startup_failed(&startup) && !robstride_startup_ready(&startup)) {
        const uint32_t previous_sends = sends;
        robstride_startup_update(&startup);
        assert(sends - previous_sends <= 1U);
        advance_time();
        assert(elapsed_ms <= ROBSTRIDE_STARTUP_TIMEOUT_MS);
    }
    assert(robstride_startup_ready(&startup) == expect_success);
    if (expect_success) {
        for (uint32_t index = 0U; index < ROBSTRIDE_STARTUP_MAX_MOTORS; ++index) {
            assert(simulated[index].enabled && simulated[index].ever_running);
            assert(motors[index].run_mode == POSITION_PP);
        }
        if (is_scenario("delayed-power") || is_scenario("tick-rollover")) assert(elapsed_ms >= 1200U);
        if (is_scenario("transient-tx")) assert(simulated[0].mode_attempts >= 3U);
        if (is_scenario("silent-config-loss")) {
            assert(simulated[0].mode_attempts >= 2U);
            assert(simulated[1].current_attempts >= 2U);
        }
        if (is_scenario("ignored-enable")) assert(simulated[0].enable_attempts >= 2U);
        if (is_scenario("lost-hold")) assert(simulated[0].hold_attempts >= 2U);
        if (is_scenario("temporary-fault")) assert(elapsed_ms >= 1000U);
        if (is_scenario("handoff-keepalive")) {
            uint32_t enable_attempts[ROBSTRIDE_STARTUP_MAX_MOTORS];
            for (uint32_t index = 0U; index < ROBSTRIDE_STARTUP_MAX_MOTORS; ++index)
                enable_attempts[index] = simulated[index].enable_attempts;
            for (uint32_t duration_ms = 0U; duration_ms < 1000U; ++duration_ms) {
                robstride_startup_update(&startup);
                advance_time();
                assert(robstride_startup_ready(&startup));
            }
            for (uint32_t index = 0U; index < ROBSTRIDE_STARTUP_MAX_MOTORS; ++index)
                assert(simulated[index].enable_attempts == enable_attempts[index]);
            interrupt_mask = 1U;
            assert(robstride_startup_ready(&startup));
            robstride_startup_update(&startup);
            assert(interrupt_mask == 1U);
            interrupt_mask = 0U;
        }
    } else {
        assert(robstride_startup_failed(&startup));
        assert(elapsed_ms == ROBSTRIDE_STARTUP_TIMEOUT_MS);
        const uint32_t previous_sends = sends;
        robstride_startup_update(&startup);
        assert(previous_sends == sends);
        if (is_scenario("missing-feedback") || is_scenario("persistent-tx")) {
            assert(simulated[0].ever_running && simulated[2].ever_running);
        }
    }
    printf("PASS %s elapsed=%lu ms sends=%lu\n", scenario,
        (unsigned long)elapsed_ms, (unsigned long)sends);
    return 0;
}
