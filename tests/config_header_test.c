#include "app_mode.h"
#include "cybergear_test_motion.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

uint32_t HAL_GetTick(void) { return 1000U; }
void HAL_Delay(uint32_t delay_ms) { (void)delay_ms; }
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef *handle)
{ (void)handle; return 3U; }
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(const FDCAN_HandleTypeDef *handle,
    FDCAN_ProtocolStatusTypeDef *status)
{ (void)handle; memset(status, 0, sizeof(*status)); return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(const FDCAN_HandleTypeDef *handle,
    FDCAN_ErrorCountersTypeDef *counters)
{ (void)handle; memset(counters, 0, sizeof(*counters)); return HAL_OK; }
uint32_t HAL_FDCAN_GetError(const FDCAN_HandleTypeDef *handle)
{ return handle->ErrorCode; }
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *handle,
    FDCAN_TxHeaderTypeDef *header, uint8_t *data)
{ (void)handle; (void)header; (void)data; assert(false); return HAL_ERROR; }

static void check_close(float actual, float expected)
{
    assert(fabsf(actual - expected) <= 1e-6f * fmaxf(1.0f, fabsf(expected)));
}

static void check_motion(void)
{
    CyberGearTestMotion motion;
    const float targets[] = {CG_TEST_AMPLITUDE_RAD, -CG_TEST_AMPLITUDE_RAD,
        CG_TEST_SMALL_AMPLITUDE_RAD, -CG_TEST_SMALL_AMPLITUDE_RAD};
    cybergear_test_motion_init(&motion, 1000U);
    uint32_t now_ms = 1000U;
    for (unsigned int target_index = 0U; target_index < 4U; ++target_index) {
        check_close(motion.target_rad, targets[target_index]);
        assert(cybergear_test_motion_update(&motion, now_ms, true, true, true,
            motion.target_rad + 1.1f * CG_TEST_REACHED_RAD, 0.0f) == CG_TEST_WAIT);
        assert(!motion.settling);
        assert(cybergear_test_motion_update(&motion, now_ms, true, true, true,
            motion.target_rad, 1.1f * CG_TEST_REACHED_SPEED_RAD_S) == CG_TEST_WAIT);
        assert(!motion.settling);
        assert(cybergear_test_motion_update(&motion, now_ms, true, true, true,
            motion.target_rad, 0.5f * CG_TEST_REACHED_SPEED_RAD_S) == CG_TEST_WAIT);
        assert(motion.settling);
        assert(cybergear_test_motion_update(&motion, now_ms + CG_TEST_DWELL_MS - 1U,
            true, true, true, motion.target_rad, 0.0f) == CG_TEST_WAIT);
        now_ms += CG_TEST_DWELL_MS;
        assert(cybergear_test_motion_update(&motion, now_ms, true, true, true,
            motion.target_rad, 0.0f) == CG_TEST_NEW_TARGET);
    }
    assert(motion.completed_legs == 4U);
    check_close(motion.target_rad, CG_TEST_AMPLITUDE_RAD);
    cybergear_test_motion_init(&motion, 1000U);
    assert(cybergear_test_motion_update(&motion, 1000U, true, true, false,
        CG_TEST_AMPLITUDE_RAD + CG_TEST_TRAVEL_GUARD_RAD + 0.001f, 0.0f) == CG_TEST_STOP_TRAVEL);
    cybergear_test_motion_init(&motion, 1000U);
    assert(cybergear_test_motion_update(&motion, 1000U + CG_TEST_LEG_TIMEOUT_MS,
        true, true, false, 0.0f, 0.0f) == CG_TEST_STOP_TIMEOUT);
}

int main(void)
{
    CyberGearConfig config;
    cybergear_config_defaults(&config);
    assert(cybergear_config_valid(&config));
    CyberGearControllerConfig controller;
    cybergear_controller_default_config(&controller);
    assert(cybergear_controller_config_valid(&controller));
    check_close(controller.current_limit_a, 10.0f);
    check_close(controller.b0_initial, 10.0f);
    check_close(controller.bandwidth_rad_s, 4.0f);
    check_close(controller.observer_rad_s, 10.0f);
#include "config_header_assertions.h"
    check_close(config.stall_current_a, 0.7f * 4.5f);
    check_close(config.dynamics.reserve_current_a, 0.3f * 4.5f);
    check_close(config.dynamics.current_slew_a_s, 4.0f);
    check_close(CG_TEST_AMPLITUDE_RAD, 0.7853981634f);
    check_close(CG_TEST_SMALL_AMPLITUDE_RAD, 0.0610865238f);
    check_close(CG_TEST_REACHED_RAD, 0.0069813170f);
    check_close(CG_TEST_TRAVEL_GUARD_RAD, 0.0349065850f);
    FDCAN_HandleTypeDef can = {0};
    CyberGearMotor motor;
    assert(cybergear_init(&motor, &can, 0x7fU, 0xfeU));
    assert(cybergear_test_configure(&motor));
    check_close(motor.config.controller.current_limit_a,
        fminf(config.controller.current_limit_a, CG_TEST_CURRENT_LIMIT_A));
    check_close(motor.config.trajectory.velocity_max_rad_s,
        fminf(config.trajectory.velocity_max_rad_s, CG_TEST_SPEED_RAD_S));
    check_close(motor.config.controller.observer_rad_s,
        fminf(config.controller.observer_rad_s, CG_TEST_OBSERVER_RAD_S));
    check_close(motor.config.controller.disturbance_limit_a, 0.4f * 2.5f);
    check_close(motor.config.dynamics.reserve_current_a, 0.45f * 2.5f);
    check_close(motor.config.stall_current_a, 0.7f * 2.5f);
    check_close(motor.config.controller.leak_fixed_s, 0.04f);
    assert(motor.config.controller.leak_mode == CYBERGEAR_LEAK_NONE);
    motor.config.controller.current_limit_a = 1.25f;
    motor.config.controller.observer_rad_s = 4.0f;
    motor.config.trajectory.velocity_max_rad_s = 0.12f;
    assert(cybergear_test_configure(&motor));
    check_close(motor.config.controller.current_limit_a, 1.25f);
    check_close(motor.config.controller.observer_rad_s, 4.0f);
    check_close(motor.config.trajectory.velocity_max_rad_s, 0.12f);
    check_motion();
    puts("Central CyberGear header: production defaults and standalone overrides passed.");
    return 0;
}
