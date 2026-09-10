#include "cybergear_test_motion.h"
#include <math.h>
#include <string.h>

static const float test_targets_rad[] = {
    CG_TEST_AMPLITUDE_RAD,
    -CG_TEST_AMPLITUDE_RAD,
    CG_TEST_SMALL_AMPLITUDE_RAD,
    -CG_TEST_SMALL_AMPLITUDE_RAD
};

bool cybergear_test_configure(CyberGearMotor *motor)
{
    if (motor == NULL) return false;
    CyberGearConfig config = motor->config;
    const float current = fminf(config.controller.current_limit_a, CG_TEST_CURRENT_LIMIT_A);
    config.controller.current_limit_a = current;
    config.controller.disturbance_limit_a = CG_TEST_DISTURBANCE_CURRENT_FRACTION * current;
    config.controller.leak_mode = CG_TEST_LEAK_MODE;
    config.controller.observer_rad_s = fminf(config.controller.observer_rad_s, CG_TEST_OBSERVER_RAD_S);
    config.stall_current_a = fminf(config.stall_current_a, CYBERGEAR_STALL_CURRENT_FRACTION * current);
    config.dynamics.acceleration_current_a = current;
    config.dynamics.braking_current_a = current;
    config.dynamics.reserve_current_a = CG_TEST_RESERVE_CURRENT_FRACTION * current;
    config.trajectory.velocity_max_rad_s = fminf(config.trajectory.velocity_max_rad_s, CG_TEST_SPEED_RAD_S);
    if (-CG_TEST_AMPLITUDE_RAD <= config.trajectory.position_min_rad ||
        CG_TEST_AMPLITUDE_RAD >= config.trajectory.position_max_rad) return false;
    return cybergear_configure(motor, &config);
}

void cybergear_test_motion_init(CyberGearTestMotion *test, uint32_t now_ms)
{
    memset(test, 0, sizeof(*test));
    test->target_rad = test_targets_rad[0];
    test->leg_started_ms = now_ms;
}

CyberGearTestResult cybergear_test_motion_update(CyberGearTestMotion *test,
    uint32_t now_ms, bool running, bool fresh, bool trajectory_done,
    float position_rad, float estimated_velocity_rad_s)
{
    if (test->halted) return CG_TEST_WAIT;
    CyberGearTestResult stop = CG_TEST_WAIT;
    if (!running || !fresh || !isfinite(position_rad) || !isfinite(estimated_velocity_rad_s))
        stop = CG_TEST_STOP_FAULT;
    else if (fabsf(position_rad) > CG_TEST_AMPLITUDE_RAD + CG_TEST_TRAVEL_GUARD_RAD)
        stop = CG_TEST_STOP_TRAVEL;
    else if ((uint32_t)(now_ms - test->leg_started_ms) >= CG_TEST_LEG_TIMEOUT_MS)
        stop = CG_TEST_STOP_TIMEOUT;
    if (stop != CG_TEST_WAIT) {
        test->halted = true;
        return stop;
    }
    /* Use the controller's velocity estimate: raw feedback was noisy at rest.
     * Also require the measured position and completed trajectory to agree. */
    if (!trajectory_done || fabsf(position_rad - test->target_rad) > CG_TEST_REACHED_RAD ||
        fabsf(estimated_velocity_rad_s) > CG_TEST_REACHED_SPEED_RAD_S) {
        test->settling = false;
        return CG_TEST_WAIT;
    }
    if (!test->settling) {
        test->settling = true;
        test->settled_since_ms = now_ms;
    }
    if ((uint32_t)(now_ms - test->settled_since_ms) < CG_TEST_DWELL_MS) return CG_TEST_WAIT;
    test->target_index = (test->target_index + 1U) %
        (sizeof(test_targets_rad) / sizeof(test_targets_rad[0]));
    test->target_rad = test_targets_rad[test->target_index];
    test->completed_legs++;
    test->leg_started_ms = now_ms;
    test->settling = false;
    return CG_TEST_NEW_TARGET;
}
