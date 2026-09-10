#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "cybergear_test_motion.h"

static CyberGearTestResult step(CyberGearTestMotion *motion, uint32_t now_ms,
    bool done, float position_rad, float velocity_rad_s)
{
    return cybergear_test_motion_update(motion, now_ms, true, true, done,
        position_rad, velocity_rad_s);
}

void test_motion(void)
{
    CyberGearTestMotion motion;
    const float amplitude = CG_TEST_AMPLITUDE_RAD;
    const float small_amplitude = CG_TEST_SMALL_AMPLITUDE_RAD;
    assert(fabsf(amplitude - 60.0f * 0.01745329252f) < 1e-6f);
    assert(fabsf(small_amplitude - 5.0f * 0.01745329252f) < 1e-6f);
    cybergear_test_motion_init(&motion, 0U);
    assert(motion.target_rad == amplitude);
    assert(step(&motion, 0U, false, 0.0f, 0.0f) == CG_TEST_WAIT);
    assert(step(&motion, 100U, true, amplitude, 0.1f) == CG_TEST_WAIT);
    assert(step(&motion, 200U, true, amplitude, 0.0f) == CG_TEST_WAIT);
    assert(step(&motion, 1199U, true, amplitude, 0.0f) == CG_TEST_WAIT);
    assert(step(&motion, 1200U, true, amplitude, 0.0f) == CG_TEST_NEW_TARGET);
    assert(motion.target_rad == -amplitude);
    assert(step(&motion, 2200U, false, motion.target_rad, 0.0f) == CG_TEST_WAIT);
    assert(step(&motion, 2500U, true, motion.target_rad, 0.0f) == CG_TEST_WAIT);
    assert(step(&motion, 3000U, true, 0.0f, 0.0f) == CG_TEST_WAIT);
    assert(step(&motion, 3500U, true, motion.target_rad, 0.0f) == CG_TEST_WAIT);
    assert(step(&motion, 4499U, true, motion.target_rad, 0.0f) == CG_TEST_WAIT);
    assert(step(&motion, 4500U, true, motion.target_rad, 0.0f) == CG_TEST_NEW_TARGET);
    assert(motion.target_rad == small_amplitude);
    assert(step(&motion, 4501U, false, -amplitude, 0.0f) == CG_TEST_WAIT);
    assert(step(&motion, 6500U, true, motion.target_rad, 0.0f) == CG_TEST_WAIT);
    assert(step(&motion, 7500U, true, motion.target_rad, 0.0f) == CG_TEST_NEW_TARGET);
    assert(motion.target_rad == -small_amplitude && motion.completed_legs == 3U);

    const float expected_targets[] = {amplitude, -amplitude, small_amplitude, -small_amplitude, amplitude};
    uint32_t now_ms = 8000U;
    for (unsigned int index = 0U; index < sizeof(expected_targets) / sizeof(expected_targets[0]); ++index) {
        assert(step(&motion, now_ms, true, motion.target_rad, 0.0f) == CG_TEST_WAIT);
        now_ms += CG_TEST_DWELL_MS;
        assert(step(&motion, now_ms, true, motion.target_rad, 0.0f) == CG_TEST_NEW_TARGET);
        assert(motion.target_rad == expected_targets[index]);
        now_ms++;
    }
    assert(motion.completed_legs == 8U);

    assert(cybergear_test_motion_update(&motion, now_ms, true, false, true, motion.target_rad, 0.0f) == CG_TEST_STOP_FAULT);
    assert(step(&motion, now_ms + 1U, true, motion.target_rad, 0.0f) == CG_TEST_WAIT && motion.halted);
    cybergear_test_motion_init(&motion, 0U);
    assert(step(&motion, CG_TEST_LEG_TIMEOUT_MS, false, 0.0f, 0.0f) == CG_TEST_STOP_TIMEOUT);
    cybergear_test_motion_init(&motion, 0U);
    assert(step(&motion, 1U, true, amplitude + CG_TEST_TRAVEL_GUARD_RAD + 0.01f, 0.0f) == CG_TEST_STOP_TRAVEL);
    cybergear_test_motion_init(&motion, 0U);
    assert(step(&motion, 1U, true, -amplitude - CG_TEST_TRAVEL_GUARD_RAD - 0.01f, 0.0f) == CG_TEST_STOP_TRAVEL);
    cybergear_test_motion_init(&motion, 0U);
    assert(step(&motion, 1U, true, NAN, 0.0f) == CG_TEST_STOP_FAULT);
    cybergear_test_motion_init(&motion, 0U);
    assert(cybergear_test_motion_update(&motion, 1U, false, true, true, 0.0f, 0.0f) == CG_TEST_STOP_FAULT);

    const uint32_t start = UINT32_MAX - 500U;
    cybergear_test_motion_init(&motion, start);
    assert(step(&motion, start, true, amplitude, 0.0f) == CG_TEST_WAIT);
    assert(step(&motion, start + 999U, true, amplitude, 0.0f) == CG_TEST_WAIT);
    assert(step(&motion, start + 1000U, true, amplitude, 0.0f) == CG_TEST_NEW_TARGET);
    puts("Reciprocation: endpoints, dwell, fault latch, timeout, travel and tick wrap passed.");
}
