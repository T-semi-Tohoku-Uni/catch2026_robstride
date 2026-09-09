#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "cybergear_test_motion.h"

static CyberGearTestResult step(CyberGearTestMotion *t, uint32_t ms, bool done, float q, float v)
{
    return cybergear_test_motion_update(t, ms, true, true, done, q, v);
}

void test_motion(void)
{
    CyberGearTestMotion t;
    const float amplitude = CG_TEST_AMPLITUDE_RAD;
    cybergear_test_motion_init(&t, 0U);
    assert(t.target_rad == amplitude);
    assert(step(&t, 0U, false, 0.0f, 0.0f) == CG_TEST_WAIT);
    assert(step(&t, 100U, true, amplitude, 0.1f) == CG_TEST_WAIT);
    assert(step(&t, 200U, true, amplitude, 0.0f) == CG_TEST_WAIT);
    assert(step(&t, 1199U, true, amplitude, 0.0f) == CG_TEST_WAIT);
    assert(step(&t, 1200U, true, amplitude, 0.0f) == CG_TEST_NEW_TARGET);
    assert(t.target_rad == -amplitude);
    assert(step(&t, 2200U, false, t.target_rad, 0.0f) == CG_TEST_WAIT);
    assert(step(&t, 2500U, true, t.target_rad, 0.0f) == CG_TEST_WAIT);
    assert(step(&t, 3000U, true, 0.0f, 0.0f) == CG_TEST_WAIT); /* Leaving tolerance resets dwell. */
    assert(step(&t, 3500U, true, t.target_rad, 0.0f) == CG_TEST_WAIT);
    assert(step(&t, 4499U, true, t.target_rad, 0.0f) == CG_TEST_WAIT);
    assert(step(&t, 4500U, true, t.target_rad, 0.0f) == CG_TEST_NEW_TARGET);
    assert(t.target_rad == amplitude);
    assert(step(&t, 6500U, true, t.target_rad, 0.0f) == CG_TEST_WAIT);
    assert(step(&t, 7500U, true, t.target_rad, 0.0f) == CG_TEST_NEW_TARGET);
    assert(t.target_rad == -amplitude && t.completed_legs == 3U);

    assert(cybergear_test_motion_update(&t, 7600U, true, false, true, t.target_rad, 0.0f) == CG_TEST_STOP_FAULT);
    assert(step(&t, 10000U, true, t.target_rad, 0.0f) == CG_TEST_WAIT && t.halted);
    cybergear_test_motion_init(&t, 0U);
    assert(step(&t, CG_TEST_LEG_TIMEOUT_MS, false, 0.0f, 0.0f) == CG_TEST_STOP_TIMEOUT);
    cybergear_test_motion_init(&t, 0U);
    assert(step(&t, 1U, true, CG_TEST_AMPLITUDE_RAD + CG_TEST_TRAVEL_GUARD_RAD + 0.01f, 0.0f) == CG_TEST_STOP_TRAVEL);
    cybergear_test_motion_init(&t, 0U);
    assert(step(&t, 1U, true, NAN, 0.0f) == CG_TEST_STOP_FAULT);
    cybergear_test_motion_init(&t, 0U);
    assert(cybergear_test_motion_update(&t, 1U, false, true, true, 0.0f, 0.0f) == CG_TEST_STOP_FAULT);

    const uint32_t start = UINT32_MAX - 500U;
    cybergear_test_motion_init(&t, start);
    assert(step(&t, start, true, amplitude, 0.0f) == CG_TEST_WAIT);
    assert(step(&t, start + 999U, true, amplitude, 0.0f) == CG_TEST_WAIT);
    assert(step(&t, start + 1000U, true, amplitude, 0.0f) == CG_TEST_NEW_TARGET);
    puts("Reciprocation: endpoints, dwell, fault latch, timeout, travel and tick wrap passed.");
}
