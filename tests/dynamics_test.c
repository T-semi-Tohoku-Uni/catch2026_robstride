#include "cybergear_dynamics.h"

#include <assert.h>
#include <math.h>

static CyberGearDynamicsConfig test_config(void)
{
    CyberGearDynamicsConfig c = {0};
    c.schedule_enabled = true;
    c.fixed_b0 = 10.0f;
    c.b0_min = 5.0f;
    c.b0_max = 20.0f;
    c.b0_rate_limit = 5.0f;
    c.posture_timeout_ms = 50U;
    c.point_count = 3U;
    c.points[0] = (CyberGearDynamicsPoint){0.0f, 10.0f};
    c.points[1] = (CyberGearDynamicsPoint){1.0f, 20.0f};
    c.points[2] = (CyberGearDynamicsPoint){2.0f, 5.0f};
    c.acceleration_current_a = 5.0f;
    c.braking_current_a = 6.0f;
    c.reserve_current_a = 1.0f;
    c.current_slew_a_s = 5.0f;
    c.reserve_slew_a_s = 1.0f;
    c.acceleration_cap_rad_s2 = 3.0f;
    c.braking_cap_rad_s2 = 4.0f;
    c.jerk_cap_rad_s3 = 100.0f;
    return c;
}

static void test_interpolation_and_current_budget(void)
{
    CyberGearDynamicsConfig cfg = test_config();
    CyberGearDynamics d;
    CyberGearDynamicsOutput out;
    assert(cybergear_dynamics_init(&d, &cfg));
    CyberGearPostureSnapshot p = {0.5f, 0U, true};
    assert(cybergear_dynamics_step(&d, &p, 0U, 0.01f, &out));
    assert(fabsf(out.b0 - 10.05f) < 1e-5f);
    assert(out.status == CYBERGEAR_MODEL_VALID);
    assert(fabsf(out.acceleration_rad_s2 - 3.0f) < 1e-6f);
    assert(fabsf(out.braking_rad_s2 - 4.0f) < 1e-6f);
    assert(fabsf(out.jerk_rad_s3 - 16.0f) < 1e-5f); /* b_dot に 0.8 A/s を予約。 */
    for (unsigned i = 0U; i < 110U; ++i) {
        const float previous = d.b0;
        assert(cybergear_dynamics_step(&d, &p, 0U, 0.01f, &out));
        assert(fabsf(out.b0 - previous) <= 0.050001f);
    }
    assert(fabsf(out.b0 - 15.0f) < 1e-5f);
    p.posture_index = 1.5f;
    for (unsigned i = 0U; i < 60U; ++i)
        assert(cybergear_dynamics_step(&d, &p, 0U, 0.01f, &out));
    assert(fabsf(out.b0 - 12.5f) < 1e-5f);
    assert(fabsf(out.jerk_rad_s3 - 16.0f) < 1e-5f); /* 中央 b0 に依存して制約を緩めない。 */

    cfg.schedule_enabled = false;
    cfg.point_count = 0U;
    cfg.acceleration_cap_rad_s2 = 100.0f;
    cfg.braking_cap_rad_s2 = 100.0f;
    assert(cybergear_dynamics_init(&d, &cfg));
    assert(cybergear_dynamics_step(&d, NULL, 0U, 0.01f, &out));
    assert(out.status == CYBERGEAR_MODEL_FIXED);
    assert(fabsf(out.acceleration_rad_s2 - 20.0f) < 1e-5f);
    assert(fabsf(out.braking_rad_s2 - 25.0f) < 1e-5f);
    assert(fabsf(out.jerk_rad_s3 - 20.0f) < 1e-5f);
}

static void test_staleness_and_invalid_models(void)
{
    CyberGearDynamicsConfig cfg = test_config();
    CyberGearDynamics d;
    CyberGearDynamicsOutput out;
    assert(cybergear_dynamics_init(&d, &cfg));
    d.b0 = 20.0f;
    CyberGearPostureSnapshot p = {1.0f, UINT32_MAX - 3U, true};
    assert(cybergear_dynamics_step(&d, &p, 3U, 0.01f, &out));
    assert(out.status == CYBERGEAR_MODEL_VALID); /* 時刻 wrap でも age=7ms。 */
    assert(cybergear_dynamics_step(&d, &p, 100U, 0.01f, &out));
    assert(out.status == CYBERGEAR_MODEL_STALE);
    assert(fabsf(out.b0 - 19.95f) < 1e-5f); /* fixed=10 へのジャンプは禁止。 */
    p = (CyberGearPostureSnapshot){3.0f, 100U, true};
    assert(cybergear_dynamics_step(&d, &p, 100U, 0.01f, &out));
    assert(out.status == CYBERGEAR_MODEL_INVALID);
    p.posture_index = NAN;
    assert(cybergear_dynamics_step(&d, &p, 100U, 0.01f, &out));
    assert(out.status == CYBERGEAR_MODEL_INVALID);
    assert(!cybergear_dynamics_step(&d, &p, 100U, NAN, &out));
    assert(!cybergear_dynamics_step(&d, &p, 100U, 0.1f, &out));
    cfg.points[1].posture_index = 0.0f;
    assert(!cybergear_dynamics_config_valid(&cfg));
    cfg = test_config();
    cfg.points[1].b0 = 0.0f;
    assert(!cybergear_dynamics_config_valid(&cfg));
    cfg = test_config();
    cfg.b0_min = -1.0f;
    assert(!cybergear_dynamics_config_valid(&cfg));
    cfg = test_config();
    cfg.reserve_current_a = cfg.acceleration_current_a;
    assert(!cybergear_dynamics_config_valid(&cfg));
    cfg = test_config();
    cfg.b0_rate_limit = 1000.0f;
    assert(!cybergear_dynamics_config_valid(&cfg)); /* FF の変化率予算を食い尽くす設定。 */
}

void test_dynamics(void)
{
    test_interpolation_and_current_budget();
    test_staleness_and_invalid_models();
}
