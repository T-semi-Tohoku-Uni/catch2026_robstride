#include "cybergear_controller.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void close_to(float actual, float expected, float tolerance)
{
    assert(isfinite(actual));
    assert(fabsf(actual - expected) <= tolerance);
}

static CyberGearControllerConfig isolated_config(void)
{
    CyberGearControllerConfig c;
    cybergear_controller_default_config(&c);
    c.compensation_gain = 0.0f;
    c.leak_mode = CYBERGEAR_LEAK_NONE;
    c.current_rise_a_s = 1000.0f;
    c.current_fall_a_s = 1000.0f;
    return c;
}

static void test_reference_and_discrete_observer(void)
{
    CyberGearController c;
    CyberGearControllerConfig cfg = isolated_config();
    CyberGearControllerOutput out;
    assert(cybergear_controller_init(&c, &cfg, 0.0f, 1.0f, 0U, 1U, 0U));
    CyberGearControllerReference ref = {0.0101f, 1.02f, 2.0f};
    CyberGearControllerMeasurement m = {0.0101f, 2U, 10U, true};
    assert(!cybergear_controller_step(&c, &ref, &m, 10U, 10.0f, &out));
    assert(cybergear_controller_commit_queued(&c, 0.2f, 0U));
    assert(cybergear_controller_step(&c, &ref, &m, 10U, 10.0f, &out));
    close_to(out.current_a, 0.2f, 1e-6f); /* 同じ q/v に追従中は加速度 FF のみ。 */
    close_to(out.innovation_rad, 0.0f, 1e-7f);
    assert(out.measurement_corrected);

    assert(cybergear_controller_init(&c, &cfg, 0.0f, 0.0f, 0U, 0U, 0U));
    assert(cybergear_controller_commit_queued(&c, 0.0f, 0U));
    m = (CyberGearControllerMeasurement){0.1f, 1U, 10U, true};
    ref = (CyberGearControllerReference){0.0f, 0.0f, 0.0f};
    assert(cybergear_controller_step(&c, &ref, &m, 10U, 10.0f, &out));
    /* wo=10,T=.01 の既知値。continuous observer の Euler 係数へ置換されないこと。 */
    close_to(c.position_rad, 0.02591818f, 2e-7f);
    close_to(c.velocity_rad_s, 0.2587509f, 2e-6f);
    close_to(c.disturbance_rad_s2, 0.8617844f, 3e-6f);
}

static void test_queue_prediction_and_b0_reexpression(void)
{
    CyberGearController c;
    CyberGearControllerConfig cfg = isolated_config();
    cfg.b0_min = 5.0f;
    cfg.b0_max = 20.0f;
    CyberGearControllerReference ref = {0};
    CyberGearControllerOutput out;
    assert(cybergear_controller_init(&c, &cfg, 0.0f, 0.0f, 0U, 1U, 0U));
    assert(cybergear_controller_commit_queued(&c, 1.0f, 0U));
    assert(cybergear_controller_step(&c, &ref, NULL, 10U, 20.0f, &out));
    close_to(c.position_rad, 0.0005f, 1e-8f); /* 旧区間へ新 b0=20 を遡及しない。 */
    close_to(c.velocity_rad_s, 0.1f, 1e-7f);
    close_to(c.disturbance_rad_s2, -10.0f, 1e-6f);
    close_to(20.0f * c.applied_current_estimate_a + out.z3_after_reexpression,
        10.0f * c.applied_current_estimate_a + out.z3_before_reexpression, 1e-6f);
    assert(out.current_a != 1.0f);
    /* 出力を送らず、前回キュー成功電流 1A のままもう 1 周期。 */
    assert(cybergear_controller_step(&c, &ref, NULL, 20U, 20.0f, &out));
    close_to(c.position_rad, 0.002f, 2e-8f);
    close_to(c.velocity_rad_s, 0.2f, 1e-7f);
    close_to(c.last_queued_current_a, 1.0f, 0.0f);
    cybergear_controller_invalidate_input(&c);
    assert(!cybergear_controller_step(&c, &ref, NULL, 30U, 20.0f, &out));
}

static void test_directional_limits_and_compensation(void)
{
    CyberGearController c;
    CyberGearControllerConfig cfg = isolated_config();
    cfg.current_limit_a = 1.0f;
    cfg.current_rise_a_s = 3.0f;
    cfg.current_fall_a_s = 7.0f;
    CyberGearControllerReference ref = {100.0f, 0.0f, 0.0f};
    CyberGearControllerOutput out;
    assert(cybergear_controller_init(&c, &cfg, 0.0f, 0.0f, 0U, 1U, 0U));
    assert(cybergear_controller_commit_queued(&c, -1.0f, 0U));
    assert(cybergear_controller_step(&c, &ref, NULL, 10U, 10.0f, &out));
    close_to(out.current_a, -0.97f, 1e-6f);
    assert(out.amplitude_limited && out.slew_limited);
    assert(cybergear_controller_init(&c, &cfg, 0.0f, 0.0f, 0U, 1U, 0U));
    assert(cybergear_controller_commit_queued(&c, 0.02f, 0U));
    ref.position_rad = -100.0f;
    assert(cybergear_controller_step(&c, &ref, NULL, 10U, 10.0f, &out));
    close_to(out.current_a, -0.05f, 1e-6f); /* 正電流から負電流への制動も fall。 */

    cfg.compensation_gain = 1.0f;
    cfg.compensation_delay_ms = 20U;
    cfg.compensation_ramp_ms = 40U;
    cfg.disturbance_limit_a = 0.25f;
    cfg.disturbance_slew_a_s = 1.0f;
    assert(cybergear_controller_init(&c, &cfg, 0.0f, 0.0f, 0U, 1U, 0U));
    assert(cybergear_controller_commit_queued(&c, 0.0f, 0U));
    c.disturbance_rad_s2 = 100.0f;
    float last_dist = 0.0f;
    for (uint32_t now = 10U; now <= 100U; now += 10U) {
        ref.position_rad = now == 50U ? 0.2f : 0.0f;
        assert(cybergear_controller_step(&c, &ref, NULL, now, 10.0f, &out));
        assert(fabsf(out.disturbance_current_a) <= 0.250001f);
        assert(fabsf(out.disturbance_current_a - last_dist) <= 0.010001f);
        if (now <= 20U) close_to(out.gamma, 0.0f, 0.0f);
        if (now == 40U) close_to(out.gamma, 0.5f, 1e-6f);
        if (now >= 60U) close_to(out.gamma, 1.0f, 1e-6f);
        last_dist = out.disturbance_current_a;
        assert(cybergear_controller_commit_queued(&c, out.current_a, now));
    }
}

static void test_timestamps_and_rejections(void)
{
    CyberGearController c;
    CyberGearControllerConfig cfg = isolated_config();
    CyberGearControllerReference ref = {0};
    CyberGearControllerOutput out;
    assert(cybergear_controller_init(&c, &cfg, 0.0f, 0.0f, 0U, 1U, 0U));
    assert(cybergear_controller_commit_queued(&c, 0.0f, 0U));
    CyberGearControllerMeasurement m = {0.1f, 2U, 5U, true};
    assert(cybergear_controller_step(&c, &ref, &m, 10U, 10.0f, &out));
    assert(out.measurement_corrected);
    assert(cybergear_controller_step(&c, &ref, &m, 20U, 10.0f, &out));
    assert(!out.measurement_corrected);
    m.rx_sequence = 3U; /* 同一 ms の別フレームは補正できる。 */
    assert(cybergear_controller_step(&c, &ref, &m, 30U, 10.0f, &out));
    assert(out.measurement_corrected);
    CyberGearController saved = c;
    m.rx_sequence = 2U;
    assert(!cybergear_controller_step(&c, &ref, &m, 40U, 10.0f, &out));
    assert(memcmp(&c, &saved, sizeof(c)) == 0);
    m.rx_sequence = 4U;
    m.timestamp_ms = 4U;
    assert(!cybergear_controller_step(&c, &ref, &m, 40U, 10.0f, &out));
    assert(!cybergear_controller_step(&c, &ref, NULL, 30U, 10.0f, &out));
    assert(!cybergear_controller_step(&c, &ref, NULL, 50U, 10.0f, &out));
    ref.acceleration_rad_s2 = NAN;
    assert(!cybergear_controller_step(&c, &ref, NULL, 40U, 10.0f, &out));
    ref.acceleration_rad_s2 = 0.0f;
    assert(!cybergear_controller_step(&c, &ref, NULL, 40U, 0.0f, &out));

    const uint32_t start = UINT32_MAX - 4U;
    assert(cybergear_controller_init(&c, &cfg, 0.0f, 0.0f, start, UINT32_MAX, start));
    assert(cybergear_controller_commit_queued(&c, 0.0f, start));
    m = (CyberGearControllerMeasurement){0.0f, 0U, 5U, true};
    assert(cybergear_controller_step(&c, &ref, &m, 5U, 10.0f, &out));
    assert(out.measurement_corrected);
    cfg.feedback_timeout_ms = 10U;
    assert(cybergear_controller_init(&c, &cfg, 0.0f, 0.0f, 0U, 0U, 0U));
    assert(cybergear_controller_commit_queued(&c, 0.0f, 0U));
    assert(cybergear_controller_step(&c, &ref, NULL, 10U, 10.0f, &out));
    assert(!cybergear_controller_step(&c, &ref, NULL, 20U, 10.0f, &out));
    cfg.b0_initial = NAN;
    assert(!cybergear_controller_config_valid(&cfg));
}

static void test_leak_options(void)
{
    CyberGearController c;
    CyberGearControllerConfig cfg = isolated_config();
    CyberGearControllerReference ref = {0};
    CyberGearControllerOutput out;
    for (int mode = CYBERGEAR_LEAK_LEGACY_ERROR; mode <= CYBERGEAR_LEAK_NONE; ++mode) {
        cfg.leak_mode = (CyberGearLeakMode)mode;
        cfg.leak_fixed_s = 0.2f;
        assert(cybergear_controller_init(&c, &cfg, 0.0f, 0.0f, 0U, 0U, 0U));
        assert(cybergear_controller_commit_queued(&c, 0.0f, 0U));
        c.disturbance_rad_s2 = 1.0f;
        assert(cybergear_controller_step(&c, &ref, NULL, 10U, 10.0f, &out));
        const float leak = mode == CYBERGEAR_LEAK_LEGACY_ERROR ? 0.5f :
            (mode == CYBERGEAR_LEAK_FIXED ? 0.2f : 0.0f);
        close_to(c.disturbance_rad_s2, expf(-0.01f * leak), 1e-7f);
    }
}

/* 合成の剛体モデルのみ。ratio は機体の保証範囲ではなく感度を見る試験入力。 */
static void test_synthetic_plants(void)
{
    const float ratios[] = {0.25f, 1.0f, 4.0f};
    for (unsigned rate = 0U; rate < 2U; ++rate) {
        for (unsigned r = 0U; r < sizeof(ratios) / sizeof(ratios[0]); ++r) {
            CyberGearControllerConfig cfg = isolated_config();
            cfg.period_ms = rate == 0U ? 10U : 5U;
            cfg.current_rise_a_s = cfg.current_fall_a_s = 5.0f;
            cfg.compensation_gain = 1.0f;
            CyberGearController c;
            CyberGearControllerOutput out;
            assert(cybergear_controller_init(&c, &cfg, 0.0f, 0.0f, 0U, 0U, 0U));
            assert(cybergear_controller_commit_queued(&c, 0.0f, 0U));
            float q = 0.0f, v = 0.0f, applied = 0.0f;
            float max_error = 0.0f;
            const float dt = (float)cfg.period_ms * 0.001f;
            for (uint32_t now = cfg.period_ms; now <= 10000U; now += cfg.period_ms) {
                const float acc = 10.0f * ratios[r] * applied - 0.1f * v;
                q += dt * v + 0.5f * dt * dt * acc;
                v += dt * acc;
                const float t = (float)now * 0.001f;
                CyberGearControllerReference ref = {
                    0.3f * (1.0f - cosf(t)), 0.3f * sinf(t), 0.3f * cosf(t)};
                CyberGearControllerMeasurement m = {q, now / cfg.period_ms, now, true};
                assert(cybergear_controller_step(&c, &ref, &m, now, 10.0f, &out));
                assert(fabsf(out.current_a) <= cfg.current_limit_a + 1e-5f);
                assert(fabsf(out.current_a - applied) <= cfg.current_rise_a_s * dt + 1e-5f);
                assert(fabsf(out.disturbance_current_a) <= cfg.disturbance_limit_a + 1e-5f);
                applied = out.current_a;
                assert(cybergear_controller_commit_queued(&c, applied, now));
                max_error = fmaxf(max_error, fabsf(ref.position_rad - q));
            }
            /* 非飽和の小軌道で発散・大幅な参照追従崩壊がない。 */
            assert(max_error < 0.15f);
            printf("controller synthetic %u Hz b/b0=%.2f max_error=%.5f rad\n",
                1000U / cfg.period_ms, (double)ratios[r], (double)max_error);
        }
    }
}

static void test_adaptation_reset_preserves_motion(void)
{
    for (unsigned int rate = 0U; rate < 2U; ++rate) {
        CyberGearControllerConfig config = isolated_config();
        config.period_ms = rate == 0U ? 10U : 5U;
        config.b0_max = 20.0f;
        config.compensation_gain = 1.0f;
        config.current_rise_a_s = 3.0f;
        config.current_fall_a_s = 7.0f;
        CyberGearController controller;
        const uint32_t now = UINT32_MAX - config.period_ms;
        assert(cybergear_controller_init(&controller, &config, 1.0f, 0.2f, now, 42U, now));
        assert(cybergear_controller_commit_queued(&controller, 0.4f, now));
        controller.b0 = 20.0f;
        controller.disturbance_rad_s2 = -5.0f;
        controller.disturbance_current_a = 0.3f;
        controller.compensation_elapsed_ms = config.compensation_delay_ms + config.compensation_ramp_ms;
        CyberGearController expected = controller;
        expected.b0 = config.b0_initial;
        expected.disturbance_rad_s2 = 0.0f;
        expected.compensation_elapsed_ms = 0U;
        cybergear_controller_reset_adaptation(&controller);
        assert(memcmp(&controller, &expected, sizeof(controller)) == 0);
        const CyberGearControllerReference reference = {1.0f, 0.2f, 0.0f};
        CyberGearControllerOutput output;
        for (uint32_t step = 1U; step <= 2U; ++step) {
            const uint32_t timestamp = now + step * config.period_ms;
            const float previous_current = controller.last_queued_current_a;
            const float previous_disturbance = controller.disturbance_current_a;
            assert(cybergear_controller_step(&controller, &reference, NULL, timestamp, config.b0_initial, &output));
            const float period_s = (float)config.period_ms * 0.001f;
            assert(output.gamma == 0.0f);
            assert(fabsf(output.disturbance_current_a - previous_disturbance) <= config.disturbance_slew_a_s * period_s + 1e-6f);
            assert(output.current_a - previous_current <= config.current_rise_a_s * period_s + 1e-6f);
            assert(previous_current - output.current_a <= config.current_fall_a_s * period_s + 1e-6f);
            assert(cybergear_controller_commit_queued(&controller, output.current_a, timestamp));
        }
    }
}

void test_controller(void)
{
    test_reference_and_discrete_observer();
    test_queue_prediction_and_b0_reexpression();
    test_directional_limits_and_compensation();
    test_timestamps_and_rejections();
    test_leak_options();
    test_adaptation_reset_preserves_motion();
    test_synthetic_plants();
}
