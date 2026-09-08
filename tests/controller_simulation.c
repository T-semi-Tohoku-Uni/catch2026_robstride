#include "cybergear_controller.h"
#include "cybergear_dynamics.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Deterministic, normalized synthetic plants. NO robot dimensions, motor Ktau,
 * resonance, damping, or measured latency are inferred by these constants.
 * Integration is 1 ms; both command transport and sensor transport are delayed
 * separately by delay_ms. Feedback reception timestamp is now, as on hardware. */
#define SIM_HISTORY 64U
#define SIM_DURATION_MS 12000U

typedef enum { SIM_RIGID, SIM_FLEXIBLE, SIM_VARYING_INERTIA } SimPlant;
typedef struct { float qm, vm, ql, vl; } SimState;
typedef struct {
    float max_motor_error, max_load_error, max_current, max_twist;
    double motor_error_squared, load_error_squared, tail_error_squared, tail_velocity_squared;
    unsigned samples, tail_samples, amplitude_limited, slew_limited, compensation_limited;
    float final_error, final_velocity;
} SimMetrics;

static CyberGearControllerReference segment(float from, float to, float t, float duration)
{
    CyberGearControllerReference r = {from, 0.0f, 0.0f};
    if (t <= 0.0f) return r;
    if (t >= duration) { r.position_rad = to; return r; }
    const float s = t / duration;
    const float d = to - from;
    r.position_rad = from + d * s*s*s * (10.0f + s * (-15.0f + 6.0f*s));
    r.velocity_rad_s = d / duration * 30.0f*s*s * (1.0f - 2.0f*s + s*s);
    r.acceleration_rad_s2 = d / (duration*duration) * 60.0f*s * (1.0f - 3.0f*s + 2.0f*s*s);
    return r;
}

static CyberGearControllerReference reference_at(float t)
{
    if (t < 3.0f) return segment(0.0f, 1.2f, t, 2.0f);
    if (t < 6.5f) return segment(1.2f, -0.8f, t - 3.0f, 2.5f);
    return segment(-0.8f, 0.3f, t - 6.5f, 2.0f);
}

static float friction(float velocity)
{
    /* Ktau=1 in normalized torque units: smooth Coulomb + viscous friction. */
    return 0.01f * tanhf(velocity / 0.02f) + 0.005f * velocity;
}

static void plant_step(SimState *s, SimPlant plant, float ratio, float current, float t)
{
    const float dt = 0.001f;
    const float base_inertia = 0.1f / ratio; /* Ktau/J = 10*ratio at neutral posture. */
    if (plant == SIM_FLEXIBLE) {
        const float jm = 0.2f * base_inertia;
        const float jl = 0.8f * base_inertia;
        const float inverse_reduced_inertia = 1.0f/jm + 1.0f/jl;
        const float resonance_rad_s = 15.0f;
        const float modal_damping = 0.04f;
        const float stiffness = resonance_rad_s * resonance_rad_s / inverse_reduced_inertia;
        const float damping = 2.0f * modal_damping * resonance_rad_s / inverse_reduced_inertia;
        const float shaft_torque = stiffness * (s->qm - s->ql) + damping * (s->vm - s->vl);
        const float am = (current - shaft_torque - 0.001f * s->vm) / jm;
        const float al = (shaft_torque - friction(s->vl)) / jl;
        s->qm += dt * s->vm + 0.5f * dt*dt * am;
        s->vm += dt * am;
        s->ql += dt * s->vl + 0.5f * dt*dt * al;
        s->vl += dt * al;
    } else {
        float inertia = base_inertia;
        float inertia_rate = 0.0f;
        if (plant == SIM_VARYING_INERTIA) {
            inertia *= 1.0f + 0.55f * sinf(0.7f*t);
            inertia_rate = base_inertia * 0.55f * 0.7f * cosf(0.7f*t);
        }
        /* d(J*v)/dt = torque: continuous J transition includes the physical Jdot*v term. */
        const float acceleration = (current - friction(s->vm) - inertia_rate * s->vm) / inertia;
        s->qm += dt * s->vm + 0.5f * dt*dt * acceleration;
        s->vm += dt * acceleration;
        s->ql = s->qm;
        s->vl = s->vm;
    }
}

static CyberGearDynamicsConfig synthetic_schedule(void)
{
    CyberGearDynamicsConfig cfg = {0};
    cfg.schedule_enabled = true;
    cfg.fixed_b0 = 10.0f;
    cfg.b0_min = 6.0f;
    cfg.b0_max = 23.0f;
    cfg.b0_rate_limit = 10.0f;
    cfg.posture_timeout_ms = 50U;
    cfg.point_count = CYBERGEAR_DYNAMICS_MAX_POINTS;
    for (size_t p = 0U; p < cfg.point_count; ++p) {
        cfg.points[p].posture_index = -1.0f + 2.0f * (float)p / (float)(cfg.point_count - 1U);
        cfg.points[p].b0 = 10.0f / (1.0f + 0.55f * cfg.points[p].posture_index);
    }
    cfg.acceleration_current_a = 10.0f;
    cfg.braking_current_a = 10.0f;
    cfg.reserve_current_a = 2.0f;
    cfg.current_slew_a_s = 5.0f;
    cfg.reserve_slew_a_s = 1.0f;
    cfg.acceleration_cap_rad_s2 = 3.0f;
    cfg.braking_cap_rad_s2 = 3.0f;
    cfg.jerk_cap_rad_s3 = 15.0f;
    return cfg;
}

static uint32_t noise_next(uint32_t *seed)
{
    *seed = *seed * UINT32_C(1664525) + UINT32_C(1013904223);
    return *seed;
}

static void run_case(SimPlant plant, unsigned period_ms, float ratio, unsigned delay_ms, bool schedule)
{
    CyberGearControllerConfig cfg;
    cybergear_controller_default_config(&cfg);
    cfg.period_ms = period_ms;
    cfg.b0_min = 1.0f;
    cfg.b0_max = 100.0f;
    CyberGearController controller;
    assert(cybergear_controller_init(&controller, &cfg, 0.0f, 0.0f, 0U, 0U, 0U));
    assert(cybergear_controller_commit_queued(&controller, 0.0f, 0U));
    CyberGearDynamics dynamics;
    CyberGearDynamicsConfig dyn_cfg = synthetic_schedule();
    assert(cybergear_dynamics_init(&dynamics, &dyn_cfg));
    SimState state = {0};
    SimMetrics metrics = {0};
    float current_history[SIM_HISTORY] = {0};
    float position_history[SIM_HISTORY] = {0};
    float last_queued = 0.0f;
    uint32_t random_state = UINT32_C(0x431279af);
    const float quantization = 25.0f / 65535.0f;
    const float control_dt = (float)period_ms * 0.001f;
    for (uint32_t now = 1U; now <= SIM_DURATION_MS; ++now) {
        /* Commands emitted at t are applied at t + delay. t=0 was explicitly zeroed. */
        const uint32_t input_tick = now - 1U;
        const float applied = input_tick >= delay_ms ?
            current_history[(input_tick - delay_ms) % SIM_HISTORY] : 0.0f;
        plant_step(&state, plant, ratio, applied, ((float)now - 0.5f) * 0.001f);
        assert(isfinite(state.qm) && isfinite(state.vm) && isfinite(state.ql) && isfinite(state.vl));
        position_history[now % SIM_HISTORY] = state.qm;
        if (now % period_ms == 0U) {
            const float t = (float)now * 0.001f;
            const CyberGearControllerReference ref = reference_at(t);
            const float noise = ((float)(noise_next(&random_state) >> 8) / 16777215.0f - 0.5f) * 0.0004f;
            const float delayed_position = now >= delay_ms ? position_history[(now-delay_ms) % SIM_HISTORY] : 0.0f;
            CyberGearControllerMeasurement measurement = {
                roundf((delayed_position + noise) / quantization) * quantization,
                now / period_ms, now, true};
            float b0 = 10.0f;
            if (schedule) {
                const float posture_time = now >= delay_ms ? (float)(now-delay_ms) * 0.001f : 0.0f;
                CyberGearPostureSnapshot posture = {sinf(0.7f*posture_time), now, true};
                CyberGearDynamicsOutput dyn_out;
                assert(cybergear_dynamics_step(&dynamics, &posture, now, control_dt, &dyn_out));
                assert(dyn_out.status == CYBERGEAR_MODEL_VALID);
                b0 = dyn_out.b0;
            }
            CyberGearControllerOutput output;
            assert(cybergear_controller_step(&controller, &ref, &measurement, now, b0, &output));
            assert(fabsf(output.current_a) <= cfg.current_limit_a + 1e-5f);
            assert(fabsf(output.current_a - last_queued) <= cfg.current_rise_a_s * control_dt + 1e-5f);
            assert(fabsf(output.disturbance_current_a) <= cfg.disturbance_limit_a + 1e-5f);
            assert(cybergear_controller_commit_queued(&controller, output.current_a, now));
            last_queued = output.current_a;
            const float em = ref.position_rad - state.qm;
            const float el = ref.position_rad - state.ql;
            metrics.max_motor_error = fmaxf(metrics.max_motor_error, fabsf(em));
            metrics.max_load_error = fmaxf(metrics.max_load_error, fabsf(el));
            metrics.max_current = fmaxf(metrics.max_current, fabsf(last_queued));
            metrics.max_twist = fmaxf(metrics.max_twist, fabsf(state.qm - state.ql));
            metrics.motor_error_squared += (double)em * em;
            metrics.load_error_squared += (double)el * el;
            ++metrics.samples;
            metrics.amplitude_limited += output.amplitude_limited ? 1U : 0U;
            metrics.slew_limited += output.slew_limited ? 1U : 0U;
            metrics.compensation_limited += output.disturbance_limited ? 1U : 0U;
            if (now > SIM_DURATION_MS - 1000U) {
                metrics.tail_error_squared += (double)el * el;
                metrics.tail_velocity_squared += (double)state.vl * state.vl;
                ++metrics.tail_samples;
            }
            metrics.final_error = el;
            metrics.final_velocity = state.vl;
        }
        current_history[now % SIM_HISTORY] = last_queued;
    }
    const double motor_rms = sqrt(metrics.motor_error_squared / (double)metrics.samples);
    const double load_rms = sqrt(metrics.load_error_squared / (double)metrics.samples);
    const double tail_rms = sqrt(metrics.tail_error_squared / (double)metrics.tail_samples);
    const double tail_vrms = sqrt(metrics.tail_velocity_squared / (double)metrics.tail_samples);
    /* Reporting threshold only: this is NOT the upper-level robot completion condition.
     * Unstable/poor regimes must be reported, never hidden by loosening a test bound. */
    const bool poor = metrics.max_load_error > 0.6f || tail_rms > 0.1 || tail_vrms > 0.2;
    const char *name = plant == SIM_RIGID ? "rigid" :
        (plant == SIM_FLEXIBLE ? "flexible" : "varying_inertia");
    printf("%s,%u,%.2f,%u,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f,%.3f,%s\n",
        name, 1000U/period_ms, (double)ratio, delay_ms, schedule ? 1U : 0U,
        (double)metrics.max_motor_error, (double)metrics.max_load_error, motor_rms, load_rms,
        tail_rms, tail_vrms, (double)metrics.max_current, (double)metrics.max_twist,
        100.0 * (double)metrics.amplitude_limited / (double)metrics.samples,
        100.0 * (double)metrics.slew_limited / (double)metrics.samples,
        100.0 * (double)metrics.compensation_limited / (double)metrics.samples,
        poor ? "poor" : "within_report_threshold");
}

void test_controller_simulation(void)
{
    const float ratios[] = {0.25f, 0.5f, 1.0f, 2.0f, 4.0f};
    const unsigned delays[] = {0U, 5U, 10U, 20U};
    puts("plant,hz,nominal_ratio,one_way_delay_ms,schedule,max_motor_error_rad,max_load_error_rad,motor_rms_rad,load_rms_rad,tail_load_rms_rad,tail_load_velocity_rms_rad_s,peak_current_a,peak_twist_rad,amplitude_limited_pct,slew_limited_pct,compensation_limited_pct,result");
    for (unsigned p = SIM_RIGID; p <= SIM_VARYING_INERTIA; ++p)
        for (unsigned rate = 0U; rate < 2U; ++rate)
            for (unsigned r = 0U; r < sizeof(ratios)/sizeof(ratios[0]); ++r)
                for (unsigned d = 0U; d < sizeof(delays)/sizeof(delays[0]); ++d)
                    run_case((SimPlant)p, rate == 0U ? 10U : 5U, ratios[r], delays[d], false);
    for (unsigned rate = 0U; rate < 2U; ++rate)
        for (unsigned d = 0U; d < sizeof(delays)/sizeof(delays[0]); ++d)
            run_case(SIM_VARYING_INERTIA, rate == 0U ? 10U : 5U, 1.0f, delays[d], true);
}

#ifdef CYBERGEAR_SIMULATION_STANDALONE
int main(void)
{
    test_controller_simulation();
    return 0;
}
#endif
