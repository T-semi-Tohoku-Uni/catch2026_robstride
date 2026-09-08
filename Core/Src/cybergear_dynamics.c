#include "cybergear_dynamics.h"

#include <math.h>
#include <string.h>

static bool positive(float value)
{
    return isfinite(value) && value > 0.0f;
}

static bool nonnegative(float value)
{
    return isfinite(value) && value >= 0.0f;
}

static float clamp(float value, float lower, float upper)
{
    return fminf(upper, fmaxf(lower, value));
}

/* 全到達姿勢の b_min を使うため、姿勢喪失時も軌道制約が不連続に変化しない。
 * i_ff=ad/b0 の微分に含まれる -ad*b0_dot/b0² もスルー予算から引く。 */
static bool current_budget(const CyberGearDynamicsConfig *c, CyberGearDynamicsOutput *out)
{
    const float accel_budget = c->acceleration_current_a - c->reserve_current_a;
    const float brake_budget = c->braking_current_a - c->reserve_current_a;
    const float accel = fminf(c->acceleration_cap_rad_s2, c->b0_min * accel_budget);
    const float brake = fminf(c->braking_cap_rad_s2, c->b0_min * brake_budget);
    const float b_rate = c->schedule_enabled ? c->b0_rate_limit : 0.0f;
    const float scheduling_slew = fmaxf(accel, brake) / c->b0_min * (b_rate / c->b0_min);
    const float remaining_slew = c->current_slew_a_s - c->reserve_slew_a_s - scheduling_slew;
    const float jerk = fminf(c->jerk_cap_rad_s3, c->b0_min * remaining_slew);
    if (!positive(accel) || !positive(brake) || !positive(remaining_slew) || !positive(jerk)) return false;
    if (out != NULL) {
        out->acceleration_rad_s2 = accel;
        out->braking_rad_s2 = brake;
        out->jerk_rad_s3 = jerk;
        out->tracking_current_budget_a = fminf(accel_budget, brake_budget);
    }
    return true;
}

bool cybergear_dynamics_config_valid(const CyberGearDynamicsConfig *c)
{
    if (c == NULL || !positive(c->fixed_b0) || !positive(c->b0_min) ||
        !positive(c->b0_max) || c->fixed_b0 < c->b0_min || c->fixed_b0 > c->b0_max ||
        !positive(c->b0_rate_limit) || c->posture_timeout_ms == 0U ||
        c->posture_timeout_ms >= UINT32_C(0x80000000) ||
        !positive(c->acceleration_current_a) || !positive(c->braking_current_a) ||
        !nonnegative(c->reserve_current_a) || c->reserve_current_a >= c->acceleration_current_a ||
        c->reserve_current_a >= c->braking_current_a || !positive(c->current_slew_a_s) ||
        !nonnegative(c->reserve_slew_a_s) || !positive(c->acceleration_cap_rad_s2) ||
        !positive(c->braking_cap_rad_s2) || !positive(c->jerk_cap_rad_s3) ||
        c->point_count > CYBERGEAR_DYNAMICS_MAX_POINTS) return false;
    if (c->schedule_enabled && c->point_count < 2U) return false;
    for (size_t i = 0U; i < c->point_count; ++i) {
        if (!isfinite(c->points[i].posture_index) || !isfinite(c->points[i].b0) ||
            c->points[i].b0 < c->b0_min || c->points[i].b0 > c->b0_max ||
            (i > 0U && c->points[i].posture_index <= c->points[i - 1U].posture_index)) return false;
    }
    return current_budget(c, NULL);
}

bool cybergear_dynamics_init(CyberGearDynamics *dynamics, const CyberGearDynamicsConfig *config)
{
    if (dynamics == NULL || !cybergear_dynamics_config_valid(config)) return false;
    memset(dynamics, 0, sizeof(*dynamics));
    dynamics->config = *config;
    dynamics->b0 = config->fixed_b0;
    dynamics->initialized = true;
    return true;
}

bool cybergear_dynamics_step(CyberGearDynamics *dynamics,
    const CyberGearPostureSnapshot *posture, uint32_t now_ms, float dt_s,
    CyberGearDynamicsOutput *output)
{
    if (output != NULL) memset(output, 0, sizeof(*output));
    if (dynamics == NULL || output == NULL || !dynamics->initialized ||
        !positive(dt_s) || dt_s > 0.05f) return false;
    const CyberGearDynamicsConfig *c = &dynamics->config;
    CyberGearDynamicsOutput next = {0};
    if (!current_budget(c, &next)) return false;
    next.status = CYBERGEAR_MODEL_FIXED;
    float target = c->fixed_b0;
    if (c->schedule_enabled) {
        next.status = CYBERGEAR_MODEL_STALE;
        if (posture != NULL && posture->valid) {
            if (!isfinite(posture->posture_index) ||
                posture->posture_index < c->points[0].posture_index ||
                posture->posture_index > c->points[c->point_count - 1U].posture_index) {
                next.status = CYBERGEAR_MODEL_INVALID;
            } else if ((uint32_t)(now_ms - posture->timestamp_ms) <= c->posture_timeout_ms) {
                next.status = CYBERGEAR_MODEL_VALID;
                for (size_t i = 1U; i < c->point_count; ++i) {
                    if (posture->posture_index <= c->points[i].posture_index) {
                        const CyberGearDynamicsPoint *lo = &c->points[i - 1U];
                        const CyberGearDynamicsPoint *hi = &c->points[i];
                        const float fraction = (posture->posture_index - lo->posture_index) /
                            (hi->posture_index - lo->posture_index);
                        target = lo->b0 + fraction * (hi->b0 - lo->b0);
                        break;
                    }
                }
            }
        }
    }
    if (!isfinite(target) || !isfinite(dynamics->b0)) return false;
    const float maximum_change = c->b0_rate_limit * dt_s;
    next.b0 = clamp(target, dynamics->b0 - maximum_change, dynamics->b0 + maximum_change);
    next.b0 = clamp(next.b0, c->b0_min, c->b0_max);
    next.b0_rate = (next.b0 - dynamics->b0) / dt_s;
    if (!positive(next.b0) || !isfinite(next.b0_rate)) return false;
    dynamics->b0 = next.b0;
    *output = next;
    return true;
}
