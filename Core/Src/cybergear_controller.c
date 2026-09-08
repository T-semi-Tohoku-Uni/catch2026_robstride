#include "cybergear_controller.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float clamp(float value, float lower, float upper)
{
    return fminf(upper, fmaxf(lower, value));
}

static bool nonnegative(float value)
{
    return isfinite(value) && value >= 0.0f;
}

static bool positive(float value)
{
    return isfinite(value) && value > 0.0f;
}

static bool forward_or_equal(uint32_t newer, uint32_t older)
{
    return (uint32_t)(newer - older) < UINT32_C(0x80000000);
}

void cybergear_controller_default_config(CyberGearControllerConfig *config)
{
    if (config == NULL) return;
    memset(config, 0, sizeof(*config));
    config->period_ms = 10U;
    config->timing_tolerance_ms = 1U;
    config->feedback_timeout_ms = 100U;
    config->bandwidth_rad_s = 4.0f;
    config->damping_ratio = 2.0f;
    config->observer_rad_s = 10.0f;
    /* 既存の仮定 b0 を維持。これは実機の校正範囲を表す値ではない。 */
    config->b0_initial = 10.0f;
    config->b0_min = 10.0f;
    config->b0_max = 10.0f;
    config->current_limit_a = 10.0f;
    config->current_rise_a_s = 5.0f;
    config->current_fall_a_s = 5.0f;
    config->disturbance_limit_a = 0.5f;
    config->disturbance_slew_a_s = 1.0f;
    config->compensation_gain = 1.0f;
    config->compensation_delay_ms = 100U;
    config->compensation_ramp_ms = 500U;
    config->leak_mode = CYBERGEAR_LEAK_LEGACY_ERROR;
    config->leak_fixed_s = 0.03f;
    config->leak_near_s = 0.5f;
    config->leak_far_s = 0.03f;
    config->leak_near_rad = 0.003f;
    config->leak_far_rad = 0.015f;
}

bool cybergear_controller_config_valid(const CyberGearControllerConfig *c)
{
    if (c == NULL) return false;
    return (c->period_ms == 10U || c->period_ms == 5U) &&
        c->timing_tolerance_ms < c->period_ms / 2U &&
        c->feedback_timeout_ms >= c->period_ms &&
        c->feedback_timeout_ms < UINT32_C(0x80000000) &&
        positive(c->bandwidth_rad_s) && positive(c->damping_ratio) &&
        positive(c->observer_rad_s) && positive(c->b0_initial) &&
        positive(c->b0_min) && positive(c->b0_max) &&
        c->b0_initial >= c->b0_min && c->b0_initial <= c->b0_max &&
        positive(c->current_limit_a) && positive(c->current_rise_a_s) &&
        positive(c->current_fall_a_s) && nonnegative(c->disturbance_limit_a) &&
        c->disturbance_limit_a <= c->current_limit_a && positive(c->disturbance_slew_a_s) &&
        nonnegative(c->compensation_gain) && c->compensation_gain <= 1.0f &&
        c->compensation_ramp_ms > 0U &&
        c->compensation_ramp_ms < UINT32_C(0x40000000) &&
        c->compensation_delay_ms < UINT32_C(0x40000000) &&
        c->leak_mode >= CYBERGEAR_LEAK_LEGACY_ERROR && c->leak_mode <= CYBERGEAR_LEAK_NONE &&
        nonnegative(c->leak_fixed_s) && nonnegative(c->leak_near_s) &&
        nonnegative(c->leak_far_s) && nonnegative(c->leak_near_rad) &&
        positive(c->leak_far_rad) && c->leak_far_rad > c->leak_near_rad;
}

bool cybergear_controller_init(CyberGearController *controller,
    const CyberGearControllerConfig *config, float position_rad,
    float velocity_rad_s, uint32_t now_ms, uint32_t rx_sequence, uint32_t rx_timestamp_ms)
{
    if (controller == NULL || !cybergear_controller_config_valid(config) ||
        !isfinite(position_rad) || !isfinite(velocity_rad_s) ||
        (uint32_t)(now_ms - rx_timestamp_ms) > config->feedback_timeout_ms) return false;
    memset(controller, 0, sizeof(*controller));
    controller->config = *config;
    controller->position_rad = position_rad;
    controller->velocity_rad_s = velocity_rad_s;
    controller->last_measured_position_rad = position_rad;
    const float dt = (float)config->period_ms * 0.001f;
    const float pole = expf(-config->observer_rad_s * dt);
    const float delta = 1.0f - pole;
    controller->observer_position_gain = 1.0f - pole * pole * pole;
    controller->observer_velocity_gain = 1.5f * delta * delta * (1.0f + pole) / dt;
    controller->observer_disturbance_gain = delta * delta * delta / (dt * dt);
    controller->b0 = config->b0_initial;
    controller->last_control_timestamp_ms = now_ms;
    controller->observer_timestamp_ms = now_ms;
    controller->last_queued_timestamp_ms = now_ms;
    controller->rx_sequence = rx_sequence;
    controller->rx_timestamp_ms = rx_timestamp_ms;
    controller->start_timestamp_ms = now_ms;
    controller->initialized = true;
    return true;
}

bool cybergear_controller_commit_queued(CyberGearController *controller,
    float current_a, uint32_t now_ms)
{
    if (controller == NULL || !controller->initialized || !isfinite(current_a) ||
        fabsf(current_a) > controller->config.current_limit_a ||
        !forward_or_equal(now_ms, controller->last_queued_timestamp_ms) ||
        (uint32_t)(now_ms - controller->last_control_timestamp_ms) >
            controller->config.period_ms + controller->config.timing_tolerance_ms) return false;
    controller->last_queued_current_a = current_a;
    controller->applied_current_estimate_a = current_a;
    controller->last_queued_timestamp_ms = now_ms;
    controller->applied_current_valid = true;
    return true;
}

void cybergear_controller_invalidate_input(CyberGearController *controller)
{
    if (controller != NULL) controller->applied_current_valid = false;
}

static float disturbance_leak(const CyberGearController *controller, float qd)
{
    const CyberGearControllerConfig *c = &controller->config;
    if (c->leak_mode == CYBERGEAR_LEAK_NONE) return 0.0f;
    if (c->leak_mode == CYBERGEAR_LEAK_FIXED) return c->leak_fixed_s;
    const float error = fabsf(qd - controller->last_measured_position_rad);
    const float blend = clamp((error - c->leak_near_rad) /
        (c->leak_far_rad - c->leak_near_rad), 0.0f, 1.0f);
    return c->leak_near_s + blend * (c->leak_far_s - c->leak_near_s);
}

bool cybergear_controller_step(CyberGearController *controller,
    const CyberGearControllerReference *reference,
    const CyberGearControllerMeasurement *measurement,
    uint32_t now_ms, float b0, CyberGearControllerOutput *output)
{
    if (output != NULL) memset(output, 0, sizeof(*output));
    if (controller == NULL || reference == NULL || output == NULL ||
        !controller->initialized || !controller->applied_current_valid ||
        !isfinite(reference->position_rad) || !isfinite(reference->velocity_rad_s) ||
        !isfinite(reference->acceleration_rad_s2) || !isfinite(b0) ||
        b0 < controller->config.b0_min || b0 > controller->config.b0_max) return false;

    const CyberGearControllerConfig *config = &controller->config;
    const uint32_t elapsed = now_ms - controller->last_control_timestamp_ms;
    if (elapsed < config->period_ms - config->timing_tolerance_ms ||
        elapsed > config->period_ms + config->timing_tolerance_ms) return false;

    CyberGearController next = *controller;
    CyberGearControllerOutput result = {0};
    bool new_measurement = false;
    if (measurement != NULL && measurement->valid) {
        if (!isfinite(measurement->position_rad) ||
            (uint32_t)(now_ms - measurement->timestamp_ms) > config->feedback_timeout_ms ||
            !forward_or_equal(measurement->rx_sequence, controller->rx_sequence) ||
            !forward_or_equal(measurement->timestamp_ms, controller->rx_timestamp_ms)) return false;
        if (measurement->rx_sequence != controller->rx_sequence) {
            next.rx_sequence = measurement->rx_sequence;
            next.rx_timestamp_ms = measurement->timestamp_ms;
            next.last_measured_position_rad = measurement->position_rad;
            new_measurement = true;
        } else if (measurement->timestamp_ms != controller->rx_timestamp_ms) {
            return false; /* シーケンスが同じなのに時刻だけを新しくして age を延ばさない。 */
        }
    }
    if ((uint32_t)(now_ms - next.rx_timestamp_ms) > config->feedback_timeout_ms) return false;

    /* 固定制御時刻に最新受信位置を近似配置する。dt は常に nominal period。
     * 旧区間は旧 b0 と最後に投入成功した制限後電流で予測する。 */
    const float dt = (float)config->period_ms * 0.001f;
    const float acceleration = next.disturbance_rad_s2 +
        next.b0 * next.applied_current_estimate_a;
    next.position_rad += dt * next.velocity_rad_s + 0.5f * dt * dt * acceleration;
    next.velocity_rad_s += dt * acceleration;
    next.disturbance_rad_s2 *= expf(-disturbance_leak(&next, reference->position_rad) * dt);

    if (new_measurement) {
        /* ZOH current observer: L1=1-p³, L2=3(1-p)²(1+p)/(2T), L3=(1-p)³/T² */
        result.innovation_rad = next.last_measured_position_rad - next.position_rad;
        next.position_rad += next.observer_position_gain * result.innovation_rad;
        next.velocity_rad_s += next.observer_velocity_gain * result.innovation_rad;
        next.disturbance_rad_s2 += next.observer_disturbance_gain * result.innovation_rad;
        result.measurement_corrected = true;
    }

    result.z3_before_reexpression = next.disturbance_rad_s2;
    next.disturbance_rad_s2 += (next.b0 - b0) * next.applied_current_estimate_a;
    next.b0 = b0;
    result.z3_after_reexpression = next.disturbance_rad_s2;
    result.b0 = b0;

    const uint32_t ramp_end = config->compensation_delay_ms + config->compensation_ramp_ms;
    const uint32_t remaining = ramp_end - next.compensation_elapsed_ms;
    next.compensation_elapsed_ms += remaining > config->period_ms ? config->period_ms : remaining;
    if (next.compensation_elapsed_ms > config->compensation_delay_ms) {
        const float phase = (float)(next.compensation_elapsed_ms - config->compensation_delay_ms) /
            (float)config->compensation_ramp_ms;
        result.gamma = config->compensation_gain * phase * phase * (3.0f - 2.0f * phase);
    }
    const float disturbance_requested = -result.gamma * next.disturbance_rad_s2 / b0;
    const float disturbance_bounded = clamp(disturbance_requested,
        -config->disturbance_limit_a, config->disturbance_limit_a);
    const float disturbance_step = config->disturbance_slew_a_s * dt;
    result.disturbance_current_a = clamp(disturbance_bounded,
        next.disturbance_current_a - disturbance_step, next.disturbance_current_a + disturbance_step);
    result.disturbance_limited = result.disturbance_current_a != disturbance_requested;
    next.disturbance_current_a = result.disturbance_current_a;

    const float wc = config->bandwidth_rad_s;
    result.tracking_current_a = (reference->acceleration_rad_s2 +
        wc * wc * (reference->position_rad - next.position_rad) +
        2.0f * config->damping_ratio * wc * (reference->velocity_rad_s - next.velocity_rad_s)) / b0;
    result.requested_current_a = result.tracking_current_a + result.disturbance_current_a;
    result.amplitude_current_a = clamp(result.requested_current_a,
        -config->current_limit_a, config->current_limit_a);
    /* rise/fall は符号ではなく数値の増減。正負の制動で同じ式になる。 */
    result.current_a = clamp(result.amplitude_current_a,
        next.last_queued_current_a - config->current_fall_a_s * dt,
        next.last_queued_current_a + config->current_rise_a_s * dt);
    result.amplitude_limited = result.amplitude_current_a != result.requested_current_a;
    result.slew_limited = result.current_a != result.amplitude_current_a;

    if (!isfinite(next.position_rad) || !isfinite(next.velocity_rad_s) ||
        !isfinite(next.disturbance_rad_s2) || !isfinite(result.tracking_current_a) ||
        !isfinite(disturbance_requested) || !isfinite(result.requested_current_a) ||
        !isfinite(result.current_a)) return false;
    next.last_control_timestamp_ms = now_ms;
    next.observer_timestamp_ms = now_ms;
    *controller = next;
    *output = result;
    return true;
}
