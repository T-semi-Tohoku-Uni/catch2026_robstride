#include "cybergear_calibration.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define CG_CAL_PI 3.14159265358979323846f

void cg_cal_config_defaults(CgCalConfig *c)
{
    if (!c) return;
    memset(c, 0, sizeof(*c));
    c->pulse_current_a = NAN;
    c->brake_current_a = NAN;
    c->current_limit_a = NAN;
    c->temp_trip_c = NAN;
    c->speed_trip_rad_s = NAN;
    c->stationary_speed_rad_s = 0.02f;
    c->travel_soft_rad = 10.0f * CG_CAL_PI / 180.0f;
    c->travel_hard_rad = 80.0f * CG_CAL_PI / 180.0f;
    c->guard_margin_rad = 2.0f * CG_CAL_PI / 180.0f;
    c->guard_lookahead_ms = 100U;
    c->position_jump_rad = 0.01f;
    c->baseline_ms = 200U;
    c->pulse_ms = 200U;
    c->brake_ms = 200U;
    c->settle_ms = 100U;
    c->feedback_timeout_ms = 30U;
    c->max_step_ms = 20U;
}

bool cg_cal_config_valid(const CgCalConfig *c)
{
    if (!c || !c->armed ||
        !isfinite(c->pulse_current_a) || !isfinite(c->brake_current_a) ||
        !isfinite(c->current_limit_a) || !isfinite(c->temp_trip_c) ||
        !isfinite(c->speed_trip_rad_s) || !isfinite(c->stationary_speed_rad_s) ||
        !isfinite(c->travel_soft_rad) || !isfinite(c->travel_hard_rad) ||
        !isfinite(c->guard_margin_rad) || !isfinite(c->position_jump_rad)) return false;
    return c->current_limit_a > 0.0f &&
        fabsf(c->pulse_current_a) > 0.0f &&
        fabsf(c->pulse_current_a) <= c->current_limit_a &&
        c->brake_current_a > 0.0f && c->brake_current_a <= c->current_limit_a &&
        c->temp_trip_c > 0.0f && c->speed_trip_rad_s > 0.0f &&
        c->speed_trip_rad_s <= 30.0f && c->stationary_speed_rad_s > 0.0f &&
        c->stationary_speed_rad_s < c->speed_trip_rad_s &&
        c->guard_margin_rad > 0.0f && c->guard_margin_rad < c->travel_soft_rad &&
        c->travel_soft_rad < c->travel_hard_rad &&
        c->travel_hard_rad < CG_CAL_ABSOLUTE_TRAVEL_RAD &&
        c->position_jump_rad > 0.0f && c->position_jump_rad < c->guard_margin_rad &&
        c->feedback_timeout_ms > 0U && c->feedback_timeout_ms <= 100U &&
        c->max_step_ms > 0U && c->max_step_ms <= 50U &&
        c->max_step_ms <= c->feedback_timeout_ms &&
        c->guard_lookahead_ms >= c->max_step_ms && c->guard_lookahead_ms <= 500U &&
        c->baseline_ms >= 3U * c->max_step_ms && c->baseline_ms <= 1000U &&
        c->pulse_ms >= 30U && c->pulse_ms >= 3U * c->max_step_ms && c->pulse_ms <= 500U &&
        c->brake_ms >= 30U && c->brake_ms >= 3U * c->max_step_ms && c->brake_ms <= 500U &&
        c->settle_ms >= 2U * c->max_step_ms && c->settle_ms <= 1000U;
}

void cg_cal_init(CgCal *cal)
{
    if (!cal) return;
    memset(cal, 0, sizeof(*cal));
    cal->phase = CG_CAL_PHASE_IDLE;
    cg_cal_config_defaults(&cal->config);
}

void cg_cal_abort(CgCal *cal, CgCalFault fault)
{
    if (!cal) return;
    if (cal->fault == CG_CAL_FAULT_NONE)
        cal->fault = fault == CG_CAL_FAULT_NONE ? CG_CAL_FAULT_REQUESTED_STOP : fault;
    cal->pending_command = false;
    cal->phase = CG_CAL_PHASE_STOPPING;
}

static CgCalFault check_feedback(CgCal *cal, const CgCalFeedback *f,
                                 uint32_t now_ms, bool initial)
{
    const CgCalConfig *c = &cal->config;
    if (!f || !f->online || (uint32_t)(now_ms - f->rx_ms) > c->feedback_timeout_ms ||
        !isfinite(f->q_rad) || !isfinite(f->v_rad_s) || !isfinite(f->temp_c))
        return CG_CAL_FAULT_FEEDBACK;
    if (f->motor_fault) return CG_CAL_FAULT_MOTOR;
    if (f->temp_c >= c->temp_trip_c) return CG_CAL_FAULT_TEMPERATURE;
    if (fabsf(f->v_rad_s) >= c->speed_trip_rad_s) return CG_CAL_FAULT_SPEED;
    if (fabsf(f->q_rad) > 12.5f) return CG_CAL_FAULT_POSITION;
    if (!initial) {
        const uint32_t seq_delta = f->rx_sequence - cal->last_rx_sequence;
        const uint32_t rx_delta = f->rx_ms - cal->last_rx_ms;
        if (seq_delta > UINT32_MAX / 2U || rx_delta > UINT32_MAX / 2U ||
            (seq_delta == 0U && (rx_delta != 0U || f->q_rad != cal->previous_q_rad ||
                                f->v_rad_s != cal->previous_v_rad_s || f->temp_c != cal->previous_temp_c)))
            return CG_CAL_FAULT_FEEDBACK;
        if (seq_delta != 0U && fabsf(f->q_rad - cal->previous_q_rad) >
            c->position_jump_rad + c->speed_trip_rad_s * (float)rx_delta * 0.001f)
            return CG_CAL_FAULT_POSITION;
    }
    const float displacement = fabsf(f->q_rad - cal->initial_position_rad);
    const float horizon_s = ((float)c->guard_lookahead_ms + (float)(now_ms - f->rx_ms)) * 0.001f;
    /* Conservative in either direction. There is deliberately no claim that
     * this constant-velocity check bounds unknown future acceleration. */
    if (displacement >= CG_CAL_ABSOLUTE_TRAVEL_RAD || displacement >= c->travel_hard_rad ||
        displacement + c->guard_margin_rad + fabsf(f->v_rad_s) * horizon_s >= c->travel_soft_rad)
        return CG_CAL_FAULT_POSITION;
    cal->last_rx_ms = f->rx_ms;
    cal->last_rx_sequence = f->rx_sequence;
    cal->previous_q_rad = f->q_rad;
    cal->previous_v_rad_s = f->v_rad_s;
    cal->previous_temp_c = f->temp_c;
    return CG_CAL_FAULT_NONE;
}

bool cg_cal_start(CgCal *cal, const CgCalConfig *config, float origin_rad,
                  const CgCalFeedback *f, uint32_t now_ms)
{
    if (!cal) return false;
    /* Repeated starts cannot silently erase an active origin or latched fault. */
    if (cal->phase != CG_CAL_PHASE_IDLE) return false;
    if (!cg_cal_config_valid(config) || !isfinite(origin_rad) ||
        fabsf(origin_rad) + config->travel_soft_rad >= 12.5f) {
        cg_cal_abort(cal, CG_CAL_FAULT_CONFIG);
        return false;
    }
    cal->config = *config;
    cal->initial_position_rad = origin_rad;
    CgCalFault fault = check_feedback(cal, f, now_ms, true);
    if (fault == CG_CAL_FAULT_NONE && fabsf(f->v_rad_s) > config->stationary_speed_rad_s)
        fault = CG_CAL_FAULT_BASELINE_MOTION;
    if (fault != CG_CAL_FAULT_NONE) {
        cg_cal_abort(cal, fault);
        return false;
    }
    cal->phase = CG_CAL_PHASE_BASELINE;
    cal->baseline_position_rad = f->q_rad;
    cal->baseline_started_ms = now_ms;
    cal->phase_ms = now_ms;
    cal->last_step_ms = now_ms;
    return true;
}

static CgCalOutput stop_output(void)
{
    const CgCalOutput out = {false, true, 0.0f, 0U};
    return out;
}

static void enter_phase(CgCal *cal, CgCalPhase phase, uint32_t now_ms)
{
    cal->phase = phase;
    cal->phase_ms = now_ms;
}

CgCalOutput cg_cal_step(CgCal *cal, const CgCalFeedback *f, uint32_t now_ms)
{
    CgCalOutput out = {false, false, 0.0f, 0U};
    if (!cal) return stop_output();
    if (cal->phase == CG_CAL_PHASE_STOPPING || cal->phase == CG_CAL_PHASE_FAULT)
        return stop_output();
    if (cal->phase == CG_CAL_PHASE_IDLE || cal->phase == CG_CAL_PHASE_DONE) return out;
    if (cal->pending_command) {
        cg_cal_abort(cal, CG_CAL_FAULT_TX);
        return stop_output();
    }
    if ((uint32_t)(now_ms - cal->last_step_ms) > cal->config.max_step_ms) {
        cg_cal_abort(cal, CG_CAL_FAULT_TIMING);
        return stop_output();
    }
    cal->last_step_ms = now_ms;
    const bool new_feedback = f && f->rx_sequence != cal->last_rx_sequence;
    const CgCalFault fault = check_feedback(cal, f, now_ms, false);
    if (fault != CG_CAL_FAULT_NONE) {
        cg_cal_abort(cal, fault);
        return stop_output();
    }

    const uint32_t elapsed = now_ms - cal->phase_ms;
    switch (cal->phase) {
    case CG_CAL_PHASE_BASELINE:
        if (fabsf(f->q_rad - cal->baseline_position_rad) > CG_CAL_BASELINE_DRIFT_RAD ||
            (uint32_t)(now_ms-cal->baseline_started_ms) >=
                cal->config.baseline_ms + CG_CAL_BASELINE_RETRY_MS) {
            cg_cal_abort(cal, CG_CAL_FAULT_BASELINE_MOTION);
            return stop_output();
        }
        if (fabsf(f->v_rad_s) > cal->config.stationary_speed_rad_s) {
            /* Keep commanding zero; require a complete quiet window again.
             * Raw speed/position trips above remain active, with a fixed
             * baseline anchor and bounded total wait. No pulse on a spike. */
            cal->phase_ms = now_ms;
        } else if (new_feedback && elapsed >= cal->config.baseline_ms) {
            enter_phase(cal, CG_CAL_PHASE_PULSE, now_ms);
        }
        break;
    case CG_CAL_PHASE_PULSE:
        if (elapsed >= cal->config.pulse_ms) {
            cal->brake_entry_v_rad_s = f->v_rad_s;
            enter_phase(cal, fabsf(f->v_rad_s) <= cal->config.stationary_speed_rad_s ?
                CG_CAL_PHASE_COAST : CG_CAL_PHASE_BRAKE, now_ms);
        }
        break;
    case CG_CAL_PHASE_BRAKE:
        /* Never keep opposing current after observed zero crossing, even if
         * current-to-acceleration polarity is negative. */
        if (fabsf(f->v_rad_s) <= cal->config.stationary_speed_rad_s ||
            f->v_rad_s * cal->brake_entry_v_rad_s <= 0.0f)
            enter_phase(cal, CG_CAL_PHASE_COAST, now_ms);
        else if (elapsed >= cal->config.brake_ms) {
            cg_cal_abort(cal, CG_CAL_FAULT_BRAKE_TIMEOUT);
            return stop_output();
        }
        break;
    case CG_CAL_PHASE_COAST:
        if (fabsf(f->v_rad_s) > cal->config.stationary_speed_rad_s) {
            cg_cal_abort(cal, CG_CAL_FAULT_BASELINE_MOTION);
            return stop_output();
        }
        if (elapsed >= cal->config.settle_ms) {
            enter_phase(cal, CG_CAL_PHASE_STOPPING, now_ms);
            return stop_output();
        }
        break;
    default:
        cg_cal_abort(cal, CG_CAL_FAULT_CONFIG);
        return stop_output();
    }

    out.write_current = true;
    if (cal->phase == CG_CAL_PHASE_PULSE) out.current_a = cal->config.pulse_current_a;
    else if (cal->phase == CG_CAL_PHASE_BRAKE)
        out.current_a = cal->config.pulse_current_a > 0.0f ?
            -cal->config.brake_current_a : cal->config.brake_current_a;
    out.generation = ++cal->command_generation;
    cal->pending_output = out;
    cal->pending_command = true;
    return out;
}

bool cg_cal_commit(CgCal *cal, const CgCalOutput *out, bool tx_ok)
{
    if (!cal) return false;
    if (!tx_ok || !out || !cal->pending_command || !out->write_current || out->stop ||
        out->generation != cal->pending_output.generation ||
        out->current_a != cal->pending_output.current_a) {
        cg_cal_abort(cal, CG_CAL_FAULT_TX);
        return false;
    }
    cal->last_current_a = out->current_a;
    cal->pending_command = false;
    return true;
}

void cg_cal_confirm_stopped(CgCal *cal)
{
    if (!cal || cal->phase != CG_CAL_PHASE_STOPPING) return;
    cal->pending_command = false;
    cal->last_current_a = 0.0f;
    cal->phase = cal->fault == CG_CAL_FAULT_NONE ? CG_CAL_PHASE_DONE : CG_CAL_PHASE_FAULT;
}

const char *cg_cal_phase_name(CgCalPhase phase)
{
    static const char *const names[] = {
        "IDLE", "BASELINE", "PULSE", "BRAKE", "COAST", "STOPPING", "DONE", "FAULT"
    };
    return (unsigned int)phase < sizeof(names) / sizeof(names[0]) ? names[phase] : "UNKNOWN";
}

const char *cg_cal_fault_name(CgCalFault fault)
{
    static const char *const names[] = {
        "NONE", "CONFIG", "FEEDBACK", "TIMING", "MOTOR", "POSITION", "SPEED",
        "TEMPERATURE", "TX", "LOG_OVERFLOW", "REQUESTED_STOP", "BASELINE_MOTION",
        "BRAKE_TIMEOUT", "STOP_TIMEOUT", "BUS"
    };
    return (unsigned int)fault < sizeof(names) / sizeof(names[0]) ? names[fault] : "UNKNOWN";
}
