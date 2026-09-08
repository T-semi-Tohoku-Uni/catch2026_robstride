#include "cybergear_calibration_app.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

_Static_assert(CG_CAL_APP_PERIOD_MS == 10U, "Calibration requires the verified 100 Hz scheduler");
_Static_assert(CG_CAL_APP_RETRY_MS >= CG_CAL_APP_PERIOD_MS &&
               CG_CAL_APP_RETRY_MS < CG_CAL_FEEDBACK_TIMEOUT_MS,
               "STOP/readback retry must fit inside the feedback timeout");
_Static_assert(CG_CAL_APP_QUIET_MS >= 2U * CG_CAL_APP_PERIOD_MS,
               "Quiet confirmation requires multiple samples");
_Static_assert(CG_CAL_APP_LOG_CAPACITY >= 2U, "Reserve a terminal log row");

static uint32_t lock(void) { uint32_t mask = __get_PRIMASK(); __disable_irq(); return mask; }

void cg_cal_app_config_defaults(CgCalConfig *c)
{
    cg_cal_config_defaults(c);
    if (c == NULL) return;
    c->armed = CG_CAL_ARMED != 0;
    c->pulse_current_a = CG_CAL_PULSE_CURRENT_A;
    c->brake_current_a = CG_CAL_BRAKE_CURRENT_A;
    c->current_limit_a = CG_CAL_CURRENT_LIMIT_A;
    c->temp_trip_c = CG_CAL_TEMPERATURE_TRIP_C;
    c->speed_trip_rad_s = CG_CAL_SPEED_TRIP_RAD_S;
    c->travel_soft_rad = CG_CAL_TRAVEL_SOFT_DEG * (CG_CAL_ABSOLUTE_TRAVEL_RAD / 90.0f);
    c->travel_hard_rad = CG_CAL_TRAVEL_HARD_DEG * (CG_CAL_ABSOLUTE_TRAVEL_RAD / 90.0f);
    c->guard_margin_rad = CG_CAL_GUARD_MARGIN_DEG * (CG_CAL_ABSOLUTE_TRAVEL_RAD / 90.0f);
    c->guard_lookahead_ms = CG_CAL_GUARD_LOOKAHEAD_MS;
    c->position_jump_rad = CG_CAL_POSITION_JUMP_RAD;
    c->stationary_speed_rad_s = CG_CAL_STATIONARY_SPEED_RAD_S;
    c->baseline_ms = CG_CAL_BASELINE_MS;
    c->pulse_ms = CG_CAL_PULSE_MS;
    c->brake_ms = CG_CAL_BRAKE_MS;
    c->settle_ms = CG_CAL_SETTLE_MS;
    c->feedback_timeout_ms = CG_CAL_FEEDBACK_TIMEOUT_MS;
    c->max_step_ms = CG_CAL_MAX_STEP_MS;
}

static CgCalFeedback snapshot(const CgCalApp *a)
{
    const CyberGearFeedback *f = &a->motor->feedback;
    return (CgCalFeedback){.q_rad=f->position_rad,.v_rad_s=f->velocity_rad_s,
        .temp_c=f->temperature_c,.rx_ms=f->last_received_ms,.rx_sequence=f->rx_sequence,
        .online=f->online,.motor_fault=f->fault_flags != 0U || a->motor->motor_fault_sequence != 0U};
}

static bool fresh(const CgCalApp *a, const CgCalFeedback *f, uint32_t now)
{
    return f->online && now-f->rx_ms <= a->config.feedback_timeout_ms;
}

static bool quiet(CgCalApp *a, const CgCalFeedback *f, uint32_t now, uint8_t mode)
{
    if (!fresh(a,f,now) || !isfinite(f->q_rad) || !isfinite(f->v_rad_s) || !isfinite(f->temp_c) ||
        f->motor_fault || a->motor->feedback.mode != mode ||
        fabsf(f->v_rad_s) > a->config.stationary_speed_rad_s) {
        a->quiet_active = false;
        return false;
    }
    if (f->rx_sequence == a->quiet_sequence) return false;
    a->quiet_sequence = f->rx_sequence;
    if (!a->quiet_active) { a->quiet_since_ms=now; a->quiet_active=true; }
    return now-a->quiet_since_ms >= CG_CAL_APP_QUIET_MS;
}

static void request_stop(CgCalApp *a, CgCalFault fault, uint32_t now)
{
    if (fault != CG_CAL_FAULT_NONE && !a->fault_snapshot_valid) {
        a->fault_snapshot_valid=true;
        a->fault_state=a->state;
        a->first_fault=fault;
        a->fault_feedback=snapshot(a);
        a->fault_timestamp_ms=now;
        a->fault_motor_mode=a->motor->feedback.mode;
    }
    if (fault != CG_CAL_FAULT_NONE) cg_cal_abort(&a->core, fault);
    if (a->state == CG_CAL_APP_STOPPING || a->state == CG_CAL_APP_FAULT) return;
    a->state=CG_CAL_APP_STOPPING;
    a->state_ms=now;
    a->stop_queued=false;
    a->reset_confirmed=false;
    a->stop_confirmed=false;
    a->quiet_active=false;
    a->quiet_sequence=a->motor->feedback.rx_sequence;
    a->stop_sequence=a->motor->feedback.rx_sequence;
    a->last_command_ms=now-CG_CAL_APP_RETRY_MS;
    a->requested_direction=0;
    a->motor->mode_read_pending=false;
}

bool cg_cal_app_init(CgCalApp *a, CyberGearMotor *m, const CgCalConfig *c)
{
    if (a == NULL || m == NULL || c == NULL) return false;
    memset(a,0,sizeof(*a));
    a->motor=m;
    a->config=*c;
    a->trial_config=*c;
    cg_cal_init(&a->core);
    if (!cybergear_claim_calibration(m,c)) {
        a->state=CG_CAL_APP_FAULT;
        cg_cal_abort(&a->core,CG_CAL_FAULT_CONFIG);
        return false;
    }
    a->state=CG_CAL_APP_CAPTURE;
    a->startup_ms=HAL_GetTick();
    a->state_ms=a->startup_ms;
    a->last_tick_ms=a->startup_ms;
    a->last_command_ms=a->startup_ms-CG_CAL_APP_RETRY_MS;
    return true;
}

bool cg_cal_app_request_trial(CgCalApp *a, int direction)
{
    if (a == NULL || (direction != 1 && direction != -1)) return false;
    uint32_t mask=lock();
    bool ok=(a->state == CG_CAL_APP_READY || a->state == CG_CAL_APP_DONE) &&
        a->origin_captured && !a->dump_pending && a->core.fault == CG_CAL_FAULT_NONE &&
        a->requested_direction == 0;
    if (ok) a->requested_direction=direction;
    __set_PRIMASK(mask);
    return ok;
}

void cg_cal_app_abort(CgCalApp *a)
{
    if (a == NULL) return;
    uint32_t mask=lock(); a->abort_requested=true; __set_PRIMASK(mask);
}

static CgCalFault guard(const CgCalApp *a, const CgCalFeedback *f, uint32_t now)
{
    if (f->motor_fault) return CG_CAL_FAULT_MOTOR;
    if (!isfinite(f->q_rad) || !isfinite(f->v_rad_s) || !isfinite(f->temp_c)) return CG_CAL_FAULT_FEEDBACK;
    if (!fresh(a,f,now)) return CG_CAL_FAULT_FEEDBACK;
    if (f->temp_c >= a->config.temp_trip_c) return CG_CAL_FAULT_TEMPERATURE;
    if (fabsf(f->v_rad_s) >= a->config.speed_trip_rad_s) return CG_CAL_FAULT_SPEED;
    if (a->origin_captured) {
        float distance=fabsf(f->q_rad-a->initial_position_rad);
        float projected=distance+fabsf(f->v_rad_s)*(float)(a->config.guard_lookahead_ms+now-f->rx_ms)*0.001f;
        if (distance >= CG_CAL_ABSOLUTE_TRAVEL_RAD || distance >= a->config.travel_hard_rad ||
            projected+a->config.guard_margin_rad >= a->config.travel_soft_rad) return CG_CAL_FAULT_POSITION;
    }
    return CG_CAL_FAULT_NONE;
}

static bool send_stop(CgCalApp *a, uint32_t now)
{
    a->motor->internal_send=true;
    bool ok=cybergear_stop(a->motor);
    a->motor->internal_send=false;
    a->last_command_ms=now;
    if (ok && !a->stop_queued) a->stop_sequence=a->motor->feedback.rx_sequence;
    a->stop_queued=a->stop_queued || ok;
    return ok;
}

static void record(CgCalApp *a, const CgCalFeedback *f, uint32_t now)
{
    if (!a->recording) return;
    /* Last slot is reserved for terminal/fault evidence. Do not silently lose
     * a plateau and then report a successful fit from the truncated trace. */
    if (a->log_count >= CG_CAL_APP_LOG_CAPACITY-1U) {
        a->dropped++;
        if (a->state != CG_CAL_APP_STOPPING && a->state != CG_CAL_APP_FAULT && a->state != CG_CAL_APP_DONE)
            request_stop(a,CG_CAL_FAULT_LOG_OVERFLOW,now);
        if (a->state != CG_CAL_APP_FAULT && a->state != CG_CAL_APP_DONE) return;
        a->log_count=CG_CAL_APP_LOG_CAPACITY-1U;
    }
    CgCalLogRow *r=&a->rows[a->log_count++];
    *r=(CgCalLogRow){.timestamp_ms=now,.feedback_timestamp_ms=f->rx_ms,.rx_sequence=f->rx_sequence,
        .trial_id=a->trial_id,.phase=a->core.phase,.fault=a->core.fault,
        .app_state=a->state,.motor_mode=a->motor->feedback.mode,
        .position_rad=f->q_rad,.velocity_rad_s=f->v_rad_s,.temperature_c=f->temp_c,
        .command_current_a=a->core.last_current_a,.dropped=a->dropped,
        .tx_failed=a->motor->tx_failed,.feedback_valid=fresh(a,f,now)};
    if (a->state == CG_CAL_APP_DONE || a->state == CG_CAL_APP_FAULT) a->recording=false;
}

void cg_cal_app_tick(CgCalApp *a, uint32_t now)
{
    if (a == NULL || a->motor == NULL || !a->motor->calibration_owned) return;
    CyberGearMotor *m=a->motor;
    CgCalFeedback f=snapshot(a);
    uint32_t dt=now-a->last_tick_ms;
    a->last_tick_ms=now;
    if (a->state == CG_CAL_APP_FAULT) return;
    if (a->abort_requested || m->fault != CG_FAULT_NONE) {
        a->abort_requested=false;
        request_stop(a,CG_CAL_FAULT_REQUESTED_STOP,now);
    }
    if (f.motor_fault) request_stop(a,CG_CAL_FAULT_MOTOR,now);
    if (a->recording && a->log_count >= CG_CAL_APP_LOG_CAPACITY-1U && a->state != CG_CAL_APP_STOPPING)
        request_stop(a,CG_CAL_FAULT_LOG_OVERFLOW,now);
    if (dt > a->config.max_step_ms) request_stop(a,CG_CAL_FAULT_TIMING,now);
    FDCAN_ProtocolStatusTypeDef bus;
    FDCAN_ErrorCountersTypeDef errors;
    if (HAL_FDCAN_GetProtocolStatus(m->hfdcan,&bus) != HAL_OK ||
        HAL_FDCAN_GetErrorCounters(m->hfdcan,&errors) != HAL_OK || bus.BusOff || bus.ErrorPassive ||
        errors.TxErrorCnt || errors.RxErrorCnt)
        request_stop(a,CG_CAL_FAULT_BUS,now);
    if (a->state != CG_CAL_APP_STOPPING && (f.online || a->state != CG_CAL_APP_CAPTURE)) {
        CgCalFault reason=guard(a,&f,now);
        if (reason != CG_CAL_FAULT_NONE) request_stop(a,reason,now);
    }
    if (a->state >= CG_CAL_APP_WRITE_MODE && a->state <= CG_CAL_APP_WAIT_RUN &&
        fresh(a,&f,now) && fabsf(f.q_rad-a->startup_position_rad) > CG_CAL_BASELINE_DRIFT_RAD)
        request_stop(a,CG_CAL_FAULT_BASELINE_MOTION,now);
    if (a->state == CG_CAL_APP_CAPTURE && !a->origin_captured && fresh(a,&f,now)) {
        /* Capture once BEFORE any mode change or ENABLE, never on p/n/re-arm. */
        a->initial_position_rad=f.q_rad;
        a->origin_sequence=f.rx_sequence;
        a->origin_captured=true;
        if (fabsf(f.q_rad)+a->config.travel_soft_rad >= 12.5f)
            request_stop(a,CG_CAL_FAULT_POSITION,now); /* no protocol wrap workaround */
    }
    bool due=now-a->last_command_ms >= CG_CAL_APP_RETRY_MS;
    bool ok=true;
    m->internal_send=true;
    switch (a->state) {
    case CG_CAL_APP_CAPTURE:
        if (now-a->startup_ms >= CG_CAL_APP_START_TIMEOUT_MS) request_stop(a,CG_CAL_FAULT_FEEDBACK,now);
        else if (a->origin_captured && a->stop_queued && f.rx_sequence != a->stop_sequence && quiet(a,&f,now,0U))
            a->state=CG_CAL_APP_READY;
        if (due && a->state == CG_CAL_APP_CAPTURE) ok=send_stop(a,now);
        break;
    case CG_CAL_APP_READY:
    case CG_CAL_APP_DONE:
        if (a->requested_direction != 0) {
            if (!quiet(a,&f,now,0U)) { if (due) ok=send_stop(a,now); break; }
            a->trial_config=a->config;
            a->startup_position_rad=f.q_rad;
            a->trial_config.pulse_current_a=fabsf(a->config.pulse_current_a)*(float)a->requested_direction;
            a->requested_direction=0;
            cg_cal_init(&a->core);
            a->fault_snapshot_valid=false;
            a->trial_id++;
            /* Include mode handshake and zero-current startup in the trace.
             * UART still drains only after STOP completes or expires. */
            a->log_count=0U; a->dropped=0U; a->recording=true;
            a->stop_confirmed=false; a->reset_confirmed=false;
            a->startup_ms=now; a->state=CG_CAL_APP_WRITE_MODE;
            ok=send_stop(a,now); /* refresh stopped feedback before mode handshake */
        } else if (due) ok=send_stop(a,now); /* keep fresh stopped feedback */
        break;
    case CG_CAL_APP_WRITE_MODE:
        ok=cybergear_set_run_mode(m,CYBERGEAR_RUN_MODE_CURRENT);
        if (ok) {
            a->state=CG_CAL_APP_READ_MODE;
            m->mode_read_valid=false; m->mode_read_pending=false;
            a->read_sequence=m->mode_read_sequence;
            a->last_command_ms=now;
            a->state_ms=now-CG_CAL_APP_RETRY_MS;
        }
        break;
    case CG_CAL_APP_READ_MODE:
        if (m->mode_read_valid && m->mode_read_sequence != a->read_sequence) {
            if (m->mode_read_value != CYBERGEAR_RUN_MODE_CURRENT) request_stop(a,CG_CAL_FAULT_MOTOR,now);
            else { m->mode_read_pending=false; a->state=CG_CAL_APP_ZERO; ok=send_stop(a,now); }
        } else if (now-a->state_ms >= CG_CAL_APP_RETRY_MS) {
            m->mode_read_pending=true;
            a->read_sequence=m->mode_read_sequence;
            ok=cybergear_read_parameter(m,CYBERGEAR_PARAM_RUN_MODE);
            a->state_ms=now;
        } else ok=send_stop(a,now); /* obtain fresh Reset feedback between reads */
        break;
    case CG_CAL_APP_ZERO:
        ok=cybergear_set_current(m,0.0f);
        if (ok) a->state=CG_CAL_APP_ENABLE;
        break;
    case CG_CAL_APP_ENABLE:
        ok=cybergear_enable_calibration(m,&a->trial_config);
        if (ok) {
            a->state=CG_CAL_APP_WAIT_RUN; a->quiet_active=false;
            a->quiet_sequence=f.rx_sequence; a->stop_sequence=f.rx_sequence;
        }
        break;
    case CG_CAL_APP_WAIT_RUN:
        /* quiet() restarts on a speed spike. Stay at zero current until the
         * full quiet window passes; startup deadline and pinned drift guard
         * above remain active. Never authorize a pulse from one quiet sample. */
        if (f.rx_sequence != a->stop_sequence && quiet(a,&f,now,2U)) {
            if (cg_cal_start(&a->core,&a->trial_config,a->initial_position_rad,&f,now)) {
                a->state=CG_CAL_APP_TRIAL; a->recording=true;
            } else request_stop(a,CG_CAL_FAULT_CONFIG,now);
        } else ok=cybergear_set_current(m,0.0f);
        break;
    case CG_CAL_APP_TRIAL: {
        if (m->feedback.mode != 2U || !m->mode_read_valid || m->mode_read_value != CYBERGEAR_RUN_MODE_CURRENT) {
            request_stop(a,CG_CAL_FAULT_MOTOR,now); break;
        }
        CgCalOutput out=cg_cal_step(&a->core,&f,now);
        if (out.stop) request_stop(a,a->core.fault,now);
        else if (out.write_current) {
            ok=cybergear_set_current(m,out.current_a);
            if (!cg_cal_commit(&a->core,&out,ok)) request_stop(a,CG_CAL_FAULT_TX,now);
        }
        break;
    }
    case CG_CAL_APP_STOPPING:
        break;
    default: break;
    }
    m->internal_send=false;
    if (!ok) request_stop(a,CG_CAL_FAULT_TX,now);
    if (a->state >= CG_CAL_APP_WRITE_MODE && a->state <= CG_CAL_APP_WAIT_RUN &&
        now-a->startup_ms >= CG_CAL_APP_START_TIMEOUT_MS) request_stop(a,CG_CAL_FAULT_FEEDBACK,now);
    if (a->state == CG_CAL_APP_STOPPING) {
        if (a->stop_queued && f.rx_sequence != a->stop_sequence && fresh(a,&f,now) && m->feedback.mode == 0U)
            a->reset_confirmed=true;
        if (a->reset_confirmed && quiet(a,&f,now,0U)) {
            a->stop_confirmed=true;
            cg_cal_confirm_stopped(&a->core);
            a->state=a->core.fault == CG_CAL_FAULT_NONE ? CG_CAL_APP_DONE : CG_CAL_APP_FAULT;
            a->dump_pending=true;
        } else if (now-a->state_ms >= CG_CAL_APP_STOP_TIMEOUT_MS) {
            cg_cal_abort(&a->core,CG_CAL_FAULT_STOP_TIMEOUT);
            a->core.phase=CG_CAL_PHASE_FAULT; /* terminal, but never claim stopped */
            a->state=CG_CAL_APP_FAULT; a->dump_pending=true;
        } else if (now-a->last_command_ms >= CG_CAL_APP_RETRY_MS) {
            if (!send_stop(a,now)) cg_cal_abort(&a->core,CG_CAL_FAULT_TX);
        }
    }
    record(a,&f,now);
}

static bool uart_text(UART_HandleTypeDef *uart, const char *text)
{
    return HAL_UART_Transmit(uart,(const uint8_t *)text,(uint16_t)strlen(text),200U) == HAL_OK;
}

bool cg_cal_app_dump(CgCalApp *a, UART_HandleTypeDef *uart)
{
    if (a == NULL || uart == NULL) return false;
    uint32_t mask=lock();
    bool ready=a->dump_pending && (a->state == CG_CAL_APP_DONE || a->state == CG_CAL_APP_FAULT);
    uint32_t count=a->log_count;
    __set_PRIMASK(mask);
    if (!ready) return false;
    char line[256];
    int written=snprintf(line,sizeof(line),"# CGCAL trial=%lu origin=%.7f fault=%s stop_queued=%u reset=%u stationary=%u\r\n",
        (unsigned long)a->trial_id,(double)a->initial_position_rad,cg_cal_fault_name(a->core.fault),
        (unsigned)a->stop_queued,(unsigned)a->reset_confirmed,(unsigned)a->stop_confirmed);
    if (written < 0 || (size_t)written >= sizeof(line) || !uart_text(uart,line)) return false;
    if (a->fault_snapshot_valid) {
        static const char *const states[]={"CAPTURE","READY","WRITE_MODE","READ_MODE",
            "ZERO","ENABLE","WAIT_RUN","TRIAL","STOPPING","DONE","FAULT"};
        const unsigned state=(unsigned)a->fault_state;
        const CgCalFeedback *f=&a->fault_feedback;
        written=snprintf(line,sizeof(line),
            "# CGCAL first_fault=%s state=%s t_ms=%lu mode=%u online=%u rx_age_ms=%lu rx_seq=%lu\r\n",
            cg_cal_fault_name(a->first_fault),state < sizeof(states)/sizeof(states[0]) ? states[state] : "UNKNOWN",
            (unsigned long)a->fault_timestamp_ms,(unsigned)a->fault_motor_mode,(unsigned)f->online,
            (unsigned long)(a->fault_timestamp_ms-f->rx_ms),(unsigned long)f->rx_sequence);
        if (written < 0 || (size_t)written >= sizeof(line) || !uart_text(uart,line)) return false;
        written=snprintf(line,sizeof(line),
            "# CGCAL fault_sample q_rad=%.7f delta_q_rad=%.7f v_rad_s=%.7f stationary_limit_rad_s=%.7f temp_c=%.2f\r\n",
            (double)f->q_rad,(double)(f->q_rad-a->initial_position_rad),(double)f->v_rad_s,
            (double)a->config.stationary_speed_rad_s,(double)f->temp_c);
        if (written < 0 || (size_t)written >= sizeof(line) || !uart_text(uart,line)) return false;
    }
    if (!uart_text(uart,"timestamp_ms,feedback_timestamp_ms,rx_sequence,trial_id,phase,position_rad,velocity_rad_s,command_current_a,rx_age_ms,fault,saturated,dropped,tx_failed,feedback_valid,initial_position_rad,temperature_c,app_state,motor_mode\r\n")) return false;
    for (uint32_t i=0U;i<count;++i) {
        const CgCalLogRow *r=&a->rows[i];
        written=snprintf(line,sizeof(line),"%lu,%lu,%lu,%lu,%s,%.7f,%.7f,%.6f,%lu,%u,0,%lu,%lu,%u,%.7f,%.2f,%u,%u\r\n",
            (unsigned long)r->timestamp_ms,(unsigned long)r->feedback_timestamp_ms,
            (unsigned long)r->rx_sequence,(unsigned long)r->trial_id,cg_cal_phase_name(r->phase),
            (double)r->position_rad,(double)r->velocity_rad_s,(double)r->command_current_a,
            (unsigned long)(r->timestamp_ms-r->feedback_timestamp_ms),(unsigned)r->fault,
            (unsigned long)r->dropped,(unsigned long)r->tx_failed,(unsigned)r->feedback_valid,
            (double)a->initial_position_rad,(double)r->temperature_c,
            (unsigned)r->app_state,(unsigned)r->motor_mode);
        if (written < 0 || (size_t)written >= sizeof(line) || !uart_text(uart,line)) return false;
    }
    mask=lock(); a->dump_pending=false; __set_PRIMASK(mask);
    return true;
}
