#include "cybergear.h"
#include "cybergear_config.h"
#include <math.h>
#include <string.h>

void cybergear_latch_fault(CyberGearMotor *m, CGFault reason, uint32_t now)
{
    CGDiagnostics *d=&m->diagnostics;
    if (d->fault!=CG_FAULT_NONE) return;
    d->fault=reason; d->state=CG_FAULT_LATCHED; d->state_since_ms=now;
    d->stop_requested=true; d->stop_attempts=0; d->stop_queued=false;
    d->reset_confirmed=false; d->mechanically_stationary=false;
    d->baseline_sequence=m->feedback.rx_sequence;
    d->applied_current_valid=false; m->adrc.active=false;
}

static void latch(CyberGearMotor *m, CGFault reason, uint32_t now)
{
    cybergear_latch_fault(m,reason,now);
}

static bool request_mode(CyberGearMotor *m)
{
    uint8_t data[8]={0x05,0x70,0,0,0,0,0,0};
    FDCAN_TxHeaderTypeDef header=m->tx_header;
    header.Identifier=(17UL<<24)|((uint32_t)m->master_id<<8)|m->motor_id;
    m->diagnostics.fifo_free=HAL_FDCAN_GetTxFifoFreeLevel(m->hfdcan);
    if (HAL_FDCAN_AddMessageToTxFifoQ(m->hfdcan,&header,data)!=HAL_OK) {
        ++m->diagnostics.tx_failed; return false;
    }
    ++m->diagnostics.tx_queued;
    return true;
}

bool cybergear_start_position_adrc(CyberGearMotor *m)
{
    if (!m || m->diagnostics.state!=CG_DISARMED || m->diagnostics.fault) return false;
    if (!cg_config_valid(&m->config)) {
        latch(m,CG_FAULT_CONFIG,HAL_GetTick());
        m->internal_owner=true; cybergear_stop(m); m->internal_owner=false;
        return false;
    }
    memset(&m->adrc,0,sizeof(m->adrc));
    memset(&m->controller,0,sizeof(m->controller));
    m->diagnostics.state=CG_STOP_WAIT;
    m->diagnostics.state_since_ms=HAL_GetTick();
    m->diagnostics.timestamp_ms=HAL_GetTick();
    m->diagnostics.stop_requested=true;
    m->diagnostics.baseline_sequence=m->feedback.rx_sequence;
    return true; /* Request accepted; RUN requires subsequent confirmed replies. */
}

static void stop_service(CyberGearMotor *m, CyberGearFeedback f, uint32_t now)
{
    CGDiagnostics *d=&m->diagnostics;
    if (f.online && f.rx_sequence!=d->baseline_sequence && now-f.last_received_ms<=20) {
        d->reset_confirmed=f.mode==0;
        d->mechanically_stationary=fabsf(f.velocity_rad_s)<0.05f;
    }
    if (!d->reset_confirmed && now-d->state_since_ms<500 && d->stop_attempts<5 &&
        (d->stop_attempts==0 || now-d->last_stop_ms>=20)) {
        ++d->stop_attempts; d->last_stop_ms=now;
        d->stop_queued=cybergear_stop(m) || d->stop_queued;
    }
}

static void transition(CyberGearMotor *m, CGState next, uint32_t now)
{
    m->state_command_queued=false;
    m->diagnostics.state=next; m->diagnostics.state_since_ms=now;
    m->diagnostics.baseline_sequence=m->feedback.rx_sequence;
}

static bool planning_budget(void *context)
{
    uint32_t start=*(uint32_t *)context;
    return DWT->CYCCNT-start < SystemCoreClock/2000U; /* 500 us; checked during root search */
}

static bool service(CyberGearMotor *m,float target)
{
    CGDiagnostics *d=&m->diagnostics;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    CyberGearFeedback f=m->feedback;
    uint32_t now=HAL_GetTick();
    __set_PRIMASK(mask);
    d->dt_ms=now-d->timestamp_ms; d->timestamp_ms=now; ++d->control_cycles;
    d->rx_sequence=f.rx_sequence; d->rx_age_ms=now-f.last_received_ms;
    FDCAN_ProtocolStatusTypeDef bus={0}; FDCAN_ErrorCountersTypeDef errors={0};
    bool bus_valid=HAL_FDCAN_GetProtocolStatus(m->hfdcan,&bus)==HAL_OK &&
        HAL_FDCAN_GetErrorCounters(m->hfdcan,&errors)==HAL_OK;
    d->busoff=bus.BusOff; d->tec=errors.TxErrorCnt; d->rec=errors.RxErrorCnt;
    d->fifo_free=HAL_FDCAN_GetTxFifoFreeLevel(m->hfdcan);
    if (d->state==CG_DISARMED) return false;
    if (!bus_valid || bus.BusOff) latch(m,CG_FAULT_BUS,now);
    if (f.fault_flags) latch(m,CG_FAULT_DEVICE,now);
    if (d->state==CG_FAULT_LATCHED) { stop_service(m,f,now); return false; }
    if (!isfinite(target) || target<m->config.motion.q_min || target>m->config.motion.q_max)
        latch(m,CG_FAULT_INPUT,now);
    if (d->state==CG_FAULT_LATCHED) { stop_service(m,f,now); return false; }
    if (d->state!=CG_RUN && now-d->state_since_ms>500) {
        latch(m,CG_FAULT_TIMEOUT,now); stop_service(m,f,now); return false;
    }
    switch (d->state) {
    case CG_STOP_WAIT:
        stop_service(m,f,now);
        if (d->reset_confirmed && d->mechanically_stationary) transition(m,CG_MODE_WRITE,now);
        return true;
    case CG_MODE_WRITE:
        if (!cybergear_set_run_mode(m,CYBERGEAR_RUN_MODE_CURRENT)) break;
        m->mode_sequence=0; transition(m,CG_MODE_READBACK,now); return true;
    case CG_MODE_READBACK:
        if (m->mode_sequence) {
            if (m->confirmed_mode!=CYBERGEAR_RUN_MODE_CURRENT) {
                latch(m,CG_FAULT_MODE,now); return false;
            }
            transition(m,CG_ZERO_COMMAND,now); return true;
        }
        /* One read request, no diagnostic traffic during RUN. */
        if (!m->state_command_queued) {
            if (!request_mode(m)) break;
            m->state_command_queued=true;
        }
        return true;
    case CG_ZERO_COMMAND:
        if (!cybergear_set_current(m,0)) break;
        d->last_queued_current=0; d->last_queued_ms=now;
        transition(m,CG_ENABLE_WAIT,now); return true;
    case CG_ENABLE_WAIT:
        if (!m->state_command_queued) {
            if (!cybergear_enable(m)) break;
            m->state_command_queued=true;
        }
        if (f.online && f.rx_sequence!=d->baseline_sequence && f.mode==2 && d->rx_age_ms<=20) {
            m->warmup_samples=0; m->consumed_sequence=f.rx_sequence;
            d->last_position=f.position_rad;
            transition(m,CG_OBSERVER_WARMUP,now);
        }
        return true;
    case CG_OBSERVER_WARMUP:
        if (f.rx_sequence!=m->consumed_sequence && d->rx_age_ms<=20 && f.mode==2) {
            m->consumed_sequence=f.rx_sequence;
            if (fabsf(f.velocity_rad_s)<0.05f) ++m->warmup_samples; else m->warmup_samples=0;
            if (m->warmup_samples>=5) {
                if (!isfinite(f.position_rad) || f.position_rad<m->config.motion.q_min ||
                    f.position_rad>m->config.motion.q_max) { latch(m,CG_FAULT_RANGE,now); return false; }
                cg_trajectory_hold(&m->trajectory,f.position_rad);
                m->controller.q=f.position_rad;
                d->last_position=f.position_rad;
                d->stall_position=f.position_rad; d->stall_since_ms=now;
                m->adrc.active=true; m->adrc.initialized=true;
                transition(m,CG_RUN,now);
            }
        }
        if (!cybergear_set_current(m,0)) break;
        return true;
    case CG_RUN: {
        const uint32_t period=1000/CYBERGEAR_CONTROL_HZ;
        /* Fixed-period approximation: latest measurement is placed at control time.
         * Do not mix variable feedback dt with the current-observer gains. */
        if (d->dt_ms!=period) { latch(m,CG_FAULT_DT,now); return false; }
        if (!f.online || d->rx_age_ms>period*2) { latch(m,CG_FAULT_TIMEOUT,now); return false; }
        if (f.mode!=2) { latch(m,CG_FAULT_MODE,now); return false; }
        if (!isfinite(f.position_rad) || !isfinite(f.velocity_rad_s) || !isfinite(f.temperature_c)) {
            latch(m,CG_FAULT_FEEDBACK,now); return false;
        }
        if (f.position_rad<m->config.guard_min || f.position_rad>m->config.guard_max) {
            latch(m,CG_FAULT_RANGE,now); return false;
        }
        if (f.temperature_c>m->config.temperature_limit) { latch(m,CG_FAULT_TEMPERATURE,now); return false; }
        if (fabsf(f.velocity_rad_s)>m->config.speed_trip) { latch(m,CG_FAULT_SPEED,now); return false; }
        bool fresh=f.rx_sequence!=m->consumed_sequence;
        if (fresh && fabsf(f.position_rad-d->last_position)>m->config.position_jump) {
            latch(m,CG_FAULT_JUMP,now); return false;
        }
        m->trajectory.elapsed=fminf(m->trajectory.duration,m->trajectory.elapsed+period*0.001f);
        CGReference r=cg_trajectory_sample(&m->trajectory,m->trajectory.elapsed);
        if (target!=m->trajectory.target) {
            uint32_t planning_start=DWT->CYCCNT;
            if (!cg_trajectory_plan_bounded(&m->trajectory,r,target,&m->config.motion,
                planning_budget,&planning_start)) {
                /* No feasible bounded plan: drive-disable stop, never keep old trajectory. */
                latch(m,CG_FAULT_PLAN,now); return false;
            }
        }
        if (!cg_controller_step(&m->controller,&m->config,r,f.position_rad,fresh,
            d->last_queued_current,period*0.001f)) { latch(m,CG_FAULT_FEEDBACK,now); return false; }
        if (fresh) {
            m->consumed_sequence=f.rx_sequence; d->last_position=f.position_rad;
            d->observer_timestamp_ms=now;
        }
        if (m->controller.amplitude_limited || m->controller.slew_limited) {
            if (!d->saturation_active) {
                d->saturation_active=true; d->saturation_since_ms=now;
            } else if (now-d->saturation_since_ms>=m->config.stall_ms) {
                latch(m,CG_FAULT_STALL,now); return false;
            }
        } else d->saturation_active=false;
        if (fabsf(f.position_rad-d->stall_position)>=m->config.stall_progress ||
            (fabsf(target-f.position_rad)<m->config.stall_error &&
            !m->controller.amplitude_limited && !m->controller.slew_limited)) {
            d->stall_position=f.position_rad; d->stall_since_ms=now;
        } else if (now-d->stall_since_ms>=m->config.stall_ms) {
            latch(m,CG_FAULT_STALL,now); return false;
        }
        if (!cybergear_set_current(m,m->controller.command)) break;
        d->last_queued_current=m->controller.command; d->last_queued_ms=now;
        /* Only a model estimate; HAL_OK does not prove on-bus delivery or actuation. */
        d->applied_current_estimate=d->last_queued_current; d->applied_current_valid=false;
        m->adrc.position_rad=m->controller.q; m->adrc.velocity_rad_s=m->controller.v;
        m->adrc.disturbance_rad_s2=m->controller.f; m->adrc.reference_rad=r.q;
        m->adrc.current_a=d->last_queued_current; m->adrc.last_update_ms=now;
        m->adrc.last_feedback_ms=f.last_received_ms;
        return true;
    }
    default: return false;
    }
    latch(m,CG_FAULT_TX,now); return false;
}

bool cybergear_control_position_adrc(CyberGearMotor *m,float target)
{
    if (!m || !m->hfdcan) return false;
    uint32_t start=DWT->CYCCNT;
    m->internal_owner=true;
    bool ok=service(m,target);
    if (!ok && m->diagnostics.state==CG_FAULT_LATCHED) {
        uint32_t mask=__get_PRIMASK(); __disable_irq();
        CyberGearFeedback f=m->feedback; __set_PRIMASK(mask);
        stop_service(m,f,HAL_GetTick());
    }
    m->internal_owner=false;
    uint32_t elapsed=DWT->CYCCNT-start;
    if (elapsed>m->diagnostics.max_execution_cycles) m->diagnostics.max_execution_cycles=elapsed;
    uint32_t head=m->log_head;
    if (head-m->log_tail<16) {
        CGLog *log=&m->logs[head%16];
        log->diagnostics=m->diagnostics; log->controller=m->controller;
        log->reference=cg_trajectory_sample(&m->trajectory,m->trajectory.elapsed);
        log->target=target; log->measured_position=m->feedback.position_rad;
        log->feedback_velocity=m->feedback.velocity_rad_s;
        __DMB(); m->log_head=head+1;
    } else ++m->diagnostics.log_dropped;
    return ok;
}

bool cybergear_log_pop(CyberGearMotor *m,CGLog *log)
{
    if (!m || !log) return false;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool available=m->log_tail!=m->log_head;
    if (available) { *log=m->logs[m->log_tail%16]; ++m->log_tail; }
    __set_PRIMASK(mask); return available;
}
