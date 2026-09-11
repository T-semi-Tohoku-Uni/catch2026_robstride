#include "robstride_startup.h"

#include <math.h>
#include <string.h>

static bool feedback_healthy(const RobstrideFeedback *feedback, uint32_t now_ms)
{
    return feedback->online && feedback->fault_flags == 0U &&
        isfinite(feedback->position_rad) &&
        now_ms - feedback->last_leceived_ms <= ROBSTRIDE_STARTUP_FEEDBACK_MS;
}

static bool startup_valid(const RobstrideStartup *startup)
{
    return startup != NULL && startup->motors != NULL &&
        startup->motor_count > 0U && startup->motor_count <= ROBSTRIDE_STARTUP_MAX_MOTORS;
}

bool robstride_startup_init(RobstrideStartup *startup, RobstrideMotor *motors,
    uint32_t motor_count, const float *current_limits_a)
{
    if (startup == NULL) return false;
    memset(startup, 0, sizeof(*startup));
    if (motors == NULL || current_limits_a == NULL || motor_count == 0U ||
        motor_count > ROBSTRIDE_STARTUP_MAX_MOTORS) {
        startup->failed = true;
        return false;
    }
    for (uint32_t index = 0U; index < motor_count; ++index) {
        if (!isfinite(current_limits_a[index]) || current_limits_a[index] < 0.0f) {
            startup->failed = true;
            return false;
        }
        startup->axes[index].current_limit_a = current_limits_a[index];
    }
    startup->motors = motors;
    startup->motor_count = motor_count;
    startup->started_ms = HAL_GetTick();
    startup->last_poll_ms = startup->started_ms - ROBSTRIDE_STARTUP_POLL_MS;
    return true;
}

bool robstride_startup_failed(RobstrideStartup *startup)
{
    if (!startup_valid(startup)) return true;
    if (HAL_GetTick() - startup->started_ms >= ROBSTRIDE_STARTUP_TIMEOUT_MS) startup->failed = true;
    return startup->failed;
}

bool robstride_startup_ready(RobstrideStartup *startup)
{
    if (!startup_valid(startup)) return false;
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    const uint32_t now_ms = HAL_GetTick();
    bool ready = !startup->failed && now_ms - startup->started_ms < ROBSTRIDE_STARTUP_TIMEOUT_MS;
    for (uint32_t index = 0U; index < startup->motor_count; ++index) {
        const RobstrideFeedback feedback = startup->motors[index].feedback;
        if (startup->axes[index].stage != RS_STARTUP_READY ||
            !feedback_healthy(&feedback, now_ms) || feedback.mode != 2U) ready = false;
    }
    __set_PRIMASK(interrupt_mask);
    return ready;
}

void robstride_startup_update(RobstrideStartup *startup)
{
    if (!startup_valid(startup) || robstride_startup_failed(startup)) return;
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    const uint32_t now_ms = HAL_GetTick();
    if (now_ms - startup->last_poll_ms < ROBSTRIDE_STARTUP_POLL_MS) {
        __set_PRIMASK(interrupt_mask);
        return;
    }
    startup->last_poll_ms = now_ms;

    for (uint32_t index = 0U; index < startup->motor_count; ++index) {
        RobstrideStartupAxis *axis = &startup->axes[index];
        const RobstrideFeedback feedback = startup->motors[index].feedback;
        const bool healthy = feedback_healthy(&feedback, now_ms);
        if ((axis->stage == RS_STARTUP_READY && (!healthy || feedback.mode != 2U)) ||
            (axis->stage >= RS_STARTUP_SET_MODE && axis->stage < RS_STARTUP_READY &&
             (!healthy || (axis->stage <= RS_STARTUP_SEND_ENABLE && feedback.mode != 0U)))) {
            axis->stage = RS_STARTUP_SEND_STOP;
            axis->waiting_for_write = false;
        }
        if (axis->stage == RS_STARTUP_WAIT_RUNNING && healthy && feedback.mode == 2U &&
            feedback.received_count != axis->baseline_count) axis->stage = RS_STARTUP_READY;
    }

    RobstrideStartupAxis *axis = &startup->axes[startup->next_axis];
    RobstrideMotor *motor = &startup->motors[startup->next_axis];
    const RobstrideFeedback feedback = motor->feedback;
    bool write_queued = false;
    if (axis->waiting_for_write) {
        if (feedback.received_count != axis->baseline_count) {
            axis->waiting_for_write = false;
            if (axis->stage == RS_STARTUP_SET_MODE) motor->run_mode = POSITION_PP;
            axis->stage = (RobstrideStartupStage)(axis->stage + 1);
        } else if (now_ms - axis->last_probe_ms >= ROBSTRIDE_STARTUP_PROBE_MS) {
            axis->waiting_for_write = false;
        }
    } else {
        switch (axis->stage) {
        case RS_STARTUP_SEND_STOP:
            axis->baseline_count = feedback.received_count;
            if (robstride_stop(motor)) {
                axis->last_probe_ms = now_ms;
                axis->stage = RS_STARTUP_WAIT_STOPPED;
            }
            break;
        case RS_STARTUP_WAIT_STOPPED:
            if (feedback_healthy(&feedback, now_ms) && feedback.mode == 0U &&
                feedback.received_count != axis->baseline_count) {
                axis->hold_position_rad = feedback.position_rad;
                axis->stage = RS_STARTUP_SET_MODE;
            } else if (now_ms - axis->last_probe_ms >= ROBSTRIDE_STARTUP_PROBE_MS) {
                robstride_stop(motor);
                axis->last_probe_ms = now_ms;
            }
            break;
        case RS_STARTUP_SET_MODE:
            write_queued = robstride_set_run_mode(motor, POSITION_PP);
            break;
        case RS_STARTUP_SET_VELOCITY:
            write_queued = robstride_set_pp_velocity_max(motor, ROBSTRIDE_STARTUP_VELOCITY_RAD_S);
            break;
        case RS_STARTUP_SET_ACCELERATION:
            write_queued = robstride_set_pp_acceleration(motor, ROBSTRIDE_STARTUP_ACCELERATION_RAD_S2);
            break;
        case RS_STARTUP_SET_CURRENT:
            write_queued = robstride_set_current_limit(motor, axis->current_limit_a);
            break;
        case RS_STARTUP_SET_HOLD:
            axis->hold_position_rad = feedback.position_rad;
            write_queued = robstride_set_position(motor, axis->hold_position_rad);
            break;
        case RS_STARTUP_SEND_ENABLE:
            axis->baseline_count = feedback.received_count;
            if (robstride_enable(motor)) {
                axis->enabled_ms = now_ms;
                axis->last_probe_ms = now_ms;
                axis->stage = RS_STARTUP_WAIT_RUNNING;
            }
            break;
        case RS_STARTUP_WAIT_RUNNING:
            if (now_ms - axis->enabled_ms >= ROBSTRIDE_STARTUP_RETRY_MS) {
                axis->stage = RS_STARTUP_SEND_STOP;
            } else if (now_ms - axis->last_probe_ms >= ROBSTRIDE_STARTUP_PROBE_MS) {
                robstride_set_current_limit(motor, axis->current_limit_a);
                axis->last_probe_ms = now_ms;
            }
            break;
        case RS_STARTUP_READY:
            if (now_ms - axis->last_probe_ms >= ROBSTRIDE_STARTUP_PROBE_MS) {
                robstride_set_current_limit(motor, axis->current_limit_a);
                axis->last_probe_ms = now_ms;
            }
            break;
        }
    }
    if (write_queued) {
        axis->baseline_count = feedback.received_count;
        axis->last_probe_ms = now_ms;
        axis->waiting_for_write = true;
    }
    startup->next_axis = (startup->next_axis + 1U) % startup->motor_count;
    __set_PRIMASK(interrupt_mask);
}
