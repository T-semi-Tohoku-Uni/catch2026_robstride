"""Run the real main startup, stop, and watchdog helpers without hardware."""
from pathlib import Path
import re
import subprocess
import sys

sys.dont_write_bytecode = True
from compile_host import compile_host


def definition(source, signature):
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 1
    for index in range(opening + 1, len(source)):
        depth += (source[index] == "{") - (source[index] == "}")
        if depth == 0:
            return source[start:index + 1]
    raise ValueError("Unbalanced function braces")


PREFIX = r'''
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "cybergear.h"
#include "robstride_startup.h"
#include "app_mode.h"

typedef struct { unsigned int unused; } TIM_HandleTypeDef;
static TIM_HandleTypeDef htim6;
static CyberGearMotor cybergear_base;
static RobstrideMotor robstride_handler[3];
static RobstrideStartup robstride_startup;
static bool motors_running, motor_return_active;
static uint32_t now_ms, scenario_started_ms, cg_ready_delay_ms, cg_fault_delay_ms;
static uint32_t cg_stop_started_ms, cg_stop_delay_ms;
static unsigned int begin_calls, control_calls, service_calls, stop_steps;
static unsigned int timer_starts, timer_stops, failure_calls, watchdog_checks;
static unsigned int rs_stop_calls[3], rs_keepalive_calls[3], rs_commands[3];
static unsigned int rs_stop_failures[3], fifo_used, fifo_rejections;
static uint32_t absent_axes, stop_tx_axes, stop_ack_axes;
static bool begin_allowed, stopping, cg_stop_requested, cg_stop_tx, cg_stop_ack;
static HAL_StatusTypeDef start_status;
static void motor_check_feedback(void);

static bool queue_frame(void)
{
    if (fifo_used == 3U) {
        fifo_rejections++;
        return false;
    }
    fifo_used++;
    return true;
}

static unsigned int axis_index(RobstrideMotor *motor)
{
    const unsigned int index = (unsigned int)(motor - robstride_handler);
    assert(index < 3U);
    assert(motor->motor_id == index + 3U);
    return index;
}

static void respond(RobstrideMotor *motor, uint8_t mode)
{
    const unsigned int index = axis_index(motor);
    if ((absent_axes & (1U << index)) != 0U) return;
    motor->feedback.online = true;
    motor->feedback.mode = mode;
    motor->feedback.fault_flags = 0U;
    motor->feedback.position_rad = 0.1f * (float)(index + 1U);
    motor->feedback.velocity_rps = 0.0f;
    motor->feedback.last_leceived_ms = now_ms;
    motor->feedback.received_count++;
}

uint32_t HAL_GetTick(void) { return now_ms; }

void HAL_Delay(uint32_t delay_ms)
{
    assert(delay_ms > 0U);
    now_ms += delay_ms;
    fifo_used = 0U;
    assert(now_ms - scenario_started_ms <= ROBSTRIDE_STARTUP_TIMEOUT_MS + 100U);
    if (!stopping) {
        watchdog_checks++;
        motor_check_feedback();
        assert(failure_calls == 0U);
    }
}

static HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *timer)
{
    assert(timer == &htim6);
    assert(motors_running);
    assert(cybergear_base.state == CG_STATE_RUNNING);
    assert(cybergear_base.first_cyclic);
    assert(robstride_startup_ready(&robstride_startup));
    assert(now_ms - scenario_started_ms >= cg_ready_delay_ms);
    timer_starts++;
    return start_status;
}

static HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *timer)
{
    assert(timer == &htim6);
    assert(!motors_running && !motor_return_active);
    timer_stops++;
    stopping = true;
    return HAL_OK;
}

static void motor_init_failed(const char *reason)
{
    assert(reason != NULL && reason[0] != '\0');
    failure_calls++;
}

bool robstride_stop(RobstrideMotor *motor)
{
    const unsigned int index = axis_index(motor);
    rs_commands[index]++;
    rs_stop_calls[index]++;
    if (stopping && (stop_tx_axes & (1U << index)) == 0U) return false;
    if (stopping && rs_stop_failures[index] != 0U) {
        rs_stop_failures[index]--;
        return false;
    }
    if (!queue_frame()) return false;
    if (!stopping || (stop_ack_axes & (1U << index)) != 0U) respond(motor, 0U);
    return true;
}

bool robstride_enable(RobstrideMotor *motor)
{
    rs_commands[axis_index(motor)]++;
    respond(motor, 2U);
    return true;
}

bool robstride_set_run_mode(RobstrideMotor *motor, RobstrideRunMode mode)
{
    assert(mode == POSITION_PP);
    rs_commands[axis_index(motor)]++;
    respond(motor, 0U);
    return true;
}

bool robstride_set_pp_velocity_max(RobstrideMotor *motor, float value)
{
    assert(value == ROBSTRIDE_STARTUP_VELOCITY_RAD_S);
    rs_commands[axis_index(motor)]++;
    respond(motor, 0U);
    return true;
}

bool robstride_set_pp_acceleration(RobstrideMotor *motor, float value)
{
    assert(value == ROBSTRIDE_STARTUP_ACCELERATION_RAD_S2);
    rs_commands[axis_index(motor)]++;
    respond(motor, 0U);
    return true;
}

bool robstride_set_current_limit(RobstrideMotor *motor, float value)
{
    const unsigned int index = axis_index(motor);
    assert(value == ROBSTRIDE_STARTUP_CURRENT_A);
    rs_commands[index]++;
    if (motor->feedback.mode == 2U) rs_keepalive_calls[index]++;
    respond(motor, motor->feedback.mode);
    return true;
}

bool robstride_set_position(RobstrideMotor *motor, float position)
{
    assert(position == motor->feedback.position_rad);
    rs_commands[axis_index(motor)]++;
    respond(motor, motor->feedback.mode);
    return true;
}

bool cybergear_begin_position_control(CyberGearMotor *motor, CyberGearRunMode mode)
{
    assert(motor == &cybergear_base);
    assert(mode == (CYBERGEAR_COMPARE_OPERATION_MODE ?
                   CYBERGEAR_RUN_MODE_OPERATION : CYBERGEAR_RUN_MODE_CURRENT));
    begin_calls++;
    assert(begin_calls == 1U);
    motor->managed = true;
    motor->state = CG_STATE_WAIT_RUN;
    return begin_allowed;
}

bool cybergear_stop(CyberGearMotor *motor)
{
    assert(motor == &cybergear_base);
    if (!cg_stop_requested) cg_stop_started_ms = now_ms;
    cg_stop_requested = true;
    if (motor->managed) {
        motor->state = CG_STATE_STOPPING;
        return true;
    }
    const bool queued = cg_stop_tx && queue_frame();
    if (queued && cg_stop_ack && now_ms - cg_stop_started_ms >= cg_stop_delay_ms) {
        motor->feedback.online = true;
        motor->feedback.mode = 0U;
        motor->feedback.velocity_rad_s = 0.0f;
        motor->feedback.rx_sequence++;
        motor->feedback.last_received_ms = now_ms;
    }
    return queued;
}

bool cybergear_control_position_adrc(CyberGearMotor *motor, float target)
{
    assert(motor == &cybergear_base);
    assert(target == (cg_stop_requested ? 0.0f : motor->requested_target_rad));
    control_calls++;
    if (cg_stop_requested) {
        assert(motor->managed);
        stop_steps++;
        if (cg_stop_tx && queue_frame()) motor->stop_queued = true;
        if (cg_stop_tx && cg_stop_ack && now_ms - cg_stop_started_ms >= cg_stop_delay_ms) {
            motor->reset_confirmed = true;
            motor->stationary = true;
            motor->feedback.online = true;
            motor->feedback.mode = 0U;
            motor->feedback.last_received_ms = now_ms;
        }
        return false;
    }
    assert(timer_starts == 0U);
    motor->first_cyclic = false;
    if (now_ms - scenario_started_ms >= cg_fault_delay_ms) {
        motor->state = CG_STATE_FAULT;
        return false;
    }
    if (now_ms - scenario_started_ms >= cg_ready_delay_ms) motor->state = CG_STATE_RUNNING;
    motor->feedback.online = true;
    motor->feedback.mode = 2U;
    motor->feedback.last_received_ms = now_ms;
    return true;
}

void cybergear_service(CyberGearMotor *motor)
{
    assert(motor == &cybergear_base);
    service_calls++;
}
'''

SUFFIX = r'''
static void reset_scenario(uint32_t period_ms, uint32_t start_ms)
{
    memset(&cybergear_base, 0, sizeof(cybergear_base));
    memset(robstride_handler, 0, sizeof(robstride_handler));
    memset(&robstride_startup, 0, sizeof(robstride_startup));
    memset(rs_stop_calls, 0, sizeof(rs_stop_calls));
    memset(rs_keepalive_calls, 0, sizeof(rs_keepalive_calls));
    memset(rs_commands, 0, sizeof(rs_commands));
    memset(rs_stop_failures, 0, sizeof(rs_stop_failures));
    fifo_used = fifo_rejections = 0U;
    now_ms = start_ms;
    scenario_started_ms = start_ms;
    cg_ready_delay_ms = 900U;
    cg_fault_delay_ms = UINT32_MAX;
    cg_stop_delay_ms = 30U;
    begin_calls = control_calls = service_calls = stop_steps = 0U;
    timer_starts = timer_stops = failure_calls = watchdog_checks = 0U;
    absent_axes = 0U;
    stop_tx_axes = stop_ack_axes = 7U;
    motors_running = motor_return_active = stopping = cg_stop_requested = false;
    begin_allowed = cg_stop_tx = cg_stop_ack = true;
    start_status = HAL_OK;
    cybergear_base.config.controller.period_ms = period_ms;
    cybergear_base.config.controller.feedback_timeout_ms = 100U;
    cybergear_base.requested_target_rad = 0.7f;
    for (unsigned int index = 0U; index < 3U; ++index)
        robstride_handler[index].motor_id = (uint8_t)(index + 3U);
}

static void startup_scenarios(uint32_t period_ms)
{
    reset_scenario(period_ms, 1000U);
    assert(motor_start_control());
    assert(motors_running && timer_starts == 1U && begin_calls == 1U);
    assert(now_ms - scenario_started_ms == cg_ready_delay_ms);
    assert(service_calls > 50U && watchdog_checks > 50U);
    for (unsigned int index = 0U; index < 3U; ++index) assert(rs_keepalive_calls[index] >= 5U);

    reset_scenario(period_ms, 1000U);
    cg_ready_delay_ms = 100U;
    assert(motor_start_control());
    assert(motors_running && timer_starts == 1U && begin_calls == 1U);
    assert(now_ms - scenario_started_ms > cg_ready_delay_ms);
    assert(cybergear_base.first_cyclic);

    reset_scenario(period_ms, UINT32_MAX - 400U);
    assert(motor_start_control());
    assert(timer_starts == 1U && now_ms - scenario_started_ms == 900U);

    for (unsigned int index = 0U; index < 3U; ++index) {
        reset_scenario(period_ms, 1000U);
        absent_axes = 1U << index;
        assert(!motor_start_control());
        assert(!motors_running && timer_starts == 0U && begin_calls == 1U);
        assert(now_ms - scenario_started_ms == ROBSTRIDE_STARTUP_TIMEOUT_MS);
        assert(failure_calls == 0U && watchdog_checks > 100U);
        assert(rs_stop_calls[index] > 10U);
    }

    reset_scenario(period_ms, 1000U);
    cg_fault_delay_ms = 650U;
    assert(!motor_start_control());
    assert(!motors_running && timer_starts == 0U && now_ms - scenario_started_ms == 650U);

    reset_scenario(period_ms, 1000U);
    begin_allowed = false;
    assert(!motor_start_control());
    assert(!motors_running && timer_starts == 0U && control_calls == 0U);

    reset_scenario(period_ms, 1000U);
    start_status = HAL_ERROR;
    assert(!motor_start_control());
    assert(!motors_running && timer_starts == 1U);
}

static void stop_scenarios(uint32_t period_ms)
{
    reset_scenario(period_ms, 1000U);
    cybergear_base.managed = true;
    motors_running = motor_return_active = true;
    assert(motor_stop_all());
    assert(!motors_running && !motor_return_active && timer_stops == 1U);
    assert(stop_steps >= 4U && now_ms - scenario_started_ms == cg_stop_delay_ms);
    assert(fifo_rejections == 0U);
    for (unsigned int index = 0U; index < 3U; ++index) assert(rs_stop_calls[index] == 1U);

    reset_scenario(period_ms, 1000U);
    assert(motor_stop_all());
    assert(stop_steps == 0U && now_ms - scenario_started_ms < 120U);

    reset_scenario(period_ms, 1000U);
    cybergear_base.managed = true;
    rs_stop_failures[2] = 1U;
    assert(motor_stop_all());
    assert(rs_stop_calls[2] >= 2U && now_ms - scenario_started_ms < 200U);
    assert(fifo_rejections == 0U);

    for (unsigned int index = 0U; index < 3U; ++index) {
        reset_scenario(period_ms, UINT32_MAX - 400U);
        cybergear_base.managed = true;
        stop_ack_axes &= ~(1U << index);
        assert(!motor_stop_all());
        assert(now_ms - scenario_started_ms == MOTOR_STOP_TIMEOUT_MS);
        assert(stop_steps == MOTOR_STOP_TIMEOUT_MS / period_ms);
        for (unsigned int axis = 0U; axis < 3U; ++axis) assert(rs_stop_calls[axis] >= 10U);
    }

    reset_scenario(period_ms, 1000U);
    cybergear_base.managed = true;
    cg_stop_ack = false;
    assert(!motor_stop_all());
    assert(now_ms - scenario_started_ms == MOTOR_STOP_TIMEOUT_MS && stop_steps > 0U);

    reset_scenario(period_ms, 1000U);
    cybergear_base.managed = true;
    cg_stop_tx = false;
    stop_tx_axes = 0U;
    assert(!motor_stop_all());
    assert(now_ms - scenario_started_ms == MOTOR_STOP_TIMEOUT_MS);
    for (unsigned int index = 0U; index < 3U; ++index) assert(rs_stop_calls[index] >= 10U);
}

static void healthy_running(uint32_t period_ms)
{
    reset_scenario(period_ms, 1000U);
    motors_running = true;
    cybergear_base.state = CG_STATE_RUNNING;
    cybergear_base.feedback.online = true;
    cybergear_base.feedback.last_received_ms = now_ms;
    for (unsigned int index = 0U; index < 3U; ++index) respond(&robstride_handler[index], 2U);
    motor_check_feedback();
    assert(failure_calls == 0U);
}

static void watchdog_scenarios(uint32_t period_ms)
{
    reset_scenario(period_ms, 50000U);
    motor_check_feedback();
    assert(failure_calls == 0U);
    for (unsigned int index = 0U; index < 3U; ++index) {
        healthy_running(period_ms);
        robstride_handler[index].feedback.last_leceived_ms = now_ms - ROBSTRIDE_STARTUP_FEEDBACK_MS;
        motor_check_feedback();
        assert(failure_calls == 1U);

        healthy_running(period_ms);
        robstride_handler[index].feedback.mode = 0U;
        motor_check_feedback();
        assert(failure_calls == 1U);

        healthy_running(period_ms);
        robstride_handler[index].feedback.fault_flags = 1U;
        motor_check_feedback();
        assert(failure_calls == 1U);
    }
    healthy_running(period_ms);
    cybergear_base.state = CG_STATE_FAULT;
    motor_check_feedback();
    assert(failure_calls == 1U);

    healthy_running(period_ms);
    cybergear_base.feedback.fault_flags = 1U;
    motor_check_feedback();
    assert(failure_calls == 1U);

    healthy_running(period_ms);
    cybergear_base.feedback.last_received_ms = now_ms - cybergear_base.config.controller.feedback_timeout_ms;
    motor_check_feedback();
    assert(failure_calls == 1U);
}

int main(void)
{
    for (uint32_t period_ms = 5U; period_ms <= 10U; period_ms += 5U) {
        startup_scenarios(period_ms);
        stop_scenarios(period_ms);
        watchdog_scenarios(period_ms);
    }
    puts("Real motor helpers: startup coordination, stop confirmation, and per-axis watchdog passed.");
    return 0;
}
'''


def main():
    compiler, repo_arg, output_arg = sys.argv[1:]
    repo, output = Path(repo_arg).resolve(), Path(output_arg).resolve()
    source = (repo / "Core/Src/main.c").read_text(encoding="utf-8")
    constants = "\n".join(re.findall(
        r"^#define (?:MOTOR_STOP_TIMEOUT_MS|CYBERGEAR_HOMING_FEEDBACK_TIMEOUT_MS)\b[^\n]*",
        source, re.MULTILINE))
    functions = "\n\n".join(definition(source, signature) for signature in (
        "static void motor_check_feedback(", "static bool motor_stop_all(",
        "static bool motor_start_control("))
    startup = (repo / "Core/Src/robstride_startup.c").read_text(encoding="utf-8")
    generated = output / "motor_startup_test_generated.c"
    generated.write_text(PREFIX + constants + "\n" + startup + "\n" + functions + SUFFIX,
                         encoding="utf-8")
    binary = output / "motor_startup_test.exe"
    compile_host(compiler, repo, generated, binary,
                 ["__MAIN_H", "APP_CYBERGEAR_STANDALONE_TEST=0"])
    subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
