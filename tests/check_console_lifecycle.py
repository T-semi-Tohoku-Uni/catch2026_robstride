"""Execute real standalone stop/reinitialize/start helpers with ordered HAL stubs."""
from pathlib import Path
import subprocess
import sys

sys.dont_write_bytecode = True
from compile_host import compile_host
from check_motor_startup import definition


PREFIX = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "cybergear_console.h"
#include "cybergear_test_motion.h"
#include "cybergear_tuning.h"

#define CYBER_GEAR_ID CYBERGEAR_MOTOR_ID
#define HOST_ID CYBERGEAR_HOST_ID
#define FDCAN_TX_BUFFER0 1U
#define FDCAN_TX_BUFFER1 2U
#define FDCAN_TX_BUFFER2 4U
#define TIM_FLAG_UPDATE 1U
#define TIM6_DAC_IRQn 6U
typedef struct { uint32_t counter; bool flag_cleared; } TIM_HandleTypeDef;
static TIM_HandleTypeDef htim6;
static FDCAN_HandleTypeDef hfdcan3;
static FDCAN_TxHeaderTypeDef motor_txheader;
static CyberGearMotor cybergear_base;
static CyberGearTestMotion cybergear_test_motion;
static CyberGearConsole cybergear_console;
static CyberGearTuning cybergear_test_tuning;
static bool cybergear_test_initialized, cybergear_test_can_started;
static volatile bool cybergear_test_timer_active;
static uint8_t cybergear_test_control_phase;
static float target_angle[4];
static uint32_t now_ms, stop_started_ms, pending_until_ms;
static unsigned int init_calls, apply_calls, request_calls, control_calls, reset_calls;
static unsigned int can_calls, timer_stops, timer_starts, home_calls, base_calls, begin_calls;
static unsigned int motion_calls, startup_steps;
static bool valid_settings, allow_ack, stale_feedback, refuse_request, fail_stop_early;
static bool pending_forever, queue_empty, reset_succeeded, cancel_on_service;
static bool fail_home, startup_active, fail_startup, pending_irq_cleared;
static HAL_StatusTypeDef can_status, timer_stop_status, timer_start_status;

uint32_t HAL_GetTick(void) { return now_ms; }
void HAL_Delay(uint32_t duration) { assert(duration > 0U); now_ms += duration; }
HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *timer)
{
    assert(timer == &htim6 && !cybergear_test_timer_active);
    timer_stops++;
    return timer_stop_status;
}
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *timer)
{
    assert(timer == &htim6 && cybergear_test_timer_active);
    assert(timer->counter == 0U && timer->flag_cleared && pending_irq_cleared);
    assert(cybergear_test_control_phase == 0U && cybergear_base.first_cyclic);
    assert(cybergear_base.last_control_ms == now_ms);
    assert(home_calls == 1U && begin_calls == 1U && reset_succeeded);
    timer_starts++;
    return timer_start_status;
}
#define __HAL_TIM_SET_COUNTER(timer, value) ((timer)->counter = (value))
#define __HAL_TIM_CLEAR_FLAG(timer, flag) ((timer)->flag_cleared = ((flag) == TIM_FLAG_UPDATE))
void HAL_NVIC_ClearPendingIRQ(uint32_t interrupt)
{ assert(interrupt == TIM6_DAC_IRQn); pending_irq_cleared = true; }
HAL_StatusTypeDef motor_CAN_RxTxSettings_init(FDCAN_TxHeaderTypeDef *header)
{ assert(header == &motor_txheader); can_calls++; return can_status; }
uint32_t HAL_FDCAN_IsTxBufferMessagePending(const FDCAN_HandleTypeDef *handle, uint32_t buffers)
{
    assert(handle == &hfdcan3 && buffers == 7U);
    queue_empty = !pending_forever && now_ms - stop_started_ms >= pending_until_ms;
    return queue_empty ? 0U : 1U;
}
bool cybergear_tuning_valid(const CyberGearTuning *settings)
{ return settings != NULL && valid_settings; }
bool cybergear_init(CyberGearMotor *motor, FDCAN_HandleTypeDef *handle,
                    uint8_t motor_id, uint8_t host_id)
{
    assert(motor == &cybergear_base && handle == &hfdcan3);
    assert(motor_id == CYBER_GEAR_ID && host_id == HOST_ID);
    assert(!cybergear_test_timer_active);
    if (cybergear_test_initialized) assert(reset_succeeded && queue_empty);
    init_calls++;
    memset(motor, 0, sizeof(*motor));
    motor->hfdcan = handle;
    motor->config.controller.period_ms = 10U;
    motor->config.stop_timeout_ms = 60U;
    motor->config.command_retry_ms = 10U;
    return true;
}
bool cybergear_configure(CyberGearMotor *motor, const CyberGearConfig *config)
{
    assert(motor == &cybergear_base && reset_succeeded && !motor->managed);
    assert(queue_empty && !cybergear_test_timer_active);
    apply_calls++;
    motor->config = *config;
    return true;
}
bool cybergear_request_reinitialize_stop(CyberGearMotor *motor)
{
    assert(motor == &cybergear_base && !cybergear_test_timer_active);
    request_calls++;
    if (refuse_request) return false;
    motor->state = CG_STATE_STOPPING;
    motor->managed = true;
    motor->reset_confirmed = motor->stationary = false;
    stop_started_ms = now_ms;
    reset_succeeded = false;
    return true;
}
bool cybergear_control_position_adrc(CyberGearMotor *motor, float target)
{
    assert(motor == &cybergear_base && !cybergear_test_timer_active);
    (void)target;
    control_calls++;
    if (cancel_on_service) cybergear_console.stop_requested = true;
    if (startup_active) {
        startup_steps++;
        motor->fault = fail_startup ? CG_FAULT_START_TIMEOUT : CG_FAULT_NONE;
        motor->quiet_active = false;
        motor->feedback.velocity_rad_s = 0.0188f;
        motor->state = fail_startup ? CG_STATE_FAULT : CG_STATE_RUNNING;
        return !fail_startup;
    }
    if (allow_ack && now_ms - stop_started_ms >= 20U) {
        motor->state = CG_STATE_FAULT;
        motor->reset_confirmed = motor->stationary = true;
    } else if (fail_stop_early && now_ms - stop_started_ms >= 20U) {
        motor->state = CG_STATE_FAULT;
    }
    return false;
}
bool cybergear_reset_fault(CyberGearMotor *motor)
{
    assert(motor == &cybergear_base);
    reset_calls++;
    if (!motor->reset_confirmed || !motor->stationary || stale_feedback) return false;
    assert(queue_empty);
    motor->state = CG_STATE_OFF;
    motor->managed = false;
    reset_succeeded = true;
    return true;
}
bool cybergear_test_motion_init_settings(CyberGearTestMotion *motion, uint32_t timestamp,
    float amplitude, float small, uint32_t dwell, uint32_t timeout)
{
    assert(motion == &cybergear_test_motion && timestamp == now_ms);
    assert(amplitude > small && dwell < timeout && apply_calls > 0U);
    memset(motion, 0, sizeof(*motion));
    motion->target_rad = CYBERGEAR_DEG_TO_RAD(amplitude);
    motion_calls++;
    return true;
}
bool cybergear_base_init(void)
{ assert(reset_succeeded && !cybergear_test_timer_active); base_calls++; return true; }
bool cybergear_homing(void)
{ assert(reset_succeeded && apply_calls >= 2U); home_calls++; return !fail_home; }
bool cybergear_begin_position_control(CyberGearMotor *motor, CyberGearRunMode mode)
{
    assert(motor == &cybergear_base && mode == CYBERGEAR_RUN_MODE_CURRENT);
    assert(home_calls == 1U && !fail_home);
    begin_calls++;
    startup_active = true;
    motor->managed = true;
    motor->state = CG_STATE_WAIT_RUN;
    motor->startup_ms = now_ms - 3010U;
    motor->feedback.online = true;
    motor->feedback.mode = 2U;
    motor->feedback.last_received_ms = now_ms - 5U;
    motor->feedback.velocity_rad_s = 0.0380f;
    motor->feedback.position_rad = 0.0143f;
    motor->config.stationary_speed_rad_s = 0.03f;
    motor->config.stationary_dwell_ms = 100U;
    motor->quiet_active = true;
    motor->quiet_since_ms = now_ms - 50U;
    motor->mode_read_valid = true;
    motor->mode_read_value = (uint8_t)mode;
    motor->requested_mode = mode;
    return true;
}
bool cybergear_stop(CyberGearMotor *motor)
{ assert(motor == &cybergear_base); motor->state = CG_STATE_STOPPING; return true; }
void cybergear_service(CyberGearMotor *motor) { assert(motor == &cybergear_base); }
'''


SUFFIX = r'''
static void prepare(void)
{
    memset(&cybergear_base, 0, sizeof(cybergear_base));
    memset(&cybergear_test_motion, 0, sizeof(cybergear_test_motion));
    memset(&cybergear_console, 0, sizeof(cybergear_console));
    memset(&cybergear_test_tuning, 0, sizeof(cybergear_test_tuning));
    cybergear_base.hfdcan = &hfdcan3;
    cybergear_base.config.controller.period_ms = 10U;
    cybergear_base.config.stop_timeout_ms = 60U;
    cybergear_base.config.command_retry_ms = 10U;
    cybergear_base.state = CG_STATE_RUNNING;
    cybergear_base.managed = true;
    cybergear_test_tuning.motor = cybergear_base.config;
    cybergear_test_tuning.motor.controller.current_limit_a = 2.0f;
    cybergear_test_tuning.amplitude_deg = 40.0f;
    cybergear_test_tuning.small_amplitude_deg = 4.0f;
    cybergear_test_tuning.dwell_ms = 100U;
    cybergear_test_tuning.leg_timeout_ms = 20000U;
    cybergear_test_initialized = true;
    cybergear_test_can_started = false;
    cybergear_test_timer_active = true;
    cybergear_test_control_phase = 7U;
    htim6.counter = 731U;
    htim6.flag_cleared = pending_irq_cleared = false;
    target_angle[0] = -0.4f;
    now_ms = 1000U;
    stop_started_ms = pending_until_ms = 0U;
    init_calls = apply_calls = request_calls = control_calls = reset_calls = 0U;
    can_calls = timer_stops = timer_starts = home_calls = base_calls = begin_calls = 0U;
    motion_calls = startup_steps = 0U;
    valid_settings = allow_ack = true;
    stale_feedback = refuse_request = fail_stop_early = pending_forever = false;
    queue_empty = reset_succeeded = cancel_on_service = false;
    fail_home = startup_active = fail_startup = false;
    can_status = timer_stop_status = timer_start_status = HAL_OK;
}
static void check_no_apply(void)
{
    assert(init_calls == 0U && apply_calls == 0U && motion_calls == 0U);
    assert(home_calls == 0U && begin_calls == 0U && timer_starts == 0U);
    assert(!cybergear_test_timer_active);
    assert(cybergear_test_tuning.motor.controller.current_limit_a == 2.0f);
}
static void receive_text(const char *text)
{
    while (*text != '\0') cybergear_console_receive(&cybergear_console, (uint8_t)*text++);
}
static void flush_input_boundary_test(void)
{
    CyberGearConsoleCommand command;
    for (unsigned int scenario = 0U; scenario < 4U; ++scenario) {
        prepare();
        if (scenario < 2U) {
            receive_text("set current 1.");
            if (scenario == 1U)
                assert(cybergear_console_poll(&cybergear_console, &command) == CG_CONSOLE_NONE);
        } else if (scenario == 2U) {
            for (unsigned int byte = 0U; byte <= CG_CONSOLE_RX_CAPACITY; ++byte)
                cybergear_console_receive(&cybergear_console, 'a');
            assert(cybergear_console.dropping);
        } else {
            cybergear_console_receive_error(&cybergear_console);
        }
        cybergear_test_console_flush();
        assert(cybergear_console.dropping && cybergear_console.receiving_line);
        receive_text("s\n");
        assert(cybergear_console_poll(&cybergear_console, &command) == CG_CONSOLE_NONE);
        receive_text("s\n");
        assert(cybergear_console_poll(&cybergear_console, &command) == CG_CONSOLE_START);
    }
    prepare();
    receive_text("s\n");
    cybergear_test_console_flush();
    assert(cybergear_console_poll(&cybergear_console, &command) == CG_CONSOLE_NONE);
    receive_text("x");
    cybergear_test_console_flush();
    assert(cybergear_console.stop_requested);
    assert(cybergear_console_poll(&cybergear_console, &command) == CG_CONSOLE_STOP);
}
int main(void)
{
    flush_input_boundary_test();
    prepare();
    receive_text("x");
    assert(!cybergear_test_reinitialize(&cybergear_test_tuning));
    check_no_apply();
    assert(cybergear_console.stop_requested);

    prepare();
    valid_settings = false;
    assert(!cybergear_test_reinitialize(&cybergear_test_tuning));
    assert(can_calls == 0U && timer_stops == 0U && request_calls == 0U && init_calls == 0U);

    prepare();
    can_status = HAL_ERROR;
    assert(!cybergear_test_reinitialize(&cybergear_test_tuning));
    check_no_apply();
    assert(!cybergear_test_can_started && request_calls == 0U);

    prepare();
    refuse_request = true;
    assert(!cybergear_test_reinitialize(&cybergear_test_tuning));
    check_no_apply();

    for (unsigned int failure = 0U; failure < 5U; ++failure) {
        prepare();
        allow_ack = failure != 0U && failure != 1U;
        fail_stop_early = failure == 0U;
        stale_feedback = failure == 2U;
        pending_forever = failure == 3U;
        cancel_on_service = failure == 4U;
        assert(!cybergear_test_reinitialize(&cybergear_test_tuning));
        check_no_apply();
        assert(request_calls == 1U && control_calls > 0U);
        if (pending_forever) assert(reset_calls == 0U);
        if (cancel_on_service) assert(cybergear_console.stop_requested);
    }

    prepare();
    timer_stop_status = HAL_ERROR;
    assert(!cybergear_test_reinitialize(&cybergear_test_tuning));
    check_no_apply();

    prepare();
    pending_until_ms = 40U;
    CyberGearTuning candidate = cybergear_test_tuning;
    candidate.motor.controller.current_limit_a = 1.5f;
    assert(cybergear_test_reinitialize(&candidate));
    assert(now_ms - stop_started_ms >= pending_until_ms);
    assert(init_calls == 1U && apply_calls == 1U && motion_calls == 1U);
    assert(home_calls == 0U && begin_calls == 0U && timer_starts == 0U);
    assert(!cybergear_test_timer_active && cybergear_test_motion.halted);
    assert(cybergear_test_control_phase == 0U && target_angle[0] == 0.0f);
    assert(cybergear_test_tuning.motor.controller.current_limit_a == 1.5f);
    assert(can_calls == 1U && cybergear_test_can_ready() && can_calls == 1U);

    prepare();
    cybergear_test_initialized = false;
    assert(cybergear_test_reinitialize(&cybergear_test_tuning));
    assert(init_calls == 2U && apply_calls == 1U && home_calls == 0U);

    prepare();
    allow_ack = false;
    assert(!cybergear_test_start());
    check_no_apply();
    assert(base_calls == 0U);

    prepare();
    fail_home = true;
    assert(!cybergear_test_start());
    assert(home_calls == 1U && begin_calls == 0U && timer_starts == 0U);

    prepare();
    fail_startup = true;
    assert(!cybergear_test_start());
    assert(home_calls == 1U && begin_calls == 1U && timer_starts == 0U);

    prepare();
    assert(cybergear_test_start());
    assert(request_calls == 1U && init_calls == 1U && apply_calls == 2U);
    assert(base_calls == 1U && home_calls == 1U && startup_steps == 1U);
    assert(timer_starts == 1U && cybergear_test_timer_active && motion_calls == 2U);
    assert(target_angle[0] == cybergear_test_motion.target_rad);

    prepare();
    timer_start_status = HAL_ERROR;
    assert(!cybergear_test_start());
    assert(timer_starts == 1U && !cybergear_test_timer_active);
    puts("Console lifecycle: STOP confirmation, queue drain, config isolation and restart timing passed.");
    return 0;
}
'''


def main():
    compiler, repo_arg, output_arg = sys.argv[1:]
    repo, output = Path(repo_arg).resolve(), Path(output_arg).resolve()
    source = (repo / "Core/Src/main.c").read_text(encoding="utf-8")
    helpers = "\n\n".join(definition(source, signature) for signature in (
        "static void cybergear_test_console_flush(",
        "static bool cybergear_test_can_ready(",
        "static bool cybergear_test_reinitialize(",
        "static bool cybergear_test_start(",
    ))
    generated = output / "console_lifecycle_generated.c"
    generated.write_text(PREFIX + helpers + SUFFIX, encoding="utf-8")
    binary = output / "console_lifecycle.exe"
    compile_host(compiler, repo, generated, binary, ["APP_CYBERGEAR_STANDALONE_TEST=1"],
                 extra_sources=[repo / "Core/Src/cybergear_console.c"])
    result = subprocess.run([str(binary)], check=True, capture_output=True, text=True)
    assert "CG startup failed: phase=5 fault=4 elapsed=3010 ms" in result.stdout
    assert "CG startup feedback: mode=2 online=1 age=5 v=0.03800 q=0.01430" in result.stdout
    assert "CG startup stationary: limit=0.03000 quiet_ms=50 required_ms=100" in result.stdout
    assert "CG startup readback: pending=0 valid=1 value=3 requested=3" in result.stdout
    print(result.stdout, end="")


if __name__ == "__main__":
    main()
