"""Exercise real console/status output across faults, silence, queries and recovery."""
from pathlib import Path
import subprocess
import sys

sys.dont_write_bytecode = True
from compile_host import compile_host
from check_motor_startup import definition


PREFIX = r'''
#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "cybergear_console.h"
#include "cybergear_test_motion.h"
#include "cybergear_tuning.h"

static CyberGearMotor cybergear_base;
static CyberGearConsole cybergear_console;
static CyberGearTestMotion cybergear_test_motion;
static CyberGearTuning cybergear_test_tuning, cybergear_test_candidate;
static bool cybergear_test_initialized, cybergear_test_timer_active;
static float target_angle[4];
static uint32_t ticks, iteration, status_times[16];
static unsigned int scenario, output_calls, status_calls, setting_calls, service_calls;
static unsigned int quiet_output_calls;
static jmp_buf finished;

static int capture_printf(const char *format, ...)
{
    ++output_calls;
    if (strstr(format, " state=%u fault=%u online=%u age=%lu") != NULL) {
        assert(status_calls < 16U);
        status_times[status_calls++] = iteration;
    }
    if (strncmp(format, "CG SET ", 7U) == 0) ++setting_calls;
    return 0;
}
#define printf capture_printf
#define Board_LED_GPIO_Port 0U
#define Board_LED_Pin 0U
#define limit_GPIO_Port 0U
#define limit_Pin 0U
static void HAL_GPIO_TogglePin(unsigned int port, unsigned int pin)
{ (void)port; (void)pin; }
static unsigned int HAL_GPIO_ReadPin(unsigned int port, unsigned int pin)
{ (void)port; (void)pin; return 0U; }

uint32_t HAL_GetTick(void) { return ticks; }
void HAL_Delay(uint32_t duration)
{
    assert(duration == CG_TEST_LOOP_DELAY_MS);
    if (iteration == 10U || iteration == 610U) quiet_output_calls = output_calls;
    if ((iteration > 10U && iteration < 600U) ||
        (iteration > 610U && iteration < 900U))
        assert(output_calls == quiet_output_calls);
    ticks += duration;
    if (++iteration == 1600U) longjmp(finished, 1);
}
static void cybergear_test_console_init(void) {}
static void cybergear_test_console_flush(void) {}
bool cybergear_tuning_defaults(CyberGearTuning *tuning)
{ memset(tuning, 0, sizeof(*tuning)); return true; }
size_t cybergear_tuning_count(void) { return 3U; }
const char *cybergear_tuning_key(size_t index) { (void)index; return "current"; }
const char *cybergear_tuning_help(size_t index) { (void)index; return "A"; }
double cybergear_tuning_value(const CyberGearTuning *tuning, size_t index)
{ (void)tuning; (void)index; return 1.0; }
bool cybergear_tuning_set(CyberGearTuning *tuning, const char *key, const char *value)
{ (void)tuning; (void)key; (void)value; return true; }

static bool cybergear_test_start(void)
{
    cybergear_test_initialized = true;
    cybergear_base.managed = true;
    cybergear_base.fault = scenario == 0U ? CG_FAULT_START_TIMEOUT : CG_FAULT_NONE;
    cybergear_base.state = scenario == 0U ? CG_STATE_STOPPING : CG_STATE_RUNNING;
    cybergear_base.feedback.online = true;
    cybergear_base.config.controller.feedback_timeout_ms = 100U;
    cybergear_test_motion.halted = false;
    return scenario != 0U;
}
static bool cybergear_test_reinitialize(const CyberGearTuning *settings)
{
    (void)settings;
    cybergear_test_initialized = true;
    cybergear_base.managed = scenario == 1U;
    cybergear_base.fault = scenario == 1U ? CG_FAULT_START_TIMEOUT : CG_FAULT_NONE;
    cybergear_base.state = scenario == 1U ? CG_STATE_FAULT : CG_STATE_OFF;
    return scenario != 1U;
}
bool cybergear_stop(CyberGearMotor *motor)
{
    if (motor->fault == CG_FAULT_NONE) motor->fault = CG_FAULT_REQUESTED_STOP;
    motor->state = CG_STATE_STOPPING;
    return true;
}
bool cybergear_request_reinitialize_stop(CyberGearMotor *motor)
{ return cybergear_stop(motor); }
bool cybergear_control_position_adrc(CyberGearMotor *motor, float target)
{
    (void)target;
    if (iteration > 3U && motor->state == CG_STATE_STOPPING) motor->state = CG_STATE_FAULT;
    return motor->fault == CG_FAULT_NONE;
}
void cybergear_service(CyberGearMotor *motor)
{
    assert(motor == &cybergear_base);
    ++service_calls;
    if (scenario == 7U && iteration == 2U) {
        motor->fault = CG_FAULT_FEEDBACK;
        motor->state = CG_STATE_STOPPING;
    }
}
CyberGearTestResult cybergear_test_motion_update(CyberGearTestMotion *motion,
    uint32_t now, bool running, bool fresh, bool trajectory_done,
    float position, float velocity)
{
    (void)now; (void)fresh; (void)trajectory_done; (void)position; (void)velocity;
    motion->halted = !running;
    return running ? CG_TEST_WAIT : CG_TEST_STOP_FAULT;
}
CyberGearConsoleAction cybergear_console_poll(CyberGearConsole *console,
    CyberGearConsoleCommand *command)
{
    (void)console; (void)command;
    cybergear_base.feedback.last_received_ms = ticks;
    if (iteration == 0U) return scenario == 1U ? CG_CONSOLE_REINIT : CG_CONSOLE_START;
    if (iteration == 1U && scenario >= 3U) return CG_CONSOLE_SHOW;
    if (iteration == 2U && scenario >= 2U && scenario != 7U) {
        if (scenario == 2U) return CG_CONSOLE_STOP;
        cybergear_base.fault = CG_FAULT_FEEDBACK;
        cybergear_base.state = CG_STATE_STOPPING;
    }
    if (iteration == 600U && scenario >= 4U)
        return scenario == 4U ? CG_CONSOLE_SHOW : CG_CONSOLE_HELP;
    if (iteration == 900U && scenario == 6U) return CG_CONSOLE_REINIT;
    if (iteration == 901U && scenario == 6U) return CG_CONSOLE_START;
    return CG_CONSOLE_NONE;
}
'''

SUFFIX = r'''
#undef printf
int main(void)
{
    for (scenario = 0U; scenario < 8U; ++scenario) {
        memset(&cybergear_base, 0, sizeof(cybergear_base));
        memset(&cybergear_test_motion, 0, sizeof(cybergear_test_motion));
        cybergear_test_initialized = cybergear_test_timer_active = false;
        ticks += 10000U;
        iteration = output_calls = status_calls = setting_calls = service_calls = 0U;
        if (setjmp(finished) == 0) cybergear_standalone_test_run();
        assert(service_calls == 1600U);
        if (scenario < 2U) {
            assert(status_calls == 1U && status_times[0] == 0U);
        } else {
            assert(status_times[0] == 0U && status_times[1] == 2U);
            if (scenario < 4U) assert(status_calls == 2U);
            else {
                assert(status_times[2] == 600U);
                if (scenario != 6U) assert(status_calls == 3U);
                else {
                    assert(status_calls >= 4U);
                    assert(status_times[3] >= 900U);
                    assert(cybergear_base.state == CG_STATE_RUNNING);
                }
            }
        }
        assert(setting_calls == (scenario < 3U ? 0U : scenario < 4U ? 1U : 4U));
    }
    puts("Console fault logging: immediate single report, silence, query, recovery and continued stop service passed.");
    return 0;
}
'''


def main():
    compiler, repo_arg, output_arg = sys.argv[1:]
    repo, output = Path(repo_arg).resolve(), Path(output_arg).resolve()
    source = (repo / "Core/Src/main.c").read_text(encoding="utf-8")
    helpers = "\n\n".join(definition(source, signature) for signature in (
        "static bool cybergear_test_trajectory_done(",
        "static void cybergear_test_status(",
        "static void cybergear_standalone_test_run(",
    ))
    generated = output / "console_fault_logging_generated.c"
    generated.write_text(PREFIX + helpers + SUFFIX, encoding="utf-8")
    binary = output / "console_fault_logging.exe"
    compile_host(compiler, repo, generated, binary, ["APP_CYBERGEAR_STANDALONE_TEST=1"])
    subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
