"""Exercise real standalone boot, UART IRQ, parser and command dispatch on host."""
from pathlib import Path
import subprocess
import sys

sys.dont_write_bytecode = True
from compile_host import compile_host
from check_scheduler import callback


PREFIX = r'''
#include <assert.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "app_mode.h"
#include "cybergear_console.h"
#include "cybergear_test_motion.h"
#include "cybergear_tuning.h"
#include "stm32g4xx_hal.h"

typedef struct { uint32_t ISR, RDR; } MockUartRegisters;
typedef struct { MockUartRegisters *Instance; } UART_HandleTypeDef;
typedef struct { uint8_t byte; uint32_t flags; } ReceiveEvent;
static MockUartRegisters uart_registers;
static UART_HandleTypeDef huart2 = {&uart_registers};
static CyberGearMotor cybergear_base;
static CyberGearConsole cybergear_console;
static CyberGearTestMotion cybergear_test_motion;
static CyberGearTuning cybergear_test_tuning, cybergear_test_candidate;
static bool cybergear_test_initialized;
static volatile bool cybergear_test_timer_active;
static float target_angle[4];
static unsigned int can1_calls, can3_calls, errors, start_calls, reinit_calls;
static unsigned int flush_calls, clear_calls, delay_calls, defaults_calls, display_calls;
static uint32_t enabled_interrupts, ticks;
static bool irq_enabled;
static const ReceiveEvent *receive_events;
static size_t receive_event_count, receive_event_index;
static jmp_buf waiting;

#define USART2_IRQn 2U
#define UART_RXDATA_FLUSH_REQUEST 1U
#define UART_FLAG_RXNE 1U
#define UART_FLAG_PE 2U
#define UART_FLAG_FE 4U
#define UART_FLAG_NE 8U
#define UART_FLAG_ORE 16U
#define UART_CLEAR_PEF UART_FLAG_PE
#define UART_CLEAR_FEF UART_FLAG_FE
#define UART_CLEAR_NEF UART_FLAG_NE
#define UART_CLEAR_OREF UART_FLAG_ORE
#define UART_IT_RXNE 1U
#define UART_IT_PE 2U
#define UART_IT_ERR 4U
#define READ_REG(reg) (reg)
#define __HAL_UART_SEND_REQ(handle, request) flush_uart(handle, request)
#define __HAL_UART_CLEAR_FLAG(handle, flags) clear_uart_flags(handle, flags)
#define __HAL_UART_ENABLE_IT(handle, interrupt) enable_uart_interrupt(handle, interrupt)

static void flush_uart(UART_HandleTypeDef *handle, uint32_t request)
{
    assert(handle == &huart2 && request == UART_RXDATA_FLUSH_REQUEST);
    uart_registers.ISR = uart_registers.RDR = 0U;
    ++flush_calls;
}
static void clear_uart_flags(UART_HandleTypeDef *handle, uint32_t flags)
{
    assert(handle == &huart2);
    assert(flags == (UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF));
    uart_registers.ISR &= ~flags;
    ++clear_calls;
}
static void enable_uart_interrupt(UART_HandleTypeDef *handle, uint32_t interrupt)
{
    assert(handle == &huart2 && irq_enabled);
    enabled_interrupts |= interrupt;
}
void HAL_NVIC_SetPriority(uint32_t irq, uint32_t priority, uint32_t subpriority)
{
    assert(irq == USART2_IRQn && priority == 2U && subpriority == 0U);
}
void HAL_NVIC_ClearPendingIRQ(uint32_t irq) { assert(irq == USART2_IRQn); }
void HAL_NVIC_EnableIRQ(uint32_t irq)
{
    assert(irq == USART2_IRQn);
    irq_enabled = true;
}
void USART2_IRQHandler(void);
uint32_t HAL_GetTick(void) { return ticks; }
void HAL_Delay(uint32_t delay_ms)
{
    assert(delay_ms == CG_TEST_LOOP_DELAY_MS);
    assert(irq_enabled && enabled_interrupts == (UART_IT_RXNE | UART_IT_PE | UART_IT_ERR));
    assert(flush_calls == 1U && clear_calls >= 1U);
    assert(start_calls == 0U && errors == 0U);
    ticks += delay_ms;
    if (++delay_calls > 1024U) longjmp(waiting, 1);
    if (receive_event_index < receive_event_count) {
        const ReceiveEvent event = receive_events[receive_event_index++];
        uart_registers.ISR = event.flags;
        uart_registers.RDR = event.byte;
        USART2_IRQHandler();
    }
}
void HAL_Init(void) {}
void SystemClock_Config(void) {}
void MX_GPIO_Init(void) {}
void MX_USART2_UART_Init(void) {}
void MX_TIM6_Init(void) {}
void MX_FDCAN1_Init(void) { ++can1_calls; }
void MX_FDCAN3_Init(void) { ++can3_calls; }
void Error_Handler(void) { ++errors; }

bool cybergear_tuning_defaults(CyberGearTuning *tuning)
{
    assert(tuning == &cybergear_test_tuning);
    memset(tuning, 0, sizeof(*tuning));
    ++defaults_calls;
    return true;
}
bool cybergear_tuning_set(CyberGearTuning *candidate, const char *key, const char *value)
{
    assert(candidate == &cybergear_test_candidate && key != NULL && value != NULL);
    if (strcmp(key, "current") != 0 || strcmp(value, "1.5") != 0) return false;
    candidate->motor.controller.current_limit_a = 1.5f;
    return true;
}
size_t cybergear_tuning_count(void) { return 2U; }
const char *cybergear_tuning_key(size_t index)
{
    assert(index < cybergear_tuning_count());
    ++display_calls;
    return "parameter";
}
const char *cybergear_tuning_help(size_t index)
{
    assert(index < cybergear_tuning_count());
    return "description";
}
double cybergear_tuning_value(const CyberGearTuning *tuning, size_t index)
{
    assert(tuning == &cybergear_test_tuning && index < cybergear_tuning_count());
    return 1.0;
}
static bool cybergear_test_start(void)
{
    assert(receive_event_index > 0U);
    const ReceiveEvent last = receive_events[receive_event_index - 1U];
    assert(last.flags == UART_FLAG_RXNE && (last.byte == '\r' || last.byte == '\n'));
    ++start_calls;
    longjmp(waiting, 2);
}
static bool cybergear_test_reinitialize(const CyberGearTuning *settings)
{
    assert(settings == &cybergear_test_candidate);
    ++reinit_calls;
    if (receive_event_index < receive_event_count) return true;
    longjmp(waiting, 3);
}
bool cybergear_stop(CyberGearMotor *motor)
{
    (void)motor;
    assert(!"No motor STOP should be sent before initialization");
    return false;
}
bool cybergear_request_reinitialize_stop(CyberGearMotor *motor)
{
    (void)motor;
    assert(!"No driver reinitialization should occur before dispatch");
    return false;
}
bool cybergear_control_position_adrc(CyberGearMotor *motor, float target)
{
    (void)motor;
    (void)target;
    assert(!"No motor control should run before initialization");
    return false;
}
void cybergear_service(CyberGearMotor *motor)
{
    (void)motor;
    assert(!"No driver service should run before initialization");
}
static bool cybergear_test_trajectory_done(void)
{
    assert(!"Trajectory evaluation should not run before start");
    return false;
}
static void cybergear_test_status(bool force)
{
    (void)force;
    assert(!"Motor status should not run before initialization");
}
CyberGearTestResult cybergear_test_motion_update(CyberGearTestMotion *test,
    uint32_t now_ms, bool running, bool fresh, bool trajectory_done,
    float position_rad, float estimated_velocity_rad_s)
{
    (void)test;
    (void)now_ms;
    (void)running;
    (void)fresh;
    (void)trajectory_done;
    (void)position_rad;
    (void)estimated_velocity_rad_s;
    assert(!"Motion should not be evaluated before start");
    return CG_TEST_WAIT;
}
'''

STARTUP_PREFIX = r'''
int firmware_startup(void)
{
'''

SUFFIX = r'''
    return 0;
}
static void check_events(const ReceiveEvent *events, size_t event_count,
                         int expected_dispatch, bool expect_display,
                         unsigned int expected_reinitializations)
{
    can1_calls = can3_calls = errors = start_calls = reinit_calls = 0U;
    flush_calls = clear_calls = delay_calls = defaults_calls = display_calls = 0U;
    ticks = enabled_interrupts = 0U;
    irq_enabled = cybergear_test_initialized = cybergear_test_timer_active = false;
    memset(&cybergear_base, 0, sizeof(cybergear_base));
    memset(&cybergear_console, 0, sizeof(cybergear_console));
    memset(&cybergear_test_motion, 0, sizeof(cybergear_test_motion));
    receive_event_index = 0U;
    receive_events = events;
    receive_event_count = event_count;
    const int dispatch = setjmp(waiting);
    if (dispatch == 0) {
        firmware_startup();
        assert(!"Standalone console must remain active");
    }
    assert(dispatch == expected_dispatch);
    assert(start_calls == (expected_dispatch == 2 ? 1U : 0U));
    assert(reinit_calls == expected_reinitializations);
    assert(can1_calls == 0U && can3_calls == 1U && errors == 0U);
    assert(defaults_calls == 1U && flush_calls == 1U && clear_calls >= 1U);
    assert((display_calls > 0U) == expect_display);
    assert(receive_event_index == event_count);
    assert(!cybergear_test_initialized && !cybergear_test_timer_active);
}
static void check_input(const char *input, int expected_dispatch, bool expect_display,
                        unsigned int expected_reinitializations)
{
    ReceiveEvent events[512];
    const size_t count = strlen(input);
    assert(count <= sizeof(events) / sizeof(events[0]));
    for (size_t index = 0U; index < count; ++index)
        events[index] = (ReceiveEvent){(uint8_t)input[index], UART_FLAG_RXNE};
    check_events(events, count, expected_dispatch, expect_display, expected_reinitializations);
}
int main(void)
{
    check_input("", 1, false, 0U);
    check_input("s", 1, false, 0U);
    check_input("S ", 1, false, 0U);
    check_input("set", 1, false, 0U);
    check_input("set observer_rad_s", 1, false, 0U);
    check_input("ss\ns nope\ninvalid\n", 1, false, 0U);
    check_input("set invalid_key 6\n", 1, false, 0U);
    check_input("show\nhelp\n?\r\n", 1, true, 0U);
    check_input("show\ns", 1, true, 0U);
    check_input("s\n", 2, false, 0U);
    check_input("S\r", 2, false, 0U);
    check_input("invalid\nshow\nhelp\nS\n", 2, true, 0U);
    check_input("reinit\n", 3, false, 1U);
    check_input("set current 1.5\n", 3, false, 1U);
    check_input("set current 1.5\ns\n", 2, false, 1U);
    check_input("x", 1, false, 0U);
    check_input("X\ns\n", 1, false, 0U);
    check_input("X\nreinit\ns\n", 2, false, 1U);
    char overlong[CG_CONSOLE_LINE_CAPACITY + 8U];
    memset(overlong, 's', sizeof(overlong));
    overlong[sizeof(overlong) - 2U] = '\n';
    overlong[sizeof(overlong) - 1U] = '\0';
    check_input(overlong, 1, false, 0U);
    const ReceiveEvent uart_error[] = {
        {'s', UART_FLAG_RXNE | UART_FLAG_FE}, {'s', UART_FLAG_RXNE},
        {'\n', UART_FLAG_RXNE}
    };
    check_events(uart_error, sizeof(uart_error) / sizeof(uart_error[0]), 1, false, 0U);
    const ReceiveEvent uart_recovery[] = {
        {'s', UART_FLAG_RXNE | UART_FLAG_ORE}, {'\n', UART_FLAG_RXNE},
        {'S', UART_FLAG_RXNE}, {'\n', UART_FLAG_RXNE}
    };
    check_events(uart_recovery, sizeof(uart_recovery) / sizeof(uart_recovery[0]), 2, false, 0U);
    puts("Standalone startup: real UART IRQ/parser, complete-line start, inert queries and CAN isolation passed.");
    return 0;
}
'''


def main():
    compiler, repo_arg, output_arg = sys.argv[1:]
    repo, output = Path(repo_arg).resolve(), Path(output_arg).resolve()
    source = (repo / "Core/Src/main.c").read_text(encoding="utf-8")
    start = source.index("  HAL_Init();", source.index("int main(void)"))
    end = source.index("  /* USER CODE END 2 */", start)
    helper_names = (
        "cybergear_test_console_flush", "USART2_IRQHandler",
        "cybergear_test_console_init", "cybergear_standalone_test_run",
    )
    helpers = "\n".join(callback(source, name) for name in helper_names)
    generated = output / "standalone_startup_generated.c"
    generated.write_text(PREFIX + helpers + STARTUP_PREFIX + source[start:end] + SUFFIX,
                         encoding="utf-8")
    binary = output / "standalone_startup.exe"
    compile_host(compiler, repo, generated, binary, ["APP_CYBERGEAR_STANDALONE_TEST=1"],
                 extra_sources=[repo / "Core/Src/cybergear_console.c"])
    subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
