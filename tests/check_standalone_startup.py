"""Execute the real main initialization in standalone mode with host stubs."""
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
#include "app_mode.h"
#include "cybergear_test_motion.h"
#include "stm32g4xx_hal.h"
typedef struct { unsigned int unused; } UART_HandleTypeDef;
typedef struct { HAL_StatusTypeDef status; uint8_t command; } ReceiveEvent;
static UART_HandleTypeDef huart2;
FDCAN_TxHeaderTypeDef motor_txheader;
static unsigned int can1_calls, can3_calls, setup_calls, test_calls, errors;
static unsigned int receive_calls, flush_calls, clear_calls;
static bool fail_can, start_received;
static const ReceiveEvent *receive_events;
static size_t receive_event_count, receive_event_index;
static jmp_buf waiting;
#define UART_RXDATA_FLUSH_REQUEST 1U
#define __HAL_UART_SEND_REQ(handle, request) flush_uart(handle, request)
#define __HAL_UART_CLEAR_OREFLAG(handle) clear_uart_overrun(handle)
static void flush_uart(UART_HandleTypeDef *handle, uint32_t request)
{
    assert(handle == &huart2 && request == UART_RXDATA_FLUSH_REQUEST);
    assert(receive_calls == 0U && setup_calls == 0U && test_calls == 0U);
    ++flush_calls;
}
static void clear_uart_overrun(UART_HandleTypeDef *handle)
{
    assert(handle == &huart2 && receive_calls == 0U);
    ++clear_calls;
}
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *handle, uint8_t *command,
                                  uint16_t size, uint32_t timeout_ms)
{
    assert(handle == &huart2 && size == 1U);
    assert(timeout_ms > 0U && timeout_ms < 1000U);
    assert(flush_calls == 1U && clear_calls == 1U);
    assert(setup_calls == 0U && test_calls == 0U && errors == 0U);
    assert(!start_received);
    if (++receive_calls > 2048U) longjmp(waiting, 1);
    if (receive_event_index == receive_event_count) return HAL_TIMEOUT;
    const ReceiveEvent event = receive_events[receive_event_index++];
    *command = event.command;
    start_received = event.status == HAL_OK &&
                     (event.command == 's' || event.command == 'S');
    return event.status;
}
void HAL_Init(void) {}
void SystemClock_Config(void) {}
void MX_GPIO_Init(void) {}
void MX_USART2_UART_Init(void) {}
void MX_TIM6_Init(void) {}
void MX_FDCAN1_Init(void) { ++can1_calls; }
void MX_FDCAN3_Init(void) { ++can3_calls; }
void Error_Handler(void) { ++errors; }
HAL_StatusTypeDef motor_CAN_RxTxSettings_init(FDCAN_TxHeaderTypeDef *header)
{
    assert(header == &motor_txheader && can3_calls > 0U);
    assert(start_received && receive_event_index == receive_event_count);
    ++setup_calls;
    return fail_can ? HAL_ERROR : HAL_OK;
}
void cybergear_standalone_test_run(void)
{
    assert(start_received && setup_calls == 1U && !fail_can);
    ++test_calls;
}
'''

STARTUP_PREFIX = r'''
int firmware_startup(void)
{
'''

SUFFIX = r'''
    return 0;
}
static void check_startup(const ReceiveEvent *events, size_t event_count,
                          bool expect_start, bool can_failure)
{
    can1_calls = can3_calls = setup_calls = test_calls = errors = 0U;
    receive_calls = flush_calls = clear_calls = 0U;
    receive_event_index = 0U;
    receive_events = events;
    receive_event_count = event_count;
    start_received = false;
    fail_can = can_failure;
    if (setjmp(waiting) == 0) {
        const int result = firmware_startup();
        assert(expect_start && result == (can_failure ? 1 : 0));
        assert(start_received && setup_calls == 1U);
        assert(test_calls == (can_failure ? 0U : 1U));
        assert(errors == (can_failure ? 1U : 0U));
    } else {
        assert(!expect_start && !start_received);
        assert(setup_calls == 0U && test_calls == 0U && errors == 0U);
    }
    assert(can1_calls == 0U && can3_calls == 1U);
    assert(flush_calls == 1U && clear_calls == 1U);
    assert(receive_event_index == event_count);
}
int main(void)
{
    const ReceiveEvent invalid[] = {
        {HAL_OK, 'x'}, {HAL_OK, 'X'}, {HAL_OK, '\r'}, {HAL_OK, '\n'},
        {HAL_OK, '?'}, {HAL_OK, 0U}, {HAL_ERROR, 's'}, {HAL_BUSY, 'S'},
        {HAL_TIMEOUT, 's'}
    };
    const ReceiveEvent lowercase[] = {{HAL_OK, 's'}};
    const ReceiveEvent uppercase[] = {{HAL_OK, 'S'}};
    const ReceiveEvent delayed[] = {
        {HAL_TIMEOUT, 's'}, {HAL_ERROR, 'S'}, {HAL_OK, 'x'},
        {HAL_OK, '\r'}, {HAL_OK, '\n'}, {HAL_OK, 'S'}
    };
    check_startup(NULL, 0U, false, false);
    check_startup(invalid, sizeof(invalid) / sizeof(invalid[0]), false, false);
    check_startup(lowercase, 1U, true, false);
    check_startup(uppercase, 1U, true, false);
    check_startup(delayed, sizeof(delayed) / sizeof(delayed[0]), true, false);
    check_startup(lowercase, 1U, true, true);
    puts("Standalone startup: UART start gate, CAN isolation and setup failure passed.");
    return 0;
}
'''


def main():
    compiler, repo_arg, output_arg = sys.argv[1:]
    repo, output = Path(repo_arg).resolve(), Path(output_arg).resolve()
    source = (repo / "Core/Src/main.c").read_text(encoding="utf-8")
    start = source.index("  HAL_Init();", source.index("int main(void)"))
    end = source.index("  /* USER CODE END 2 */", start)
    helper_source = source[source.rindex("static void cybergear_test_wait_for_start("):]
    helper = callback(helper_source, "cybergear_test_wait_for_start")
    generated = output / "standalone_startup_generated.c"
    generated.write_text(PREFIX + helper + STARTUP_PREFIX + source[start:end] + SUFFIX,
                         encoding="utf-8")
    binary = output / "standalone_startup.exe"
    compile_host(compiler, repo, generated, binary, ["APP_CYBERGEAR_STANDALONE_TEST=1"])
    subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
