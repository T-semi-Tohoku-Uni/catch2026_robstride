"""Exercise real CAN1 command handling and feedback watchdog in standalone mode."""
from pathlib import Path
import subprocess
import sys
sys.dont_write_bytecode = True
from check_scheduler import callback
from compile_host import compile_host

PREFIX = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "stm32g4xx_hal.h"
#include "app_mode.h"
#define FDCAN_IT_RX_FIFO1_NEW_MESSAGE 1U
#define FDCAN_RX_FIFO1 1U
#define FDCAN_DLC_BYTES_4 0x00040000U
#define FDCAN_DLC_BYTES_16 0x000a0000U
#define MOTOR_INIT_CANID 0x500U
#define CANID 0x200U
#define MOTOR_INIT_FEEDBACK_TIMEOUT_MS 3000U
int can1_instance;
#define FDCAN1 ((void *)&can1_instance)
bool motor_return_active, motors_running, motor_init_requested;
uint32_t motor_return_started_ms, motor_last_feedback_ms;
float target_angle[4] = {1, 2, 3, 4};
unsigned int next_frame, resets;
uint32_t HAL_GetTick(void) { return 100000U; }
void NVIC_SystemReset(void) { ++resets; }
void motor_init_print_can_status(void) {}
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *h, uint32_t fifo)
{ (void)h; assert(fifo == FDCAN_RX_FIFO1); return 3U - next_frame; }
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h, uint32_t fifo,
    FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{
    (void)h; assert(fifo == FDCAN_RX_FIFO1);
    memset(header, 0, sizeof(*header));
    memset(data, 0, 64U);
    header->Identifier = next_frame < 2U ? MOTOR_INIT_CANID : CANID;
    header->DataLength = next_frame < 2U ? FDCAN_DLC_BYTES_4 : FDCAN_DLC_BYTES_16;
    data[3] = (uint8_t)next_frame++; /* A changing start/return command. */
    return HAL_OK;
}
void u8_to_int(uint8_t *data, int32_t *value, uint32_t len)
{ (void)len; *value = data[3]; }
void u8_to_float(uint8_t *data, float *values, uint32_t len)
{ (void)data; (void)len; for (unsigned int i = 0; i < 4U; ++i) values[i] = 99.0f; }
'''
SUFFIX = r'''
int main(void)
{
    FDCAN_HandleTypeDef can = {FDCAN1, 0};
    for (unsigned int running = 0; running < 2U; ++running) {
        motors_running = running != 0U;
        next_frame = 0U;
        HAL_FDCAN_RxFifo1Callback(&can, FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
        assert(next_frame == 3U);
        assert(!motor_init_requested && !motor_return_active);
        for (unsigned int i = 0; i < 4U; ++i) assert(target_angle[i] == (float)(i + 1U));
    }
    motor_check_feedback(); /* No feedback for 100 s must not reboot/re-home. */
    assert(resets == 0U);
    puts("Standalone: external start/return/targets ignored; no feedback reboot.");
    return 0;
}
'''


def main():
    compiler, repo_arg, output_arg = sys.argv[1:]
    repo, output = Path(repo_arg).resolve(), Path(output_arg).resolve()
    source = (repo / "Core/Src/main.c").read_text(encoding="utf-8")
    generated = output / "standalone_commands_generated.c"
    generated.write_text(PREFIX + callback(source, "HAL_FDCAN_RxFifo1Callback")
                         + callback(source, "motor_check_feedback") + SUFFIX, encoding="utf-8")
    binary = output / "standalone_commands.exe"
    compile_host(compiler, repo, generated, binary, ["APP_CYBERGEAR_STANDALONE_TEST=1"])
    subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
