"""Exercise real CAN1 commands and the return timer using an in-memory FIFO."""
from pathlib import Path
import subprocess
import sys

sys.dont_write_bytecode = True
from check_scheduler import callback
from compile_host import compile_host


PREFIX = r'''
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "cybergear.h"
#include "app_mode.h"
#include "board_protocol.h"
#define __MAIN_H
#include "robstride_startup.h"
#define FDCAN_IT_RX_FIFO1_NEW_MESSAGE 1U
#define FDCAN_RX_FIFO1 1U
#define FDCAN_DLC_BYTES_4 0x00040000U
#define FDCAN_DLC_BYTES_16 0x000a0000U
#define MOTOR_INIT_CANID 0x500U
#define CANID 0x200U
#define MOTOR_RETURN_IGNORE_MS 2000U
int can1_instance;
#define FDCAN1 ((void *)&can1_instance)
FDCAN_HandleTypeDef hfdcan1 = {FDCAN1, 0U};
CyberGearMotor cybergear_base;
RobstrideMotor robstride_handler[3];
bool motor_return_active, motors_running, motor_init_requested;
uint32_t motor_return_started_ms, motor_last_feedback_ms;
float target_angle[4] = {1.0f, 2.0f, 3.0f, 4.0f};
uint32_t tick_ms = 1000U;
unsigned int failures, resets;
typedef struct { FDCAN_RxHeaderTypeDef header; uint8_t data[64]; } QueuedFrame;
QueuedFrame frames[32];
unsigned int next_frame, frame_count;
uint32_t HAL_GetTick(void) { return tick_ms; }
void NVIC_SystemReset(void) { ++resets; }
void motor_init_print_can_status(void) {}
void motor_init_failed(const char *reason) { assert(reason != NULL); ++failures; }
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *handle, uint32_t fifo)
{
    assert(handle == &hfdcan1 && fifo == FDCAN_RX_FIFO1);
    return frame_count - next_frame;
}
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *handle, uint32_t fifo,
    FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{
    assert(handle == &hfdcan1 && fifo == FDCAN_RX_FIFO1);
    assert(next_frame < frame_count);
    *header = frames[next_frame].header;
    memcpy(data, frames[next_frame++].data, 64U);
    return HAL_OK;
}
QueuedFrame *queue_frame(uint32_t identifier, uint32_t length)
{
    if (next_frame == frame_count) next_frame = frame_count = 0U;
    assert(frame_count < sizeof(frames) / sizeof(frames[0]));
    QueuedFrame *frame = &frames[frame_count++];
    memset(frame, 0, sizeof(*frame));
    frame->header.Identifier = identifier;
    frame->header.IdType = FDCAN_STANDARD_ID;
    frame->header.RxFrameType = FDCAN_DATA_FRAME;
    frame->header.DataLength = length;
    return frame;
}
QueuedFrame *queue_command(int32_t command)
{
    QueuedFrame *frame = queue_frame(MOTOR_INIT_CANID, FDCAN_DLC_BYTES_4);
    const uint32_t raw = (uint32_t)command;
    frame->data[0] = (uint8_t)(raw >> 24);
    frame->data[1] = (uint8_t)(raw >> 16);
    frame->data[2] = (uint8_t)(raw >> 8);
    frame->data[3] = (uint8_t)raw;
    return frame;
}
QueuedFrame *queue_angles(const float values[4])
{
    QueuedFrame *frame = queue_frame(CANID, FDCAN_DLC_BYTES_16);
    board_encode_angles(values, frame->data);
    return frame;
}
void assert_targets(const float expected[4])
{
    for (unsigned int index = 0U; index < 4U; ++index)
        assert(target_angle[index] == expected[index]);
}
'''

SUFFIX = r'''
void receive(void)
{
    HAL_FDCAN_RxFifo1Callback(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
    assert(next_frame == frame_count);
}

int main(void)
{
    const float initial[4] = {1.0f, 2.0f, 3.0f, 4.0f};
    const float requested[4] = {0.5f, -1.25f, 2.0f, 3.5f};
    const uint8_t requested_bytes[16] = {
        0x3f, 0x00, 0x00, 0x00, 0xbf, 0xa0, 0x00, 0x00,
        0x40, 0x00, 0x00, 0x00, 0x40, 0x60, 0x00, 0x00};
    QueuedFrame *frame = queue_frame(CANID, FDCAN_DLC_BYTES_16);
    memcpy(frame->data, requested_bytes, sizeof(requested_bytes));
    FDCAN_HandleTypeDef other_can = {NULL, 0U};
    HAL_FDCAN_RxFifo1Callback(&other_can, FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
    HAL_FDCAN_RxFifo1Callback(&hfdcan1, 0U);
    assert(next_frame == 0U);
    assert_targets(initial);
    receive();
    assert_targets(requested);
    assert(!motors_running && !motor_init_requested);

    const float invalid[] = {NAN, INFINITY, -INFINITY};
    for (unsigned int axis = 0U; axis < 4U; ++axis) {
        for (unsigned int kind = 0U; kind < 3U; ++kind) {
            float values[4] = {9.0f, 8.0f, 7.0f, 6.0f};
            values[axis] = invalid[kind];
            queue_angles(values);
            receive();
            assert_targets(requested);
        }
    }
    for (unsigned int malformed = 0U; malformed < 4U; ++malformed) {
        frame = queue_angles(initial);
        if (malformed == 0U) frame->header.IdType = FDCAN_EXTENDED_ID;
        if (malformed == 1U) frame->header.RxFrameType = FDCAN_REMOTE_FRAME;
        if (malformed == 2U) frame->header.DataLength = FDCAN_DLC_BYTES_12;
        if (malformed == 3U) frame->header.Identifier = CANID + 1U;
        receive();
        assert_targets(requested);
    }

    frame = queue_command(0);
    frame->header.IdType = FDCAN_EXTENDED_ID;
    receive();
    queue_command(1);
    receive();
    assert(!motor_init_requested);
    queue_command(2);
    queue_command(-1);
    queue_command(1);
    receive();
    assert(!motor_init_requested);
    queue_command(0);
    receive();
    assert(!motor_init_requested);
    queue_command(1);
    receive();
    assert(motor_init_requested);
    motor_init_requested = false;
    queue_command(1);
    receive();
    assert(!motor_init_requested);

    motors_running = true;
    tick_ms = UINT32_MAX - 1000U;
    queue_command(0);
    receive();
    assert(motor_return_active);
    assert(motor_return_started_ms == tick_ms);
    const float returning[4] = {0.140f, -0.900f, 1.98f, requested[3]};
    assert_targets(returning);
    const uint32_t return_started = tick_ms;
    queue_command(7);
    queue_angles(initial);
    receive();
    assert_targets(returning);
    assert(motor_return_started_ms == return_started);
    tick_ms = return_started + 1999U;
    motor_return_update();
    assert(motor_return_active);
    queue_command(9);
    queue_angles(initial);
    tick_ms = return_started + 2000U;
    motor_return_update();
    assert(next_frame == frame_count);
    assert(!motor_return_active);
    assert_targets(returning);
    queue_command(9);
    receive();
    assert(!motor_return_active);
    queue_angles(initial);
    receive();
    assert_targets(initial);
    queue_command(10);
    receive();
    assert(motor_return_active);
    assert(failures == 0U && resets == 0U);
    puts("Real CAN1 callback: complete finite frames, startup edge, and 2 s return gating passed.");
    return 0;
}
'''


def main():
    compiler, repo_arg, output_arg = sys.argv[1:]
    repo, output = Path(repo_arg).resolve(), Path(output_arg).resolve()
    source = (repo / "Core/Src/main.c").read_text(encoding="utf-8")
    protocol = (repo / "Core/Src/board_protocol.c").read_text(encoding="utf-8")
    generated = output / "board_commands_generated.c"
    generated.write_text(PREFIX + protocol
                         + callback(source, "HAL_FDCAN_RxFifo1Callback")
                         + callback(source, "motor_return_update") + SUFFIX,
                         encoding="utf-8")
    binary = output / "board_commands.exe"
    compile_host(compiler, repo, generated, binary, ["APP_CYBERGEAR_STANDALONE_TEST=0"])
    subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
