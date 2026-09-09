"""Compile actual FDCAN3 RX dispatch with a counted, in-memory frame FIFO."""
from pathlib import Path
import subprocess
import sys
sys.dont_write_bytecode = True
from check_scheduler import callback
from compile_host import compile_host

PREFIX = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "cybergear.h"
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 1U
#define FDCAN_RX_FIFO0 0U
#define HOST_ID 0xfeU
#define CYBER_GEAR_ID 0x7fU
#define RIGHT_RS03_ID 3U
#define LEFT_RS03_ID 4U
#define EL05_ID 5U
#define FeedbackId 2U
int peripheral3;
#define FDCAN3 ((void *)&peripheral3)
typedef struct { float position_rad; } MockFeedback;
typedef struct { uint8_t motor_id; MockFeedback feedback; } MockRobstride;
MockRobstride robstride_handler[3] = {{3, {0}}, {4, {0}}, {5, {0}}};
CyberGearMotor cybergear_base;
uint32_t motor_can_rx_count, motor_can_last_rx_id, motor_can_last_rx_dlc;
uint32_t motor_can_last_rx_id_type, motor_last_feedback_ms;
bool motor_init_feedback_received;
FDCAN_RxHeaderTypeDef frames[8];
unsigned int next_frame, frame_count, forwarded[32], rs_parsed;
uint32_t HAL_GetTick(void) { return 1000U; }
uint8_t robstride_get_communication_type(uint32_t id) { return (id >> 24) & 31U; }
uint8_t robstride_get_destination_id(uint32_t id) { return id & 255U; }
uint16_t robstride_get_area_2(uint32_t id) { return (id >> 8) & 65535U; }
uint32_t HAL_FDCAN_GetRxFifoFillLevel(const FDCAN_HandleTypeDef *h, uint32_t fifo)
{ (void)h; assert(fifo == FDCAN_RX_FIFO0); return frame_count - next_frame; }
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h, uint32_t fifo,
    FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{ (void)h; (void)fifo; *header = frames[next_frame++]; memset(data, 0, 8); return HAL_OK; }
bool cybergear_process_rx(CyberGearMotor *m, const FDCAN_RxHeaderTypeDef *header, const uint8_t *data)
{
    (void)m; (void)data;
    if (header->IdType != FDCAN_EXTENDED_ID || header->RxFrameType != FDCAN_DATA_FRAME ||
        header->DataLength != FDCAN_DLC_BYTES_8 ||
        (header->Identifier & 0xffffU) != 0x7ffeU) return false;
    unsigned int type = (header->Identifier >> 24) & 31U;
    if (type != 2U && type != 17U && type != 21U) return false;
    ++forwarded[type];
    return true;
}
bool cybergear_parse_feedback(CyberGearMotor *m, uint32_t id, const uint8_t *data)
{
    FDCAN_RxHeaderTypeDef header = {0};
    header.Identifier = id;
    header.IdType = FDCAN_EXTENDED_ID;
    header.DataLength = FDCAN_DLC_BYTES_8;
    return cybergear_process_rx(m, &header, data);
}
bool robstride_parse_feedback(uint32_t id, const uint8_t *data, MockFeedback *feedback)
{
    (void)data; (void)feedback;
    assert((id & 0xffffU) == 0x03feU && ((id >> 24) & 31U) == 2U);
    ++rs_parsed;
    return true;
}
'''
SUFFIX = r'''
int main(void)
{
    const uint32_t ids[] = {0x11007ffeU, 0x15007ffeU, 0x02807ffeU,
        0x028003feU, 0x110003feU, 0x02807ffeU, 0x01007ffeU};
    frame_count = sizeof(ids) / sizeof(ids[0]);
    for (unsigned int i = 0U; i < frame_count; ++i) {
        frames[i].Identifier = ids[i];
        frames[i].IdType = FDCAN_EXTENDED_ID;
        frames[i].DataLength = FDCAN_DLC_BYTES_8;
    }
    frames[5].DataLength = FDCAN_DLC_BYTES_7;
    FDCAN_HandleTypeDef can = {FDCAN3, 0};
    HAL_FDCAN_RxFifo0Callback(&can, 0U);
    assert(next_frame == 0U);
    HAL_FDCAN_RxFifo0Callback(&can, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    assert(next_frame == frame_count && motor_can_rx_count == frame_count);
    assert(forwarded[2] == 1U && forwarded[17] == 1U && forwarded[21] == 1U);
    assert(rs_parsed == 1U);
    assert(motor_init_feedback_received && motor_last_feedback_ms == 1000U);
    puts("Real FDCAN3 callback: type 2/17/21 routing and RobStride isolation passed.");
    return 0;
}
'''


def main() -> None:
    compiler, repo_arg, output_arg = sys.argv[1:]
    repo, output = Path(repo_arg).resolve(), Path(output_arg).resolve()
    source = callback((repo / "Core/Src/main.c").read_text(encoding="utf-8"),
                      "HAL_FDCAN_RxFifo0Callback")
    generated = output / "rx_dispatch_generated.c"
    generated.write_text(PREFIX + source + SUFFIX, encoding="utf-8")
    binary = output / "rx_dispatch.exe"
    compile_host(compiler, repo, generated, binary)
    subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
