#ifndef BOARD_PROTOCOL_H
#define BOARD_PROTOCOL_H

#include <stdint.h>

#define BOARD_TARGET_CAN_ID 0x200U
#define BOARD_FEEDBACK_CAN_ID 0x210U
#define BOARD_INIT_CAN_ID 0x500U
#define BOARD_AXIS_COUNT 4U
#define BOARD_ANGLE_PAYLOAD_BYTES 16U

typedef enum
{
    BOARD_AXIS_BASE = 0,
    BOARD_AXIS_LEFT = 1,
    BOARD_AXIS_RIGHT = 2,
    BOARD_AXIS_EL05 = 3
} BoardAxis;

/* Fixed-size buffers are required. Integers and IEEE-754 floats are big-endian. */
int32_t board_decode_init_command(const uint8_t data[4]);
void board_decode_angles(const uint8_t data[BOARD_ANGLE_PAYLOAD_BYTES],
                         float angles[BOARD_AXIS_COUNT]);
void board_encode_angles(const float angles[BOARD_AXIS_COUNT],
                         uint8_t data[BOARD_ANGLE_PAYLOAD_BYTES]);
/* Coordinate conversion only; no clamping or finite-value validation. */
float board_target_to_motor(BoardAxis axis, float angle_rad);
float board_position_from_motor(BoardAxis axis, float position_rad);

#endif
