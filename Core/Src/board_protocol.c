#include "board_protocol.h"

#include <float.h>
#include <string.h>

_Static_assert(sizeof(float) == sizeof(uint32_t) && FLT_RADIX == 2 &&
               FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128,
               "Board protocol requires binary32 float");

/* Keep double literals for feedback and float arithmetic for commands, as before. */
#define RIGHT_POSITION_OFFSET_RAD 1.884
#define LEFT_POSITION_OFFSET_RAD 0.0
#define EL05_POSITION_OFFSET_RAD 2.963

static uint32_t read_u32_be(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
           ((uint32_t)data[2] << 8) | (uint32_t)data[3];
}

static void write_u32_be(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value >> 24);
    data[1] = (uint8_t)(value >> 16);
    data[2] = (uint8_t)(value >> 8);
    data[3] = (uint8_t)value;
}

int32_t board_decode_init_command(const uint8_t data[4])
{
    const uint32_t raw = read_u32_be(data);
    int32_t command;
    memcpy(&command, &raw, sizeof(command));
    return command;
}

void board_decode_angles(const uint8_t data[BOARD_ANGLE_PAYLOAD_BYTES],
                         float angles[BOARD_AXIS_COUNT])
{
    for (uint32_t i = 0; i < BOARD_AXIS_COUNT; ++i)
    {
        const uint32_t raw = read_u32_be(&data[4U * i]);
        memcpy(&angles[i], &raw, sizeof(raw));
    }
}

void board_encode_angles(const float angles[BOARD_AXIS_COUNT],
                         uint8_t data[BOARD_ANGLE_PAYLOAD_BYTES])
{
    for (uint32_t i = 0; i < BOARD_AXIS_COUNT; ++i)
    {
        uint32_t raw;
        memcpy(&raw, &angles[i], sizeof(raw));
        write_u32_be(&data[4U * i], raw);
    }
}

float board_target_to_motor(BoardAxis axis, float angle_rad)
{
    switch (axis)
    {
        case BOARD_AXIS_RIGHT: return angle_rad - (float)RIGHT_POSITION_OFFSET_RAD;
        case BOARD_AXIS_LEFT: return -angle_rad - (float)LEFT_POSITION_OFFSET_RAD;
        case BOARD_AXIS_EL05: return -angle_rad - (float)EL05_POSITION_OFFSET_RAD;
        default: return angle_rad;
    }
}

float board_position_from_motor(BoardAxis axis, float position_rad)
{
    switch (axis)
    {
        case BOARD_AXIS_RIGHT: return (float)(position_rad + RIGHT_POSITION_OFFSET_RAD);
        case BOARD_AXIS_LEFT: return (float)(-(position_rad + LEFT_POSITION_OFFSET_RAD));
        case BOARD_AXIS_EL05: return (float)(-(position_rad + EL05_POSITION_OFFSET_RAD));
        default: return position_rad;
    }
}
