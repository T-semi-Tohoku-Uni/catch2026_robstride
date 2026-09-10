#include "board_protocol.h"

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "line %d: %s\n", __LINE__, #condition); exit(1); \
} } while (0)

int main(void)
{
    const uint8_t values[16] = {
        0x00, 0x00, 0x00, 0x00, /* +0 */
        0xbf, 0x80, 0x00, 0x00, /* -1 */
        0x40, 0x00, 0x00, 0x00, /* +2 */
        0x80, 0x00, 0x00, 0x00  /* -0 */
    };
    float angles[4];
    uint8_t encoded[16];
    board_decode_angles(values, angles);
    CHECK(angles[0] == 0.0f && !signbit(angles[0]));
    CHECK(angles[1] == -1.0f);
    CHECK(angles[2] == 2.0f);
    CHECK(angles[3] == 0.0f && signbit(angles[3]));
    board_encode_angles(angles, encoded);
    CHECK(memcmp(encoded, values, sizeof(values)) == 0);

    const uint8_t commands[][4] = {
        {0, 0, 0, 0}, {0, 0, 0, 1}, {0xff, 0xff, 0xff, 0xff},
        {0x80, 0, 0, 0}, {0x7f, 0xff, 0xff, 0xff}, {0x12, 0x34, 0x56, 0x78}
    };
    const int32_t expected[] = {0, 1, -1, INT32_MIN, INT32_MAX, 0x12345678};
    for (unsigned i = 0; i < sizeof(expected) / sizeof(expected[0]); ++i)
        CHECK(board_decode_init_command(commands[i]) == expected[i]);

    /* Existing transport preserves special floats; validation belongs to control. */
    const uint8_t special[16] = {
        0x7f, 0x80, 0, 0, 0xff, 0x80, 0, 0,
        0x7f, 0xc0, 0x12, 0x34, 0, 0, 0, 1
    };
    board_decode_angles(special, angles);
    CHECK(isinf(angles[0]) && angles[0] > 0);
    CHECK(isinf(angles[1]) && angles[1] < 0);
    CHECK(isnan(angles[2]));
    CHECK(angles[3] > 0 && angles[3] < 1.0e-37f);
    board_encode_angles(angles, encoded);
    CHECK(memcmp(encoded, special, sizeof(special)) == 0);

    const float targets[4] = {0.140f, -0.900f, 1.98f, 0.0f};
    const float motor_expected[4] = {0.140f, -0.100f, 0.096f, -2.963f};
    for (unsigned axis = 0; axis < BOARD_AXIS_COUNT; ++axis)
    {
        const float motor = board_target_to_motor((BoardAxis)axis, targets[axis]);
        CHECK(fabsf(motor - motor_expected[axis]) < 0.000001f);
        CHECK(fabsf(board_position_from_motor((BoardAxis)axis, motor) - targets[axis]) < 0.000001f);
    }
    puts("board protocol: byte order, signed commands, special floats and axes passed");
    return 0;
}
