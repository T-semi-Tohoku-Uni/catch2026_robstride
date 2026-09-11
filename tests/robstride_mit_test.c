#include "robstride_app.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "line %d: %s\n", __LINE__, #condition); exit(1); \
} } while (0)

FDCAN_HandleTypeDef hfdcan3;
static struct { uint32_t id; uint8_t data[8]; } frames[16];
static unsigned frame_count;
static unsigned fail_frame;
static uint32_t tick, mask;

uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick += ms; }
uint32_t __get_PRIMASK(void) { return mask; }
void __disable_irq(void) { mask = 1U; }
void __set_PRIMASK(uint32_t value) { mask = value; }

HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *can,
    const FDCAN_TxHeaderTypeDef *header, const uint8_t *data)
{
    CHECK(can == &hfdcan3 && mask == 1U && frame_count < 16U);
    frames[frame_count].id = header->Identifier;
    memcpy(frames[frame_count].data, data, 8);
    ++frame_count;
    return frame_count == fail_frame ? HAL_ERROR : HAL_OK;
}

static RobstrideMotor setup(RobstrideModel model)
{
    RobstrideMotor motor = {0};
    motor.model = model;
    motor.motor_id = model == ROBSTRIDE_MODEL_EL05 ? 5U : 3U;
    motor.host_id = 0xfe;
    motor.run_mode = MIT_MODE;
    frame_count = fail_frame = 0U;
    tick = mask = 0U;
    return motor;
}

static void check_frame(unsigned index, uint32_t id, const uint8_t data[8])
{
    CHECK(index < frame_count);
    CHECK(frames[index].id == id);
    CHECK(memcmp(frames[index].data, data, 8) == 0);
}

static void test_wire_format(void)
{
    /* Hand-calculated protocol vectors exercise per-model scaling and BE order. */
    const uint8_t rs03_vector[8] = {0x80,0x00,0xbf,0xff,0x00,0x42,0x02,0x8f};
    const uint8_t el05_vector[8] = {0x80,0x00,0x99,0x99,0x02,0x8f,0x33,0x33};
    RobstrideMotor motor = setup(ROBSTRIDE_MODEL_RS03);
    CHECK(robstride_control_mit(&motor, 0, 10, 5, 1, 3));
    check_frame(0, 0x01866603U, rs03_vector);
    CHECK(mask == 0U);
    motor = setup(ROBSTRIDE_MODEL_EL05);
    mask = 1U; /* Sending must also preserve an already-masked caller. */
    CHECK(robstride_control_mit(&motor, 0, 10, 5, 1, 3));
    check_frame(0, 0x01bfff05U, el05_vector);
    CHECK(mask == 1U);

    const uint8_t minimum[8] = {0};
    const uint8_t maximum[8] = {255,255,255,255,255,255,255,255};
    CHECK(robstride_control_mit(&motor, -12.57f, -50, 0, 0, -6));
    check_frame(1, 0x01000005U, minimum);
    CHECK(robstride_control_mit(&motor, 12.57f, 50, 500, 5, 6));
    check_frame(2, 0x01ffff05U, maximum);
    motor = setup(ROBSTRIDE_MODEL_RS03);
    CHECK(robstride_control_mit(&motor, -12.57f, -20, 0, 0, -60));
    check_frame(0, 0x01000003U, minimum);
    CHECK(robstride_control_mit(&motor, 12.57f, 20, 5000, 100, 60));
    check_frame(1, 0x01ffff03U, maximum);
}

static void test_rejection(void)
{
    RobstrideMotor motor = setup(ROBSTRIDE_MODEL_EL05);
    const float invalid[][5] = {
        {NAN,0,0,0,0}, {0,INFINITY,0,0,0}, {0,0,NAN,0,0},
        {0,0,0,INFINITY,0}, {0,0,0,0,NAN},
        {13,0,0,0,0}, {-13,0,0,0,0}, {0,51,0,0,0}, {0,-51,0,0,0},
        {0,0,-1,0,0}, {0,0,501,0,0}, {0,0,0,-1,0}, {0,0,0,6,0},
        {0,0,0,0,7}, {0,0,0,0,-7}
    };
    for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
        CHECK(!robstride_control_mit(&motor, invalid[i][0], invalid[i][1],
            invalid[i][2], invalid[i][3], invalid[i][4]));
    CHECK(!robstride_control_mit(NULL, 0,0,0,0,0));
    motor.run_mode = POSITION_PP;
    CHECK(!robstride_control_mit(&motor, 0,0,0,0,0));
    motor.run_mode = MIT_MODE;
    motor.model = ROBSTRIDE_MODEL_UNSPECIFIED;
    CHECK(!robstride_control_mit(&motor, 0,0,0,0,0));
    CHECK(!robstride_start_mit_mode(&motor));
    CHECK(frame_count == 0U);
    motor.model = ROBSTRIDE_MODEL_EL05;
    fail_frame = 1U;
    CHECK(!robstride_control_mit(&motor, 0,0,0,0,0));
    CHECK(mask == 0U);
}

static void test_startup_and_retry(void)
{
    const uint8_t empty[8] = {0};
    const uint8_t mode[8] = {0x05,0x70,0,0,0,0,0,0};
    const uint8_t neutral[8] = {0x80,0,0x80,0,0,0,0,0};
    for (unsigned model = ROBSTRIDE_MODEL_RS03; model <= ROBSTRIDE_MODEL_EL05; ++model)
    {
        RobstrideMotor motor = setup((RobstrideModel)model);
        motor.run_mode = POSITION_PP;
        for (unsigned attempt = 0; attempt < 2; ++attempt)
        {
            frame_count = 0;
            CHECK(robstride_start_mit_mode(&motor));
            CHECK(motor.run_mode == MIT_MODE && frame_count == 4U);
            check_frame(0, 0x0400fe00U | motor.motor_id, empty);
            check_frame(1, 0x1200fe00U | motor.motor_id, mode);
            check_frame(2, 0x0300fe00U | motor.motor_id, empty);
            check_frame(3, 0x01800000U | motor.motor_id, neutral);
        }
        CHECK(tick == 60U);
    }
    for (unsigned failure = 1; failure <= 4; ++failure)
    {
        RobstrideMotor motor = setup(ROBSTRIDE_MODEL_EL05);
        motor.run_mode = POSITION_PP;
        fail_frame = failure;
        CHECK(!robstride_start_mit_mode(&motor));
        CHECK(frame_count == (failure == 4 ? 5U : failure));
        if (failure <= 2) CHECK(motor.run_mode == POSITION_PP);
        if (failure == 4) check_frame(4, 0x0400fe05U, empty);
        CHECK(mask == 0U);
    }
}

static void test_feedback(void)
{
    const uint8_t maximum[8] = {255,255,255,255,255,255,1,144};
    const uint8_t minimum[8] = {0};
    for (unsigned model = ROBSTRIDE_MODEL_RS03; model <= ROBSTRIDE_MODEL_EL05; ++model)
    {
        RobstrideMotor motor = setup((RobstrideModel)model);
        uint32_t id = 0x028000feU | ((uint32_t)motor.motor_id << 8);
        tick = 123;
        robstride_parse_feedback(id, maximum, &motor);
        CHECK(motor.feedback.online && motor.feedback.received_count == 1U);
        CHECK(motor.feedback.mode == 2 && motor.feedback.fault_flags == 0);
        CHECK(motor.feedback.last_leceived_ms == 123);
        CHECK(motor.feedback.position_rad == 12.57f && motor.feedback.temperature_c == 40);
        CHECK(motor.feedback.velocity_rps == (model == ROBSTRIDE_MODEL_EL05 ? 50 : 20));
        CHECK(motor.feedback.torqe_nm == (model == ROBSTRIDE_MODEL_EL05 ? 6 : 60));
        robstride_parse_feedback(id, minimum, &motor);
        CHECK(motor.feedback.velocity_rps == (model == ROBSTRIDE_MODEL_EL05 ? -50 : -20));
        CHECK(motor.feedback.torqe_nm == (model == ROBSTRIDE_MODEL_EL05 ? -6 : -60));
        robstride_parse_feedback(id ^ 1U, maximum, &motor); /* Other host. */
        robstride_parse_feedback(id ^ 0x100U, maximum, &motor); /* Other motor. */
        robstride_parse_feedback(id ^ 0x1000000U, maximum, &motor); /* Other type. */
        robstride_parse_feedback(id, NULL, &motor);
        CHECK(motor.feedback.received_count == 2U);
    }
}

int main(void)
{
    test_wire_format();
    test_rejection();
    test_startup_and_retry();
    test_feedback();
    puts("RobStride MIT protocol tests passed");
    return 0;
}
