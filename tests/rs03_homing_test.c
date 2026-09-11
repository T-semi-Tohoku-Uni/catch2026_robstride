#include "motor_app.h"
#include "robstride_app.h"
#include "can_init.h"
#include "rs03_homing_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <setjmp.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "line %d: %s\n", __LINE__, #x); exit(1); } } while (0)

FDCAN_HandleTypeDef hfdcan3;
static uint32_t tick, mask;
static struct {
    bool enabled, zero;
    uint8_t mode;
    unsigned moves, holds;
    float speed;
} sim[5];
static unsigned zeros, scenario;
static uint8_t pending;
static uint8_t reply_motor;
static jmp_buf unexpected_error;

uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return mask; }
void __disable_irq(void) { mask = 1U; }
void __set_PRIMASK(uint32_t value) { mask = value; }
void Error_Handler(void) { longjmp(unexpected_error, 1); }

void HAL_Delay(uint32_t ms)
{
    tick += ms;
    /* Simulate active feedback from enabled motors. */
    for (uint8_t id = 3; id <= 4; ++id)
    {
        if (sim[id].enabled) pending |= (uint8_t)(1U << id);
        if (scenario == 3 || (scenario == 4 && sim[id].moves >= 2))
            pending &= (uint8_t)~(1U << id);
    }
    motor_app_receive_feedback(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
}

GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin)
{
    (void)port;
    CHECK(pin == 1U || pin == 2U);
    const unsigned id = pin == 2U ? 4U : 3U;
    if (scenario == 1) return GPIO_PIN_SET;
    if (scenario == 2 || scenario == 4) return GPIO_PIN_RESET;
    return sim[id].moves >= 3 ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

HAL_StatusTypeDef motor_CAN_RxTxSettings_init(FDCAN_TxHeaderTypeDef *header)
{
    header->Identifier = 0;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *can,
    const FDCAN_TxHeaderTypeDef *header, const uint8_t *data)
{
    CHECK(can == &hfdcan3 && mask == 1U);
    const uint8_t id = (uint8_t)header->Identifier;
    const uint8_t type = (uint8_t)(header->Identifier >> 24);
    CHECK(id == 3 || id == 4); /* No CyberGear or EL05 commands. */
    CHECK(((header->Identifier >> 8) & 0xffffU) == 0xfeU);
    if (type == 3)
    {
        if (id == 3) CHECK(sim[4].zero && sim[4].enabled);
        sim[id].enabled = true;
    }
    else if (type == 4) sim[id].enabled = false;
    else if (type == 6)
    {
        CHECK(!sim[id].enabled && sim[id].speed == 0.0f);
        CHECK(data[0] == 1);
        for (unsigned i = 1; i < 8; ++i) CHECK(data[i] == 0);
        CHECK(id == (zeros == 0 ? 4 : 3));
        ++zeros;
        sim[id].zero = scenario != 6;
    }
    else if (type == 0x12)
    {
        const uint16_t index = convert_u8_u16_le(data);
        const float value = convert_u8_f32_le(data + 4);
        if (index == RUN_MODE) sim[id].mode = data[4];
        if (index == SPEED_REF)
        {
            if (value != 0.0f)
            {
                CHECK(sim[id].enabled && sim[id].mode == VELOCITY);
                CHECK(id == 4 ? value > 0 : value < 0);
                if (id == 3) CHECK(sim[4].zero && sim[4].holds > 0 && sim[4].enabled);
                ++sim[id].moves;
                if (scenario == 5) return HAL_ERROR;
            }
            sim[id].speed = value;
        }
        if (index == POSITION_REF)
        {
            CHECK(sim[id].zero && value == 0.0f && sim[id].mode == POSITION_CSP);
            ++sim[id].holds;
        }
    }
    else CHECK(false);
    pending |= (uint8_t)(1U << id);
    return HAL_OK;
}

uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *can, uint32_t fifo)
{
    (void)can; (void)fifo;
    return pending ? 1U : 0U;
}

HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *can, uint32_t fifo,
    FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{
    (void)can; (void)fifo;
    reply_motor = (pending & (1U << 3)) != 0U ? 3 : 4;
    *header = (FDCAN_RxHeaderTypeDef){0x020000feU | ((uint32_t)reply_motor << 8),
        FDCAN_EXTENDED_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_8};
    if (scenario == 7 && sim[reply_motor].moves >= 2) header->Identifier |= 0x10000U;
    memset(data, 0, 8);
    const uint16_t raw = sim[reply_motor].zero ? 32768U : 35000U;
    data[0] = (uint8_t)(raw >> 8);
    data[1] = (uint8_t)raw;
    pending &= (uint8_t)~(1U << reply_motor);
    return HAL_OK;
}

static void run(unsigned test_case)
{
    scenario = test_case;
    tick = test_case == 8 ? UINT32_MAX - 100U : 0;
    mask = zeros = 0;
    pending = false;
    memset(sim, 0, sizeof(sim));
    CHECK(setjmp(unexpected_error) == 0);
    motor_app_start();
    for (unsigned i = 0; i < 10; ++i) motor_app_process();
    if (test_case == 0 || test_case == 1 || test_case == 8)
    {
        CHECK(zeros == 2 && sim[4].enabled && sim[3].enabled);
        CHECK(sim[4].holds > 10 && sim[3].holds >= 10);
        if (test_case == 1) CHECK(sim[3].moves == 0 && sim[4].moves == 0);
        else CHECK(sim[3].moves >= 3 && sim[4].moves >= 3);
    }
    else
    {
        CHECK(!sim[3].enabled && !sim[4].enabled && sim[3].moves == 0);
        CHECK(zeros == (test_case == 6 ? 1U : 0U));
    }
}

int main(void)
{
    for (unsigned i = 0; i <= 8; ++i) run(i);
    puts("RS03 sequential homing tests passed");
    return 0;
}
