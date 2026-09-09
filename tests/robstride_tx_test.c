#include <assert.h>
#include <stdio.h>
#include "robstride_app.h"

FDCAN_HandleTypeDef hfdcan3;
static unsigned int tx_count;
uint32_t HAL_GetTick(void) { return 0U; }
void HAL_Delay(uint32_t ms) { (void)ms; }
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(
    FDCAN_HandleTypeDef *can, FDCAN_TxHeaderTypeDef *header, uint8_t *data)
{
    (void)can; (void)header; (void)data;
    ++tx_count;
    return HAL_OK;
}

int main(void)
{
    for (uint8_t id = 3U; id <= 5U; ++id) {
        RobstrideMotor motor = {0};
        motor.host_id = 0xfeU;
        motor.motor_id = id;
        uint8_t data[8] = {0};
        for (uint8_t type = 0U; type < 32U; ++type)
            assert(!send_robstride(&motor, robstride_make_can_id(type, 0xfeU, id), data));
        assert(!robstride_enable(&motor));
        assert(!robstride_stop(&motor));
        assert(!robstride_clear_fault(&motor));
        assert(!robstride_set_zero(&motor));
        assert(!robstride_set_current(&motor, 1.0f));
        assert(!robstride_set_velocity(&motor, 0.5f));
        assert(!robstride_set_position(&motor, 0.1f));
        assert(!robstride_start_current_mode(&motor));
        assert(!robstride_start_velocity_mode(&motor, 1.0f));
        assert(!robstride_start_position_pp_mode(&motor, 1.0f, 1.0f, 1.0f));
        assert(!robstride_start_position_csp_mode(&motor, 1.0f, 1.0f));
    }
    assert(tx_count == 0U);
    puts("Standalone test: all RobStride IDs and commands blocked at transport.");
    return 0;
}
