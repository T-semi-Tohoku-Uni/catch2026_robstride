#include "motor_app.h"
#include "motor_config.h"
#include "rs03_homing_config.h"
#include "robstride_app.h"
#include "can_init.h"

#include <math.h>
#include <stdio.h>

extern FDCAN_HandleTypeDef hfdcan3;

/* This application replaces motor_app.c in the homing-only build. */
static RobstrideMotor right;
static bool right_homed;
static bool failed;

static RobstrideFeedback feedback_snapshot(RobstrideMotor *motor)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const RobstrideFeedback feedback = motor->feedback;
    __set_PRIMASK(mask);
    return feedback;
}

static bool feedback_ok(RobstrideMotor *motor)
{
    const RobstrideFeedback feedback = feedback_snapshot(motor);
    return feedback.online && feedback.fault_flags == 0U &&
        (uint32_t)(HAL_GetTick() - feedback.last_leceived_ms) < RS03_HOME_FEEDBACK_TIMEOUT_MS;
}

static void stop_right(void)
{
    failed = true;
    /* Keep retrying STOP from process(), including after a full CAN queue. */
    robstride_stop(&right);
}

static bool hold_zero(void)
{
    if (right_homed && (!feedback_ok(&right) || !robstride_set_position(&right, 0.0f)))
        return false;
    return true;
}

static bool wait_with_hold(uint32_t duration_ms)
{
    const uint32_t started_ms = HAL_GetTick();
    do
    {
        if (!hold_zero()) return false;
        HAL_Delay(RS03_HOME_POLL_MS);
    } while ((uint32_t)(HAL_GetTick() - started_ms) < duration_ms);
    return true;
}

static bool home_one(RobstrideMotor *motor, GPIO_TypeDef *port, uint16_t pin, float speed)
{
    /* Configure a zero speed and current limit before enabling. */
    if (!robstride_set_run_mode(motor, VELOCITY) || !wait_with_hold(10U) ||
        !robstride_set_current_limit(motor, RS03_HOME_CURRENT_LIMIT_A) || !wait_with_hold(10U) ||
        !robstride_set_velocity(motor, 0.0f) || !wait_with_hold(10U) ||
        !robstride_enable(motor) || !wait_with_hold(10U)) return false;

    const uint32_t started_ms = HAL_GetTick();
    while (HAL_GPIO_ReadPin(port, pin) != RS03_HOME_ACTIVE_STATE)
    {
        if (!feedback_ok(motor) ||
            (uint32_t)(HAL_GetTick() - started_ms) >= RS03_HOME_TIMEOUT_MS ||
            !robstride_set_velocity(motor, speed) || !wait_with_hold(RS03_HOME_POLL_MS))
            return false;
    }

    /* Brake before disabling and assigning the stopped position as zero. */
    if (!feedback_ok(motor) || !robstride_set_velocity(motor, 0.0f) ||
        !wait_with_hold(RS03_HOME_SETTLE_MS) || !feedback_ok(motor) ||
        !robstride_stop(motor) || !wait_with_hold(10U) ||
        !robstride_set_run_mode(motor, POSITION_CSP) || !wait_with_hold(10U)) return false;

    const uint32_t previous_count = feedback_snapshot(motor).received_count;
    if (!robstride_set_zero(motor) || !wait_with_hold(10U)) return false;
    const uint32_t zero_started_ms = HAL_GetTick();
    while (1)
    {
        const RobstrideFeedback feedback = feedback_snapshot(motor);
        if (feedback.received_count != previous_count && feedback_ok(motor) &&
            fabsf(feedback.position_rad) < 0.02f) break;
        if ((uint32_t)(HAL_GetTick() - zero_started_ms) >= RS03_HOME_FEEDBACK_TIMEOUT_MS ||
            !wait_with_hold(RS03_HOME_POLL_MS)) return false;
    }

    /* Set the new coordinate target before enable, avoiding an old target jump. */
    if (!robstride_set_speed_limit(motor, fabsf(speed)) || !wait_with_hold(10U) ||
        !robstride_set_position(motor, 0.0f) || !wait_with_hold(10U) ||
        !robstride_enable(motor) || !wait_with_hold(10U)) return false;
    return true;
}

void motor_app_start(void)
{
    FDCAN_TxHeaderTypeDef header = {0};
    right = (RobstrideMotor){.motor_id = RIGHT_RS03_ID, .host_id = HOST_ID};
    right_homed = failed = false;
    if (motor_CAN_RxTxSettings_init(&header) != HAL_OK)
    {
        failed = true;
        Error_Handler();
        return;
    }
    right.txheader = header;

    /* Only right participates: no commands or feedback dependency for left. */
    printf("RS03 zero setup: right CW (PC0)\r\n");
    const uint32_t started_ms = HAL_GetTick();
    do
    {
        if (!robstride_stop(&right)) goto fail;
        HAL_Delay(RS03_HOME_POLL_MS);
        if ((uint32_t)(HAL_GetTick() - started_ms) >= MOTOR_INIT_FEEDBACK_TIMEOUT_MS)
            goto fail;
    } while (!feedback_ok(&right));

    if (!home_one(&right, RS03_HOME_RIGHT_PORT, RS03_HOME_RIGHT_PIN, RS03_HOME_RIGHT_SPEED_RAD_S))
        goto fail;
    right_homed = true;
    printf("RS03 zero setup complete; holding right at zero\r\n");
    return;

fail:
    stop_right();
    printf("RS03 zero setup failed; stopped (reset MCU to retry)\r\n");
}

void motor_app_process(void)
{
    if (failed || !hold_zero()) stop_right();
    HAL_Delay(RS03_HOME_POLL_MS);
}

void motor_app_receive_feedback(FDCAN_HandleTypeDef *hfdcan, uint32_t interrupts)
{
    if (hfdcan != &hfdcan3 || (interrupts & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) return;
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        FDCAN_RxHeaderTypeDef header;
        uint8_t data[64];
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK) break;
        if (header.IdType != FDCAN_EXTENDED_ID || header.RxFrameType != FDCAN_DATA_FRAME ||
            header.DataLength != FDCAN_DLC_BYTES_8 ||
            robstride_get_communication_type(header.Identifier) != FeedbackId ||
            robstride_get_destination_id(header.Identifier) != HOST_ID) continue;
        const uint8_t id = (uint8_t)robstride_get_area_2(header.Identifier);
        if (id == right.motor_id) robstride_parse_feedback(header.Identifier, data, &right.feedback);
    }
}

void motor_app_receive_command(FDCAN_HandleTypeDef *hfdcan, uint32_t interrupts)
{
    (void)hfdcan;
    (void)interrupts;
}

void motor_app_control_tick(TIM_HandleTypeDef *htim)
{
    (void)htim;
}
