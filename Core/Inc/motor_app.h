#ifndef MOTOR_APP_H
#define MOTOR_APP_H

#include "main.h"

/* Call once after CubeMX peripheral initialization. Blocks until startup ends. */
void motor_app_start(void);
/* Call from the main loop; includes the existing 10 ms polling delay. */
void motor_app_process(void);
/* HAL callback adapters. Blocking startup must never run in an interrupt. */
void motor_app_receive_feedback(FDCAN_HandleTypeDef *hfdcan, uint32_t interrupts);
void motor_app_receive_command(FDCAN_HandleTypeDef *hfdcan, uint32_t interrupts);
void motor_app_control_tick(TIM_HandleTypeDef *htim);

#endif
