#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

/* Board wiring, addressing and application timing. */
#define HOST_ID 0xfe
#define RIGHT_RS03_ID 3
#define LEFT_RS03_ID 4
#define EL05_ID 5
#define CYBER_GEAR_ID 0x7f

#define RIGHT_RS03_INDEX 0
#define LEFT_RS03_INDEX 1
#define EL05_INDEX 2
#define ROBSTRIDE_MOTOR_COUNT 3U

#define CYBERGEAR_DEBUG_INTERVAL_MS 200U
#define MOTOR_INIT_FEEDBACK_TIMEOUT_MS 3000U
#define MOTOR_RETURN_IGNORE_MS 2000U
/* TIM6 ticks every 1 ms. Four motor slots require at least four phases. */
#define MOTOR_CONTROL_PHASE_COUNT 5U
#define BOARD_FEEDBACK_INTERVAL_MS 5U
#if MOTOR_CONTROL_PHASE_COUNT < 4U
#error "Motor control cycle must include all four motor slots"
#endif
#if BOARD_FEEDBACK_INTERVAL_MS < 1U
#error "Board feedback interval must be at least one TIM6 tick"
#endif
#define MOTOR_POLL_INTERVAL_MS 10U
#define MOTOR_PROBE_INTERVAL_MS 100U
#define MOTOR_PP_VELOCITY_RAD_S 10.0f
#define MOTOR_PP_ACCELERATION_RAD_S2 1.0f
#define MOTOR_PP_CURRENT_LIMIT_A 10.0f

#define CYBERGEAR_HOMING_REVERSE_ANGLE_RAD 1.0471975512f
#define CYBERGEAR_HOMING_SLOW_SPEED_RAD_S 0.4f
#define CYBERGEAR_HOMING_CLEARANCE_RAD 0.034906585f
#define CYBERGEAR_HOMING_FEEDBACK_TIMEOUT_MS 100U
#define CYBERGEAR_HOMING_PHASE_TIMEOUT_MS 15000U
#define CYBERGEAR_HOMING_DEBOUNCE_MS 20U

#endif
