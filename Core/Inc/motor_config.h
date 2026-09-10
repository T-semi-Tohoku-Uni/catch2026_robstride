#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

/* Board wiring, addressing, application timing and motor tuning. */
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
#define MOTOR_CONTROL_PHASE_COUNT 10U
#define MOTOR_POLL_INTERVAL_MS 10U
#define MOTOR_PROBE_INTERVAL_MS 100U
/* Provisional MIT gains; tune on the actual mechanism. No PP motion limits apply.
 * The board supplies position only; velocity and feedforward torque are zero. */
#define RIGHT_RS03_MIT_KP 5.0f
#define RIGHT_RS03_MIT_KD 1.0f
#define LEFT_RS03_MIT_KP 5.0f
#define LEFT_RS03_MIT_KD 1.0f
#define EL05_MIT_KP 1.0f
#define EL05_MIT_KD 0.1f

#define CYBERGEAR_HOMING_REVERSE_ANGLE_RAD 1.0471975512f
#define CYBERGEAR_HOMING_SLOW_SPEED_RAD_S 0.4f
#define CYBERGEAR_HOMING_CLEARANCE_RAD 0.034906585f
#define CYBERGEAR_HOMING_FEEDBACK_TIMEOUT_MS 100U
#define CYBERGEAR_HOMING_PHASE_TIMEOUT_MS 15000U
#define CYBERGEAR_HOMING_DEBOUNCE_MS 20U

#endif
