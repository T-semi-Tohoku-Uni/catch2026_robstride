#ifndef CYBERGEAR_CONFIG_H
#define CYBERGEAR_CONFIG_H

#include <math.h>

#define CYBERGEAR_CONTROL_BANDWIDTH_RAD_S            8.0f
#define CYBERGEAR_CONTROL_DAMPING_RATIO              1.0f

#define CYBERGEAR_B0_FIXED                           1.0

#define CYBERGEAR_TRAJECTORY_ACCEL_RAD_S2            2.0f
#define CYBERGEAR_TRAJECTORY_BRAKE_RAD_S2            2.0f
#define CYBERGEAR_TRAJECTORY_JERK_RAD_S3             8.0f

#define CYBERGEAR_CURRENT_RISE_A_S                   10.0f
#define CYBERGEAR_CURRENT_FALL_A_S                   10.0f

#define CYBERGEAR_DISTURBANCE_SLEW_A_S               1.5f
#define CYBERGEAR_DYNAMICS_RESERVE_SLEW_A_S          2.5f

#define CYBERGEAR_COMPENSATION_GAIN                  1.0f
#define CYBERGEAR_LEAK_FIXED_S                       0.03f

#define CYBERGEAR_CURRENT_LIMIT_A                    5.0f
#define CYBERGEAR_TRAJECTORY_SPEED_RAD_S             0.8f
#define CYBERGEAR_OBSERVER_RAD_S                     6.0f

#define CYBERGEAR_SPEED_TRIP_RAD_S                   26.0f
#define CYBERGEAR_TEMPERATURE_TRIP_C                 85.0f

#define CYBERGEAR_SOFT_MIN_RAD                       -6.28
#define CYBERGEAR_SOFT_MAX_RAD                       6.28
#define CYBERGEAR_HARD_MIN_RAD                       -12.5f
#define CYBERGEAR_HARD_MAX_RAD                       12.5f

#define CYBERGEAR_B0_MIN                             0.35
#define CYBERGEAR_B0_MAX                             3.0

#define CYBERGEAR_UART_BAUD_RATE                     115200U
#define CYBERGEAR_UART_TX_TIMEOUT_MS                 10U

#define CYBERGEAR_DEG_TO_RAD(degrees)                ((degrees) * 0.01745329252f)

#ifndef CYBERGEAR_USE_200_HZ
#define CYBERGEAR_USE_200_HZ                         0
#endif

#define CYBERGEAR_TIMING_TOLERANCE_MS                1U
#define CYBERGEAR_FEEDBACK_TIMEOUT_MS                100U

#define CYBERGEAR_DISTURBANCE_LIMIT_A                2.5f

#define CYBERGEAR_COMPENSATION_DELAY_MS              100U
#define CYBERGEAR_COMPENSATION_RAMP_MS               500U

#define CYBERGEAR_LEAK_MODE                          CYBERGEAR_LEAK_FIXED
#define CYBERGEAR_LEAK_NEAR_S                        0.5f
#define CYBERGEAR_LEAK_FAR_S                         0.03f
#define CYBERGEAR_LEAK_NEAR_RAD                      0.003f
#define CYBERGEAR_LEAK_FAR_RAD                       0.015f

#define CYBERGEAR_HOST_ID                            0xfeU
#define CYBERGEAR_MOTOR_ID                           0x7fU
#define CYBERGEAR_HARDWARE_CONFIRMED                 1

#define CYBERGEAR_DYNAMICS_RESERVE_CURRENT_FRACTION  0.5f

#define CYBERGEAR_TRAJECTORY_DURATION_MIN_S          0.1f
#define CYBERGEAR_TRAJECTORY_DURATION_MAX_S          60.0f
#define CYBERGEAR_TRAJECTORY_TARGET_TOLERANCE_RAD    0.0001f
#define CYBERGEAR_TRAJECTORY_SEARCH_ITERATIONS       32U

#define CYBERGEAR_POSITION_JUMP_RAD                  0.02f

#define CYBERGEAR_TRACKING_ERROR_RAD                 0.15f
#define CYBERGEAR_TRACKING_TIMEOUT_MS                200U
#define CYBERGEAR_TRACKING_RECOVERY_ENABLED          1

#define CYBERGEAR_SATURATION_TIMEOUT_MS              1000U
#define CYBERGEAR_SATURATION_RECOVERY_ENABLED        1
#define CYBERGEAR_RECOVERY_ZERO_MS                   200U

#define CYBERGEAR_STALL_CURRENT_FRACTION             0.8f
#define CYBERGEAR_STALL_PROGRESS_RAD                 0.003f
#define CYBERGEAR_STALL_TIMEOUT_MS                   1000U

#define CYBERGEAR_STATIONARY_SPEED_RAD_S             0.03f
#define CYBERGEAR_STATIONARY_DWELL_MS                100U

#define CYBERGEAR_STARTUP_TIMEOUT_MS                 3000U
#define CYBERGEAR_COMMAND_RETRY_MS                   50U
#define CYBERGEAR_STOP_TIMEOUT_MS                    1000U
#define CYBERGEAR_STOP_MAX_ATTEMPTS                  20U

#define CYBERGEAR_PLANNER_LEAD_MS                    50U
#define CYBERGEAR_PLANNER_TIMEOUT_MS                 500U

#define CYBERGEAR_HOMING_FAST_SPEED_RAD_S            1.0f
#define CYBERGEAR_HOMING_SLOW_SPEED_RAD_S            0.4f
#define CYBERGEAR_HOMING_REVERSE_ANGLE_DEG           120.0f
#define CYBERGEAR_HOMING_CLEARANCE_DEG               2.0f
#define CYBERGEAR_HOMING_REVERSE_ANGLE_RAD           CYBERGEAR_DEG_TO_RAD(CYBERGEAR_HOMING_REVERSE_ANGLE_DEG)
#define CYBERGEAR_HOMING_CLEARANCE_RAD               CYBERGEAR_DEG_TO_RAD(CYBERGEAR_HOMING_CLEARANCE_DEG)

#define CYBERGEAR_HOMING_STARTUP_TIMEOUT_MS          3000U
#define CYBERGEAR_HOMING_PHASE_TIMEOUT_MS            15000U
#define CYBERGEAR_HOMING_FEEDBACK_TIMEOUT_MS         100U

#define CYBERGEAR_HOMING_DEBOUNCE_MS                 20U
#define CYBERGEAR_HOMING_POLL_MS                     10U
#define CYBERGEAR_HOMING_PROBE_INTERVAL_MS           100U

#define CYBERGEAR_B0_RATE_LIMIT                      1.0f
#define CYBERGEAR_POSTURE_TIMEOUT_MS                 100U

#define CYBERGEAR_COMPARE_OPERATION_MODE             0
#define CYBERGEAR_OPERATION_KP                       NAN
#define CYBERGEAR_OPERATION_KD                       NAN
#define CYBERGEAR_OPERATION_TORQUE_LIMIT_NM          NAN

#define CYBERGEAR_LOG_UART                           0

static inline int cybergear_control_phase_due(unsigned int phase)
{
    return phase == 0U || (CYBERGEAR_USE_200_HZ && phase == 5U);
}

#endif
