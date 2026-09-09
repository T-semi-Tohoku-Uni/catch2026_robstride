#ifndef CYBERGEAR_TEST_MOTION_H
#define CYBERGEAR_TEST_MOTION_H

#include "cybergear.h"

#define CG_TEST_FIRMWARE_TAG "SWING30_V4_3A"
#define CG_TEST_AMPLITUDE_RAD 0.5235987756f /* +/-30 degrees (60 degrees total) */
#define CG_TEST_SPEED_RAD_S 0.3f
#define CG_TEST_CURRENT_LIMIT_A 3.0f /* ADRC phase only; homing is unchanged. */
#define CG_TEST_DWELL_MS 1000U
#define CG_TEST_LEG_TIMEOUT_MS 20000U
#define CG_TEST_REACHED_RAD 0.0087266463f /* 0.5 degree */
#define CG_TEST_TRAVEL_GUARD_RAD 0.0523598776f /* 3 degrees beyond endpoints */

typedef struct {
    float target_rad;
    uint32_t leg_started_ms, settled_since_ms, completed_legs;
    bool settling, halted;
} CyberGearTestMotion;

typedef enum {
    CG_TEST_WAIT, CG_TEST_NEW_TARGET, CG_TEST_STOP_FAULT,
    CG_TEST_STOP_TRAVEL, CG_TEST_STOP_TIMEOUT
} CyberGearTestResult;

bool cybergear_test_configure(CyberGearMotor *motor);
void cybergear_test_motion_init(CyberGearTestMotion *test, uint32_t now_ms);
/* Call with an atomic snapshot. Faults latch until explicit initialization. */
CyberGearTestResult cybergear_test_motion_update(CyberGearTestMotion *test,
    uint32_t now_ms, bool running, bool fresh, bool trajectory_done,
    float position_rad, float estimated_velocity_rad_s);

#endif
