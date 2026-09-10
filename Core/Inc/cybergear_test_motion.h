#ifndef CYBERGEAR_TEST_MOTION_H
#define CYBERGEAR_TEST_MOTION_H

#include "cybergear.h"

typedef struct {
    float target_rad;
    uint32_t leg_started_ms, settled_since_ms, completed_legs;
    uint8_t target_index;
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
