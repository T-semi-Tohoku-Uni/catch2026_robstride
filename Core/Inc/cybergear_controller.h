#ifndef CYBERGEAR_CONTROLLER_H
#define CYBERGEAR_CONTROLLER_H
#include <stdint.h>
#include "cybergear_trajectory.h"
typedef struct {
    CGLimits motion;
    float current_limit, current_slew, disturbance_limit, disturbance_slew;
    float b0, temperature_limit, speed_trip, position_jump, stall_error, stall_progress;
    float guard_min, guard_max;
    uint32_t stall_ms;
    bool approved, b0_verified, legacy_leak;
} CGConfig;
typedef struct {
    float q,v,f,dist,track,requested,command,gamma;
    bool amplitude_limited, slew_limited;
} CGController;
bool cg_config_valid(const CGConfig *c);
void cg_observer_gains(float dt, float *l1, float *l2, float *l3);
bool cg_controller_step(CGController *s, const CGConfig *c, CGReference r,
    float measurement, bool fresh, float queued_current, float dt);
#endif
