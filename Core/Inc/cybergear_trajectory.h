#ifndef CYBERGEAR_TRAJECTORY_H
#define CYBERGEAR_TRAJECTORY_H
#include <stdbool.h>
typedef struct { float q, v, a; } CGReference;
typedef struct { float q_min, q_max, velocity, acceleration, jerk; } CGLimits;
typedef struct { float c[6], duration, elapsed, target; } CGTrajectory;
bool cg_limits_valid(const CGLimits *l);
bool cg_trajectory_plan(CGTrajectory *t, CGReference start, float target, const CGLimits *l);
typedef bool (*CGBudgetAvailable)(void *context);
bool cg_trajectory_plan_bounded(CGTrajectory *t, CGReference start, float target,
    const CGLimits *l, CGBudgetAvailable available, void *context);
CGReference cg_trajectory_sample(const CGTrajectory *t, float seconds);
void cg_trajectory_hold(CGTrajectory *t, float q);
#endif
