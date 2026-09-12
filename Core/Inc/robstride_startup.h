#ifndef ROBSTRIDE_STARTUP_H
#define ROBSTRIDE_STARTUP_H

#include <stdbool.h>
#include <stdint.h>
#include "robstride_app.h"

#define ROBSTRIDE_STARTUP_MAX_MOTORS 3U
#define ROBSTRIDE_STARTUP_TIMEOUT_MS 15000U
#define ROBSTRIDE_STARTUP_POLL_MS 10U
#define ROBSTRIDE_STARTUP_PROBE_MS 50U
#define ROBSTRIDE_STARTUP_RETRY_MS 300U
#define ROBSTRIDE_STARTUP_FEEDBACK_MS 100U
#define ROBSTRIDE_STARTUP_VELOCITY_RAD_S 10.0f
#define ROBSTRIDE_STARTUP_ACCELERATION_RAD_S2 1.0f
/* Per-motor current limits [A]. Applied on every startup and retry. */
#define ROBSTRIDE_RIGHT_RS03_CURRENT_LIMIT_A 20.0f
#define ROBSTRIDE_LEFT_RS03_CURRENT_LIMIT_A 20c.0f
#define ROBSTRIDE_EL05_CURRENT_LIMIT_A 10.0f

typedef enum {
    RS_STARTUP_SEND_STOP,
    RS_STARTUP_WAIT_STOPPED,
    RS_STARTUP_SET_MODE,
    RS_STARTUP_SET_VELOCITY,
    RS_STARTUP_SET_ACCELERATION,
    RS_STARTUP_SET_CURRENT,
    RS_STARTUP_SET_HOLD,
    RS_STARTUP_SEND_ENABLE,
    RS_STARTUP_WAIT_RUNNING,
    RS_STARTUP_READY
} RobstrideStartupStage;

typedef struct {
    RobstrideStartupStage stage;
    bool waiting_for_write;
    uint32_t baseline_count;
    uint32_t last_probe_ms;
    uint32_t enabled_ms;
    float hold_position_rad;
    float current_limit_a;
} RobstrideStartupAxis;

typedef struct {
    RobstrideMotor *motors;
    uint32_t motor_count;
    uint32_t started_ms;
    uint32_t last_poll_ms;
    uint32_t next_axis;
    bool failed;
    RobstrideStartupAxis axes[ROBSTRIDE_STARTUP_MAX_MOTORS];
} RobstrideStartup;

bool robstride_startup_init(RobstrideStartup *startup, RobstrideMotor *motors,
    uint32_t motor_count, const float *current_limits_a);
void robstride_startup_update(RobstrideStartup *startup);
bool robstride_startup_ready(RobstrideStartup *startup);
bool robstride_startup_failed(RobstrideStartup *startup);

#endif
