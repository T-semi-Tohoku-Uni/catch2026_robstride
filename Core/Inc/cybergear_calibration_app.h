#ifndef CYBERGEAR_CALIBRATION_APP_H
#define CYBERGEAR_CALIBRATION_APP_H
#include "cybergear.h"
#include "cybergear_calibration.h"
#include "cybergear_calibration_config.h"

typedef enum {
    CG_CAL_APP_CAPTURE, CG_CAL_APP_READY, CG_CAL_APP_WRITE_MODE,
    CG_CAL_APP_READ_MODE, CG_CAL_APP_ZERO, CG_CAL_APP_ENABLE,
    CG_CAL_APP_WAIT_RUN, CG_CAL_APP_TRIAL, CG_CAL_APP_STOPPING,
    CG_CAL_APP_DONE, CG_CAL_APP_FAULT
} CgCalAppState;

typedef struct {
    uint32_t timestamp_ms, feedback_timestamp_ms, rx_sequence, trial_id;
    CgCalPhase phase;
    CgCalFault fault;
    CgCalAppState app_state;
    uint8_t motor_mode;
    float position_rad, velocity_rad_s, command_current_a, temperature_c;
    uint32_t dropped, tx_failed;
    bool feedback_valid;
} CgCalLogRow;

typedef struct {
    CyberGearMotor *motor;
    CgCalConfig config, trial_config;
    CgCal core;
    CgCalAppState state;
    float initial_position_rad;
    float startup_position_rad; /* Pinned per trial throughout the zero-current handshake. */
    bool origin_captured, quiet_active, stop_queued, reset_confirmed;
    bool stop_confirmed, dump_pending, recording, abort_requested;
    int requested_direction;
    uint32_t origin_sequence, quiet_sequence, quiet_since_ms;
    uint32_t state_ms, startup_ms, last_tick_ms, last_command_ms;
    uint32_t stop_sequence, read_sequence, trial_id, log_count, dropped;
    uint32_t last_recorded_rx_sequence;
    bool recorded_rx_sequence;
    /* First fault evidence, captured before STOP changes the state/feedback. */
    bool fault_snapshot_valid;
    CgCalAppState fault_state;
    CgCalFault first_fault;
    CgCalFeedback fault_feedback;
    uint32_t fault_timestamp_ms;
    uint8_t fault_motor_mode;
    CgCalLogRow rows[CG_CAL_APP_LOG_CAPACITY];
} CgCalApp;

void cg_cal_app_config_defaults(CgCalConfig *config);
bool cg_cal_app_init(CgCalApp *app, CyberGearMotor *motor, const CgCalConfig *config);
/* TIM6 ISR only, every CG_CAL_APP_PERIOD_MS. FDCAN RX must not preempt this
 * single-owner tick (main.c configures the same preemption priority).
 * Main-loop requests below use short interrupt-masked handoffs. */
void cg_cal_app_tick(CgCalApp *app, uint32_t now_ms);
bool cg_cal_app_request_trial(CgCalApp *app, int direction);
void cg_cal_app_abort(CgCalApp *app);
/* Blocking UART only after STOP has finished/expired. No live printf logging. */
bool cg_cal_app_dump(CgCalApp *app, UART_HandleTypeDef *uart);

#endif
