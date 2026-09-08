#ifndef CYBERGEAR_CALIBRATION_H
#define CYBERGEAR_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

/* Robot-mounted trial: this fixed envelope cannot be enlarged by configuration.
 * This is a software trip boundary, NOT a guarantee that inertia/gravity stop
 * the mechanism at that boundary. No homing, angle wrapping or zeroing occurs. */
#define CG_CAL_ABSOLUTE_TRAVEL_RAD 1.5707963267948966f
/* Zero-current BASELINE only: about 2.6 position feedback counts.
 * Never re-anchor on a noisy speed sample; accumulated drift still aborts. */
#define CG_CAL_BASELINE_DRIFT_RAD 0.001f
#define CG_CAL_BASELINE_RETRY_MS 1000U

typedef enum {
    CG_CAL_PHASE_IDLE, CG_CAL_PHASE_BASELINE, CG_CAL_PHASE_PULSE,
    CG_CAL_PHASE_BRAKE, CG_CAL_PHASE_COAST, CG_CAL_PHASE_STOPPING,
    CG_CAL_PHASE_DONE, CG_CAL_PHASE_FAULT
} CgCalPhase;

typedef enum {
    CG_CAL_FAULT_NONE, CG_CAL_FAULT_CONFIG, CG_CAL_FAULT_FEEDBACK,
    CG_CAL_FAULT_TIMING, CG_CAL_FAULT_MOTOR, CG_CAL_FAULT_POSITION,
    CG_CAL_FAULT_SPEED, CG_CAL_FAULT_TEMPERATURE, CG_CAL_FAULT_TX,
    CG_CAL_FAULT_LOG_OVERFLOW, CG_CAL_FAULT_REQUESTED_STOP,
    CG_CAL_FAULT_BASELINE_MOTION, CG_CAL_FAULT_BRAKE_TIMEOUT,
    CG_CAL_FAULT_STOP_TIMEOUT, CG_CAL_FAULT_BUS
} CgCalFault;

typedef struct {
    bool armed;                 /* Explicit operator opt-in; default false. */
    float pulse_current_a;      /* Signed pulse [A], nonzero, default NAN.
                                 * One sign per run; no auto repeat/increase. */
    float brake_current_a;      /* Positive magnitude [A], default NAN.
                                 * The applied sign is opposite the pulse. */
    float current_limit_a;      /* Approved mechanism current ceiling [A], NAN.
                                 * Both magnitudes must already fit; no clipping. */
    float temp_trip_c;          /* Approved motor temperature trip [deg C], NAN. */
    float speed_trip_rad_s;     /* Approved measured speed ceiling [rad/s], NAN. */
    float stationary_speed_rad_s; /* Start/settle/brake-zero threshold [rad/s],
                                  * default 0.02, greater than feedback noise. */
    float travel_soft_rad;      /* Trip at this displacement from pinned origin
                                 * [rad]; default 10 deg. This is not a target. */
    float travel_hard_rad;      /* Additional trip [rad], default 80 deg;
                                 * soft < hard < fixed 90 deg, never equal. */
    float guard_margin_rad;    /* Reserve within soft trip [rad], default 2 deg.
                                 * Increase for motion/encoder uncertainty. */
    uint32_t guard_lookahead_ms; /* Constant-velocity lookahead [ms], default 100,
                                 * plus feedback age; NOT a braking model. */
    float position_jump_rad;   /* Extra accepted encoder delta [rad] on top of
                                 * speed_trip * RX dt; default 0.01 rad. */
    uint32_t baseline_ms;      /* Continuous quiet observation [ms], default 200.
                                 * Speed spikes restart this zero-current window;
                                 * fixed position drift/total wait bounds apply. */
    uint32_t pulse_ms;         /* One current pulse [ms], default 200, 30..500. */
    uint32_t brake_ms;         /* Max opposite-current interval [ms], default
                                 * 200, 30..500; nonzero speed at end faults. */
    uint32_t settle_ms;        /* Zero-current observation [ms], default 100.
                                 * Movement above quiet threshold faults. */
    uint32_t feedback_timeout_ms; /* Max feedback age [ms], default 30. */
    uint32_t max_step_ms;      /* Max caller interval [ms], default 20.
                                 * Step must run periodically even without RX. */
} CgCalConfig;

typedef struct {
    float q_rad, v_rad_s, temp_c;
    uint32_t rx_ms, rx_sequence;
    bool online, motor_fault;
} CgCalFeedback;

typedef struct {
    bool write_current, stop;
    float current_a;
    uint32_t generation;       /* Identifies the command passed to commit. */
} CgCalOutput;

typedef struct {
    CgCalConfig config;
    CgCalPhase phase;
    CgCalFault fault;
    float initial_position_rad; /* Immutable from start to terminal state. */
    float last_current_a;      /* Last successfully QUEUED command, not Iq. */
    float previous_q_rad, previous_v_rad_s, previous_temp_c, brake_entry_v_rad_s;
    uint32_t phase_ms, last_step_ms, last_rx_ms, last_rx_sequence;
    uint32_t command_generation;
    float baseline_position_rad;
    uint32_t baseline_started_ms;
    CgCalOutput pending_output;
    bool pending_command;
} CgCal;

void cg_cal_config_defaults(CgCalConfig *config);
bool cg_cal_config_valid(const CgCalConfig *config);
void cg_cal_init(CgCal *cal);

/* Call only after the adapter has verified STOP, current-mode readback, zero
 * command and ENABLE, with a fresh stationary sample. origin_rad MUST be the
 * pre-drive origin retained by the adapter (not re-zeroed or re-captured here).
 * To perform a new trial, explicit adapter/operator re-arm is required. */
bool cg_cal_start(CgCal *cal, const CgCalConfig *config, float origin_rad,
                  const CgCalFeedback *feedback, uint32_t now_ms);

/* Pure state machine; never performs CAN I/O. stop takes precedence over
 * write_current. The adapter must retry STOP and independently confirm fresh
 * Reset + stationary feedback. No more drive output after a fault or DONE. */
CgCalOutput cg_cal_step(CgCal *cal, const CgCalFeedback *feedback, uint32_t now_ms);

/* Call for EVERY write_current output before next step. tx_ok means queued,
 * not measured motor Iq. Failure/missing commit immediately latches TX fault;
 * the adapter must request STOP in that same cycle after a failed commit. */
bool cg_cal_commit(CgCal *cal, const CgCalOutput *output, bool tx_ok);
void cg_cal_abort(CgCal *cal, CgCalFault fault);
void cg_cal_confirm_stopped(CgCal *cal);
const char *cg_cal_phase_name(CgCalPhase phase);
const char *cg_cal_fault_name(CgCalFault fault);

#endif
