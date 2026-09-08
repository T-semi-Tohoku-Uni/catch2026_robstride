#ifndef __CYBERGEAR_H
#define __CYBERGEAR_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "cybergear_controller.h"

typedef enum
{
	CYBERGEAR_COMM_GET_DEVICE_ID = 0,
	CYBERGEAR_COMM_MOTION_CONTROL = 1,
	CYBERGEAR_COMM_FEEDBACK = 2,
	CYBERGEAR_COMM_ENABLE = 3,
	CYBERGEAR_COMM_STOP = 4,
	CYBERGEAR_COMM_SET_ZERO = 6,
	CYBERGEAR_COMM_READ_PARAMETER = 17,
	CYBERGEAR_COMM_WRITE_PARAMETER = 18,
	CYBERGEAR_COMM_FAULT_FEEDBACK = 21
} CyberGearCommunicationType;

typedef enum
{
	CYBERGEAR_RUN_MODE_OPERATION = 0,
	CYBERGEAR_RUN_MODE_POSITION = 1,
	CYBERGEAR_RUN_MODE_SPEED = 2,
	CYBERGEAR_RUN_MODE_CURRENT = 3
} CyberGearRunMode;

typedef enum
{
	CYBERGEAR_PARAM_RUN_MODE = 0x7005,
	CYBERGEAR_PARAM_IQ_REF = 0x7006,
	CYBERGEAR_PARAM_SPEED_REF = 0x700A,
	CYBERGEAR_PARAM_LIMIT_TORQUE = 0x700B,
	CYBERGEAR_PARAM_POSITION_REF = 0x7016,
	CYBERGEAR_PARAM_LIMIT_SPEED = 0x7017,
	CYBERGEAR_PARAM_LIMIT_CURRENT = 0x7018
} CyberGearParameter;

typedef struct
{
	float position_rad;
	float velocity_rad_s;
	float torque_nm;
	float temperature_c;

	uint8_t motor_id;
	uint8_t mode;
	uint8_t fault_flags;

	uint32_t last_received_ms;
	uint32_t rx_sequence;
	bool online;
} CyberGearFeedback;

typedef struct
{
	float position_rad;
	float velocity_rad_s;
	float disturbance_rad_s2;
	float reference_rad;
	float current_a;
	uint32_t last_update_ms;
	uint32_t last_feedback_ms;
	bool initialized;
	bool active;
} CyberGearAdrcState;

typedef enum { CG_DISARMED, CG_STOP_WAIT, CG_MODE_WRITE, CG_MODE_READBACK,
    CG_ZERO_COMMAND, CG_ENABLE_WAIT, CG_OBSERVER_WARMUP, CG_RUN,
    CG_FAULT_LATCHED } CGState;
typedef enum { CG_FAULT_NONE, CG_FAULT_CONFIG, CG_FAULT_TX, CG_FAULT_TIMEOUT,
    CG_FAULT_MODE, CG_FAULT_INPUT, CG_FAULT_FEEDBACK, CG_FAULT_DT,
    CG_FAULT_RANGE, CG_FAULT_TEMPERATURE, CG_FAULT_SPEED, CG_FAULT_JUMP,
    CG_FAULT_STALL, CG_FAULT_PLAN, CG_FAULT_BUS, CG_FAULT_DEVICE } CGFault;
typedef struct {
    uint32_t timestamp_ms, rx_sequence, rx_age_ms, dt_ms, tx_queued, tx_failed;
    uint32_t fifo_free, stop_attempts, state_since_ms, last_stop_ms, baseline_sequence;
    uint32_t observer_timestamp_ms, last_queued_ms, stall_since_ms, control_cycles;
    uint32_t tec, rec, busoff, log_dropped, max_execution_cycles;
    uint32_t saturation_since_ms;
    float last_queued_current, applied_current_estimate, stall_position, last_position;
    CGState state;
    CGFault fault;
    bool stop_requested, stop_queued, reset_confirmed, mechanically_stationary;
    bool applied_current_valid;
    bool saturation_active;
} CGDiagnostics;
typedef struct {
    CGDiagnostics diagnostics;
    CGController controller;
    CGReference reference;
    float target, measured_position, feedback_velocity;
} CGLog;

typedef struct
{
	FDCAN_HandleTypeDef *hfdcan;
	uint8_t motor_id;
	uint8_t master_id;
	CyberGearRunMode run_mode;

	FDCAN_TxHeaderTypeDef tx_header;
	CyberGearFeedback feedback;
	CyberGearAdrcState adrc;
	CGConfig config;
	CGTrajectory trajectory;
	CGController controller;
	CGDiagnostics diagnostics;
	uint32_t consumed_sequence, mode_sequence, warmup_samples;
	uint8_t confirmed_mode;
	bool internal_owner;
	bool state_command_queued;
	CGLog logs[16];
	volatile uint32_t log_head, log_tail;
} CyberGearMotor;

bool cybergear_dispatch(CyberGearMotor *motor, const FDCAN_RxHeaderTypeDef *header,
    const uint8_t *data);
bool cybergear_log_pop(CyberGearMotor *motor, CGLog *log);
/* Internal driver/state-machine entry; preserves the first reason until re-init. */
void cybergear_latch_fault(CyberGearMotor *motor, CGFault reason, uint32_t now);

bool cybergear_init(
	CyberGearMotor *motor,
	FDCAN_HandleTypeDef *hfdcan,
	uint8_t motor_id,
	uint8_t master_id
);

bool cybergear_enable(CyberGearMotor *motor);
bool cybergear_stop(CyberGearMotor *motor);
bool cybergear_clear_fault(CyberGearMotor *motor);
bool cybergear_set_zero(CyberGearMotor *motor);

bool cybergear_control(
	CyberGearMotor *motor,
	float position_rad,
	float velocity_rad_s,
	float kp,
	float kd,
	float feedforward_torque_nm
);

bool cybergear_set_run_mode(
	CyberGearMotor *motor,
	CyberGearRunMode mode
);

bool cybergear_write_float(
	CyberGearMotor *motor,
	uint16_t index,
	float value
);

bool cybergear_set_position(
	CyberGearMotor *motor,
	float position_rad
);

bool cybergear_set_velocity(
	CyberGearMotor *motor,
	float velocity_rad_s
);

bool cybergear_set_current(
	CyberGearMotor *motor,
	float current_a
);

bool cybergear_start_position_adrc(CyberGearMotor *motor);
bool cybergear_control_position_adrc(CyberGearMotor *motor, float position_rad);

bool cybergear_parse_feedback(
	CyberGearMotor *motor,
	uint32_t can_id,
	const uint8_t *rx_data
);

#endif
