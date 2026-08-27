#ifndef __CYBERGEAR_H
#define __CYBERGEAR_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

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
	bool online;
} CyberGearFeedback;

typedef struct
{
	FDCAN_HandleTypeDef *hfdcan;
	uint8_t motor_id;
	uint8_t master_id;
	CyberGearRunMode run_mode;

	FDCAN_TxHeaderTypeDef tx_header;
	CyberGearFeedback feedback;
} CyberGearMotor;

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

bool cybergear_parse_feedback(
	CyberGearMotor *motor,
	uint32_t can_id,
	const uint8_t *rx_data
);

#endif
