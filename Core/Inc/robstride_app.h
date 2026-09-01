#ifndef __ROBSTRIDE__APP_H
#define __ROBSTRIDE__APP_H

#include <stdbool.h>
#include <string.h>
#include "main.h"

typedef enum
{
	POSITION_PP  = 1,
	VELOCITY     = 2,
	CURRENT      = 3,
	POSITION_CSP = 5
} RobstrideRunMode;

typedef enum
{
	FeedbackId = 0x02,
	EnableId = 0x03,
	StopId = 0x04,
	ResetPosId = 0x06
} RobstrideCanId;

typedef enum
{
	RUN_MODE = 0x7005,
	IQ_REF = 0x7006,
	SPEED_REF = 0x700A,
	LIMIT_TORQUE = 0x700B,

	POSITION_REF = 0x7016,
	LIMIT_SPEED = 0x7017,
	LIMIT_CURRENT = 0x7018,

	PP_VELOCITY_MAX = 0x7024,
	PP_ACCELERATION = 0x7025
} RobstrideParameter;

typedef struct
{
	float position_rad;
	float velocity_rps;
	float torqe_nm;
	float temperature_c;

	uint8_t motor_id;
	uint8_t mode;
	uint8_t fault_flags;

	uint32_t last_leceived_ms;
	bool online;
} RobstrideFeedback;

typedef struct
{
	uint8_t motor_id;
	uint8_t host_id;

	RobstrideFeedback feedback;
	RobstrideRunMode run_mode;

	FDCAN_TxHeaderTypeDef txheader;
} RobstrideMotor;

/* id 関連 */
uint32_t robstride_make_can_id(
	uint8_t communication_type,
	uint16_t data_area_2,
	uint8_t destination_id
);

uint8_t robstride_get_communication_type(uint32_t id);
uint8_t robstride_get_destination_id(uint32_t id);
uint16_t robstride_get_area_2(uint32_t id);

/* endian convert */
void convert_u16_to_u8_le(uint8_t *dst, uint16_t value);
void convert_u32_to_u8_le(uint8_t *dst, uint32_t value);
void convert_f32_to_u8_le(uint8_t *dst, float value);

uint16_t convert_u8_u16_le(const uint8_t *src);
uint32_t convert_u8_u32_le(const uint8_t *src);
float convert_u8_f32_le(const uint8_t *src);
uint16_t convert_u8_u16_be(const uint8_t *src);

uint16_t robstride_float_to_u16(
	float value,
	float min_value,
	float max_value
);
float robstride_u16_to_float(
	uint16_t raw,
	float min_value,
	float max_value
);

/* robstride app */
void robstride_delay(void);
bool send_robstride(RobstrideMotor *motor, uint32_t id, uint8_t *txdata);
bool robstride_enable(RobstrideMotor *motor);
bool robstride_stop(RobstrideMotor *motor);
bool robstride_clear_fault(RobstrideMotor *motor);

void robstride_parse_feedback(
	uint32_t canid,
	const uint8_t *rxdata,
	RobstrideFeedback *feedback
);

/* parameter raw func */
bool robstride_write_parameter_raw(
    RobstrideMotor *motor,
    uint16_t index,
    const uint8_t *value,
	uint8_t value_size
);
bool robstride_write_u8(
    RobstrideMotor *motor,
    uint16_t index,
    uint8_t value
);
bool robstride_write_u16(
    RobstrideMotor *motor,
    uint16_t index,
    uint16_t value
);
bool robstride_write_u32(
    RobstrideMotor *motor,
    uint16_t index,
    uint32_t value
);
bool robstride_write_float(
    RobstrideMotor *motor,
    uint16_t index,
    float value
);

/* parameter set func */
bool robstride_set_current(
    RobstrideMotor *motor,
    float current_a
);
bool robstride_set_velocity(
    RobstrideMotor *motor,
    float velocity_rad_s
);
bool robstride_set_position(
    RobstrideMotor *motor,
    float position_rad
);
bool robstride_set_current_limit(
    RobstrideMotor *motor,
    float current_limit_a
);
bool robstride_set_speed_limit(
    RobstrideMotor *motor,
    float speed_limit_rad_s
);
bool robstride_set_pp_velocity_max(
    RobstrideMotor *motor,
    float velocity_max_rad_s
);
bool robstride_set_pp_acceleration(
    RobstrideMotor *motor,
    float acceleration_rad_s2
);

/* mode */
bool robstride_set_run_mode(
	RobstrideMotor *motor,
	RobstrideRunMode mode
);
bool robstride_start_current_mode(
	RobstrideMotor *motor
);
bool robstride_start_velocity_mode(
	RobstrideMotor *motor,
	float current_limit_a
);
bool robstride_start_position_pp_mode(
    RobstrideMotor *motor,
    float velocity_max_rad_s,
    float acceleration_rad_s2,
    float current_limit_a
);
bool robstride_start_position_csp_mode(
	RobstrideMotor *motor,
	float speed_limit_rad_s,
	float current_limit_a
);

bool robstride_set_zero(RobstrideMotor *motor);

#endif
