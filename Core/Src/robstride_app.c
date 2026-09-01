#include "robstride_app.h"

#include "main.h"

extern FDCAN_HandleTypeDef hfdcan3;

/* id 関連 */

uint32_t robstride_make_can_id(
	uint8_t communication_type,
	uint16_t data_area_2,
	uint8_t destination_id
)
{
	uint32_t can_id = 
		((uint32_t)(communication_type & 0x1f) << 24) |
		((uint32_t)data_area_2 << 8) |
		destination_id;

	return can_id;
}

uint8_t robstride_get_communication_type(uint32_t id)
{
	return (uint8_t)((id >> 24) & 0x1f);
}

uint8_t robstride_get_destination_id(uint32_t id)
{
	return (uint8_t)(id & 0xff);
}

uint16_t robstride_get_area_2(uint32_t id)
{
	return (uint16_t)((id >> 8) & 0xffff);
}

/* endian convert */

void convert_u16_to_u8_le(uint8_t *dst, uint16_t value)
{
	dst[0] = (uint8_t)(value & 0xff);
	dst[1] = (uint8_t)(value >> 8);
}

void convert_u32_to_u8_le(uint8_t *dst, uint32_t value)
{
	dst[0] = (uint8_t)(value & 0xff);
	dst[1] = (uint8_t)(value >> 8);
	dst[2] = (uint8_t)(value >> 16);
	dst[3] = (uint8_t)(value >> 24);
}

void convert_f32_to_u8_le(uint8_t *dst, float value)
{
	union f32_u32
	{
		float value_f32;
		uint32_t value_u32;
	};
	union f32_u32 convert_union;
	convert_union.value_f32 = value;

	convert_u32_to_u8_le(dst, convert_union.value_u32);
}

uint16_t convert_u8_u16_le(const uint8_t *src)
{
	uint16_t dst = (uint16_t)(((uint16_t)src[1] << 8) | src[0]);

	return dst;
}

uint32_t convert_u8_u32_le(const uint8_t *src)
{
	uint32_t dst = (uint32_t)(((uint32_t)src[3] << 24) | ((uint32_t)src[2] << 16) | ((uint32_t)src[1] << 8) | src[0]);

	return dst;
}

float convert_u8_f32_le(const uint8_t *src)
{
	union f32_u32
	{
		float value_f32;
		uint32_t value_u32;
	};
	union f32_u32 convert_union;
	convert_union.value_u32 = convert_u8_u32_le(src);

	return convert_union.value_f32;
}

uint16_t convert_u8_u16_be(const uint8_t *src)
{
	uint16_t dst = (uint16_t)(((uint16_t)src[0] << 8) | src[1]);

	return dst;
}

uint16_t robstride_float_to_u16(
	float value,
	float min_value,
	float max_value
)
{
	if (value < min_value) value = min_value;
	if (value > max_value) value = max_value;

	const float normalized = (value - min_value) / (max_value - min_value);
	return (uint16_t)(normalized * 65535.0 + 0.5);
}

float robstride_u16_to_float(
	uint16_t raw,
	float min_value,
	float max_value
)
{
	return min_value + (float)raw * (max_value - min_value) / 65535.0f;
}

/* robstride app */

void robstride_delay(void)
{
	HAL_Delay(10);
}

bool send_robstride(RobstrideMotor *motor, uint32_t id, uint8_t *txdata)
{
	motor->txheader.Identifier = id;
	if (HAL_OK!= HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &motor->txheader, txdata))
	{
		return false;
	}

	return true;
}

bool robstride_enable(RobstrideMotor *motor)
{
	uint8_t data[8] = {0};
	const uint32_t id = robstride_make_can_id(
		EnableId,
		motor->host_id,
		motor->motor_id
	);

	if (!send_robstride(motor, id, data))
	{
		return false;
	}

	return true;
}

bool robstride_stop(RobstrideMotor *motor)
{
	uint8_t data[8] = {0};
	const uint32_t id = robstride_make_can_id(
		StopId,
		motor->host_id,
		motor->motor_id
	);

	if (!send_robstride(motor, id, data))
	{
		return false;
	}

	return true;
}

bool robstride_clear_fault(RobstrideMotor *motor)
{
	uint8_t data[8] = {0};
	data[0] = 1;
	const uint32_t id = robstride_make_can_id(
		StopId,
		motor->host_id,
		motor->motor_id
	);

	if (!send_robstride(motor, id, data))
	{
		return false;
	}

	return true;
}

void robstride_parse_feedback(
	uint32_t canid,
	const uint8_t *rxdata,
	RobstrideFeedback *feedback
)
{
	if (0x02 != robstride_get_communication_type(canid)) return;

	const uint16_t position_raw = convert_u8_u16_be(&rxdata[0]);
	const uint16_t velocity_raw = convert_u8_u16_be(&rxdata[2]);
	const uint16_t torque_raw = convert_u8_u16_be(&rxdata[4]);
	const uint16_t temp_raw = convert_u8_u16_be(&rxdata[6]);

	const uint16_t data2 = robstride_get_area_2(canid);

	feedback->motor_id = (uint8_t)(data2 & 0xff);
	feedback->fault_flags = (uint8_t)((data2 >> 8) & 0x3f);
	feedback->mode = (uint8_t)((data2 >> 14) & 0x03);

	feedback->position_rad = robstride_u16_to_float(position_raw, -12.57, 12.57);
	feedback->velocity_rps = robstride_u16_to_float(velocity_raw, -20.0f, 20.0f);
	feedback->torqe_nm = robstride_u16_to_float(torque_raw, -60.0f, 60.0f);
	feedback->temperature_c = (float)temp_raw / 10.0;
	feedback->last_leceived_ms = HAL_GetTick();
	feedback->online = true;
}

/* parameter raw func */
bool robstride_write_parameter_raw(
    RobstrideMotor *motor,
    uint16_t index,
    const uint8_t *value,
	uint8_t value_size
)
{
	if (NULL == value || value_size > 4) return false;

	uint8_t txdata[8] = {0};
	uint32_t canid = robstride_make_can_id(
		0x12,
		motor->host_id,
		motor->motor_id
	);
	convert_u16_to_u8_le(&txdata[0], index);
	memcpy(&txdata[4], value, value_size);

	return send_robstride(motor, canid, txdata);
}

bool robstride_write_u8(
    RobstrideMotor *motor,
    uint16_t index,
    uint8_t value
)
{
	uint8_t txdata[1] = {value};
	return robstride_write_parameter_raw(
		motor,
		index,
		txdata,
		sizeof(txdata)
	);
}

bool robstride_write_u16(
    RobstrideMotor *motor,
    uint16_t index,
    uint16_t value
)
{
	uint8_t txdata[2];
	convert_u16_to_u8_le(txdata, value);

	return robstride_write_parameter_raw(
		motor,
		index,
		txdata,
		sizeof(txdata)
	);
}

bool robstride_write_u32(
    RobstrideMotor *motor,
    uint16_t index,
    uint32_t value
)
{
	uint8_t txdata[4];
	convert_u32_to_u8_le(txdata, value);

	return robstride_write_parameter_raw(
		motor,
		index,
		txdata,
		sizeof(txdata)
	);
}

bool robstride_write_float(
    RobstrideMotor *motor,
    uint16_t index,
    float value
)
{
	uint8_t txdata[4];
	convert_f32_to_u8_le(txdata, value);

	return robstride_write_parameter_raw(
		motor,
		index,
		txdata,
		sizeof(txdata)
	);
}

/* parameter set func */
bool robstride_set_current(
    RobstrideMotor *motor,
    float current_a
)
{
	return robstride_write_float(
		motor,
		IQ_REF,
		current_a
	);
}

bool robstride_set_velocity(
    RobstrideMotor *motor,
    float velocity_rad_s
)
{
	return robstride_write_float(
		motor,
		SPEED_REF,
		velocity_rad_s
	);
}

bool robstride_set_position(
    RobstrideMotor *motor,
    float position_rad
)
{
	return robstride_write_float(
		motor,
		POSITION_REF,
		position_rad
	);
}

bool robstride_set_current_limit(
    RobstrideMotor *motor,
    float current_limit_a
)
{
	return robstride_write_float(
		motor,
		LIMIT_CURRENT,
		current_limit_a
	);
}
bool robstride_set_speed_limit(
    RobstrideMotor *motor,
    float speed_limit_rad_s
)
{
	return robstride_write_float(
		motor,
		LIMIT_SPEED,
		speed_limit_rad_s
	);
}

bool robstride_set_pp_velocity_max(
    RobstrideMotor *motor,
    float velocity_max_rad_s)
{
    return robstride_write_float(
        motor,
        PP_VELOCITY_MAX,
        velocity_max_rad_s
    );
}

bool robstride_set_pp_acceleration(
    RobstrideMotor *motor,
    float acceleration_rad_s2)
{
    return robstride_write_float(
        motor,
        PP_ACCELERATION,
        acceleration_rad_s2
    );
}

/* mode */
bool robstride_set_run_mode(
	RobstrideMotor *motor,
	RobstrideRunMode mode
)
{
	const uint8_t value = (uint8_t)mode;

	return robstride_write_u8(
		motor,
		RUN_MODE,
		value
	);
}

bool robstride_start_current_mode(
	RobstrideMotor *motor
)
{
	if (!robstride_stop(motor))
	{
		return false;
	}
	robstride_delay();

	if (!robstride_set_run_mode(
			motor,
			CURRENT
	))
	{
		return false;
	}
	robstride_delay();

	return robstride_enable(motor);
}

bool robstride_start_velocity_mode(
	RobstrideMotor *motor,
	float current_limit_a
)
{
	if (!robstride_stop(motor))
	{
		return false;
	}
	robstride_delay();
	if (!robstride_set_run_mode(
		motor,
		VELOCITY
	))
	{
		return false;
	}
	robstride_delay();

	if (!robstride_enable(motor))
	{
		return false;
	}
	robstride_delay();

	return robstride_set_current_limit(
		motor,
		current_limit_a
	);
}

bool robstride_start_position_pp_mode(
    RobstrideMotor *motor,
    float velocity_max_rad_s,
    float acceleration_rad_s2,
    float current_limit_a
)
{
	if (!robstride_stop(motor))
    {
        return false;
    }
	robstride_delay();

    if (!robstride_set_run_mode(
		motor,
		POSITION_PP
	))
    {
        return false;
    }
	robstride_delay();

    if (!robstride_enable(motor))
    {
        return false;
    }
	robstride_delay();

    if (!robstride_set_pp_velocity_max(
		motor,
		velocity_max_rad_s
	))
    {
        return false;
    }
	robstride_delay();

    if (!robstride_set_pp_acceleration(
		motor,
		acceleration_rad_s2
	))
    {
        return false;
    }
	robstride_delay();

    return robstride_set_current_limit(
        motor,
        current_limit_a
    );
}

bool robstride_start_position_csp_mode(
	RobstrideMotor *motor,
	float speed_limit_rad_s,
	float current_limit_a
)
{
	if (!robstride_stop(motor))
	{
		return false;
	}
	robstride_delay();

	if (!robstride_set_run_mode(
		motor,
		POSITION_CSP
	))
	{
		return false;
	}
	robstride_delay();

	if (!robstride_enable(motor))
	{
		return false;
	}
	robstride_delay();

	if (!robstride_set_speed_limit(
		motor,
		speed_limit_rad_s
	))
	{
		return false;
	}
	robstride_delay();

	return robstride_set_current_limit(
		motor,
		current_limit_a
	);
}

bool robstride_set_zero(RobstrideMotor *motor)
{
    uint8_t data[8] = {0};
    
    // Set Zero の通信タイプ（仮に0x03とする。仕様書に合わせてください）
    const uint8_t set_zero_comm_type = 0x03; 

    const uint32_t id = robstride_make_can_id(
        set_zero_comm_type, motor->host_id, motor->motor_id
    );

    if (!send_robstride(motor, id, data))
    {
        return false;
    }
    return true;
}
