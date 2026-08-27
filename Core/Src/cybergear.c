#include "cybergear.h"

#include <string.h>

#define CYBERGEAR_POSITION_MIN_RAD        (-12.5f)
#define CYBERGEAR_POSITION_MAX_RAD        (12.5f)
#define CYBERGEAR_VELOCITY_MIN_RAD_S      (-30.0f)
#define CYBERGEAR_VELOCITY_MAX_RAD_S      (30.0f)
#define CYBERGEAR_KP_MIN                  (0.0f)
#define CYBERGEAR_KP_MAX                  (500.0f)
#define CYBERGEAR_KD_MIN                  (0.0f)
#define CYBERGEAR_KD_MAX                  (5.0f)
#define CYBERGEAR_TORQUE_MIN_NM           (-12.0f)
#define CYBERGEAR_TORQUE_MAX_NM           (12.0f)

static uint32_t cybergear_make_can_id(
	CyberGearCommunicationType communication_type,
	uint16_t data_area_2,
	uint8_t destination_id
)
{
	return ((uint32_t)(communication_type & 0x1fU) << 24) |
		((uint32_t)data_area_2 << 8) |
		destination_id;
}

static uint8_t cybergear_get_communication_type(uint32_t can_id)
{
	return (uint8_t)((can_id >> 24) & 0x1fU);
}

static uint8_t cybergear_get_destination_id(uint32_t can_id)
{
	return (uint8_t)(can_id & 0xffU);
}

static uint16_t cybergear_get_data_area_2(uint32_t can_id)
{
	return (uint16_t)((can_id >> 8) & 0xffffU);
}

static uint16_t cybergear_float_to_u16(
	float value,
	float min_value,
	float max_value
)
{
	if (value < min_value)
	{
		value = min_value;
	}
	else if (value > max_value)
	{
		value = max_value;
	}

	return (uint16_t)(
		(value - min_value) * 65535.0f / (max_value - min_value)
	);
}

static float cybergear_u16_to_float(
	uint16_t raw,
	float min_value,
	float max_value
)
{
	return min_value +
		(float)raw * (max_value - min_value) / 65535.0f;
}

static void cybergear_write_u16_be(uint8_t *destination, uint16_t value)
{
	destination[0] = (uint8_t)(value >> 8);
	destination[1] = (uint8_t)(value & 0xffU);
}

static uint16_t cybergear_read_u16_be(const uint8_t *source)
{
	return (uint16_t)(((uint16_t)source[0] << 8) | source[1]);
}

static bool cybergear_send(
	CyberGearMotor *motor,
	uint32_t can_id,
	uint8_t *tx_data
)
{
	if (motor == NULL || motor->hfdcan == NULL || tx_data == NULL)
	{
		return false;
	}

	motor->tx_header.Identifier = can_id;

	return HAL_FDCAN_AddMessageToTxFifoQ(
		motor->hfdcan,
		&motor->tx_header,
		tx_data
	) == HAL_OK;
}

static bool cybergear_send_empty_command(
	CyberGearMotor *motor,
	CyberGearCommunicationType communication_type,
	uint8_t first_data_byte
)
{
	if (motor == NULL)
	{
		return false;
	}

	uint8_t tx_data[8] = {0};
	tx_data[0] = first_data_byte;

	const uint32_t can_id = cybergear_make_can_id(
		communication_type,
		motor->master_id,
		motor->motor_id
	);

	return cybergear_send(motor, can_id, tx_data);
}

static bool cybergear_write_u8(
	CyberGearMotor *motor,
	uint16_t index,
	uint8_t value
)
{
	if (motor == NULL)
	{
		return false;
	}

	uint8_t tx_data[8] = {0};
	tx_data[0] = (uint8_t)(index & 0xffU);
	tx_data[1] = (uint8_t)(index >> 8);
	tx_data[4] = value;

	const uint32_t can_id = cybergear_make_can_id(
		CYBERGEAR_COMM_WRITE_PARAMETER,
		motor->master_id,
		motor->motor_id
	);

	return cybergear_send(motor, can_id, tx_data);
}

bool cybergear_init(
	CyberGearMotor *motor,
	FDCAN_HandleTypeDef *hfdcan,
	uint8_t motor_id,
	uint8_t master_id
)
{
	if (motor == NULL || hfdcan == NULL)
	{
		return false;
	}

	memset(motor, 0, sizeof(*motor));

	motor->hfdcan = hfdcan;
	motor->motor_id = motor_id;
	motor->master_id = master_id;
	motor->run_mode = CYBERGEAR_RUN_MODE_OPERATION;
	motor->feedback.motor_id = motor_id;

	motor->tx_header.IdType = FDCAN_EXTENDED_ID;
	motor->tx_header.TxFrameType = FDCAN_DATA_FRAME;
	motor->tx_header.DataLength = FDCAN_DLC_BYTES_8;
	motor->tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	motor->tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	motor->tx_header.FDFormat = FDCAN_CLASSIC_CAN;
	motor->tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	motor->tx_header.MessageMarker = 0;

	return true;
}

bool cybergear_enable(CyberGearMotor *motor)
{
	return cybergear_send_empty_command(
		motor,
		CYBERGEAR_COMM_ENABLE,
		0
	);
}

bool cybergear_stop(CyberGearMotor *motor)
{
	return cybergear_send_empty_command(
		motor,
		CYBERGEAR_COMM_STOP,
		0
	);
}

bool cybergear_clear_fault(CyberGearMotor *motor)
{
	return cybergear_send_empty_command(
		motor,
		CYBERGEAR_COMM_STOP,
		1
	);
}

bool cybergear_set_zero(CyberGearMotor *motor)
{
	return cybergear_send_empty_command(
		motor,
		CYBERGEAR_COMM_SET_ZERO,
		1
	);
}

bool cybergear_control(
	CyberGearMotor *motor,
	float position_rad,
	float velocity_rad_s,
	float kp,
	float kd,
	float feedforward_torque_nm
)
{
	if (motor == NULL)
	{
		return false;
	}

	const uint16_t position_raw = cybergear_float_to_u16(
		position_rad,
		CYBERGEAR_POSITION_MIN_RAD,
		CYBERGEAR_POSITION_MAX_RAD
	);
	const uint16_t velocity_raw = cybergear_float_to_u16(
		velocity_rad_s,
		CYBERGEAR_VELOCITY_MIN_RAD_S,
		CYBERGEAR_VELOCITY_MAX_RAD_S
	);
	const uint16_t kp_raw = cybergear_float_to_u16(
		kp,
		CYBERGEAR_KP_MIN,
		CYBERGEAR_KP_MAX
	);
	const uint16_t kd_raw = cybergear_float_to_u16(
		kd,
		CYBERGEAR_KD_MIN,
		CYBERGEAR_KD_MAX
	);
	const uint16_t torque_raw = cybergear_float_to_u16(
		feedforward_torque_nm,
		CYBERGEAR_TORQUE_MIN_NM,
		CYBERGEAR_TORQUE_MAX_NM
	);

	uint8_t tx_data[8];
	cybergear_write_u16_be(&tx_data[0], position_raw);
	cybergear_write_u16_be(&tx_data[2], velocity_raw);
	cybergear_write_u16_be(&tx_data[4], kp_raw);
	cybergear_write_u16_be(&tx_data[6], kd_raw);

	const uint32_t can_id = cybergear_make_can_id(
		CYBERGEAR_COMM_MOTION_CONTROL,
		torque_raw,
		motor->motor_id
	);

	return cybergear_send(motor, can_id, tx_data);
}

bool cybergear_set_run_mode(
	CyberGearMotor *motor,
	CyberGearRunMode mode
)
{
	if (motor == NULL ||
		mode < CYBERGEAR_RUN_MODE_OPERATION ||
		mode > CYBERGEAR_RUN_MODE_CURRENT)
	{
		return false;
	}

	if (!cybergear_write_u8(
		motor,
		CYBERGEAR_PARAM_RUN_MODE,
		(uint8_t)mode
	))
	{
		return false;
	}

	motor->run_mode = mode;
	return true;
}

bool cybergear_write_float(
	CyberGearMotor *motor,
	uint16_t index,
	float value
)
{
	if (motor == NULL)
	{
		return false;
	}

	uint8_t tx_data[8] = {0};
	tx_data[0] = (uint8_t)(index & 0xffU);
	tx_data[1] = (uint8_t)(index >> 8);

	uint32_t raw_value;
	memcpy(&raw_value, &value, sizeof(raw_value));
	tx_data[4] = (uint8_t)(raw_value & 0xffU);
	tx_data[5] = (uint8_t)((raw_value >> 8) & 0xffU);
	tx_data[6] = (uint8_t)((raw_value >> 16) & 0xffU);
	tx_data[7] = (uint8_t)((raw_value >> 24) & 0xffU);

	const uint32_t can_id = cybergear_make_can_id(
		CYBERGEAR_COMM_WRITE_PARAMETER,
		motor->master_id,
		motor->motor_id
	);

	return cybergear_send(motor, can_id, tx_data);
}

bool cybergear_set_position(
	CyberGearMotor *motor,
	float position_rad
)
{
	return cybergear_write_float(
		motor,
		CYBERGEAR_PARAM_POSITION_REF,
		position_rad
	);
}

bool cybergear_set_velocity(
	CyberGearMotor *motor,
	float velocity_rad_s
)
{
	return cybergear_write_float(
		motor,
		CYBERGEAR_PARAM_SPEED_REF,
		velocity_rad_s
	);
}

bool cybergear_set_current(
	CyberGearMotor *motor,
	float current_a
)
{
	return cybergear_write_float(
		motor,
		CYBERGEAR_PARAM_IQ_REF,
		current_a
	);
}

bool cybergear_parse_feedback(
	CyberGearMotor *motor,
	uint32_t can_id,
	const uint8_t *rx_data
)
{
	if (motor == NULL || rx_data == NULL ||
		cybergear_get_communication_type(can_id) != CYBERGEAR_COMM_FEEDBACK ||
		cybergear_get_destination_id(can_id) != motor->master_id)
	{
		return false;
	}

	const uint16_t data_area_2 = cybergear_get_data_area_2(can_id);
	const uint8_t source_motor_id = (uint8_t)(data_area_2 & 0xffU);

	if (source_motor_id != motor->motor_id)
	{
		return false;
	}

	const uint16_t position_raw = cybergear_read_u16_be(&rx_data[0]);
	const uint16_t velocity_raw = cybergear_read_u16_be(&rx_data[2]);
	const uint16_t torque_raw = cybergear_read_u16_be(&rx_data[4]);
	const uint16_t temperature_raw = cybergear_read_u16_be(&rx_data[6]);

	motor->feedback.position_rad = cybergear_u16_to_float(
		position_raw,
		CYBERGEAR_POSITION_MIN_RAD,
		CYBERGEAR_POSITION_MAX_RAD
	);
	motor->feedback.velocity_rad_s = cybergear_u16_to_float(
		velocity_raw,
		CYBERGEAR_VELOCITY_MIN_RAD_S,
		CYBERGEAR_VELOCITY_MAX_RAD_S
	);
	motor->feedback.torque_nm = cybergear_u16_to_float(
		torque_raw,
		CYBERGEAR_TORQUE_MIN_NM,
		CYBERGEAR_TORQUE_MAX_NM
	);
	motor->feedback.temperature_c = (float)temperature_raw / 10.0f;
	motor->feedback.motor_id = source_motor_id;
	motor->feedback.fault_flags = (uint8_t)((data_area_2 >> 8) & 0x3fU);
	motor->feedback.mode = (uint8_t)((data_area_2 >> 14) & 0x03U);
	motor->feedback.last_received_ms = HAL_GetTick();
	motor->feedback.online = true;

	return true;
}
