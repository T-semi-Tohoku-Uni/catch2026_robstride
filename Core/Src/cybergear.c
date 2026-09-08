#include "cybergear.h"

#include <math.h>

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

static void cybergear_latch_fault(CyberGearMotor *motor, CyberGearFault reason);

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
	/* Homing runs before managed cyclic control. A type-21 latch must also
	 * prevent that legacy path from issuing further drive/mode commands. */
	if (motor->motor_fault_sequence != 0U &&
		cybergear_get_communication_type(can_id) != CYBERGEAR_COMM_STOP) return false;

	motor->tx_header.Identifier = can_id;
	motor->tx_fifo_free = HAL_FDCAN_GetTxFifoFreeLevel(motor->hfdcan);
	const bool queued = HAL_FDCAN_AddMessageToTxFifoQ(
		motor->hfdcan,
		&motor->tx_header,
		tx_data
	) == HAL_OK;
	if (queued) motor->tx_queued++;
	else motor->tx_failed++;
	return queued;
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
	cybergear_config_defaults(&motor->config);
	motor->state = CG_STATE_OFF;

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
	if (motor == NULL || (motor->managed && !motor->internal_send) ||
		!cybergear_config_valid(&motor->config)) return false;
	return cybergear_send_empty_command(
		motor,
		CYBERGEAR_COMM_ENABLE,
		0
	);
}

bool cybergear_stop(CyberGearMotor *motor)
{
	if (motor == NULL) return false;
	if (motor->managed && !motor->internal_send) {
		cybergear_latch_fault(motor, CG_FAULT_REQUESTED_STOP);
		return true; /* Request accepted. STOP delivery is tracked separately. */
	}

	return cybergear_send_empty_command(
		motor,
		CYBERGEAR_COMM_STOP,
		0
	);
}

bool cybergear_clear_fault(CyberGearMotor *motor)
{
	if (motor == NULL || motor->managed) return false;

	return cybergear_send_empty_command(
		motor,
		CYBERGEAR_COMM_STOP,
		1
	);
}

bool cybergear_set_zero(CyberGearMotor *motor)
{
	if (motor == NULL || motor->managed) return false;

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
	if (motor == NULL || (motor->managed && !motor->internal_send) ||
		!isfinite(position_rad) || !isfinite(velocity_rad_s) || !isfinite(kp) ||
		!isfinite(kd) || !isfinite(feedforward_torque_nm))
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
	if (motor == NULL || (motor->managed && !motor->internal_send) ||
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
	if (motor == NULL || (motor->managed && !motor->internal_send) || !isfinite(value))
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
	motor->feedback.rx_sequence++;
	motor->feedback.online = true;

	return true;
}

bool cybergear_claim_calibration(CyberGearMotor *motor, const CgCalConfig *config)
{
    if (motor == NULL || motor->hfdcan == NULL || motor->managed ||
        motor->state != CG_STATE_OFF || !cg_cal_config_valid(config)) return false;
    motor->managed = true;
    motor->calibration_owned = true;
    return true;
}

bool cybergear_enable_calibration(CyberGearMotor *motor, const CgCalConfig *config)
{
    if (motor == NULL || !motor->calibration_owned || !motor->managed ||
        !motor->internal_send || !cg_cal_config_valid(config)) return false;
    return cybergear_send_empty_command(motor, CYBERGEAR_COMM_ENABLE, 0);
}

static bool positive(float x) { return isfinite(x) && x > 0.0f; }
static bool nonnegative(float x) { return isfinite(x) && x >= 0.0f; }
static uint32_t lock_state(void) { uint32_t mask = __get_PRIMASK(); __disable_irq(); return mask; }
static bool time_reached(uint32_t now, uint32_t when) { return (int32_t)(now - when) >= 0; }

void cybergear_config_defaults(CyberGearConfig *c)
{
    if (c == NULL) return;
    memset(c, 0, sizeof(*c));
    cybergear_controller_default_config(&c->controller);
    c->hardware_confirmed = CYBERGEAR_HARDWARE_CONFIRMED != 0;
    c->controller.period_ms = CYBERGEAR_USE_200_HZ ? 5U : 10U;
    c->controller.b0_initial = CYBERGEAR_B0_FIXED;
    c->controller.b0_min = CYBERGEAR_B0_MIN;
    c->controller.b0_max = CYBERGEAR_B0_MAX;
    c->controller.current_limit_a = CYBERGEAR_CURRENT_LIMIT_A;
    c->trajectory = (CgTrajectoryLimits){
        .position_min_rad = CYBERGEAR_SOFT_MIN_RAD,
        .position_max_rad = CYBERGEAR_SOFT_MAX_RAD,
        .velocity_max_rad_s = CYBERGEAR_TRAJECTORY_SPEED_RAD_S,
        .acceleration_max_rad_s2 = CYBERGEAR_TRAJECTORY_ACCEL_RAD_S2,
        .braking_max_rad_s2 = CYBERGEAR_TRAJECTORY_BRAKE_RAD_S2,
        .jerk_max_rad_s3 = CYBERGEAR_TRAJECTORY_JERK_RAD_S3,
        .duration_min_s = 0.1f, .duration_max_s = 60.0f,
        .target_tolerance_rad = 0.0001f, .search_iterations = 32U
    };
    c->hard_min_rad = CYBERGEAR_HARD_MIN_RAD;
    c->hard_max_rad = CYBERGEAR_HARD_MAX_RAD;
    c->speed_trip_rad_s = CYBERGEAR_SPEED_TRIP_RAD_S;
    c->temperature_trip_c = CYBERGEAR_TEMPERATURE_TRIP_C;
    c->position_jump_rad = 0.02f;
    c->tracking_error_rad = 0.15f;
    c->tracking_timeout_ms = 500U;
    c->saturation_timeout_ms = 1000U;
    c->stall_current_a = 0.8f * c->controller.current_limit_a;
    c->stall_progress_rad = 0.003f;
    c->stall_timeout_ms = 1000U;
    c->stationary_speed_rad_s = 0.03f;
    c->stationary_dwell_ms = 100U;
    c->startup_timeout_ms = 3000U;
    c->command_retry_ms = 50U;
    c->stop_timeout_ms = 1000U;
    c->stop_max_attempts = 20U;
    c->planner_lead_ms = 50U;
    c->planner_timeout_ms = 500U;
    c->brake_guaranteed_rad_s2 = CYBERGEAR_BRAKE_GUARANTEED_RAD_S2;
    c->outward_accel_rad_s2 = CYBERGEAR_OUTWARD_ACCEL_RAD_S2;
    c->stop_margin_rad = CYBERGEAR_STOP_MARGIN_RAD;
    c->transport_delay_ms = 100U;
    c->operation_kp = NAN;
    c->operation_kd = NAN;
    c->operation_torque_limit_nm = NAN;
    c->dynamics = (CyberGearDynamicsConfig){
        .fixed_b0 = c->controller.b0_initial,
        .b0_min = c->controller.b0_min, .b0_max = c->controller.b0_max,
        .b0_rate_limit = 1.0f, .posture_timeout_ms = 100U,
        .acceleration_current_a = c->controller.current_limit_a,
        .braking_current_a = c->controller.current_limit_a,
        .reserve_current_a = 0.5f * c->controller.current_limit_a,
        .current_slew_a_s = fminf(c->controller.current_rise_a_s, c->controller.current_fall_a_s),
        .reserve_slew_a_s = 2.5f,
        .acceleration_cap_rad_s2 = c->trajectory.acceleration_max_rad_s2,
        .braking_cap_rad_s2 = c->trajectory.braking_max_rad_s2,
        .jerk_cap_rad_s3 = c->trajectory.jerk_max_rad_s3
    };
}

bool cybergear_config_valid(const CyberGearConfig *c)
{
    if (c == NULL || !c->hardware_confirmed ||
        !cybergear_controller_config_valid(&c->controller) ||
        !cybergear_dynamics_config_valid(&c->dynamics)) return false;
    CgTrajectory check;
    CgTrajectoryPoint origin = {0};
    origin.q_rad = c->trajectory.position_min_rad;
    if (!cg_trajectory_reset(&check, &origin, &c->trajectory)) return false;
    return isfinite(c->hard_min_rad) && isfinite(c->hard_max_rad) &&
        c->hard_min_rad >= CYBERGEAR_POSITION_MIN_RAD &&
        c->hard_max_rad <= CYBERGEAR_POSITION_MAX_RAD &&
        c->hard_min_rad < c->trajectory.position_min_rad &&
        c->hard_max_rad > c->trajectory.position_max_rad &&
        positive(c->speed_trip_rad_s) && c->speed_trip_rad_s <= 30.0f &&
        c->speed_trip_rad_s > c->trajectory.velocity_max_rad_s &&
        positive(c->temperature_trip_c) && positive(c->position_jump_rad) &&
        positive(c->tracking_error_rad) && positive(c->stall_current_a) &&
        c->stall_current_a <= c->controller.current_limit_a && positive(c->stall_progress_rad) &&
        positive(c->stationary_speed_rad_s) && c->stationary_speed_rad_s < c->speed_trip_rad_s &&
        positive(c->brake_guaranteed_rad_s2) && nonnegative(c->outward_accel_rad_s2) &&
        positive(c->stop_margin_rad) && c->transport_delay_ms >= c->controller.feedback_timeout_ms &&
        c->transport_delay_ms < 10000U && c->tracking_timeout_ms > 0U && c->tracking_timeout_ms < 60000U &&
        c->saturation_timeout_ms > 0U && c->saturation_timeout_ms < 60000U &&
        c->stall_timeout_ms > 0U && c->stall_timeout_ms < 60000U &&
        c->stationary_dwell_ms > 0U && c->startup_timeout_ms > c->stationary_dwell_ms * 2U &&
        c->startup_timeout_ms < 60000U && c->command_retry_ms >= c->controller.period_ms &&
        c->stop_timeout_ms >= c->command_retry_ms && c->stop_timeout_ms < 60000U &&
        c->stop_max_attempts > 0U && c->stop_max_attempts <= 100U &&
        c->planner_lead_ms >= c->controller.period_ms * 2U &&
        c->planner_lead_ms % c->controller.period_ms == 0U &&
        c->planner_timeout_ms > c->planner_lead_ms && c->planner_timeout_ms < 60000U &&
        c->dynamics.fixed_b0 == c->controller.b0_initial &&
        c->dynamics.b0_min == c->controller.b0_min && c->dynamics.b0_max == c->controller.b0_max &&
        c->dynamics.acceleration_current_a <= c->controller.current_limit_a &&
        c->dynamics.braking_current_a <= c->controller.current_limit_a &&
        c->dynamics.reserve_current_a >= c->controller.disturbance_limit_a &&
        c->dynamics.current_slew_a_s <= fminf(c->controller.current_rise_a_s, c->controller.current_fall_a_s);
}

bool cybergear_configure(CyberGearMotor *m, const CyberGearConfig *c)
{
    if (m == NULL || m->managed || m->state != CG_STATE_OFF || !cybergear_config_valid(c)) return false;
    m->config = *c;
    return true;
}

bool cybergear_read_parameter(CyberGearMotor *m, uint16_t index)
{
    if (m == NULL || (m->managed && !m->internal_send)) return false;
    uint8_t data[8] = { (uint8_t)index, (uint8_t)(index >> 8), 0, 0, 0, 0, 0, 0 };
    return cybergear_send(m, cybergear_make_can_id(CYBERGEAR_COMM_READ_PARAMETER,
        m->master_id, m->motor_id), data);
}

bool cybergear_process_rx(CyberGearMotor *m, const FDCAN_RxHeaderTypeDef *h, const uint8_t *data)
{
    if (m == NULL || h == NULL || data == NULL || m->hfdcan == NULL ||
        h->IdType != FDCAN_EXTENDED_ID || h->RxFrameType != FDCAN_DATA_FRAME ||
        h->DataLength != FDCAN_DLC_BYTES_8 || h->Identifier > 0x1fffffffU) return false;
    const uint8_t type = cybergear_get_communication_type(h->Identifier);
    const uint8_t source = (uint8_t)(h->Identifier >> 8);
    const uint8_t destination = (uint8_t)h->Identifier;
    const bool addressed = source == m->motor_id && destination == m->master_id;
    /* Published type-21 documents disagree on direction. Accept either exact
     * known motor/host pair only for faults; retain raw payload, never decode
     * an undocumented bit layout or accept another motor's packet. */
    const bool reverse_fault = type == CYBERGEAR_COMM_FAULT_FEEDBACK &&
        source == m->master_id && destination == m->motor_id;
    if (!addressed && !reverse_fault) return false;
    if (type == CYBERGEAR_COMM_FEEDBACK) return cybergear_parse_feedback(m, h->Identifier, data);
    if (type == CYBERGEAR_COMM_READ_PARAMETER) {
        const uint16_t index = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
        if (index != CYBERGEAR_PARAM_RUN_MODE) return false;
        m->mode_read_sequence++;
        if (m->mode_read_pending && (m->state == CG_STATE_WAIT_MODE || m->calibration_owned) &&
            ((h->Identifier >> 16) & 0xffU) == 0U && data[2] == 0U && data[3] == 0U) {
            m->mode_read_value = data[4];
            m->mode_read_ms = HAL_GetTick();
            m->mode_read_valid = true;
        }
        return true;
    }
    if (type == CYBERGEAR_COMM_FAULT_FEEDBACK) {
        /* Preserve bytes: installed FW fault layout/endian has not been confirmed.
         * Any nonzero fault/warning payload latches; do not infer individual bits. */
        memcpy(m->fault_payload, data, 8);
        for (unsigned int i = 0; i < 8U; ++i) {
            if (data[i] != 0U) { m->motor_fault_sequence++; break; }
        }
        return true;
    }
    return false;
}

static void cybergear_latch_fault(CyberGearMotor *m, CyberGearFault reason)
{
    if (m->state == CG_STATE_STOPPING || m->state == CG_STATE_FAULT) return;
    m->fault = reason;
    m->state = CG_STATE_STOPPING;
    m->managed = true;
    m->stop_started_ms = HAL_GetTick();
    m->stop_rx_sequence = m->feedback.rx_sequence;
    m->stop_last_sequence = m->feedback.rx_sequence;
    m->stop_attempts = 0;
    m->stop_queued = false;
    m->reset_confirmed = false;
    m->stationary = false;
    m->quiet_active = false;
    m->prepared_ready = false;
    m->plan_generation++;
    cybergear_controller_invalidate_input(&m->controller);
}

static bool feedback_fresh(const CyberGearMotor *m, uint32_t now)
{
    return m->feedback.online && now - m->feedback.last_received_ms <= m->config.controller.feedback_timeout_ms;
}

static bool quiet_feedback(CyberGearMotor *m, uint32_t now, uint8_t mode)
{
    if (!feedback_fresh(m, now) || m->feedback.mode != mode ||
        fabsf(m->feedback.velocity_rad_s) > m->config.stationary_speed_rad_s) {
        m->quiet_active = false;
        return false;
    }
    if (m->feedback.rx_sequence == m->quiet_last_sequence) return false;
    m->quiet_last_sequence = m->feedback.rx_sequence;
    if (!m->quiet_active) { m->quiet_since_ms = now; m->quiet_active = true; }
    return now - m->quiet_since_ms >= m->config.stationary_dwell_ms;
}

static void service_stop(CyberGearMotor *m, uint32_t now)
{
    if (m->stop_queued && m->feedback.rx_sequence != m->stop_rx_sequence &&
        feedback_fresh(m, now) && m->feedback.mode == 0U) m->reset_confirmed = true;
    m->stationary = quiet_feedback(m, now, 0U);
    if (m->state == CG_STATE_FAULT) return;
    if ((m->reset_confirmed && m->stationary) || now - m->stop_started_ms >= m->config.stop_timeout_ms ||
        m->stop_attempts >= m->config.stop_max_attempts) {
        m->state = CG_STATE_FAULT;
        return;
    }
    if (m->stop_attempts == 0U || now - m->last_command_ms >= m->config.command_retry_ms) {
        m->internal_send = true;
        const bool queued = cybergear_stop(m);
        m->internal_send = false;
        m->stop_attempts++;
        m->last_command_ms = now;
        if (queued && !m->stop_queued) m->stop_rx_sequence = m->feedback.rx_sequence;
        m->stop_queued = m->stop_queued || queued;
    }
}

bool cybergear_reset_fault(CyberGearMotor *m)
{
    if (m == NULL || m->calibration_owned) return false;
    const uint32_t mask = lock_state();
    bool ready = m->state == CG_STATE_FAULT && m->reset_confirmed && m->stationary &&
        feedback_fresh(m, HAL_GetTick()) && m->feedback.mode == 0U && m->feedback.fault_flags == 0U &&
        fabsf(m->feedback.velocity_rad_s) <= m->config.stationary_speed_rad_s && !m->bus_off;
    if (ready) {
        m->state = CG_STATE_OFF;
        m->fault = CG_FAULT_NONE;
        m->managed = false;
        m->motor_fault_sequence = 0;
        memset(m->fault_payload, 0, sizeof(m->fault_payload));
    }
    __set_PRIMASK(mask);
    return ready;
}

bool cybergear_begin_position_control(CyberGearMotor *m, CyberGearRunMode mode)
{
    if (m == NULL || m->hfdcan == NULL || m->managed || m->state != CG_STATE_OFF) return false;
    if (!cybergear_config_valid(&m->config)) { cybergear_latch_fault(m, CG_FAULT_CONFIG); return false; }
    if (mode != CYBERGEAR_RUN_MODE_CURRENT && mode != CYBERGEAR_RUN_MODE_OPERATION) return false;
    if (mode == CYBERGEAR_RUN_MODE_OPERATION &&
        (!positive(m->config.operation_kp) || m->config.operation_kp > 500.0f ||
         !positive(m->config.operation_kd) || m->config.operation_kd > 5.0f ||
         !positive(m->config.operation_torque_limit_nm) || m->config.operation_torque_limit_nm > 12.0f)) return false;
    m->managed = true;
    m->state = CG_STATE_WAIT_STOP;
    m->requested_mode = mode;
    m->startup_ms = HAL_GetTick();
    m->state_ms = m->startup_ms;
    m->last_command_ms = m->startup_ms - m->config.command_retry_ms;
    m->stop_rx_sequence = m->feedback.rx_sequence;
    m->quiet_last_sequence = m->feedback.rx_sequence;
    m->quiet_active = false;
    m->mode_read_valid = false;
    m->mode_read_pending = false;
    m->stop_queued = false;
    m->plan_generation++;
    m->stop_attempts = 0U;
    return true;
}

static void enter_state(CyberGearMotor *m, CyberGearState state, uint32_t now)
{
    m->state = state;
    m->state_ms = now;
}

static bool begin_running(CyberGearMotor *m, uint32_t now)
{
    if (m->feedback.position_rad < m->config.trajectory.position_min_rad ||
        m->feedback.position_rad > m->config.trajectory.position_max_rad) return false;
    if (!cybergear_dynamics_init(&m->dynamics, &m->config.dynamics) ||
        !cybergear_dynamics_step(&m->dynamics, &m->posture, now,
            (float)m->config.controller.period_ms * 0.001f, &m->dynamics_output) ||
        m->dynamics_output.status == CYBERGEAR_MODEL_STALE ||
        m->dynamics_output.status == CYBERGEAR_MODEL_INVALID) return false;
    m->effective_limits = m->config.trajectory;
    m->effective_limits.acceleration_max_rad_s2 = fminf(m->effective_limits.acceleration_max_rad_s2, m->dynamics_output.acceleration_rad_s2);
    m->effective_limits.braking_max_rad_s2 = fminf(m->effective_limits.braking_max_rad_s2, m->dynamics_output.braking_rad_s2);
    m->effective_limits.jerk_max_rad_s3 = fminf(m->effective_limits.jerk_max_rad_s3, m->dynamics_output.jerk_rad_s3);
    CgTrajectoryPoint origin = { .q_rad = m->feedback.position_rad };
    if (!cg_trajectory_reset(&m->trajectory, &origin, &m->effective_limits) ||
        !cybergear_controller_init(&m->controller, &m->config.controller,
            origin.q_rad, 0.0f, now, m->feedback.rx_sequence, m->feedback.last_received_ms) ||
        !cybergear_controller_commit_queued(&m->controller, 0.0f, now)) return false;
    m->trajectory_start_ms = now;
    m->last_control_ms = now;
    m->first_cyclic = true;
    m->requested_target_rad = origin.q_rad;
    m->requested_since_ms = now;
    m->prepared_ready = false;
    m->planning_fault = false;
    m->tracking_active = false;
    m->saturation_active = false;
    m->stall_active = false;
    memset(&m->output, 0, sizeof(m->output));
    enter_state(m, CG_STATE_RUNNING, now);
    return true;
}

static void startup_step(CyberGearMotor *m, uint32_t now)
{
    if (now - m->startup_ms > m->config.startup_timeout_ms) {
        cybergear_latch_fault(m, CG_FAULT_START_TIMEOUT); return;
    }
    const bool due = now - m->last_command_ms >= m->config.command_retry_ms;
    bool ok = true;
    m->internal_send = true;
    switch (m->state) {
    case CG_STATE_WAIT_STOP:
        if (m->stop_queued && m->feedback.rx_sequence != m->stop_rx_sequence && quiet_feedback(m, now, 0U)) {
            enter_state(m, CG_STATE_WRITE_MODE, now);
        } else if (due) {
            ok = cybergear_stop(m); m->stop_queued = m->stop_queued || ok; m->last_command_ms = now;
        }
        break;
    case CG_STATE_WRITE_MODE:
        ok = cybergear_set_run_mode(m, m->requested_mode);
        if (ok) { enter_state(m, CG_STATE_WAIT_MODE, now); m->last_command_ms = now; }
        break;
    case CG_STATE_WAIT_MODE:
        if (m->mode_read_valid && m->mode_read_sequence != m->mode_request_sequence) {
            if (m->mode_read_value != (uint8_t)m->requested_mode) {
                cybergear_latch_fault(m, CG_FAULT_MODE); break;
            }
            m->mode_read_pending = false;
            enter_state(m, CG_STATE_ZERO, now);
        } else if (due) {
            m->mode_request_sequence = m->mode_read_sequence;
            m->mode_read_pending = true;
            ok = cybergear_read_parameter(m, CYBERGEAR_PARAM_RUN_MODE);
            m->last_command_ms = now;
        }
        break;
    case CG_STATE_ZERO:
        /* One queued frame per tick. Internal current limit application in current
         * mode is FW-dependent; the verified outer limit remains authoritative. */
        if (m->stop_attempts == 0U) {
            ok = m->requested_mode == CYBERGEAR_RUN_MODE_CURRENT ? cybergear_set_current(m, 0.0f) :
                cybergear_write_float(m, CYBERGEAR_PARAM_LIMIT_TORQUE, m->config.operation_torque_limit_nm);
            if (ok) m->stop_attempts = 1U;
        } else if (m->requested_mode == CYBERGEAR_RUN_MODE_OPERATION && m->stop_attempts == 1U) {
            ok = cybergear_control(m, m->feedback.position_rad, 0.0f, 0.0f, 0.0f, 0.0f);
            if (ok) m->stop_attempts = 2U;
        } else {
            ok = cybergear_enable(m);
            if (ok) {
                m->stop_rx_sequence = m->feedback.rx_sequence;
                m->quiet_last_sequence = m->feedback.rx_sequence;
                m->quiet_active = false;
                enter_state(m, CG_STATE_WAIT_RUN, now);
            }
        }
        break;
    case CG_STATE_WAIT_RUN:
        if (m->feedback.rx_sequence != m->stop_rx_sequence && quiet_feedback(m, now, 2U)) {
            if (!begin_running(m, now)) cybergear_latch_fault(m, CG_FAULT_MODEL);
        } else if (due) {
            ok = m->requested_mode == CYBERGEAR_RUN_MODE_CURRENT ? cybergear_set_current(m, 0.0f) :
                cybergear_control(m, m->feedback.position_rad, 0.0f, 0.0f, 0.0f, 0.0f);
            m->last_command_ms = now;
        }
        break;
    default: break;
    }
    m->internal_send = false;
    if (!ok) cybergear_latch_fault(m, CG_FAULT_TX);
}

static void record_log(CyberGearMotor *m, uint32_t now, uint32_t dt)
{
    const bool full = m->log_head - m->log_tail >= CYBERGEAR_LOG_CAPACITY;
    if (full) m->log_drops++;
    CyberGearLog *l = &m->latest_log;
    *l = (CyberGearLog){
        .timestamp_ms=now, .rx_sequence=m->feedback.rx_sequence,
        .rx_age_ms=now-m->feedback.last_received_ms, .control_dt_ms=dt,
        .state=m->state, .fault=m->fault, .target_rad=m->requested_target_rad,
        .qd=m->trajectory.point.q_rad, .vd=m->trajectory.point.v_rad_s, .ad=m->trajectory.point.a_rad_s2,
        .q=m->feedback.position_rad, .v_feedback=m->feedback.velocity_rad_s,
        .z1=m->controller.position_rad, .z2=m->controller.velocity_rad_s, .z3=m->controller.disturbance_rad_s2,
        .i_track=m->output.tracking_current_a, .i_dist=m->output.disturbance_current_a,
        .i_req=m->output.requested_current_a, .i_cmd=m->controller.last_queued_current_a,
        .b0=m->dynamics_output.b0, .b0_rate=m->dynamics_output.b0_rate,
        .accel_limit=m->effective_limits.acceleration_max_rad_s2,
        .brake_limit=m->effective_limits.braking_max_rad_s2, .jerk_limit=m->effective_limits.jerk_max_rad_s3,
        .posture_index=m->posture.posture_index, .posture_timestamp_ms=m->posture.timestamp_ms,
        .model_status=m->dynamics_output.status, .z3_before_reexpression=m->output.z3_before_reexpression,
        .z3_after_reexpression=m->output.z3_after_reexpression,
        .tx_queued=m->tx_queued, .tx_failed=m->tx_failed, .tx_fifo_free=m->tx_fifo_free,
        .tec=m->tec, .rec=m->rec, .drops=m->log_drops, .amp_limited=m->output.amplitude_limited,
        .slew_limited=m->output.slew_limited, .compensation_limited=m->output.disturbance_limited,
        .bus_off=m->bus_off, .applied_estimate_valid=m->controller.applied_current_valid,
        .stop_queued=m->stop_queued, .reset_confirmed=m->reset_confirmed, .stationary=m->stationary
    };
    if (full) return;
    m->logs[m->log_head % CYBERGEAR_LOG_CAPACITY] = *l;
    m->log_head++;
}

bool cybergear_pop_log(CyberGearMotor *m, CyberGearLog *l)
{
    if (m == NULL || l == NULL) return false;
    uint32_t mask = lock_state();
    bool available = m->log_head != m->log_tail;
    if (available) { *l = m->logs[m->log_tail % CYBERGEAR_LOG_CAPACITY]; m->log_tail++; }
    __set_PRIMASK(mask);
    return available;
}

void cybergear_set_posture(CyberGearMotor *m, const CyberGearPostureSnapshot *p)
{
    if (m == NULL || p == NULL) return;
    uint32_t mask = lock_state(); m->posture = *p; __set_PRIMASK(mask);
}

void cybergear_service(CyberGearMotor *m)
{
    if (m == NULL) return;
    uint32_t mask = lock_state();
    if (m->state != CG_STATE_RUNNING || m->prepared_ready || m->planning_fault ||
        fabsf(m->requested_target_rad - m->trajectory.target_rad) <= m->effective_limits.target_tolerance_rad) {
        __set_PRIMASK(mask); return;
    }
    CgTrajectory previous = m->trajectory;
    CgTrajectoryLimits limits = m->effective_limits;
    const float target = m->requested_target_rad;
    const uint32_t generation = m->plan_generation;
    const uint32_t activation = m->last_control_ms + m->config.planner_lead_ms;
    const uint32_t previous_start = m->trajectory_start_ms;
    __set_PRIMASK(mask);
    /* Polynomial/root searches occur here in main, never in the timer ISR.
     * Evaluate the OLD curve at a future activation instant; new q/v/a match
     * that exact instant even while the old curve continues in the ISR. */
    CgTrajectoryPoint point;
    CgTrajectory candidate;
    const bool ok = cg_trajectory_evaluate(&previous, (double)(activation - previous_start) * 0.001, &point) &&
        cg_trajectory_reset(&candidate, &point, &limits) && cg_trajectory_plan(&candidate, target, &limits);
    mask = lock_state();
    if (m->state == CG_STATE_RUNNING && m->plan_generation == generation &&
        !time_reached(HAL_GetTick(), activation)) {
        if (ok) { m->prepared = candidate; m->prepared_start_ms = activation; m->prepared_ready = true; }
        else m->planning_fault = true;
    }
    __set_PRIMASK(mask);
}

static bool timed_condition(bool condition, bool *active, uint32_t *since, uint32_t now, uint32_t timeout)
{
    if (!condition) { *active = false; return false; }
    if (!*active) { *active = true; *since = now; }
    return now - *since >= timeout;
}

static CyberGearFault running_protection(CyberGearMotor *m, uint32_t now)
{
    const CyberGearConfig *c = &m->config;
    const float error = fabsf(m->trajectory.point.q_rad - m->feedback.position_rad);
    if (timed_condition(error > c->tracking_error_rad, &m->tracking_active, &m->tracking_since_ms,
        now, c->tracking_timeout_ms)) return CG_FAULT_TRACKING;
    if (timed_condition(m->output.amplitude_limited || m->output.slew_limited,
        &m->saturation_active, &m->saturation_since_ms, now, c->saturation_timeout_ms)) return CG_FAULT_SATURATION;
    if (error > c->tracking_error_rad && fabsf(m->controller.last_queued_current_a) >= c->stall_current_a) {
        if (!m->stall_active) {
            m->stall_active = true; m->stall_since_ms = now; m->stall_start_rad = m->feedback.position_rad;
        } else if (fabsf(m->feedback.position_rad - m->stall_start_rad) >= c->stall_progress_rad) {
            m->stall_since_ms = now; m->stall_start_rad = m->feedback.position_rad;
        } else if (now - m->stall_since_ms >= c->stall_timeout_ms) return CG_FAULT_STALL;
    } else m->stall_active = false;
    /* Conservative distance to HARD boundary, including existing current reversal.
     * This is an active-drive feasibility monitor, not a guarantee after STOP/CAN loss. */
    const float velocity = m->feedback.velocity_rad_s;
    if (fabsf(velocity) > c->stationary_speed_rad_s) {
        const float slew = fminf(c->controller.current_rise_a_s, c->controller.current_fall_a_s);
        const float previous_current = m->requested_mode == CYBERGEAR_RUN_MODE_CURRENT ?
            fabsf(m->controller.last_queued_current_a) : c->controller.current_limit_a;
        const float reversal = (previous_current + c->dynamics.braking_current_a) / slew;
        const float delay = (float)c->transport_delay_ms * 0.001f + reversal;
        const float speed = fabsf(velocity);
        const float after = speed + c->outward_accel_rad_s2 * delay;
        const float margin = speed * delay + 0.5f * c->outward_accel_rad_s2 * delay * delay +
            after * after / (2.0f * c->brake_guaranteed_rad_s2) + c->stop_margin_rad;
        const float available = velocity > 0.0f ? c->hard_max_rad - m->feedback.position_rad :
            m->feedback.position_rad - c->hard_min_rad;
        if (!isfinite(margin) || margin >= available) return CG_FAULT_STOP_MARGIN;
    }
    return CG_FAULT_NONE;
}

bool cybergear_control_position_adrc(CyberGearMotor *m, float target)
{
    if (m == NULL || !m->managed || m->calibration_owned) return false;
    const uint32_t now = HAL_GetTick();
    const uint32_t elapsed = now - m->last_control_ms;
    FDCAN_ProtocolStatusTypeDef bus;
    FDCAN_ErrorCountersTypeDef counters;
    if (HAL_FDCAN_GetProtocolStatus(m->hfdcan, &bus) != HAL_OK ||
        HAL_FDCAN_GetErrorCounters(m->hfdcan, &counters) != HAL_OK) {
        cybergear_latch_fault(m, CG_FAULT_BUS);
    } else {
        m->bus_off = bus.BusOff != 0U; m->tec = counters.TxErrorCnt; m->rec = counters.RxErrorCnt;
        if (m->bus_off || bus.ErrorPassive != 0U || m->tec != 0U) cybergear_latch_fault(m, CG_FAULT_BUS);
    }
    if (m->state == CG_STATE_STOPPING || m->state == CG_STATE_FAULT) {
        service_stop(m, now); record_log(m, now, elapsed); return false;
    }
    if (m->motor_fault_sequence != 0U || (m->feedback.online && m->feedback.fault_flags != 0U)) {
        cybergear_latch_fault(m, CG_FAULT_MOTOR);
    } else if (m->feedback.online && (!isfinite(m->feedback.position_rad) ||
        !isfinite(m->feedback.velocity_rad_s) || !isfinite(m->feedback.temperature_c))) {
        cybergear_latch_fault(m, CG_FAULT_NUMERIC);
    } else if (m->feedback.online && m->feedback.temperature_c >= m->config.temperature_trip_c) {
        cybergear_latch_fault(m, CG_FAULT_TEMPERATURE);
    } else if (m->feedback.online && fabsf(m->feedback.velocity_rad_s) > m->config.speed_trip_rad_s) {
        cybergear_latch_fault(m, CG_FAULT_SPEED);
    } else if (m->state != CG_STATE_RUNNING) {
        startup_step(m, now);
    } else if (!feedback_fresh(m, now)) {
        cybergear_latch_fault(m, CG_FAULT_FEEDBACK);
    } else if (m->feedback.mode != 2U || !m->mode_read_valid || m->mode_read_value != (uint8_t)m->requested_mode) {
        cybergear_latch_fault(m, CG_FAULT_MODE);
    } else if (!isfinite(target) || target < m->config.trajectory.position_min_rad ||
        target > m->config.trajectory.position_max_rad) {
        cybergear_latch_fault(m, CG_FAULT_TARGET);
    } else if (m->feedback.position_rad <= m->config.hard_min_rad || m->feedback.position_rad >= m->config.hard_max_rad) {
        cybergear_latch_fault(m, CG_FAULT_POSITION);
    } else if (elapsed > 0U) {
        const uint32_t period = m->config.controller.period_ms;
        const uint32_t tolerance = m->config.controller.timing_tolerance_ms;
        if (m->first_cyclic && elapsed < period) goto done;
        const uint32_t difference = elapsed > period ? elapsed-period : period-elapsed;
        if (difference > tolerance) { cybergear_latch_fault(m, CG_FAULT_TIMING); goto done; }
        m->first_cyclic = false;
        if (m->feedback.rx_sequence != m->controller.rx_sequence) {
            const uint32_t rx_dt = m->feedback.last_received_ms - m->controller.rx_timestamp_ms;
            const float max_jump = m->config.position_jump_rad + m->config.speed_trip_rad_s * (float)rx_dt * 0.001f;
            if (rx_dt > m->config.controller.feedback_timeout_ms ||
                fabsf(m->feedback.position_rad - m->controller.last_measured_position_rad) > max_jump) {
                cybergear_latch_fault(m, CG_FAULT_POSITION); goto done;
            }
        }
        if (fabsf(target - m->requested_target_rad) > m->effective_limits.target_tolerance_rad) {
            const bool waiting = fabsf(m->requested_target_rad - m->trajectory.target_rad) > m->effective_limits.target_tolerance_rad;
            m->requested_target_rad = target;
            if (!waiting) m->requested_since_ms = now;
            /* Coalesce streaming targets. An immutable future plan must get its
             * activation slot; invalidating it on every 10 ms target would
             * starve a 50 ms lookahead forever. Newest target is planned next. */
        }
        if (m->planning_fault) { cybergear_latch_fault(m, CG_FAULT_PLANNER); goto done; }
        if (m->prepared_ready && time_reached(now, m->prepared_start_ms)) {
            m->trajectory = m->prepared; m->trajectory_start_ms = m->prepared_start_ms;
            m->prepared_ready = false; m->plan_generation++; m->requested_since_ms = now;
        }
        if (fabsf(m->requested_target_rad - m->trajectory.target_rad) > m->effective_limits.target_tolerance_rad &&
            now - m->requested_since_ms > m->config.planner_timeout_ms) {
            cybergear_latch_fault(m, CG_FAULT_PLANNER); goto done;
        }
        if (!cg_trajectory_evaluate(&m->trajectory, (double)(now-m->trajectory_start_ms)*0.001, &m->trajectory.point)) {
            cybergear_latch_fault(m, CG_FAULT_PLANNER); goto done;
        }
        m->trajectory.elapsed_s = fmin((double)(now-m->trajectory_start_ms)*0.001, m->trajectory.duration_s);
        m->trajectory.active = m->trajectory.elapsed_s < m->trajectory.duration_s;
        if (m->trajectory.duration_s > 0.0 &&
            (double)(now-m->trajectory_start_ms)*0.001 >= m->trajectory.duration_s) {
            /* Rebase completed motion to a true hold, so uint32_t time wrap
             * after a long idle cannot replay the original polynomial. */
            CgTrajectoryPoint hold = m->trajectory.point;
            if (!cg_trajectory_reset(&m->trajectory, &hold, &m->effective_limits)) {
                cybergear_latch_fault(m, CG_FAULT_PLANNER); goto done;
            }
            m->trajectory_start_ms = now;
        }
        if (!cybergear_dynamics_step(&m->dynamics, &m->posture, now, (float)period*0.001f, &m->dynamics_output) ||
            m->dynamics_output.status == CYBERGEAR_MODEL_STALE || m->dynamics_output.status == CYBERGEAR_MODEL_INVALID) {
            cybergear_latch_fault(m, CG_FAULT_MODEL); goto done;
        }
        CyberGearControllerReference reference = {m->trajectory.point.q_rad, m->trajectory.point.v_rad_s, m->trajectory.point.a_rad_s2};
        CyberGearControllerMeasurement measurement = {m->feedback.position_rad,m->feedback.rx_sequence,m->feedback.last_received_ms,true};
        if (m->requested_mode == CYBERGEAR_RUN_MODE_CURRENT &&
            !cybergear_controller_step(&m->controller, &reference, &measurement, now, m->dynamics_output.b0, &m->output)) {
            cybergear_latch_fault(m, CG_FAULT_NUMERIC); goto done;
        }
        if (m->requested_mode == CYBERGEAR_RUN_MODE_OPERATION) {
            /* The internal PD current is unknown. Do not feed a fictitious zero
             * into an ESO or use an unsent external-current saturation flag. */
            cybergear_controller_invalidate_input(&m->controller);
            m->controller.rx_sequence = m->feedback.rx_sequence;
            m->controller.rx_timestamp_ms = m->feedback.last_received_ms;
            m->controller.last_measured_position_rad = m->feedback.position_rad;
        }
        m->last_control_ms = now;
        const CyberGearFault protection = running_protection(m, now);
        if (protection != CG_FAULT_NONE) { cybergear_latch_fault(m, protection); goto done; }
        m->internal_send = true;
        bool sent;
        if (m->requested_mode == CYBERGEAR_RUN_MODE_CURRENT) sent = cybergear_set_current(m, m->output.current_a);
        else sent = cybergear_control(m, reference.position_rad, reference.velocity_rad_s,
            m->config.operation_kp, m->config.operation_kd, 0.0f);
        m->internal_send = false;
        if (!sent) cybergear_latch_fault(m, CG_FAULT_TX);
        else if (m->requested_mode == CYBERGEAR_RUN_MODE_CURRENT &&
            !cybergear_controller_commit_queued(&m->controller, m->output.current_a, now))
            cybergear_latch_fault(m, CG_FAULT_NUMERIC);
    }
done:
    if (m->state == CG_STATE_STOPPING) service_stop(m, now);
    record_log(m, now, elapsed);
    return m->state != CG_STATE_FAULT && m->state != CG_STATE_STOPPING;
}

bool cybergear_start_position_adrc(CyberGearMotor *m)
{
    if (!cybergear_begin_position_control(m, CYBERGEAR_COMPARE_OPERATION_MODE ?
        CYBERGEAR_RUN_MODE_OPERATION : CYBERGEAR_RUN_MODE_CURRENT)) return false;
    /* Before TIM6 starts, service the same bounded state machine in main.
     * Failure drains STOP retries before caller's existing Error_Handler. */
    while (m->state != CG_STATE_RUNNING && m->state != CG_STATE_FAULT) {
        const uint32_t mask = lock_state();
        cybergear_control_position_adrc(m, 0.0f);
        __set_PRIMASK(mask);
        if (m->state != CG_STATE_RUNNING) HAL_Delay(m->config.controller.period_ms);
    }
    return m->state == CG_STATE_RUNNING;
}
