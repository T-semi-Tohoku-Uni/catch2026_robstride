#ifndef __CYBERGEAR_H
#define __CYBERGEAR_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32g4xx_hal.h"
#include "cybergear_config.h"
#include "cybergear_controller.h"
#include "cybergear_dynamics.h"
#include "cybergear_trajectory.h"

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

typedef enum {
    CG_STATE_OFF, CG_STATE_WAIT_STOP, CG_STATE_WRITE_MODE, CG_STATE_WAIT_MODE,
    CG_STATE_ZERO, CG_STATE_WAIT_RUN, CG_STATE_RUNNING, CG_STATE_STOPPING, CG_STATE_FAULT
} CyberGearState;

typedef enum {
    CG_FAULT_NONE, CG_FAULT_CONFIG, CG_FAULT_TX, CG_FAULT_BUS,
    CG_FAULT_START_TIMEOUT, CG_FAULT_MODE, CG_FAULT_FEEDBACK,
    CG_FAULT_MOTOR, CG_FAULT_TARGET, CG_FAULT_TIMING, CG_FAULT_POSITION,
    CG_FAULT_SPEED, CG_FAULT_TEMPERATURE, CG_FAULT_TRACKING,
    CG_FAULT_STALL, CG_FAULT_SATURATION, CG_FAULT_STOP_MARGIN,
    CG_FAULT_MODEL, CG_FAULT_PLANNER, CG_FAULT_NUMERIC, CG_FAULT_REQUESTED_STOP
} CyberGearFault;

typedef struct {
    bool hardware_confirmed; /* 機械値/正方向 b0/停止経路の確認済みフラグ。既定 false。 */
    CgTrajectoryLimits trajectory;
    CyberGearControllerConfig controller;
    CyberGearDynamicsConfig dynamics;
    float hard_min_rad, hard_max_rad; /* 実測外側保護境界 [rad]。soft範囲を厳密に包含。 */
    float speed_trip_rad_s;           /* 実測速度の絶対上限 [rad/s]。参照Vより大きくする。 */
    float temperature_trip_c;         /* 停止温度 [deg C]。連続運転・冷却条件から決める。 */
    float position_jump_rad;          /* 位置飛びの余裕 [rad]。速度上限×受信dtに加算する。 */
    float tracking_error_rad;         /* |qd-q| の許容 [rad]。最終目標からの距離ではない。 */
    uint32_t tracking_timeout_ms;     /* 誤差超過の連続時間 [ms]。移動の正常遅れより長く。 */
    uint32_t saturation_timeout_ms;   /* 振幅/スルー飽和が続く上限 [ms]。0で無効にはしない。 */
    bool recover_on_saturation;
    bool recover_on_tracking;
    uint32_t recovery_zero_ms;
    float stall_current_a;            /* 拘束疑い電流 [A]。正常保持電流より大きく、Imax以下。 */
    float stall_progress_rad;         /* 拘束観測窓内の最小移動 [rad]。量子化幅より大きく。 */
    uint32_t stall_timeout_ms;        /* 誤差と電流があるのに進行しない観測窓 [ms]。 */
    float stationary_speed_rad_s;     /* 起動/再armの静止速度しきい値 [rad/s]。量子化を考慮。 */
    uint32_t stationary_dwell_ms;     /* 新鮮なReset/Run feedbackで低速を保つ時間 [ms]。 */
    uint32_t startup_timeout_ms;      /* STOP→readback→ENABLE 全体の期限 [ms]。 */
    uint32_t command_retry_ms;        /* 起動read要求とSTOP再試行間隔 [ms]。共有バスを占有しない。 */
    uint32_t stop_timeout_ms;         /* STOP再試行期限 [ms]。期限後FAULT、到達未確認はログに残る。 */
    uint16_t stop_max_attempts;       /* STOPの最大投入試行回数。失敗も回数に含む。 */
    uint32_t planner_lead_ms;         /* mainで将来状態から計画する先行時間 [ms]。周期の整数倍。 */
    uint32_t planner_timeout_ms;      /* 新目標の計画が有効にならない最長時間 [ms]。 */
    float operation_kp, operation_kd; /* 比較用の内蔵PD。Kp [Nm/rad]、Kd [Nm s/rad]。未設定NAN。 */
    float operation_torque_limit_nm; /* 比較用トルク上限レジスタ [Nm]。FW適用確認が必要。 */
} CyberGearConfig;

/* ISRでは数値だけ記録し、mainで取り出す。全列の意味は調整説明書に記載。 */
typedef struct {
    uint32_t timestamp_ms, rx_sequence, rx_age_ms, control_dt_ms;
    CyberGearState state;
    CyberGearFault fault;
    float target_rad, qd, vd, ad, q, v_feedback, z1, z2, z3;
    float i_track, i_dist, i_req, i_cmd, b0, b0_rate;
    float accel_limit, brake_limit, jerk_limit, posture_index;
    float z3_before_reexpression, z3_after_reexpression;
    uint32_t posture_timestamp_ms;
    CyberGearModelStatus model_status;
    uint32_t tx_queued, tx_failed, tx_fifo_free, tec, rec, drops;
    bool amp_limited, slew_limited, compensation_limited, bus_off;
    bool applied_estimate_valid, stop_queued, reset_confirmed, stationary;
} CyberGearLog;

#define CYBERGEAR_LOG_CAPACITY 32U /* 約320 ms @100Hz。満杯時drop、制御を待たせない。 */

typedef struct
{
	FDCAN_HandleTypeDef *hfdcan;
	uint8_t motor_id;
	uint8_t master_id;
	CyberGearRunMode run_mode;

	FDCAN_TxHeaderTypeDef tx_header;
	CyberGearFeedback feedback;
    CyberGearConfig config;
    CyberGearState state;
    CyberGearFault fault;
    CyberGearRunMode requested_mode;
    CyberGearController controller;
    CyberGearDynamics dynamics;
    CyberGearDynamicsOutput dynamics_output;
    CyberGearControllerOutput output;
    CyberGearPostureSnapshot posture;
    CgTrajectory trajectory, prepared;
    CgTrajectoryLimits effective_limits;
    uint32_t trajectory_start_ms, prepared_start_ms, plan_generation;
    uint32_t requested_since_ms, last_control_ms, startup_ms, state_ms, last_command_ms;
    uint32_t stop_started_ms, stop_rx_sequence, stop_attempts, stop_last_sequence;
    uint32_t quiet_since_ms, quiet_last_sequence;
    uint32_t tracking_since_ms, saturation_since_ms, stall_since_ms;
    uint32_t recovery_count, recovery_started_ms;
    bool recovery_zero_active;
    float requested_target_rad, stall_start_rad;
    bool managed, internal_send, prepared_ready, planning_fault, quiet_active, first_cyclic;
    bool tracking_active, saturation_active, stall_active;
    bool mode_read_pending, mode_read_valid;
    uint8_t mode_read_value, fault_payload[8];
    uint32_t mode_read_ms, mode_read_sequence, mode_request_sequence;
    uint32_t motor_fault_sequence;
    bool stop_queued, reset_confirmed, stationary;
    uint32_t tx_queued, tx_failed, tx_fifo_free, tec, rec;
    bool bus_off;
    CyberGearLog latest_log; /* Ring fullでも最新状態を保持。停止理由を古いログに埋もれさせない。 */
    CyberGearLog logs[CYBERGEAR_LOG_CAPACITY];
    uint32_t log_head, log_tail, log_drops;
} CyberGearMotor;

void cybergear_config_defaults(CyberGearConfig *config);
bool cybergear_config_valid(const CyberGearConfig *config);
bool cybergear_configure(CyberGearMotor *motor, const CyberGearConfig *config);
bool cybergear_begin_position_control(CyberGearMotor *motor, CyberGearRunMode mode);
bool cybergear_reset_fault(CyberGearMotor *motor); /* 明示操作専用。自動呼出し禁止。 */
void cybergear_service(CyberGearMotor *motor);    /* main専用。CAN送信なし、計画のみ。 */
void cybergear_set_posture(CyberGearMotor *motor, const CyberGearPostureSnapshot *posture);
bool cybergear_pop_log(CyberGearMotor *motor, CyberGearLog *log);
bool cybergear_process_rx(CyberGearMotor *motor, const FDCAN_RxHeaderTypeDef *header,
    const uint8_t *data);
bool cybergear_read_parameter(CyberGearMotor *motor, uint16_t index);

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
