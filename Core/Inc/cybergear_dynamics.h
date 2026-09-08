#ifndef CYBERGEAR_DYNAMICS_H
#define CYBERGEAR_DYNAMICS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CYBERGEAR_DYNAMICS_MAX_POINTS 8U

typedef struct {
    float posture_index; /* 校正済み実姿勢のスカラー座標。意味・単位は機構担当が定義する。 */
    float b0;            /* 同じ姿勢・最大想定荷物条件で校正した公称入力ゲイン [rad/s²/A]。 */
} CyberGearDynamicsPoint;

typedef struct {
    bool schedule_enabled;        /* true は校正テーブルと実姿勢 API の接続確認後のみ。既定 false。 */
    float fixed_b0;               /* 無効時・情報失効時の退避用入力ゲイン [rad/s²/A]。検証済み値。 */
    float b0_min;                 /* 到達全姿勢・荷物・誤差を含む入力ゲイン下限 [rad/s²/A] >0。
                                    Ktau/J_hi 相当。制御用の中央推定値と区別する。 */
    float b0_max;                 /* 校正済み入力ゲイン上限 [rad/s²/A] >= b0_min。 */
    float b0_rate_limit;          /* b0 の最大変化率 [rad/s²/A/s] >0。情報失効時の復帰も同じ制限。 */
    uint32_t posture_timeout_ms;  /* 実姿勢 age の許容 [ms]。古いモデルへの新規高速移動許可はしない。 */
    size_t point_count;           /* 2..MAX_POINTS。schedule_enabled=false では 0 可。 */
    CyberGearDynamicsPoint points[CYBERGEAR_DYNAMICS_MAX_POINTS]; /* index 昇順。校正範囲外は失効扱い。 */
    float acceleration_current_a; /* 加速に利用可能な絶対電流 [A]。機体で確認した温度・電源条件込み。 */
    float braking_current_a;      /* 減速に利用可能な絶対電流 [A]。回生・電源条件込みで別設定。 */
    float reserve_current_a;      /* 摩擦・軸間干渉・PD・外乱補償に予約する電流 [A] >=0。 */
    float current_slew_a_s;       /* 正負スルーの小さい方 [A/s]。実コントローラの制限以下にする。 */
    float reserve_slew_a_s;       /* PD・補償のため FF に使わない電流変化率 [A/s] >=0。 */
    float acceleration_cap_rad_s2;/* 軌道の機械的加速度上限 [rad/s²] >0。電流予算の上限と小さい方。 */
    float braking_cap_rad_s2;     /* 軌道の機械的減速度上限 [rad/s²] >0。 */
    float jerk_cap_rad_s3;        /* 軌道の機械的 jerk 上限 [rad/s³] >0。 */
} CyberGearDynamicsConfig;

typedef struct {
    float posture_index;
    uint32_t timestamp_ms;
    bool valid;
} CyberGearPostureSnapshot;

typedef enum {
    CYBERGEAR_MODEL_FIXED = 0,
    CYBERGEAR_MODEL_VALID,
    CYBERGEAR_MODEL_STALE,
    CYBERGEAR_MODEL_INVALID
} CyberGearModelStatus;

typedef struct {
    float b0;
    float b0_rate;
    float acceleration_rad_s2;
    float braking_rad_s2;
    float jerk_rad_s3;
    float tracking_current_budget_a;
    CyberGearModelStatus status;
} CyberGearDynamicsOutput;

typedef struct {
    CyberGearDynamicsConfig config;
    float b0;
    bool initialized;
} CyberGearDynamics;

bool cybergear_dynamics_config_valid(const CyberGearDynamicsConfig *config);
bool cybergear_dynamics_init(CyberGearDynamics *dynamics,
    const CyberGearDynamicsConfig *config);
/* dt_s は controller と同じ固定周期。姿勢の受信間隔から作らない。
 * b0 は連続退避、制約は全姿勢 b0_min の固定保守上限。
 * STALE/INVALID は正常な b0 退避値を返すが、継続/停止判断はドライバの責任。 */
bool cybergear_dynamics_step(CyberGearDynamics *dynamics,
    const CyberGearPostureSnapshot *posture, uint32_t now_ms, float dt_s,
    CyberGearDynamicsOutput *output);

#endif
