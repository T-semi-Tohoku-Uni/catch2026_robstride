#ifndef CYBERGEAR_CONTROLLER_H
#define CYBERGEAR_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

/* HAL 非依存。角度・速度・加速度は同じ CyberGear 出力軸座標で扱う。 */
typedef enum {
    CYBERGEAR_LEAK_LEGACY_ERROR = 0,
    CYBERGEAR_LEAK_FIXED = 1,
    CYBERGEAR_LEAK_NONE = 2
} CyberGearLeakMode;

typedef struct {
    uint32_t period_ms;             /* 固定計算周期 [ms]。10=100 Hz、5=200 Hz のみ。受信間隔を dt にしない。 */
    uint32_t timing_tolerance_ms;   /* 実呼出し間隔と period_ms の差の許容 [ms]。超過は失敗。通常 1。 */
    uint32_t feedback_timeout_ms;   /* 新測定の最長許容 age [ms]。制動余裕から決める。 */
    float bandwidth_rad_s;         /* wc [rad/s]。位置剛性 wc²。移動速度ではなく誤差修正の帯域。 */
    float damping_ratio;           /* zeta [-]。速度項 2*zeta*wc。大きいほど過減衰になる。 */
    float observer_rad_s;          /* wo [rad/s]。量子化、遅延、共振を確認して調整。 */
    float b0_initial;              /* 初期公称入力ゲイン [rad/s²/A]。正方向の符号を実測確認する。 */
    float b0_min;                  /* b0 許可下限 [rad/s²/A] > 0。校正範囲外値を拒否する。 */
    float b0_max;                  /* b0 許可上限 [rad/s²/A] >= b0_min。 */
    float current_limit_a;         /* 出力電流の絶対値上限 [A]。連続・ピーク許容の小さい方以内。 */
    float current_rise_a_s;        /* 数値として増える電流の上限 [A/s]（負電流からゼロへの変化も含む）。 */
    float current_fall_a_s;        /* 数値として減る電流の上限 [A/s]（正電流から負方向への制動も含む）。 */
    float disturbance_limit_a;     /* 外乱補償電流の絶対値上限 [A]。保持に必要な電流と余裕から設定。 */
    float disturbance_slew_a_s;    /* 外乱補償の変化率上限 [A/s]。最終出力のスルー制限とは別。 */
    float compensation_gain;       /* gamma の最終値 [0,1]。0 なら補償無効、ESO 推定は継続する。 */
    uint32_t compensation_delay_ms;/* 初期化から gamma=0 を保つ時間 [ms]。目標変更では再始動しない。 */
    uint32_t compensation_ramp_ms; /* delay 後に gamma を smoothstep で上げる時間 [ms]。>0。 */
    CyberGearLeakMode leak_mode;   /* legacy 誤差依存 / 固定 / なし。初回比較は legacy を保持する。 */
    float leak_fixed_s;            /* 固定リーク率 [1/s] >=0。小さいほど負荷補償を長く保持。 */
    float leak_near_s;             /* legacy: |qd-qmeas| が near 以下のリーク率 [1/s]。 */
    float leak_far_s;              /* legacy: 誤差が far 以上のリーク率 [1/s]。 */
    float leak_near_rad;           /* legacy: near しきい値 [rad] >=0。最終目標ではなく生成参照を使う。 */
    float leak_far_rad;            /* legacy: far しきい値 [rad] > near。間は連続線形補間。 */
} CyberGearControllerConfig;

typedef struct {
    float position_rad;
    uint32_t rx_sequence;          /* 受信フレームごとに増加。0 を含み、uint32_t wrap を許す。 */
    uint32_t timestamp_ms;         /* 受信時刻。モーター内部の測定時刻ではない。 */
    bool valid;
} CyberGearControllerMeasurement;

typedef struct {
    float position_rad;
    float velocity_rad_s;
    float acceleration_rad_s2;
} CyberGearControllerReference;

typedef struct {
    float current_a;
    float tracking_current_a;
    float disturbance_current_a;
    float requested_current_a;
    float amplitude_current_a;
    float innovation_rad;
    float gamma;
    float b0;
    float z3_before_reexpression;
    float z3_after_reexpression;
    bool amplitude_limited;
    bool slew_limited;
    bool disturbance_limited;
    bool measurement_corrected;
} CyberGearControllerOutput;

typedef struct {
    CyberGearControllerConfig config;
    float position_rad;
    float velocity_rad_s;
    float disturbance_rad_s2;
    float b0;
    float disturbance_current_a;
    float last_queued_current_a;
    float applied_current_estimate_a; /* キュー投入値を ZOH と仮定した近似。実電流測定ではない。 */
    float last_measured_position_rad;
    float observer_position_gain;  /* 固定周期・wo から初期化時に算出。ISR 内で exp を再計算しない。 */
    float observer_velocity_gain;
    float observer_disturbance_gain;
    uint32_t last_control_timestamp_ms;
    uint32_t observer_timestamp_ms;
    uint32_t last_queued_timestamp_ms;
    uint32_t rx_sequence;
    uint32_t rx_timestamp_ms;
    uint32_t start_timestamp_ms;
    uint32_t compensation_elapsed_ms; /* 飽和カウンタ。長時間運転の tick wrap でランプを再始動しない。 */
    bool initialized;
    bool applied_current_valid;
} CyberGearController;

void cybergear_controller_default_config(CyberGearControllerConfig *config);
bool cybergear_controller_config_valid(const CyberGearControllerConfig *config);
bool cybergear_controller_init(CyberGearController *controller,
    const CyberGearControllerConfig *config, float position_rad,
    float velocity_rad_s, uint32_t now_ms, uint32_t rx_sequence, uint32_t rx_timestamp_ms);
/* 必ず period_ms ごとに呼ぶ。予測を旧 b0・最後の投入電流で行い、新測定を最大 1 回補正し、
 * その後 b0 を再表現する。false は入力/時刻/数値異常。出力を送らず上位で停止する。 */
bool cybergear_controller_step(CyberGearController *controller,
    const CyberGearControllerReference *reference,
    const CyberGearControllerMeasurement *measurement,
    uint32_t now_ms, float b0, CyberGearControllerOutput *output);
/* HAL の投入成功時のみ呼ぶ。失敗した未送信要求値を ESO に帰還しない。
 * init 後も ZERO_COMMAND 投入成功をこの API で明示する。 */
bool cybergear_controller_commit_queued(CyberGearController *controller,
    float current_a, uint32_t now_ms);
void cybergear_controller_invalidate_input(CyberGearController *controller);
void cybergear_controller_reset_adaptation(CyberGearController *controller);

#endif
