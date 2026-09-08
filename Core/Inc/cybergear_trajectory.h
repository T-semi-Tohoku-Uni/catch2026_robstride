#ifndef CYBERGEAR_TRAJECTORY_H
#define CYBERGEAR_TRAJECTORY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float q_rad;          /* 同じ機械座標の位置 [rad]。実測値への毎周期の引き戻しは禁止。 */
    float v_rad_s;        /* 参照速度 [rad/s]。再計画時にも連続に引き継ぐ。 */
    float a_rad_s2;       /* 参照加速度 [rad/s^2]。再計画時にも連続に引き継ぐ。 */
    float jerk_rad_s3;    /* 参照加加速度 [rad/s^3]。五次式の接続で連続性は要求しない。 */
} CgTrajectoryPoint;

typedef struct {
    float position_min_rad;       /* 機械可動域の下端 [rad]。端点だけでなく全軌道を検査。 */
    float position_max_rad;       /* 機械可動域の上端 [rad]。min より大きい有限値が必須。 */
    float velocity_max_rad_s;     /* |参照速度| の上限 [rad/s]。正値。上げると停止距離も増す。 */
    float acceleration_max_rad_s2;/* 加速能力の上限 [rad/s^2]。正値。電流・慣性から別途算出。 */
    float braking_max_rad_s2;     /* 減速能力の上限 [rad/s^2]。正値。生成には加速上限との小さい方を使用。 */
    float jerk_max_rad_s3;        /* |参照加加速度| の上限 [rad/s^3]。正値。小さいほど加減速が緩やか。 */
    float duration_min_s;         /* 探索する最短軌道時間 [s]。1 us 以上。制御周期以上を推奨。 */
    float duration_max_s;         /* 探索する最長軌道時間 [s]。min 以上。超過時は計画失敗を返す。 */
    float target_tolerance_rad;   /* 同一目標とみなす差 [rad]。0 以上。微小差の再始動を防ぎ、古い終点を保持。 */
    uint16_t search_iterations;   /* 候補時間の検証回数上限 [回]。2..256。増加で見落とし減・main負荷増。 */
} CgTrajectoryLimits;

typedef struct {
    double coefficients[6]; /* q(s)=sum(c[k]*s^k), s=t/T。計画後は呼出側で変更しない。 */
    double duration_s;
    double elapsed_s;
    CgTrajectoryPoint initial;
    CgTrajectoryPoint point;
    CgTrajectoryLimits limits;
    float target_rad;
    uint16_t plan_iterations;
    bool initialized;
    bool active;
} CgTrajectory;

/* HAL非依存。どの関数も失敗時には出力先・軌道を変更しない。
 * reset は非ゼロ v/a も保存し jerk は 0 にする。移動中の初期化は必ず次に plan を行う。
 * plan は現在の point を接続点とし、同一目標・同一制約なら時刻を戻さない。
 * 制約変更時は同じ目標でも再計画。失敗時の制動/停止判断は呼出側の責任。
 * evaluate の time_s は軌道開始からの経過秒。現在 point からの相対時間ではない。
 * plan/validate は main で実行。ISR では advance/evaluate のみを使用する。
 * main と ISR 間の係数引渡し・接続時刻整合・排他は呼出側が保証する。 */
bool cg_trajectory_reset(CgTrajectory *trajectory, const CgTrajectoryPoint *initial,
                         const CgTrajectoryLimits *limits);
bool cg_trajectory_plan(CgTrajectory *trajectory, float target_rad,
                        const CgTrajectoryLimits *limits);
bool cg_trajectory_evaluate(const CgTrajectory *trajectory, double time_s,
                            CgTrajectoryPoint *point);
bool cg_trajectory_advance(CgTrajectory *trajectory, float dt_s, CgTrajectoryPoint *point);
bool cg_trajectory_validate(const CgTrajectory *trajectory, const CgTrajectoryLimits *limits);

#ifdef __cplusplus
}
#endif
#endif
