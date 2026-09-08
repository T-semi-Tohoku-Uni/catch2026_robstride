#ifndef CYBERGEAR_CONFIG_H
#define CYBERGEAR_CONFIG_H

#include <math.h>

/* 実機設定の入口。NAN は「未設定」であり、起動を拒否する。
 * docs/05_CyberGear実装と調整.md と各 *_parameters.md を参照。
 * 機構寸法・定格・符号を推測して値を埋めない。ホスト試験は別の fixture を使う。
 */
#define CYBERGEAR_HARDWARE_CONFIRMED       0
/* 原点設定後の出力軸座標 [rad]。通信表現範囲 ±12.5 rad とは別物。
 * 内側は目標/参照の限界、外側は実測位置の保護限界。内側を厳密に包含する。 */
#define CYBERGEAR_SOFT_MIN_RAD            -6.28
#define CYBERGEAR_SOFT_MAX_RAD            6.28
#define CYBERGEAR_HARD_MIN_RAD            -12.56
#define CYBERGEAR_HARD_MAX_RAD            12.56
/* 実機で許容した電流 [A]、実測速度 [rad/s]、温度 [deg C]。
 * 10 A は旧ソフト値であり定格ではない。連続運転で許容する値を使う。 */
#define CYBERGEAR_CURRENT_LIMIT_A         6.0f
#define CYBERGEAR_SPEED_TRIP_RAD_S        26.0f
#define CYBERGEAR_TEMPERATURE_TRIP_C      85.0f
/* 正方向電流に対する正方向加速度を確認した b0 [rad/s²/A]。
 * MIN は到達可能全姿勢・荷物・モデル誤差を含めた下限、MAX は上限。
 * FIXED は制御の代表値。軌道には MIN を使い、制御と混同しない。 */
#define CYBERGEAR_B0_FIXED                NAN
#define CYBERGEAR_B0_MIN                  NAN
#define CYBERGEAR_B0_MAX                  NAN
/* 軌道の希望上限。最終加減速/jerkは電流予算でも抑える。未同定の試験開始候補。
 * 上げる順序は V → A/J → 必要なときだけ帯域。実機許容値に調整する。 */
#define CYBERGEAR_TRAJECTORY_SPEED_RAD_S  0.4f
#define CYBERGEAR_TRAJECTORY_ACCEL_RAD_S2 1.0f
#define CYBERGEAR_TRAJECTORY_BRAKE_RAD_S2 1.0f
#define CYBERGEAR_TRAJECTORY_JERK_RAD_S3  5.0f
/* 制動の確認値。遅延中の外向き加速度と、その後保証できる減速度 [rad/s²]。
 * 停止余裕は delay + 電流符号反転時間も含めて評価。CAN断時の停止保証ではない。 */
#define CYBERGEAR_BRAKE_GUARANTEED_RAD_S2 NAN
#define CYBERGEAR_OUTWARD_ACCEL_RAD_S2   NAN
#define CYBERGEAR_STOP_MARGIN_RAD        NAN
/* 0=100 Hz（初期値）、1=200 Hz。TIM6/他軸の周期は変えない。
 * 200 Hz は受信遅延分布/最悪ISR時間を実機確認後に使用する。 */
#ifndef CYBERGEAR_USE_200_HZ
#define CYBERGEAR_USE_200_HZ              0
#endif
/* 0=電流＋外側制御、1=内蔵PD比較。停止・readback・再初期化時だけ反映。
 * 比較モードのkp/kd/torque上限はconfigureで必須設定。自動切替はない。 */
#define CYBERGEAR_COMPARE_OPERATION_MODE  0
/* 1でmainから診断を排出。制御は常にリングへ保存、満杯ならdrop。
 * UART出力によるmainの計画遅延もplanner_timeoutで監視する。 */
#define CYBERGEAR_LOG_UART               0

static inline int cybergear_control_phase_due(unsigned int phase)
{
    return phase == 0U || (CYBERGEAR_USE_200_HZ && phase == 5U);
}

#endif