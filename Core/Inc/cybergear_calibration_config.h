#ifndef CYBERGEAR_CALIBRATION_CONFIG_H
#define CYBERGEAR_CALIBRATION_CONFIG_H
#include <math.h>

/* CMake -DCYBERGEAR_CALIBRATION_BUILD=ON selects the isolated measurement
 * firmware. Normal homing/RobStride/position commands never run in that build. */
#ifndef CYBERGEAR_CALIBRATION_BUILD
#define CYBERGEAR_CALIBRATION_BUILD 0
#endif

/* 計測専用の許可。通常制御の HARDWARE_CONFIRMED/b0 は変更しない。
 * 取付済み機構で許容した電流/温度/速度を入力してから1。
 * p/nキーを受けるまではSTOPのみ。試行ごとの自動増幅・自動繰返しはない。 */
#define CG_CAL_ARMED                  0
#define CG_CAL_PULSE_CURRENT_A        NAN /* パルス振幅 [A]。p=正、n=負に適用。 */
#define CG_CAL_BRAKE_CURRENT_A        NAN /* 逆電流の振幅 [A]。許容値を超えない。 */
#define CG_CAL_CURRENT_LIMIT_A        NAN /* 機構で許容した試験電流上限 [A]。自動探索しない。 */
#define CG_CAL_TEMPERATURE_TRIP_C     NAN /* 試験を中止する温度 [℃]。定格から決める。 */
#define CG_CAL_SPEED_TRIP_RAD_S       NAN /* 試験中の実測速度上限 [rad/s]。 */

/* 初期位置からの試験範囲。90°固定境界のかなり内側から始める。
 * SOFTは到達目標ではなく中止境界。margin+速度先読みでさらに早く中止する。
 * 原点は起動後最初の新鮮なfeedbackで一度だけ固定。p/nでは更新しない。 */
#define CG_CAL_TRAVEL_SOFT_DEG        10.0f
#define CG_CAL_TRAVEL_HARD_DEG        80.0f /* 必須: soft < hard < 90。90以上は設定拒否。 */
#define CG_CAL_GUARD_MARGIN_DEG       2.0f
#define CG_CAL_GUARD_LOOKAHEAD_MS     100U /* 現在速度×(先読み+受信鮮度)を余裕に加算。加速度保証ではない。 */
#define CG_CAL_POSITION_JUMP_RAD      0.01f /* 速度上限×受信間隔に加えて許す位置差 [rad]。不連続を拒否。 */
#define CG_CAL_STATIONARY_SPEED_RAD_S 0.02f /* 静止判定幅 [rad/s]。起動・制動終了・停止確認で使用。 */
#define CG_CAL_BASELINE_MS            200U /* パルス前のゼロ電流観測。重力等で動けば中止。 */
#define CG_CAL_PULSE_MS               200U /* max(30, 3×MAX_STEP)..500 ms。同定S/Nと移動量が増す。 */
#define CG_CAL_BRAKE_MS               200U /* max(30, 3×MAX_STEP)..500 ms。静止/反転検出で先に切る。 */
#define CG_CAL_SETTLE_MS              100U /* 制動後のゼロ電流観測。低速度でこの時間を経てSTOP。 */
#define CG_CAL_FEEDBACK_TIMEOUT_MS    30U /* 最後の受信からこの時間を超えると中止。長期化しない。 */
#define CG_CAL_MAX_STEP_MS            20U /* TIM6計測呼出しの許容最大間隔。遅延超過で中止。 */

#define CG_CAL_APP_PERIOD_MS          10U /* 計測固定100 Hz。通常の200 Hz設定とは独立。 */
#define CG_CAL_APP_START_TIMEOUT_MS   3000U /* STOP/readback/Run各起動全体の期限。 */
#define CG_CAL_APP_STOP_TIMEOUT_MS    1000U /* STOP再試行期限。到達不明ならFAULT。 */
#define CG_CAL_APP_RETRY_MS           20U /* STOP/readback間隔。feedback timeoutより短く。 */
#define CG_CAL_APP_QUIET_MS           100U /* 新鮮な低速度feedbackを連続確認する時間。 */
#define CG_CAL_APP_LOG_CAPACITY       512U /* 約5秒分。満杯なら中止し、停止後UART出力。 */

#endif
