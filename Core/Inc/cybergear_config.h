#ifndef CYBERGEAR_CONFIG_H
#define CYBERGEAR_CONFIG_H

#include <math.h>

/*
 * CyberGear 実機設定の入口
 * ======================
 * このファイルを編集して再ビルド・書込みすると反映される。UART からの設定変更ではない。
 * 通常運転と単体試験の設定を集約し、初期値は集約前の動作値を維持している。
 * 実機の寸法・定格・電流の符号を推測して変更しない。NAN は「未設定」を表す。
 * 汎用制御ライブラリのホスト試験用既定値とは別に、実機の config_defaults へ適用する。
 *
 * まず変更する場所:
 *   試験 ON/OFF → 1、角度・待機・試験電流 → 2、到達性・振動 → 3、
 *   機械の許容範囲 → 4、軌道 → 5、保護停止 → 6、原点探索 → 7。
 * 既定の単体試験: +60 → -60 → +5 → -5 度、各点で1秒待機、
 *   電流上限3 A、軌道速度上限0.3 rad/s、観測器帯域6 rad/s、外乱補償上限1.5 A。
 * 到達条件は位置誤差0.5度以内・推定速度0.03 rad/s以下・軌道完了のまま。
 *
 * 注意: 試験電流・速度上限は原点探索後の ADRC に適用する。
 * 原点探索は別の速度制御であり、UART 停止文字も原点探索中は処理しない。
 * 0.3 rad/s は参照軌道の上限であり、実測速度の保護停止閾値ではない。
 * GPIO/CAN配線・クロック・TIM6レジスターはCubeMX側の設定を使用する。
 * 単体試験でもCyberGear通信用CAN3とPA0原点センサーは必要。CAN1/他軸は使用しない。
 */

/* 1. 起動モード・UART
 * 1=CyberGear単体試験、0=通常4軸運転。起動経路を #if で切り替える。
 * CMake の APP_CYBERGEAR_STANDALONE_TEST=HEADER（新規構成の既定値）でこの値を使う。
 * 既存の CMake キャッシュが ON/OFF の場合はそちらが優先されるため、
 * ヘッダーに統一するには -DAPP_CYBERGEAR_STANDALONE_TEST=HEADER で再構成する。
 * コンパイラの -DAPP_CYBERGEAR_STANDALONE_TEST=0/1 もこの既定値より優先する。
 */
#ifndef APP_CYBERGEAR_STANDALONE_TEST
#define APP_CYBERGEAR_STANDALONE_TEST       1
#endif

/* USART2: 8N1、フロー制御なし。端末側のボーレートも合わせる。
 * TX_TIMEOUT は1回のログ送信の待ち上限[ms]。増やすとmainの計画処理を遅らせ得る。 */
#define CYBERGEAR_UART_BAUD_RATE           115200U
#define CYBERGEAR_UART_TX_TIMEOUT_MS       10U

/* 開始は次のいずれか1文字を正常受信した場合だけ。改行不要、無入力なら無期限待機。
 * 停止文字は往復中のみ有効。停止後の再開にはリセットと新たな開始指令が必要。
 * 文字を変える場合は4つを重複させない。UPPER は別の受理文字であり自動変換ではない。 */
#define CG_TEST_START_COMMAND             's'
#define CG_TEST_START_COMMAND_UPPER       'S'
#define CG_TEST_STOP_COMMAND              'x'
#define CG_TEST_STOP_COMMAND_UPPER        'X'
/* 開始待ちUART受信1回の待ち上限[ms]。自動開始までの時間ではない。 */
#define CG_TEST_UART_START_POLL_MS         100U
/* 定期診断の間隔[ms]。短くするとUART占有が増え、計画処理を遅らせる。 */
#define CG_TEST_LOG_INTERVAL_MS            500U
/* 往復ループ/停止確認ループの休止[ms]。制御ISR周期とは別。 */
#define CG_TEST_LOOP_DELAY_MS              1U
#define CG_TEST_STOP_POLL_MS               10U
/* ログ識別子。数値は起動時に別途表示するため、調整のたびに名前を変える必要はない。 */
#define CG_TEST_FIRMWARE_TAG               "CG_HEADER_CONFIG_V1"

/* 2. 単体往復試験（通常運転には適用しない）
 * 度で編集する。RAD は内部用の換算値なので直接編集しない。
 * 0 < SMALL_AMPLITUDE_DEG <= AMPLITUDE_DEG とし、±振幅をソフト位置限界の内側に置く。
 * 原点設定後の出力軸角度。60度は片側振幅であり、全幅は120度。 */
#define CYBERGEAR_DEG_TO_RAD(degrees)      ((degrees) * 0.01745329252f)
#define CG_TEST_AMPLITUDE_DEG              60.0f
#define CG_TEST_SMALL_AMPLITUDE_DEG        5.0f
#define CG_TEST_AMPLITUDE_RAD              CYBERGEAR_DEG_TO_RAD(CG_TEST_AMPLITUDE_DEG)
#define CG_TEST_SMALL_AMPLITUDE_RAD        CYBERGEAR_DEG_TO_RAD(CG_TEST_SMALL_AMPLITUDE_DEG)

/* 往復時の総電流上限[A]と参照軌道速度上限[rad/s]。
 * 実効値=min(通常設定, 試験設定)。通常の上限を超えて引き上げる設定ではない。
 * 電流を上げる前に機械干渉・電源・発熱を確認する。原点探索の制限にはならない。 */
#define CG_TEST_CURRENT_LIMIT_A           3.0f
#define CG_TEST_SPEED_RAD_S               0.3f
/* 観測器帯域の試験用上限[rad/s]。実効値=min(CYBERGEAR_OBSERVER_RAD_S, この値)。
 * 6は遅延・ノイズで電流変化率制限が持続するのを抑えるための設定。
 * 下げるとノイズへの反応は弱まるが、負荷変動の推定も遅くなる。速度上限ではない。 */
#define CG_TEST_OBSERVER_RAD_S            6.0f
/* 外乱補償に割り当てる総電流の割合[0以上1未満]。3 A × 0.5 = 1.5 A。
 * 試験では通常の DISTURBANCE_LIMIT_A をこの計算値で置き換える。
 * RESERVE は軌道生成から取り置く電流割合で、DISTURBANCE以上かつ1未満が必要。
 * 両方を増やすと負荷保持の余地が増える一方、軌道の加減速に使える電流が減る。 */
#define CG_TEST_DISTURBANCE_CURRENT_FRACTION 0.5f
#define CG_TEST_RESERVE_CURRENT_FRACTION   0.5f
/* 試験では固定リークを使用。固定係数は共通の CYBERGEAR_LEAK_FIXED_S を参照。
 * 通常の LEAK_MODE と区別する。モードの意味は3節を参照。 */
#define CG_TEST_LEAK_MODE                 CYBERGEAR_LEAK_FIXED

/* 各目標で到達条件を連続して満たす必要がある時間[ms]。外れると待機をやり直す。
 * LEG_TIMEOUT は1目標の移動開始からの上限[ms]で、待機時間も含む。
 * 振幅増加・速度低下時は移動所要時間との整合を確認。時間切れは到達扱いにしない。 */
#define CG_TEST_DWELL_MS                   1000U
#define CG_TEST_LEG_TIMEOUT_MS             20000U
/* 到達判定の実測位置誤差[度]と推定速度の絶対値[rad/s]。軌道完了も必須。
 * 到達性を上げるために判定を緩めず、まず補償・帯域・負荷を調整する。 */
#define CG_TEST_REACHED_DEG                0.5f
#define CG_TEST_REACHED_RAD                CYBERGEAR_DEG_TO_RAD(CG_TEST_REACHED_DEG)
#define CG_TEST_REACHED_SPEED_RAD_S        0.03f
/* 大振幅端点から外側の追加許容角度[度]。既定は実測±63度を超えるとSTOP要求。
 * 停止完了位置を保証する値ではない。機械端までの余裕と停止距離を別途確保する。 */
#define CG_TEST_TRAVEL_GUARD_DEG           3.0f
#define CG_TEST_TRAVEL_GUARD_RAD           CYBERGEAR_DEG_TO_RAD(CG_TEST_TRAVEL_GUARD_DEG)

/* 3. ADRC（通常運転の基準値。上記の試験上書きがある項目は実効値に注意）
 * 0=100 Hz/10 ms、1=200 Hz/5 ms。TIM6/他軸の周期は変えない。
 * 200 Hzは受信遅延分布と最悪ISR時間を実機確認してから使用する。 */
#ifndef CYBERGEAR_USE_200_HZ
#define CYBERGEAR_USE_200_HZ               0
#endif
/* 制御呼出周期の許容ずれ[ms]、最後のフィードバックからの許容時間[ms]。 */
#define CYBERGEAR_TIMING_TOLERANCE_MS      1U
#define CYBERGEAR_FEEDBACK_TIMEOUT_MS      100U
/* wc[rad/s]と減衰比zeta[無次元]。位置ゲインwc²、速度ゲイン2*zeta*wc。
 * wc増加は追従を速めるが電流変動・振動を増やし得る。zeta増加は速度への減衰を強める。
 * wo[rad/s]は位置/速度/外乱推定の帯域。上げると応答とノイズ感度がともに上がる。 */
#define CYBERGEAR_CONTROL_BANDWIDTH_RAD_S  4.0f
#define CYBERGEAR_CONTROL_DAMPING_RATIO    2.0f
#define CYBERGEAR_OBSERVER_RAD_S           10.0f
/* 符号付き電流の数値が増加/減少する速さの上限[A/s]。
 * 絶対値の増減ではない。最終電流が3 A未満でもここに連続して掛かると飽和停止し得る。 */
#define CYBERGEAR_CURRENT_RISE_A_S         5.0f
#define CYBERGEAR_CURRENT_FALL_A_S         5.0f
/* 通常運転の外乱補償電流上限[A]、補償電流の変化率上限[A/s]、補償倍率[0..1]。
 * 上限不足では静止負荷を相殺できず誤差が残り得る。総電流上限内でのみ作用する。
 * 補償の予約電流（5節）を超えて設定できない。単体試験は上限だけ割合で上書きする。
 * GAIN=0では推定を継続したまま補償電流の寄与を無効にする。 */
#define CYBERGEAR_DISTURBANCE_LIMIT_A      0.5f
#define CYBERGEAR_DISTURBANCE_SLEW_A_S     1.0f
#define CYBERGEAR_COMPENSATION_GAIN        1.0f
/* 制御開始後に補償を抑える時間と、その後の立上げ時間[ms]。DELAYは0以上、RAMPは正。 */
#define CYBERGEAR_COMPENSATION_DELAY_MS    100U
#define CYBERGEAR_COMPENSATION_RAMP_MS     500U
/* 推定外乱を徐々に減衰させるリーク:
 * CYBERGEAR_LEAK_LEGACY_ERROR = 参照軌道と実測位置の誤差でNEAR/FARを補間。
 * CYBERGEAR_LEAK_FIXED = 誤差に関係なくFIXEDを使う（単体試験の既定値）。
 * CYBERGEAR_LEAK_NONE = リークによる減衰なし。外乱推定の更新は継続する。
 * 係数の単位は[1/s]。小さいほど保持負荷の補償が残るが、誤推定も残りやすい。
 * NEAR/FARの誤差は最終目標との差ではなく、その時点の参照軌道との差。
 * NEAR_RAD < FAR_RADが必要。固定モードでも設定自体の妥当性検査は行う。 */
#define CYBERGEAR_LEAK_MODE                CYBERGEAR_LEAK_LEGACY_ERROR
#define CYBERGEAR_LEAK_FIXED_S             0.03f
#define CYBERGEAR_LEAK_NEAR_S              0.5f
#define CYBERGEAR_LEAK_FAR_S               0.03f
#define CYBERGEAR_LEAK_NEAR_RAD            0.003f
#define CYBERGEAR_LEAK_FAR_RAD             0.015f

/* 4. 通信ID・機械保護範囲・入力ゲイン
 * IDは実機と一致させる。HOST_IDはmain内の他軸通信でも共有する。
 * HARDWARE_CONFIRMED=0なら管理制御の起動を拒否する。全起動経路の非常停止ではない。
 * 1は安全性の自動確認を意味しない。各値は機構・負荷に合わせて実機確認が必要。 */
#define CYBERGEAR_HOST_ID                  0xfeU
#define CYBERGEAR_MOTOR_ID                 0x7fU
#define CYBERGEAR_HARDWARE_CONFIRMED        1
/* 原点設定後の出力軸座標[rad]。SOFTは目標/参照、HARDは実測位置の保護限界。
 * HARD_MIN < SOFT_MIN < SOFT_MAX < HARD_MAX とし、通信範囲±12.5 rad内に置く。
 * 既定値は機械端の実測を保証しない。試験の±63度保護とは別に適用される。 */
#define CYBERGEAR_SOFT_MIN_RAD             -6.28
#define CYBERGEAR_SOFT_MAX_RAD             6.28
#define CYBERGEAR_HARD_MIN_RAD             -12.5f
#define CYBERGEAR_HARD_MAX_RAD             12.5f
/* 通常の総電流上限[A]、実測速度の停止閾値[rad/s]、温度の停止閾値[℃]。
 * 26 rad/sは実測保護値で、軌道の0.3/0.4 rad/sとは用途が異なる。
 * メーカー定格や連続運転の許容値を保証する初期値ではない。 */
#define CYBERGEAR_CURRENT_LIMIT_A          6.0f
#define CYBERGEAR_SPEED_TRIP_RAD_S         26.0f
#define CYBERGEAR_TEMPERATURE_TRIP_C       85.0f
/* 正方向電流に対する加速度のゲインb0[rad/s²/A]。符号が正であることを確認する。
 * FIXEDは代表値、MIN/MAXは姿勢・荷物・モデル誤差を含む上下限。
 * 軌道の電流予算にはMINを使う。到達性だけを見て無根拠に増やさない。 */
#define CYBERGEAR_B0_FIXED                 1.0
#define CYBERGEAR_B0_MIN                   0.35
#define CYBERGEAR_B0_MAX                   3.0

/* 5. 軌道生成・電流予算
 * 参照軌道の希望上限: 速度[rad/s]、加速/減速[rad/s²]、jerk[rad/s³]。
 * 実効加減速・jerkはb0下限と電流/変化率予算でも抑えるため、設定値とは限らない。 */
#define CYBERGEAR_TRAJECTORY_SPEED_RAD_S   0.4f
#define CYBERGEAR_TRAJECTORY_ACCEL_RAD_S2  1.0f
#define CYBERGEAR_TRAJECTORY_BRAKE_RAD_S2  1.0f
#define CYBERGEAR_TRAJECTORY_JERK_RAD_S3   5.0f
/* 通常運転で補償用に取り置く電流割合[0以上1未満]と変化率[A/s]。
 * 予約電流>=外乱補償上限、予約変化率<min(RISE,FALL)が必要。
 * 軌道用の電流/変化率は総予算から予約分を引いた残り。試験では割合を2節で上書き。 */
#define CYBERGEAR_DYNAMICS_RESERVE_CURRENT_FRACTION 0.5f
#define CYBERGEAR_DYNAMICS_RESERVE_SLEW_A_S 2.5f
/* 軌道時間の探索範囲[s]。MAXが短すぎると要求した移動を計画できず拒否する。
 * TARGET_TOLERANCE[rad]は軌道計画の一致許容で、試験到達判定0.5度とは別。
 * SEARCH_ITERATIONSは探索回数（2～256）。増加は計画計算時間を増やす。 */
#define CYBERGEAR_TRAJECTORY_DURATION_MIN_S 0.1f
#define CYBERGEAR_TRAJECTORY_DURATION_MAX_S 60.0f
#define CYBERGEAR_TRAJECTORY_TARGET_TOLERANCE_RAD 0.0001f
#define CYBERGEAR_TRAJECTORY_SEARCH_ITERATIONS 32U

/* 6. 制御中の保護・管理制御の起動停止
 * 位置ジャンプの追加許容量[rad]。前回位置から速度上限×受信間隔を超えた
 * 不連続量に対する余裕。実際の実装条件と合わせて変更する。 */
#define CYBERGEAR_POSITION_JUMP_RAD        0.02f
/* 参照軌道との位置誤差[rad]が閾値を超え続ける許容時間[ms]。
 * 試験の最終目標との誤差や1区間タイムアウトとは別の保護。 */
#define CYBERGEAR_TRACKING_ERROR_RAD       0.15f
#define CYBERGEAR_TRACKING_TIMEOUT_MS      500U
/* 総電流の振幅制限または変化率制限が連続する許容時間[ms]。fault=15の条件。
 * 外乱補償だけのクリップとは別。原因を調べず延長して保護を回避しない。 */
#define CYBERGEAR_SATURATION_TIMEOUT_MS    1000U
/* 停滞検出: 総電流上限に対する指令電流の割合、進捗角度[rad]、監視時間[ms]。
 * 参照軌道との誤差がTRACKING_ERROR_RADを超え、大電流が続き進捗不足なら停止。
 * 試験でも実効電流上限の同じ割合を上限に使う。 */
#define CYBERGEAR_STALL_CURRENT_FRACTION   0.8f
#define CYBERGEAR_STALL_PROGRESS_RAD       0.003f
#define CYBERGEAR_STALL_TIMEOUT_MS         1000U
/* 管理制御の停止確認などに使う静止速度[rad/s]と継続時間[ms]。
 * 試験の到達判定とは別なので、2節を変えてもこの値は変わらない。 */
#define CYBERGEAR_STATIONARY_SPEED_RAD_S   0.03f
#define CYBERGEAR_STATIONARY_DWELL_MS       100U
/* 管理制御の起動待ち、コマンド再試行間隔、停止応答待ち[ms]、停止送信の最大回数。
 * 試験開始失敗時の未管理停止も再試行間隔/最大回数を共有する。
 * 初期フィードバック確認のSTOP再送間隔は7節のPROBE_INTERVALを使う。
 * 値を0にして保護を無効化する用途には使わない。設定が不正なら起動を拒否する。 */
#define CYBERGEAR_STARTUP_TIMEOUT_MS       3000U
#define CYBERGEAR_COMMAND_RETRY_MS         50U
#define CYBERGEAR_STOP_TIMEOUT_MS          1000U
#define CYBERGEAR_STOP_MAX_ATTEMPTS        20U
/* mainの計画先行時間[ms]と計画処理が進まない場合の許容時間[ms]。
 * LEADは制御周期の2倍以上かつ整数倍、TIMEOUTはLEADより大きくする。
 * UARTログの出し過ぎ等によるmainの遅延も監視対象。 */
#define CYBERGEAR_PLANNER_LEAD_MS          50U
#define CYBERGEAR_PLANNER_TIMEOUT_MS       500U

/* 7. 原点探索（UART開始後、ADRC往復制御より前に実行）
 * PA0原点センサーを使用。FASTは初期探索、SLOWは原点端の再探索[rad/s]。
 * 大きくすると衝突・行き過ぎの危険が増える。CG_TEST_SPEED_RAD_Sは作用しない。
 * REVERSEは初期方向にこの角度[度]進んでもセンサーが変わらないとき、1回だけ反転する閾値。
 * 反転後の移動量を指定する値ではない。CLEARANCEはセンサー端からの退避角[度]。 */
#define CYBERGEAR_HOMING_FAST_SPEED_RAD_S  1.0f
#define CYBERGEAR_HOMING_SLOW_SPEED_RAD_S  0.4f
#define CYBERGEAR_HOMING_REVERSE_ANGLE_DEG 60.0f
#define CYBERGEAR_HOMING_CLEARANCE_DEG     2.0f
#define CYBERGEAR_HOMING_REVERSE_ANGLE_RAD CYBERGEAR_DEG_TO_RAD(CYBERGEAR_HOMING_REVERSE_ANGLE_DEG)
#define CYBERGEAR_HOMING_CLEARANCE_RAD     CYBERGEAR_DEG_TO_RAD(CYBERGEAR_HOMING_CLEARANCE_DEG)
/* 最初の応答/enable待ち上限、各探索段階の上限、探索中の受信鮮度上限[ms]。 */
#define CYBERGEAR_HOMING_STARTUP_TIMEOUT_MS 3000U
#define CYBERGEAR_HOMING_PHASE_TIMEOUT_MS  15000U
#define CYBERGEAR_HOMING_FEEDBACK_TIMEOUT_MS 100U
/* センサーの連続一致時間、探索確認/初期化コマンド間の待ち、応答確認再送間隔[ms]。
 * センサーの実効判定時間はPOLL刻みになる。通信やセンサーに合わせて変更する。 */
#define CYBERGEAR_HOMING_DEBOUNCE_MS       20U
#define CYBERGEAR_HOMING_POLL_MS           10U
#define CYBERGEAR_HOMING_PROBE_INTERVAL_MS 100U

/* 8. 高度な設定・比較モード・ログ
 * モデル由来b0の変化率上限[(rad/s²/A)/s]、姿勢情報の鮮度上限[ms]。
 * 通常の固定b0運用では姿勢モデルを有効にする設定ではない。 */
#define CYBERGEAR_B0_RATE_LIMIT            1.0f
#define CYBERGEAR_POSTURE_TIMEOUT_MS       100U
/* 0=電流＋外側制御、1=モーター内蔵PDとの比較。停止・再初期化時のみ反映。
 * 比較時はkp/kd/トルク上限[N m]の実機設定が必須。NANのままでは起動拒否。
 * モード変更で電流上限とトルク上限を混同しない。自動的なモード切替はない。 */
#define CYBERGEAR_COMPARE_OPERATION_MODE  0
#define CYBERGEAR_OPERATION_KP            NAN
#define CYBERGEAR_OPERATION_KD            NAN
#define CYBERGEAR_OPERATION_TORQUE_LIMIT_NM NAN
/* 1=制御診断リングをmainからUART出力。試験の定期CG TESTログとは別。
 * 制御は常にリングに記録し、満杯ならdropする。出力量とmainの遅延に注意する。 */
#define CYBERGEAR_LOG_UART                 0

static inline int cybergear_control_phase_due(unsigned int phase)
{
    return phase == 0U || (CYBERGEAR_USE_200_HZ && phase == 5U);
}

#endif
