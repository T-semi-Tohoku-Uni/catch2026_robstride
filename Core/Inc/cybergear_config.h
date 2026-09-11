#ifndef CYBERGEAR_CONFIG_H
#define CYBERGEAR_CONFIG_H

#include <math.h>

/*
 * CyberGear 実機設定の入口
 * ======================
 * このファイルは起動時の既定値。編集後に再ビルド・書込みすると反映される。
 * 冒頭の重要パラメーターは単体試験中にUSART2からも変更できる（通常4軸運転は対象外）。
 *   set current 2.5\n  : キーと値を空白で区切り、末尾に改行を送信する。
 *   reinit\n           : 現在の設定を保持したまま停止・再初期化する。
 *   s\n または S\n     : 再初期化後、原点探索と往復を開始する。
 *   x または X        : 即時停止要求。開始とは異なり改行を待たない。
 * 設定値を受理すると必ず安全停止を確認してから再初期化し、自動では再始動しない。
 * 再設定後は再度開始指令が必要。不正値・整合しない組合せは拒否する。
 * 原点探索中はx/Xの停止のみ受け付け、set/reinit等の入力は破棄する。
 * UART変更はRAM内のみで、電源断/MCUリセットではこのヘッダーの既定値へ戻る。
 * 通常運転と単体試験の設定を集約し、初期値は集約前の動作値を維持している。
 * 実機の寸法・定格・電流の符号を推測して変更しない。NAN は「未設定」を表す。
 * 汎用制御ライブラリのホスト試験用既定値とは別に、実機の config_defaults へ適用する。
 *
 * まず変更する場所:
 *   重要な調整値・上限 → 0、試験 ON/OFF・UART → 1、到達判定 → 2、
 *   ADRC詳細 → 3、通信ID → 4、軌道詳細 → 5、時間監視 → 6、原点探索 → 7。
 * 既定の単体試験: +60 → -60 → +5 → -5 度、各点で1秒待機、
 *   電流上限3 A、軌道速度上限0.3 rad/s、観測器帯域6 rad/s、外乱補償上限1.5 A。
 * 到達条件は位置誤差0.5度以内・推定速度0.03 rad/s以下・軌道完了のまま。
 *
 * 注意: 試験電流・速度上限は原点探索後の ADRC に適用する。
 * 原点探索は別の速度制御であり、原点探索の速度は7節で指定する。
 * 0.3 rad/s は参照軌道の上限であり、実測速度の保護停止閾値ではない。
 * GPIO/CAN配線・クロック・TIM6レジスターはCubeMX側の設定を使用する。
 * 単体試験でもCyberGear通信用CAN3とPA0原点センサーは必要。CAN1/他軸は使用しない。
 */

/* 0. 重要な調整値（コメントの UART キーで実行中にも変更可能）
 * ---------------------------------------------------------------------
 * 以下は初期値であり、UART変更でマクロやFlash自体が書き換わるわけではない。
 * set は1項目ずつ検証する。依存関係のある値は成立する順に変更する。
 * 例: 外乱割合を増やすなら先に reserve_fraction を増やす。
 */

/* UART: current [A]、speed [rad/s]。往復ADRCの総電流/参照軌道速度の上限。
 * 起動時はmin(通常設定, 試験設定)。UARTでは指定値を使用するが、固定上限は超えられない。
 * 電流増加は発熱・機械への力を増やす。試験電流上限は原点探索には作用しない。
 * speedは実測速度の停止閾値ではなく、参照軌道の上限である。 */
#define CG_TEST_CURRENT_LIMIT_A           5.0f
#define CG_TEST_SPEED_RAD_S               0.8f

/* UART: wc [rad/s]、zeta [無次元]。位置ゲインwc²、速度ゲイン2*zeta*wc。
 * wcを上げると追従が速くなる一方、電流変動・振動が増え得る。
 * zetaを上げると速度への減衰が強くなるが、目標付近の整定が遅くなる場合がある。 */
#define CYBERGEAR_CONTROL_BANDWIDTH_RAD_S  8.0f
#define CYBERGEAR_CONTROL_DAMPING_RATIO    1.0f
/* UART: wo [rad/s]。位置/速度/外乱推定の試験用帯域。
 * 起動時はmin(CYBERGEAR_OBSERVER_RAD_S, この値)。UARTでは固定上限内で指定可能。速度制限ではない。
 * 6は遅延・ノイズによる電流変化率制限の持続を抑えるための既定値。
 * 下げるとノイズ感度が下がる一方、負荷変化の推定も遅くなる。 */
#define CG_TEST_OBSERVER_RAD_S            6.0f
/* UART: b0 [rad/s²/A]。正方向電流に対する加速度ゲインの代表値。
 * 符号・慣性を実機確認する。値だけを増やして到達性を保証するものではない。
 * 下記B0_MIN/MAX内で設定し、軌道の電流予算には保守側のB0_MINを用いる。 */
#define CYBERGEAR_B0_FIXED                 1.0

/* UART: accel、brake [rad/s²]、jerk [rad/s³]。参照軌道の希望上限。
 * 実効値はb0下限・電流/変化率予算からも抑制されるため、この値とは限らない。
 * UARTからはこの起動時上限を超えて増やせない。引上げにはヘッダー変更と再ビルドが必要。
 * 増加は加減速力・反転時の衝撃を増やし得る。brakeは実機停止距離の保証ではない。 */
#define CYBERGEAR_TRAJECTORY_ACCEL_RAD_S2  2.0f
#define CYBERGEAR_TRAJECTORY_BRAKE_RAD_S2  2.0f
#define CYBERGEAR_TRAJECTORY_JERK_RAD_S3   8.0f
/* UART: rise、fall [A/s]。符号付き指令電流の数値が増加/減少する速さの上限。
 * 電流絶対値の増減ではない。総電流が上限未満でも連続して掛かると飽和復帰の対象になる。
 * 予約変化率reserve_slewより大きい値が必要。大きくすると電流の急変を許す。 */
#define CYBERGEAR_CURRENT_RISE_A_S         10.0f
#define CYBERGEAR_CURRENT_FALL_A_S         10.0f

/* UART: dist_fraction、reserve_fraction [0以上1未満]。
 * 外乱補償上限=総電流×dist_fraction。既定は3 A×0.5=1.5 A。
 * reserve_fractionは軌道生成から取り置く電流割合で、dist_fraction以上が必要。
 * 両者を増やすと負荷保持の余地が増える一方、加減速用の電流が減る。
 * 単体試験では通常のDISTURBANCE_LIMIT_A/予約割合をこの計算値で置き換える。 */
#define CG_TEST_DISTURBANCE_CURRENT_FRACTION 0.5f
#define CG_TEST_RESERVE_CURRENT_FRACTION   0.5f
/* UART: dist_slew、reserve_slew [A/s]。
 * dist_slewは補償電流の変化率上限、reserve_slewは軌道生成から取り置く変化率。
 * reserve_slew < min(rise, fall) が必要。予約後の残りを軌道生成で使用する。 */
#define CYBERGEAR_DISTURBANCE_SLEW_A_S     1.5f
#define CYBERGEAR_DYNAMICS_RESERVE_SLEW_A_S 2.5f
/* UART: gain [0..1]。推定外乱から補償電流への倍率。0は推定を残して補償のみ無効。
 * UART: leak [1/s]。固定リーク係数。小さいほど負荷補償も誤推定も残りやすい。
 * leakは試験既定の固定モードで作用する。誤差依存/無リークモードの詳細は3節。 */
#define CYBERGEAR_COMPENSATION_GAIN        1.0f
#define CYBERGEAR_LEAK_FIXED_S             0.03f

/* UART: amp、small [度]。原点設定後の +amp → -amp → +small → -small を反復。
 * 60度は片側振幅であり全幅120度。0 < small <= amp としソフト限界の内側に置く。
 * 到達判定は2節に固定し、振幅変更では緩めない。実測位置の試験保護範囲もampに追従する。 */
#define CG_TEST_AMPLITUDE_DEG              120.0f
#define CG_TEST_SMALL_AMPLITUDE_DEG        5.0f
/* UART: dwell_ms、leg_ms [ms、整数]。
 * dwell_msは到達条件を連続して満たす時間。外れると待機をやり直す。
 * leg_msは1目標の移動開始からの上限で待機時間も含む。0 < dwell_ms < leg_ms が必要。
 * 振幅増加/速度低下時は所要時間との整合を確認。時間切れは到達扱いにしない。 */
#define CG_TEST_DWELL_MS                   1000U
#define CG_TEST_LEG_TIMEOUT_MS             30000U

/* 0-B. 固定の上限・機械保護（UARTから変更不可、変更には再ビルドが必要）
 * 通常の総電流上限[A]、参照軌道速度上限[rad/s]、観測器帯域上限[rad/s]。
 * 単体試験はこれらを超えない。既定値は機械の許容値を保証するものではない。
 * 通常4軸運転では上記の試験用設定ではなく、これらを直接使用する。 */
#define CYBERGEAR_CURRENT_LIMIT_A          6.0f
#define CYBERGEAR_TRAJECTORY_SPEED_RAD_S   1.5f
#define CYBERGEAR_OBSERVER_RAD_S           10.0f
/* UARTで許可するwc[rad/s]・減衰比・1区間時間[ms]の検査限界。
 * 帯域/減衰比は上限以下、区間時間は上限未満。機構の安定性を保証する上限ではない。 */
#define CG_TUNING_MAX_BANDWIDTH_RAD_S      20.0f
#define CG_TUNING_MAX_DAMPING_RATIO        10.0f
#define CG_TUNING_LEG_TIMEOUT_CEILING_MS   60000U
/* 実測速度の停止閾値[rad/s]、温度の停止閾値[℃]。
 * 26 rad/sは実測保護であり、軌道の0.3/0.4 rad/sとは用途が異なる。
 * メーカー定格や連続運転の許容値を保証する初期値ではない。 */
#define CYBERGEAR_SPEED_TRIP_RAD_S         26.0f
#define CYBERGEAR_TEMPERATURE_TRIP_C       85.0f
/* 原点設定後の出力軸座標[rad]。SOFTは目標/参照、HARDは実測位置の保護限界。
 * HARD_MIN < SOFT_MIN < SOFT_MAX < HARD_MAX とし、通信範囲±12.5 rad内に置く。
 * 既定値は機械端の実測を保証しない。試験のamp+3度の保護とは別に適用される。 */
#define CYBERGEAR_SOFT_MIN_RAD             -6.28
#define CYBERGEAR_SOFT_MAX_RAD             6.28
#define CYBERGEAR_HARD_MIN_RAD             -12.5f
#define CYBERGEAR_HARD_MAX_RAD             12.5f
/* b0の上下限[rad/s²/A]。姿勢・荷物・モデル誤差を含め、実機に合わせて確認する。
 * 軌道の電流予算にはMINを使う。到達性だけを見て無根拠に増やさない。 */
#define CYBERGEAR_B0_MIN                   0.35
#define CYBERGEAR_B0_MAX                   3.0

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

/* 開始は次の文字と改行を受信した場合だけ。無入力なら無期限待機。
 * 停止文字は改行を待たず処理する。再開には再初期化と新たな開始指令が必要。
 * 文字を変える場合は4つを重複させない。UPPER は別の受理文字であり自動変換ではない。 */
#define CG_TEST_START_COMMAND             's'
#define CG_TEST_START_COMMAND_UPPER       'S'
#define CG_TEST_STOP_COMMAND              'x'
#define CG_TEST_STOP_COMMAND_UPPER        'X'
/* コマンド1行と割込み受信リングの容量[byte]。終端領域も含む。
 * 入力超過を許可するために保護を外さず、端末から1行ずつ送信する。 */
#define CG_CONSOLE_LINE_CAPACITY           96U
#define CG_CONSOLE_RX_CAPACITY             256U
/* 定期診断の間隔[ms]。短くするとUART占有が増え、計画処理を遅らせる。
 * 異常・開始失敗・停止時は間隔を待たず状況を1回出力し、以降の定期送信を停止する。
 * 次のUART指示には応答する。show/helpは状況と設定を出力後、再び待機する。
 * 再初期化成功後に定期送信を再開する。UART受信・CAN停止処理は継続する。 */
#define CG_TEST_LOG_INTERVAL_MS            500U
/* 往復ループ/停止確認ループの休止[ms]。制御ISR周期とは別。 */
#define CG_TEST_LOOP_DELAY_MS              1U
#define CG_TEST_STOP_POLL_MS               10U
/* ログ識別子。数値は起動時に別途表示するため、調整のたびに名前を変える必要はない。 */
#define CG_TEST_FIRMWARE_TAG               "CG_UART_TUNING_V2"

/* 2. 単体往復試験の詳細（通常運転には適用しない）
 * 振幅・電流・速度・補償割合・待機時間の既定値は冒頭0節。
 * RADは内部用の換算値なので直接編集しない。 */
#define CYBERGEAR_DEG_TO_RAD(degrees)      ((degrees) * 0.01745329252f)
#define CG_TEST_AMPLITUDE_RAD              CYBERGEAR_DEG_TO_RAD(CG_TEST_AMPLITUDE_DEG)
#define CG_TEST_SMALL_AMPLITUDE_RAD        CYBERGEAR_DEG_TO_RAD(CG_TEST_SMALL_AMPLITUDE_DEG)
/* 試験では固定リークを使用。固定係数は共通の CYBERGEAR_LEAK_FIXED_S を参照。
 * 通常の LEAK_MODE と区別する。モードの意味は3節を参照。 */
#define CG_TEST_LEAK_MODE                 CYBERGEAR_LEAK_FIXED

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
/* 通常運転の外乱補償電流上限[A]。変化率上限/補償倍率は冒頭0節。
 * 上限不足では静止負荷を相殺できず誤差が残り得る。総電流上限内でのみ作用する。
 * 補償の予約電流（5節）を超えて設定できない。単体試験は上限だけ割合で上書きする。
 */
#define CYBERGEAR_DISTURBANCE_LIMIT_A      0.5f
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
#define CYBERGEAR_LEAK_NEAR_S              0.5f
#define CYBERGEAR_LEAK_FAR_S               0.03f
#define CYBERGEAR_LEAK_NEAR_RAD            0.003f
#define CYBERGEAR_LEAK_FAR_RAD             0.015f

/* 4. 通信ID（機械保護範囲・入力ゲインは冒頭0節）
 * IDは実機と一致させる。HOST_IDはmain内の他軸通信でも共有する。
 * HARDWARE_CONFIRMED=0なら管理制御の起動を拒否する。全起動経路の非常停止ではない。
 * 1は安全性の自動確認を意味しない。各値は機構・負荷に合わせて実機確認が必要。 */
#define CYBERGEAR_HOST_ID                  0xfeU
#define CYBERGEAR_MOTOR_ID                 0x7fU
#define CYBERGEAR_HARDWARE_CONFIRMED        1

/* 5. 軌道生成・電流予算（速度・加減速・jerk上限は冒頭0節）
 * 通常運転で補償用に取り置く電流割合[0以上1未満]。予約変化率は冒頭0節。
 * 予約電流>=外乱補償上限、予約変化率<min(RISE,FALL)が必要。
 * 軌道用の電流/変化率は総予算から予約分を引いた残り。試験では割合を0節で上書き。 */
#define CYBERGEAR_DYNAMICS_RESERVE_CURRENT_FRACTION 0.5f
/* 軌道時間の探索範囲[s]。MAXが短すぎると要求した移動を計画できず拒否する。
 * TARGET_TOLERANCE[rad]は軌道計画の一致許容で、試験到達判定0.5度とは別。
 * SEARCH_ITERATIONSは探索回数（2～256）。増加は計画計算時間を増やす。 */
#define CYBERGEAR_TRAJECTORY_DURATION_MIN_S 0.1f
#define CYBERGEAR_TRAJECTORY_DURATION_MAX_S 60.0f
#define CYBERGEAR_TRAJECTORY_TARGET_TOLERANCE_RAD 0.0001f
#define CYBERGEAR_TRAJECTORY_SEARCH_ITERATIONS 32U

/* 6. 制御中の保護・管理制御の起動停止
 * CANバス状態・エラーカウンターによるfault=3の自動停止判定は行わない。
 * 受信鮮度・送信キュー投入失敗の停止判定は有効。
 * 位置ジャンプの追加許容量[rad]。前回位置から速度上限×受信間隔を超えた
 * 不連続量に対する余裕。実際の実装条件と合わせて変更する。 */
#define CYBERGEAR_POSITION_JUMP_RAD        0.02f
/* 参照軌道との位置誤差[rad]が閾値を超え続ける許容時間[ms]。
 * 試験の最終目標との誤差や1区間タイムアウトとは別の保護。 */
#define CYBERGEAR_TRACKING_ERROR_RAD       0.15f
#define CYBERGEAR_TRACKING_TIMEOUT_MS      200U
#define CYBERGEAR_TRACKING_RECOVERY_ENABLED 1
/* 総電流の振幅制限または変化率制限が連続する許容時間[ms]。
 * 外乱補償だけのクリップとは別。RECOVERY_ENABLED=1ではb0と外乱推定値を
 * 運転開始時の初期値へ戻し、補償ランプを再開して運転を継続する。
 * 追従異常もTRACKING_RECOVERY_ENABLED=1で同じ復帰処理を行う。
 * 電流モードでは復帰検出周期から電流を即座に0 Aにし、RECOVERY_ZERO_MS保持する。
 * ゼロへの切替だけは電流変化率制限を適用しない。再開時は0 Aから制限を適用する。
 * 位置・速度推定と原点・最終目標を保持し、原点探索・再初期化は行わない。
 * 古い準備済み軌道を破棄し、ゼロ保持終了時の実測位置から最終目標へ再計画する。
 * 再計画待ちはその位置を保持する参照（速度・加速度0）を使用する。
 * 各有効設定が0なら従来のfault=13/15で停止。内蔵PD比較モードも停止を維持する。
 * 停滞・温度・通信などの停止判定は別途有効。 */
#define CYBERGEAR_SATURATION_TIMEOUT_MS    1000U
#define CYBERGEAR_SATURATION_RECOVERY_ENABLED 1
#define CYBERGEAR_RECOVERY_ZERO_MS         200U
/* 停滞検出: 総電流上限に対する指令電流の割合、進捗角度[rad]、監視時間[ms]。
 * 参照軌道との誤差がTRACKING_ERROR_RADを超え、大電流が続き進捗不足なら停止。
 * 試験でも実効電流上限の同じ割合を上限に使う。 */
#define CYBERGEAR_STALL_CURRENT_FRACTION   0.8f
#define CYBERGEAR_STALL_PROGRESS_RAD       0.003f
#define CYBERGEAR_STALL_TIMEOUT_MS         1000U
/* 管理制御の停止確認などに使う静止速度[rad/s]と継続時間[ms]。
 * 試験の到達判定とは別なので、2節を変えてもこの値は変わらない。
 * ENABLE後は静止継続を待たず、新しいRun応答の実測位置・速度からADRCへ引き継ぐ。 */
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
#define CYBERGEAR_HOMING_REVERSE_ANGLE_DEG 120.0f
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
