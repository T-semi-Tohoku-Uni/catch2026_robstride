# CyberGear 実装・パラメーター調整・空試験

2026-09-09。作業開始HEAD: `8373ec41985510667bbdd81a819d807d03bf001f`。
元説明書の基準 `2a888ed1e31dda6a063dd62f70259de9111d552a` との差分は、生成物の追跡解除とmainのEL05診断/return変更等。開始時の追跡済みソースに未コミット差分はなく、`docs/` は未追跡だった。これらの既存変更を保持した。

## 1. 実装した内容と設定場所

通常制御は電流モードを維持し、位置だけのランプから、速度・加速度・jerk・可動域を検証した五次軌道へ変更した。制御は `ad + wc²(qd-z1) + 2*zeta*wc(vd-z2)` をb0で割り、制限付きESO補償を加える。速度の目標値がD項にも入るため、従来の「動く参照に対して速度ゼロを追う」構造を解消する。

| ファイル | 役割 |
|---|---|
| `Core/Inc/cybergear_config.h` | 機械固有の必須値、速度/加減速/jerkの開始候補、100/200 Hz、比較モード、UARTログの入口 |
| `Core/Src/cybergear.c` の `cybergear_config_defaults()` | 全体保護、計画期限、初期化/停止の既定値。共通configの組立て |
| `Core/Inc/cybergear.h` | configの全フィールドと単位、状態/故障理由、ログ/API |
| `cybergear_controller.h/.c` | HAL非依存の2自由度制御、ZOH/current ESO、電流/補償制限 |
| `cybergear_dynamics.h/.c` | 校正済みb0の連続補間、モデル鮮度、全姿勢の下限b0による電流/jerk予算 |
| `cybergear_trajectory.h/.c` | 五次軌道、全区間の極値検査、任意q/v/aからの再計画 |
| `Core/Src/main.c` | CG受信の振分け、起動設定チェック、mainでの計画、CG周期、専用ログの接続 |
| `tests/` | 実機を使わないHALモック、純C計算試験、実際のTIM6/RXコールバック回帰、数値モデル |

各パラメーターの詳細は本書に加え、[軌道の全設定](trajectory_parameters.md)、[制御・慣性の全設定](controller_parameters.md)を参照する。プロトコルの±12.5 rad、±30 rad/s、±12 Nm、Kp 0..500、Kd 0..5はパケット表現の定数であり、機体の許容範囲ではない。

## 2. 実機設定の未確定値

既定ファームウェアは `CYBERGEAR_HARDWARE_CONFIRMED=0` と機械値の `NAN` により、**CyberGearのホーミングを始める前に起動を拒否する**。設定不足を埋めずに作ったELFもビルドできるが、駆動用に完成した設定ではない。ホストテストの±2 rad、2 A等は架空のfixtureであり転記しない。

| マクロ | 単位・決め方・調整時の影響 |
|---|---|
| `CYBERGEAR_HARDWARE_CONFIRMED` | 0/1。以下の機械値、b0の正方向、停止経路を確認してから1。数値チェックを飛ばすスイッチではない |
| `CYBERGEAR_SOFT_MIN_RAD/MAX_RAD` | 原点設定後のモーター出力軸rad。目標と軌道全体の範囲。狭くすると反転軌道が実行不能になる場合がある |
| `CYBERGEAR_HARD_MIN_RAD/MAX_RAD` | 実測位置の停止境界rad。softを厳密に包含し、通信範囲内。外側に機械衝突までの実際の余裕を残す |
| `CYBERGEAR_CURRENT_LIMIT_A` | 最終iqの絶対上限A。冷却・負荷・通電時間を含め連続運転で許容する値。旧10 Aを定格と扱わない |
| `CYBERGEAR_SPEED_TRIP_RAD_S` | 実測速度の停止しきい値rad/s。参照速度より大きく、機構上限以下。量子化と追従の余裕を含む |
| `CYBERGEAR_TEMPERATURE_TRIP_C` | feedback温度で停止するdeg C。搭載FWのセンサ位置と冷却条件を確認 |
| `CYBERGEAR_B0_FIXED` | 正の加速度/電流rad/s²/A。制御用の代表値。正電流で正方向に加速する座標のみ対応。負符号を絶対値化してごまかさない |
| `CYBERGEAR_B0_MIN/MAX` | 全到達姿勢・最大荷物・誤差を含む正値範囲。MINは `Ktau/J_hi` 相当で軌道制約用、FIXEDは中央推定の制御用 |
| `CYBERGEAR_BRAKE_GUARANTEED_RAD_S2` | 通常駆動が有効なときの保証減速度rad/s²。電源/回生/慣性の最悪条件で決める |
| `CYBERGEAR_OUTWARD_ACCEL_RAD_S2` | 通信/検出/電流反転待ち中に残り得る外向き加速度rad/s²。0という設定にも根拠が必要 |
| `CYBERGEAR_STOP_MARGIN_RAD` | 停止計算に加える位置不確かさrad。境界ぎりぎりに調整しない |

パラメーター変更は停止中に行い、`cybergear_configure()`で検証した一式を設定してから再起動する。実行中の構造体をデバッガで部分書換えする方式は対象外。既定値を直した再ビルド、またはアプリの停止状態で同APIを使う。永続保存や上位CANでのパラメーター転送は追加していない。

## 3. 速度を決める値

`CYBERGEAR_TRAJECTORY_SPEED_RAD_S=0.4`、`ACCEL_RAD_S2=1`、`BRAKE_RAD_S2=1`、`JERK_RAD_S3=5`は低速で構造を比較する開始候補。実機安全値ではない。電流余裕とb0下限から得る上限との小さい方を採用する。加速と制動の上限は別に保持するが、五次式全体にはその小さい方を適用するため、方向切替時の誤った加速度分類を避ける。

希望速度を上げても加速度/jerkが低ければ短い移動では最高速に達しない。低い電流スルーで滑らかさを作ろうとすると制動も遅れるため、滑らかさはまず軌道A/Jで調整する。電流の正負反転時間をログから確認して必要な場合にスルーを変更する。

`CYBERGEAR_USE_200_HZ=0`ではCGはphase 0のみ、1ではphase 0/5。TIM6は1 ms、RSはphase 1/2/3、上位返信はphase 9のまま。周波数は移動速度や制御帯域とは別物。200 Hzへ変えた場合、controllerの固定計算周期も5 msとなる。

## 4. 保護と起動の全設定

以下は `CyberGearConfig` のフィールド。既定値は試験開始候補で、機構固有の安全値ではない。下位controller/dynamics/trajectoryの全項目は各説明書に記載する。

| フィールド | 既定値 | 説明・調整指針 |
|---|---:|---|
| `position_jump_rad` | 0.02 rad | 許容位置変化を `speed_trip * 受信dt + この余裕` とする。1カウント約0.0003815 radに対する余裕。原点変更は運転中に禁止 |
| `tracking_error_rad` | 0.15 rad | 生成qdと実測qの差。最終目標との差ではない。実際の正常追従遅れより大きく、機械余裕より小さく |
| `tracking_timeout_ms` | 500 ms | 追従誤差超過の継続時間。低くすると加速中に停止しやすく、高くすると拘束時の通電が長くなる |
| `saturation_timeout_ms` | 1000 ms | 振幅またはスルー制限が続く上限。飽和をESOで押し切らず停止。意図的な長いスルー飽和も検出する |
| `stall_current_a` | Imax×0.8 A | 大電流かつ追従誤差がある場合に進行を監視。保持電流を超える値にする |
| `stall_progress_rad` | 0.003 rad | 観測開始位置からこの距離動けば拘束窓を更新。ノイズ・量子化より大きくする |
| `stall_timeout_ms` | 1000 ms | 進行不足の許容時間。tracking保護と同時なら先に成立した理由で停止する。保護は外力の一意推定ではない |
| `stationary_speed_rad_s` | 0.03 rad/s | Reset/Run時に静止とみなす速度幅。通信速度値の量子化を考慮。ゼロ電流が静止を意味するわけではない |
| `stationary_dwell_ms` | 100 ms | 新しいfeedbackを受けながら低速を継続する時間。新フレームなしでは静止確認を完了しない |
| `startup_timeout_ms` | 3000 ms | STOP、モード書込/readback、ゼロ指令、ENABLE、Run/低速確認までの全体期限 |
| `command_retry_ms` | 50 ms | 起動のSTOPとread要求、故障時STOPの再試行間隔。制御より低頻度で共有バス負荷を制限 |
| `stop_timeout_ms` | 1000 ms | STOPを再試行する最長時間。期限後はFAULTに留まり、再び通常電流を出さない |
| `stop_max_attempts` | 20回 | STOP送信試行の上限。キュー投入失敗も数える。期限との早い方で終了 |
| `planner_lead_ms` | 50 ms | mainで旧軌道の将来q/v/aを評価して新軌道を作る先行時間。5/10 ms周期の整数倍。小さ過ぎると計画結果が間に合わない |
| `planner_timeout_ms` | 500 ms | 計画が有効化されない最大時間。main停止/過負荷や解なしを無期限に放置しない |
| `transport_delay_ms` | 100 ms | 検出＋CANの保守遅延。feedback timeout以上。現在電流から制動電流までの反転時間をさらに足す |
| `operation_kp` | NAN | 比較用内蔵PDのKp [Nm/rad]、0より大きく500以下。wc²/b0 [A/rad]と混同しない |
| `operation_kd` | NAN | 比較用内蔵PDのKd [Nm s/rad]、0より大きく5以下。内部の単位/実装をFWで確認 |
| `operation_torque_limit_nm` | NAN | 比較用の0x700Bへ書くNm上限、0より大きく12以下。搭載FWでの適用確認が必要 |

`hard_min/max`、温度/速度、制動/外向き加速度、停止余裕は2章の必須マクロから設定する。`hardware_confirmed`は数値の整合性検証とは別に要求される。

停止距離の監視は、外向き速度v、遅延中加速度a_out、反転待ちを含むt_delay、保証減速度a_brakeに対し、`v*t_delay + a_out*t_delay²/2 + (v+a_out*t_delay)²/(2*a_brake) + margin` と外側境界までの距離を比較する。反転時間は `(abs(last_current)+braking_current)/min(rise,fall)`。比較PDモードでは実電流不明のためlast_currentにImaxを使う。

この式は駆動が有効なときの余裕監視であり、**STOPは能動制動ではない**。CAN断、電源断、駆動停止後の惰性をこの式で保証しない。計画失敗・モデル情報失効・境界余裕不足も、理由付きのSTOP要求へ遷移する。実機のSTOP動作/外部停止方法が不明なため、架空の能動制動動作や未知のwatchdogレジスタを追加していない。

## 5. 起動、通信、異常後の扱い

通常制御の所有権を状態機械に移した後、一般のenable/mode/current/motion/zero APIでは書き込めない。STOP要求だけは受け付ける。従来のホーミングは所有権を移す前のAPIで行い、センサ端の幾何・原点決定方法は変更していない。

起動は次の順序。固定Delayだけでは確認を代用しない。

1. STOP投入と、その後の新しいReset/低速feedbackを確認。
2. 0x7005へ運転モードを書き込み、type17でreadback。feedbackのmode=2はRun状態であって電流モードの証明ではない。
3. 電流モードは0 A、比較モードはトルク制限とゼロゲイン/ゼロトルクのmotionを送信。
4. ENABLE後、新しいRun/低速feedbackを待ち、現在位置で参照/ESOを初期化。
5. TIM6の最初のphaseが起動から1 ms等で来ても、最初の固定周期が経過するまでは制御計算を始めない。

type2/17/21は、mainの既存RS用type2フィルタより前で、CGのID、フレーム種別、DLCを検証する。type21の資料にはID方向の不一致があるため、type21だけは既知のmotor/hostの完全な組を両方向とも受け付ける。他モーターは消費しない。故障/警告8 byteは生のまま保存し、どこか非ゼロなら停止する。未検証のbit意味やendianを断定しない。

type21のラッチ後は、管理制御開始前のホーミング経路でもSTOP以外の送信を拒否する。通常制御へ入るまで故障を無視して速度指令を出し続けることはない。

TXのHAL_OKはキュー投入成功。`last_queued_current_a`とZOH入力の`applied_current_estimate_a`を区別し、後者のvalidは「正常バス・小さい遅延を仮定したモデル入力が使用可能」を表す。実電流の測定/到達ACKではない。失敗した電流はcommitしない。bus-off、error-passive、TX error counter非ゼロ、HAL状態取得失敗で入力を無効化し停止する。共有FIFO破棄・再送設定・IRQ設定は変更しない。比較PDモードでは内部電流が不明なのでESO補償を使わず、入力estimateも無効表示する。

STOP要求、STOP投入成功、Reset応答、機械的静止は別のフラグ。再試行を終えても到達できなければFAULTのまま。故障後の自動再armはない。明示的な`cybergear_reset_fault()`も、新鮮なReset/低速/故障なしと停止確認が必要で、その後も明示起動が必要。mainには自動呼出しを追加していない。

## 6. 計画と時刻の設計

根探索を含む計画は`cybergear_service()`がmainで実行する。旧軌道の将来時点から新しいq/v/aを連続に接続し、計画完了が切替時刻より前なら短い割込み禁止区間で公開する。間に合わない計画や別の軌道世代の計画は採用しない。タイマー側は時刻に従う評価と準備済み軌道への切替を行う。

同一目標の再受信では再計画しない。連続更新目標は、既に準備した軌道を所定時刻で有効化し、最新目標を次の計画へまとめる。更新ごとに準備済み軌道を破棄すると50 ms未満の目標更新で永遠に始まらないためである。追従対象は最大で計画先行時間とmain実行間隔程度遅れる。上位軌道を厳密な時刻でストリーミング再現するインターフェースではない。

完了軌道は定位置保持へ置き直し、長時間保持後のuint32 tick wrapで元の移動が再生されるのを防ぐ。

ESOはdocsで許された**固定100/200 Hzの近似**を採用。予測は前回の投入成功電流で毎周期、測定補正は新しいrx_sequenceに対して一度だけ。受信位置をその制御時刻の測定として近似配置する。受信間隔をdtへ混ぜず、通常周期±1 msを超える呼出しは停止。受信タイムスタンプは鮮度・逆行検出に使う。真のモーター測定時刻や送信区間履歴に完全整合する遅延補償ではないため、高帯域化・200 Hz化前に遅延分布を実測する。

## 7. 可変慣性への接続

姿勢モデルを使わなくても、全姿勢を含むb0下限と電流予算でA/Jを設定できる。b0スケジューリングは既定で無効。機構の寸法、他軸の原点、payload状態が未確定なので、既存のtarget_angleを実姿勢とみなす接続は追加していない。

校正したスカラー姿勢指標とb0の2〜8点を `config.dynamics.points[]` に設定し、`cybergear_set_posture()`へ実姿勢の値、元の測定時刻、validを渡すと有効化できる。機構が複数の独立姿勢軸を持つなら、外部で検証した投影/モデルが必要。単に他軸の目標角を代入することはできない。

b0は上下限・変化率制限を通り、ESOは旧区間を旧b0で予測した後、`z3_new=z3_old+(b_old-b_new)*i_est`で再表現する。既知の干渉FFや摩擦FFは追加せず、総外乱の定義を維持。J、Ktの独立値は未確定なのでJ_hat/J_hiを架空計算せず、校正対象のb0と保守下限で表現する。

姿勢情報が古い/範囲外ならdynamicsは有界な固定b0への退避値を計算するが、ドライバは運転を継続せずSTOPへ遷移する。誤ったモデルへの自動継続/再armはしない。全姿勢制約は固定なので、姿勢失効時に参照加速度を突然クリップすることもない。

## 8. ログと比較

`CYBERGEAR_LOG_CAPACITY=32`。100 Hzなら約320 ms、200 Hzなら約160 ms分。タイマーでは数値構造体だけを記録し、満杯ではdropを数える。`latest_log`は満杯でも更新するので直近の故障理由を取得できる。`cybergear_pop_log()`でmainから排出する。ログ未排出なら最初の32件と最新1件が残る。

`CYBERGEAR_LOG_UART=1`でmainから1件ずつ短い行に分けて出す。これは既存UARTの帯域では全件連続記録を保証しない。全列は`CyberGearLog`/`latest_log`、完全取得は別の十分な帯域の排出系で扱う。計画より先に長文出力を置かない。

時刻/鮮度/dtはms。q/qd/z1はrad、vd/v_feedback/z2はrad/s、ad/z3はrad/s²、i_track/i_dist/i_req/i_cmdはA。b0はrad/s²/A、b0_rateはその毎秒変化、A/B/Jは有効な軌道制約。posture_indexは校正した指標、posture_timestampは実姿勢測定の元時刻。z3_before/afterはモデル再表現前後。tx_queued/failedは投入の累計、fifo_freeは投入直前の空き数、tec/recはCANエラーカウンタ。applied_estimate_validは実電流測定の有効性を意味しない。内蔵PDではA単位の要求/飽和ログは未使用のゼロ値として扱う。

比較PDは`CYBERGEAR_COMPARE_OPERATION_MODE=1`と未設定のkp/kd/torque上限を設定した上で明示再起動。電流制御と同じqd/vdを送り、トルクFFは0。走行中の自動モード切替はない。

## 9. 空試験と実機に残す作業

本作業では書込み、OpenOCD、ST-Link、デバッグセッション、CAN接続を実行していない。全てPC上のコンパイル/HALモック/合成モデルによる試験。

```powershell
# PATHにCMake/Ninja、ネイティブGCCまたはCLion同梱GCC、Pythonを用意
powershell -ExecutionPolicy Bypass -File tests/run_host_tests.ps1

# Arm GNU Toolchainのあるシェルで、ビルドのみ
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release
```

テスト内容/実行結果と模擬モデルの制限は[検証記録](06_空試験結果.md)、[数値モデル結果](offline_simulation_results.md)を参照。Windows同梱MinGWでASan/UBSanの実行時ライブラリが無い場合、未実行を合格と扱わない。対応環境では`-Sanitizers`オプションを使える。

実機ではまず機械必須値、原点後の静止、run_mode readback、正負のb0、STOPの動作と通信タイムアウトを確認し、低いV/A/Jでログを取る。その後、最小/最大慣性、伸縮中、荷物変化、正負反転を同じ動作で比較してパラメーターを調整する。モデル上の良好な追従は実機安定性の証明ではない。

## 10. 対象外・戻し方

RobStrideの値/軌道/周期/角度オフセット、上位CAN ID/DLC/endian、return待ち、全体監視/reset、ホーミングの幾何、共有CAN設定/GPIO/IRQは変更していない。指定された`stm32g4xx_hal_msp.c`も確認したが、共有設定なので変更不要と判断した。

`03_範囲外の不具合候補.md`の全体Error_Handler、上位値の符号付きシフト、他軸個別監視、return完了判定等は引き続き別作業。モードreadbackを必須にしたため、応答形式が異なる搭載FWでは起動が失敗し得る。実機ログで合わせる。

コミットは作成していない。元に戻す場合は今回の `cybergear.c/.h`、mainのCG接続、CMakeの新規3ソース登録を作業開始HEADの該当差分に戻し、新規モジュールをビルド対象から外す。README/追加説明/テストは残しても駆動に影響しない。作業開始時から存在したdocsや他の人の後続変更を一括削除・resetしない。比較モードやgamma=0は旧実装へのロールバックとは異なる。
