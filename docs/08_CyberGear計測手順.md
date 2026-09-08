# CyberGear 取付状態での計測手順

設定入口は [cybergear_calibration_config.h](../Core/Inc/cybergear_calibration_config.h)。
**初期位置から左右90°が外側の固定監視上限。最初は既定±10°の内側で、短い電流試験を一回ずつ行う。**
モーターの位置feedbackを使い、原点探索・ゼロ設定・自動復帰は行わない。通常のADRC、RobStride各軸の起動、上位CANからの駆動指令は計測ビルドでは実行しない。

## 最短の手順

1. 下表の必須5値を、取付状態で許容した値に設定する。`CG_CAL_ARMED=1` にするのは設定確認後。通常制御用 `CYBERGEAR_HARDWARE_CONFIRMED=0` と未測定の b0 はそのままでよい。
2. 下記コマンドで空試験と計測ビルドを行う。既定の `ARMED=0`・`NAN` のままでもビルドできるが、実行時は起動を拒否する。
3. 後日実機を使う際、機構がゼロ電流でも落下・回転しない姿勢・支持条件を整える。UART2を **115200 bps / 8 bit / パリティなし / stop 1 bit** で開き、受信全文のファイル保存を始める。
4. `# CGCAL READY origin=...` を待つ。`p` を一文字送ると正電流の試験一回、`n` なら負電流の試験一回。Enterは不要。物理的な左右は電流・エンコーダーの符号を実測して確認する。
5. 各試行は **ゼロ電流観測 → 一定電流 → 逆電流制動 → ゼロ電流観測 → STOP**。停止確認後にCSVを出力する。出力完了後、正常終了時だけ次の `p` / `n` を受け付ける。自動で振幅を上げたり往復したりしない。`x` は中止して故障状態を保持する。
6. 同一起動のログを `capture.csv` として保存し、解析する。採用された観測値と誤差を読み、[通常設定手順](07_CyberGear簡易設定手順.md)へ反映する値を決める。

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_host_tests.ps1
cmake -S . -B build/calibration -G Ninja '-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake' '-DCMAKE_BUILD_TYPE=Debug' '-DCYBERGEAR_CALIBRATION_BUILD=ON'
cmake --build build/calibration
py -3 tools/analyze_cybergear_calibration.py capture.csv --json observation.json
```

解析はログ取得後に実行する。ビルド成果物は `build/calibration/catch2026_robstride.elf`。
上記のビルド・試験コマンドは実機への接続や書込みを行わない。
Arm GCC、CMake、Ninja、Python 3、ホスト試験用のネイティブGCCが必要。
Releaseの場合は別の出力先 `build/calibration-release` と `CMAKE_BUILD_TYPE=Release` を指定する。

## 初期位置と回転範囲

基準は起動後最初の新鮮な位置feedbackで一度だけ保存する。モード変更・ENABLEより前に取得し、`p` / `n` や試行終了で更新しない。
同一起動の試行を繰り返しても、元の基準からの変位を監視し続ける。再起動すると基準が変わるため、基準をずらして可動域を広げず、次の起動前に元の初期姿勢との関係を確認する。
角度はCyberGear出力軸feedbackのrad。外部リンク・減速機による機構角度の変換は含まない。

```text
0 < GUARD_MARGIN < TRAVEL_SOFT < TRAVEL_HARD < 90°
既定値: 2°               10°          80°

中止条件:
|現在位置 − 初期位置| + GUARD_MARGIN
    + |速度| × (GUARD_LOOKAHEAD + feedbackの古さ) ≥ TRAVEL_SOFT
```

`TRAVEL_SOFT` は目標角度ではなく中止境界なので、既定値では静止時でも変位8°以上で中止する。
`TRAVEL_HARD` と固定90°にも独立した比較を行い、90°以上のHARD設定は拒否する。
位置表現の端±12.5 radをまたぐ回避処理や角度wrapは行わず、その近傍での試験は拒否する。

この監視とSTOP送信だけでは、慣性・重力・通信断・電源断がある実機を±90°内に止め切る保証にはならない。
未知のb0で重力を支える保持制御は行わず、起動時・ゼロ電流観測中に動けば中止する。支持条件を変えると同定される負荷も変わるため、測定時の姿勢・荷物・支持状態を記録する。

## 設定値の説明

表では共通接頭辞 `CG_CAL_` を省略。全て上記の計測専用ヘッダーで設定する。
既定の電流値を推測して補う処理はない。定格や機構で許容する値が分からない場合は `NAN` のままにする。

| 設定 | 既定値・単位 | 意味・設定方法 |
|---|---|---|
| `ARMED` | 0 | 1で計測専用の起動を許可する。必須値が不正なら1でも拒否。キー入力までSTOPのみ |
| `PULSE_CURRENT_A` | **NAN・必須** [A] | 一定電流の振幅。絶対値を使用し、`p`で正、`n`で負にする。小さい許容値から始め、解析で分解できない場合も自動増幅しない |
| `BRAKE_CURRENT_A` | **NAN・必須** [A] | パルスと逆符号に出す制動電流の正の大きさ。回生・機構の許容値内とする。速度ゼロ付近・反転検出で解除 |
| `CURRENT_LIMIT_A` | **NAN・必須** [A] | 計測用の指令電流上限。両振幅がこれを超えれば設定拒否。モーターの実電流を測定して遮断する値ではない |
| `TEMPERATURE_TRIP_C` | **NAN・必須** [℃] | feedback温度がこの値以上で中止。機体の許容値を定格から決める。温度を上げて限界を探す試験は行わない |
| `SPEED_TRIP_RAD_S` | **NAN・必須** [rad/s] | 実測速度の絶対値がこの値以上で中止。静止判定幅より大きく、通信表現の30 rad/s以下。30は機構の許容速度ではない |
| `TRAVEL_SOFT_DEG` | 10° | 固定した初期位置からの試験中止境界。余裕と速度先読みで、実際にはさらに内側で中止する |
| `TRAVEL_HARD_DEG` | 80° | 実測変位の追加中止境界。SOFTより大きく90°未満。境界へ走らせる指令ではない |
| `GUARD_MARGIN_DEG` | 2° | SOFTの内側に確保する固定余裕。量子化・応答の不確かさに応じて大きくする。余裕を削って距離を稼がない |
| `GUARD_LOOKAHEAD_MS` | 100 ms | 現在速度でこの時間と受信鮮度分だけ進む距離を加算。`MAX_STEP_MS`以上、500以下。未知加速度を含む停止距離モデルではない |
| `POSITION_JUMP_RAD` | 0.01 rad | 新しい受信位置間の差が「速度上限×受信時刻差＋この値」を超えれば中止。正でGUARD_MARGIN未満。ノイズのために無制限に広げない |
| `STATIONARY_SPEED_RAD_S` | 0.02 rad/s | 静止・制動終了の速度判定幅。正でSPEED_TRIP未満。センサー量子化を上回り、許容できる低速度内に置く |
| `BASELINE_MS` | 200 ms | パルス前のゼロ電流観測時間。`3×MAX_STEP_MS`以上、1000以下。この間に動けば基準が成立しないため中止 |
| `PULSE_MS` | 200 ms | 一定電流を出す最大時間。長くすると同定しやすくなる一方、速度と移動量が増す。下限は `max(30, 3×MAX_STEP_MS)`、上限500 |
| `BRAKE_MS` | 200 ms | 逆電流制動の最大時間。下限・上限はPULSEと同じ。静止/反転検出で先に解除し、期限内に減速しなければ故障STOP |
| `SETTLE_MS` | 100 ms | 制動解除後のゼロ電流観測時間。`2×MAX_STEP_MS`以上、1000以下。速度が静止幅を超えれば中止 |
| `FEEDBACK_TIMEOUT_MS` | 30 ms | 最後の受信からこれを超えたら中止。1～100、MAX_STEP以上。遅い通信を通す目的で広げず原因を調べる |
| `MAX_STEP_MS` | 20 ms | 計測tick間隔の許容最大値。1～50、FEEDBACK_TIMEOUT以下。実周期は10 msなのでそれ未満では継続できない |
| `APP_PERIOD_MS` | **10 ms固定** | TIM6で100 Hz。通常制御用200 Hzスイッチとは独立。検証した周期を変えたビルドは拒否する |
| `APP_START_TIMEOUT_MS` | 3000 ms | 最初のSTOP・静止確認、および各試行のモード設定～Run静止確認、それぞれ全体の期限。期限超過で中止 |
| `APP_STOP_TIMEOUT_MS` | 1000 ms | STOP再送・Reset応答・静止継続を待つ最大時間。過ぎたら停止未確認のFAULT。故障後は再試行を受け付けない |
| `APP_RETRY_MS` | 20 ms | STOPとモード読出しの再試行間隔。PERIOD以上、FEEDBACK_TIMEOUT未満。STOP応答で待機中もfeedbackを更新する |
| `APP_QUIET_MS` | 100 ms | 起動前・Run確認・STOP確認で新しい低速度feedbackが続く必要時間。最低2周期。古い同じサンプルで時間を稼がない |
| `APP_LOG_CAPACITY` | 512行 | RAMへ保存する行数。100 Hzで約5秒分、最後の1行を終端情報に予約。満杯なら中止。増やす場合はビルド時のRAM使用量を確認 |

設定から構造体への反映は [cybergear_calibration_app.c](../Core/Src/cybergear_calibration_app.c) の `cg_cal_app_config_defaults()`。
許容組合せは [cybergear_calibration.c](../Core/Src/cybergear_calibration.c) の `cg_cal_config_valid()` で検査する。

## ログと中止時の扱い

制御中はRAMへ記録し、CSVのUART出力はSTOP完了またはSTOP期限終了後に行う。
`# CGCAL trial=... fault=... stop_queued=... reset=... stationary=...` の3つの結果を区別する。
`stop_queued=1` はCAN送信キューに入っただけ、`reset=1` はSTOP後の新しいReset feedback、`stationary=1` は低速度の継続を確認した状態。
故障時は最初の故障理由を保持するため、STOPも失敗した場合は `reset` / `stationary` も確認する。
起動前の故障では計測行がなく、説明行とCSVヘッダーだけの場合がある。

| 表示 | 対応 |
|---|---|
| `disabled` / `CONFIG` | 必須値・ARMED・組合せを確認。計測用のモーターIDは `main.c` の `CYBER_GEAR_ID`（現状0x7f）、ホストは0xfe |
| `FEEDBACK` / `TX` / `BUS` / `MOTOR` | CAN配線・ID・モード読戻し・モーター故障を調べる。自動clear faultや再起動は行わない |
| `POSITION` / `SPEED` / `TEMPERATURE` | 設定した試験範囲を超えた。許容範囲を広げる前に電流・時間・支持条件を見直す |
| `BASELINE_MOTION` | 起動/ゼロ電流区間で動いている。姿勢・荷物・支持条件を見直す。静止判定を緩めて重力運動を通さない |
| `BRAKE_TIMEOUT` / `STOP_TIMEOUT` | 制動終了またはSTOP後の静止を確認できていない。該当試行のゲインを採用しない |
| `TIMING` / `LOG_OVERFLOW` | 周期超過または記録満杯。割込み・記録量を調べる |
| `REQUESTED_STOP` | `x` または外部STOP要求による中止。次の試行は受け付けない |

`command_current_a` は最後に正常キュー投入した指令で、実測iqではない。`saturated=0` はこの計測コードが指令をクリップしていないことを表し、モーター内部の飽和・電流到達は確認できない。
入力ゲインはこの指令に対する有効ゲインの観測値。トルクfeedbackを実電流に読み替えない。
CSV列・解析の採否条件・全オプションは [解析説明](calibration_analysis.md) を参照。

## 実測値の反映と通常版への戻し方

| 得られる観測 | 本番設定への用途 |
|---|---|
| 正負の局所入力ゲインと推定誤差 | `B0_FIXED` の材料。負値を絶対値にせず符号確認。`B0_MIN/MAX` は全姿勢・荷物の変動と不確かさを別途含める |
| 減速度・停止変位・静止確認 | `BRAKE_GUARANTEED_RAD_S2` / `STOP_MARGIN_RAD` の検討材料。観測最小値をそのまま保証値としない |
| パルス区間の加速度 | `OUTWARD_ACCEL_RAD_S2` の検討材料。遅延中の最悪外向き加速度を一試行で網羅したことにはならない |
| feedback鮮度・受信間隔・位置近似残差 | timeoutや位置不連続判定の調整材料。物理的な指令遅延や最悪応答時間の測定ではない |

機械可動域・許容電流・許容速度・許容温度は、図面・定格・機構から先に決める。
計測で機械端に当てたり、電流・温度限界まで走らせたりして自動決定しない。
通常制御の設定をレビューしてから、通常版を別ディレクトリでビルドする。

```powershell
cmake --preset Debug -DCYBERGEAR_CALIBRATION_BUILD=OFF
cmake --build --preset Debug
```

通常版には既存の原点探索があるため、取付状態でそのまま起動せず、通常設定手順の原点・可動域確認も行う。

## 今回の空試験

実機・ST-LINK・CANアダプターを使わず、ホスト試験7/7と通常版・計測版のDebug/Releaseビルドを確認した。
ホスト試験は模擬HALの起動から停止、初期位置保持、±90°と内側保護、異常時の停止、UARTログから解析までを含む。
合成データは実機パラメーターの根拠には使わない。検査範囲は [tests/README.md](../tests/README.md) を参照。
