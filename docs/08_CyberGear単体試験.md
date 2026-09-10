# CyberGear単体試験

`APP_CYBERGEAR_STANDALONE_TEST` を `1` にすると、電源投入後はUARTの開始指令を待ちます。
`s` / `S` を受信してからCyberGearだけを初期化・原点探索し、
原点基準で **+60度 → -60度 → +5度 → -5度 → +60度…** を繰り返します。
「左右60度」は片側60度（全幅120度）、「左右5度」は片側5度（全幅10度）です。
外部基板の開始指令は不要です。既定値は `1`（単体試験ON）です。`0` で通常の4軸制御に戻せます。

## 必要な接続

- CyberGear：CAN3（RX PA8 / TX PA15）、モーターID `0x7f`、ホストID `0xfe`。
- 原点探索センサー：PA0。既存の原点探索を使うため、このセンサーは必要です。
- 開始指令・ログ：USB-UARTのTXをUSART2 RX PA3へ、RXをUSART2 TX PA2へ接続し、GNDを共通にします。
  115200 bps・8N1・フロー制御なしで、`s` / `S` を送信して開始します。改行は不要です。
  UART未接続・未入力では待機を続け、モーターの初期化・原点探索・往復動作を開始しません。

CyberGearへの指令とフィードバックにCAN3は必須です。
基板間CAN1は初期化・開始せず、送受信しません。
RobStrideのID 3・4・5は初期化せず、応答も待ちません。STOPを含む全RobStride送信を禁止します。
他軸用のPC0/PC1リミットスイッチ入力とEXTI割り込みも初期化しないため、未配線で使用できます。
接続されている他軸がある場合は、試験開始前に停止させてください。

## 動作と停止条件

1. 起動時のUART受信残りを破棄し、`waiting for UART s/S to start homing` を表示します。
   `s` / `S` の正常受信まで時間制限なく待機します。他の文字や改行、受信エラーでは開始しません。
2. 開始指令の受信後、CAN3通信を開始し、CyberGearの初期化・応答確認を行います。
3. PA0センサーを使って既存の原点探索を1回実施し、原点を設定します。
4. ADRC位置制御を開始し、+60度 → -60度 → +5度 → -5度の順に目標を切り替えます。
5. 各目標で軌道完了・位置誤差±0.5度以内・推定速度0.03 rad/s以下を1秒維持してから次へ進みます。

位置制御の電流上限は3 A、目標速度上限は0.3 rad/sです。元の設定がこれより低ければ低い値を使います。
負荷を打ち消す外乱補償電流には、既存の予約電流枠（全体上限の半分、3 A設定時は1.5 A）を割り当てます。
外乱推定のリークは固定モード（既定0.03 /s）を使い、目標付近でも負荷補償を維持します。
以前は補償が0.5 Aで頭打ちになり、さらに目標付近でリークが最大0.5 /sに増えるため、
保持負荷があると位置誤差が残りました。全体電流上限・電流変化率・到達判定・保護条件は維持します。
位置フィードバックの遅延・揺れに対する電流指令の変動を抑えるため、単体試験では観測器帯域を
10 rad/sから最大6 rad/sに抑えます。位置制御帯域4 rad/sと減衰比2は維持します。
観測器を遅くすると負荷変動への応答も遅くなるため、任意の機構・負荷への適合を保証する設定ではありません。
原点探索の速度制御は既存設定のままです。
1区間20秒のタイムアウト、原点から±63度を超える実測位置、フィードバック/制御異常でSTOPを要求します。
±63度は停止要求の判定境界であり、停止完了位置ではありません。
往復中はUARTの `x` / `X` でSTOPを要求できます。
停止後は `s` / `S` を再送しても再始動せず、再試験には基板リセットと新たな開始指令が必要です。
UARTの停止入力は原点探索中には処理されません。

## 有効化・無効化とビルド

設定の編集先は **`Core/Inc/cybergear_config.h`** に統一しています。
このヘッダーの `APP_CYBERGEAR_STANDALONE_TEST` を `1` / `0` にすると、
`main.c` の `#if` で起動経路を切り替えます。`app_mode.h` はこの設定を取り込むだけです。
CMakeの `-DAPP_CYBERGEAR_STANDALONE_TEST=HEADER`（新規構成の既定値）でヘッダーに従います。
以前のビルドディレクトリに `ON` / `OFF` が保存されている場合は、次のように `HEADER` で再構成してください。
`ON` / `OFF` を明示した場合は、従来どおりヘッダーより優先して `1` / `0` を定義します。
CMake以外でもコンパイラの同名 `-D...=1` / `0` がヘッダーより優先されます。
変更は再ビルド・書込み後に反映されます。UARTからの実行時パラメーター変更ではありません。

CMake・Ninja・標準Cライブラリ付きArm GCC（STM32CubeCLTなど）をPATHに設定し、リポジトリ直下で実行します。

```sh
cmake -S . -B build/cybergear-swing-60-5 -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCATCH_FIRMWARE_BASENAME=cybergear_swing60_5_3a \
  -DAPP_CYBERGEAR_STANDALONE_TEST=HEADER
cmake --build build/cybergear-swing-60-5
```

書込み対象は `build/cybergear-swing-60-5/cybergear_swing60_5_3a.elf` です。
起動ログの識別文字列は `CG_HEADER_CONFIG_V1` です。角度・開始/停止文字は設定から表示します。
既定の目標値は `1.0472 / -1.0472 / 0.0873 / -0.0873 rad` です。
定期ログの `err` は実測位置誤差[rad]、`vest` は到達判定に使う推定速度[rad/s]、
`done` は軌道完了、`dwell` は到達条件を継続して満たしている時間[ms]です。
`itrk` / `idist` は追従/外乱補償電流[A]、`dlim` は補償電流上限[A]、
`dclip`（旧ログでは `clipped`）は補償電流の振幅・変化率制限の有無を示します。
`ireq` は最終制限前の要求電流[A]、`amp` / `slew` は全体電流の振幅/変化速度制限、
`sat_ms` はいずれかの制限が連続している時間[ms]、`track` は生成軌道と実測位置の差[rad]です。
`fault=15` は全体電流の振幅または変化速度制限が1000 ms続いたことによる停止です。
電流が3 A未満でも変化速度制限が続けば停止します。この保護は変更していません。
停止判定は通常電流の送信前なので、停止ログの `i_cmd` は最後に送信キューへ入った値です。
無効化するときはCMakeで `-DAPP_CYBERGEAR_STANDALONE_TEST=OFF` を指定して再構成・再ビルドしてください。

## パラメーターの変更方法

`Core/Inc/cybergear_config.h` に、単位・増減時の影響・通常運転と単体試験の違い・安全上の注意を日本語で記載しています。
`main.c` や `cybergear_test_motion.h` 内の定数を探して編集する必要はありません。
ヘッダーの節ごとの主な設定は次のとおりです。

| 節 | 変更できる内容 | 主なマクロ |
| --- | --- | --- |
| 1 | 試験ON/OFF、UART速度・開始/停止文字、ログ周期 | `APP_CYBERGEAR_STANDALONE_TEST`, `CG_TEST_START_COMMAND`, `CG_TEST_LOG_INTERVAL_MS` |
| 2 | 大小振幅、到達待機、区間タイムアウト、試験用電流・速度・観測器 | `CG_TEST_AMPLITUDE_DEG`, `CG_TEST_SMALL_AMPLITUDE_DEG`, `CG_TEST_DWELL_MS`, `CG_TEST_CURRENT_LIMIT_A` |
| 3 | ADRC帯域・減衰、電流変化率、外乱補償・リーク | `CYBERGEAR_CONTROL_BANDWIDTH_RAD_S`, `CYBERGEAR_CURRENT_RISE_A_S`, `CYBERGEAR_LEAK_FIXED_S` |
| 4 | 通信ID、ソフト/ハード位置範囲、速度・温度保護、入力ゲイン | `CYBERGEAR_MOTOR_ID`, `CYBERGEAR_SOFT_MIN_RAD`, `CYBERGEAR_B0_FIXED` |
| 5 | 軌道速度・加減速・jerk、補償用予約電流 | `CYBERGEAR_TRAJECTORY_SPEED_RAD_S`, `CYBERGEAR_DYNAMICS_RESERVE_CURRENT_FRACTION` |
| 6 | 追従誤差・電流飽和・停滞の停止、起動停止・計画時間 | `CYBERGEAR_SATURATION_TIMEOUT_MS`, `CYBERGEAR_TRACKING_TIMEOUT_MS` |
| 7 | 原点探索の速度・角度・デバウンス・タイムアウト | `CYBERGEAR_HOMING_FAST_SPEED_RAD_S`, `CYBERGEAR_HOMING_PHASE_TIMEOUT_MS` |
| 8 | 姿勢モデル、内蔵PD比較、制御診断ログ | `CYBERGEAR_COMPARE_OPERATION_MODE`, `CYBERGEAR_LOG_UART` |

例えば大振幅だけ変える場合は `CG_TEST_AMPLITUDE_DEG` を編集します。内部用の `_RAD` は度から自動換算します。
大小振幅は `0 < 小振幅 <= 大振幅` とし、機械の可動範囲と区間所要時間を確認してください。
`CG_TEST_CURRENT_LIMIT_A`・`CG_TEST_SPEED_RAD_S`・`CG_TEST_OBSERVER_RAD_S` は通常設定との小さい方が実効値です。
外乱補償は `CG_TEST_DISTURBANCE_CURRENT_FRACTION × 実効電流上限`、予約電流は
`CG_TEST_RESERVE_CURRENT_FRACTION × 実効電流上限` です。補償割合以下に予約割合を下げないでください。
原点探索は別設定であり、試験の3 A・0.3 rad/s制限やUART停止文字では保護されません。
到達判定・電流制限・保護停止の既定値は変更していません。停止を回避する目的だけで保護値を緩めないでください。

設定検査と既存の制御保護を使用します。
停止距離の予測保護モデルは既存実装で省略されており、この試験でも追加しません。

ホストテストはヘッダーの変更値の反映、UART開始待ち、目標の順序・待機・範囲逸脱・故障ラッチ・時刻の周回、CAN1/他軸との分離を検査します。
±0.94 A相当の一定負荷を与える合成プラントで、実ドライバー・軌道・ADRC・量子化された位置フィードバックを
組み合わせ、到達誤差±0.5度・推定速度0.03 rad/s以下・待機1秒のまま2周完了することも検査します。
これは実機の負荷を同定したモデルではありません。
さらに11 ms周期の受信、指令・位置それぞれ10 msの遅延、0.003 rad振幅の位置ノイズを与える試験で、
旧観測器帯域10 rad/sの `fault=15` を再現し、6 rad/sでは同じ保護・到達条件で2周完了することを確認します。
持続する振幅制限・変化速度制限（増減の反転を含む）で1000 ms後に停止する回帰テストも実施します。
実機での往復動作、機械許容範囲、停止位置は未検証です。
