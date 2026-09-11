# CyberGear 本番制御パラメーター

設定入口は `Core/Inc/cybergear_config.h`。`cybergear_config_defaults()` が通常4軸運転へ適用する値を示す。
汎用の `cybergear_controller_default_config()` の既定値とは区別する。
設定変更は停止中に行い、再ビルド・書込み・再初期化で反映する。
このブランチにUARTによる設定変更や単体往復試験はない。
試験からの取込み方法は [cybergear_promotion.md](cybergear_promotion.md) を参照する。

## 採用した設定

初回の出典は試験ブランチの `1760d43e008fe0dfa589e8733b10e2f285bf8d82`。
試験用上書きを含めた実効値を通常運転の値へ移している。
ヘッダーに保存されていないUART上のRAM設定はこの出典に含まれない。

2026-09-11の軽微異常からの復帰・通常停止の変更は、利用者の指示により
`20_cybergear_fix` へ直接適用した。上記出典の実機試験済み制御とは区別し、
今回の変更はホスト回帰試験とARMビルドで確認する。実機での動作確認は未実施。

| 設定 | 本番での値 | 意味 |
|---|---:|---|
| `CYBERGEAR_CURRENT_LIMIT_A` | 5 A | 最終指令電流の絶対値上限 |
| `CYBERGEAR_TRAJECTORY_SPEED_RAD_S` | 0.8 rad/s | 参照軌道の速度上限 |
| `CYBERGEAR_CONTROL_BANDWIDTH_RAD_S` | 8 rad/s | 制御帯域 `wc` |
| `CYBERGEAR_CONTROL_DAMPING_RATIO` | 1 | 減衰比 `zeta` |
| `CYBERGEAR_OBSERVER_RAD_S` | 6 rad/s | 観測器帯域 `wo` |
| `CYBERGEAR_B0_FIXED` | 1 rad/s²/A | 固定公称入力ゲイン |
| `CYBERGEAR_B0_MIN` / `CYBERGEAR_B0_MAX` | 0.35 / 3 rad/s²/A | 入力ゲインの許可範囲 |
| `CYBERGEAR_TRAJECTORY_ACCEL_RAD_S2` / `CYBERGEAR_TRAJECTORY_BRAKE_RAD_S2` | 2 / 2 rad/s² | 軌道の加速・減速上限 |
| `CYBERGEAR_TRAJECTORY_JERK_RAD_S3` | 8 rad/s³ | 軌道のjerk上限 |
| `CYBERGEAR_CURRENT_RISE_A_S` / `CYBERGEAR_CURRENT_FALL_A_S` | 10 / 10 A/s | 符号付き指令電流の増加・減少率上限 |
| `CYBERGEAR_DISTURBANCE_LIMIT_A` | 2.5 A | 外乱補償電流の絶対値上限 |
| `CYBERGEAR_DISTURBANCE_SLEW_A_S` | 1.5 A/s | 外乱補償だけの変化率上限 |
| `CYBERGEAR_DYNAMICS_RESERVE_CURRENT_FRACTION` | 0.5 | 軌道用に使わず予約する電流割合。5 A × 0.5 = 2.5 A |
| `CYBERGEAR_DYNAMICS_RESERVE_SLEW_A_S` | 2.5 A/s | 軌道用に使わず予約する電流変化率 |
| `CYBERGEAR_COMPENSATION_GAIN` | 1 | 外乱補償倍率の到達値 |
| `CYBERGEAR_COMPENSATION_DELAY_MS` / `CYBERGEAR_COMPENSATION_RAMP_MS` | 100 / 500 ms | 補償開始の待ち時間・立上げ時間 |
| `CYBERGEAR_LEAK_MODE` / `CYBERGEAR_LEAK_FIXED_S` | `CYBERGEAR_LEAK_FIXED` / 0.03 s⁻¹ | 誤差に依存しない固定リーク |
| `CYBERGEAR_USE_200_HZ` | 0 | 100 Hz、10 ms周期 |

角度・速度・加速度はCyberGear出力軸座標で統一する。
速度0.8 rad/sは参照軌道の上限であり、実測速度の保護閾値や原点探索速度ではない。
電流のrise/fallは符号付き数値の増減であり、絶対値の増減ではない。

## 制御と軌道の実効制約

```text
i_track = (ad + wc²*(qd-z1) + 2*zeta*wc*(vd-z2)) / b0
i_dist_requested = -gamma*z3 / b0
i_dist = 外乱補償の振幅制限・変化率制限(i_dist_requested)
i_requested = i_track + i_dist
i_amplitude = clamp(i_requested, -current_limit, +current_limit)
i_command = clamp(i_amplitude, last_queued-fall*dt, last_queued+rise*dt)
```

生成位置 `qd`、速度 `vd`、加速度 `ad` を同時に参照する。
`z3` は負荷・摩擦・軸間干渉・モデル誤差を含む総外乱加速度で、独立した負荷FFは加えていない。
最終的な軌道制約はヘッダー上の加速度・jerkだけでなく、保守側の `b0_min` と電流予算から決まる。
現在は姿勢によるb0スケジュールを無効とし、固定b0で運転する。

```text
accel = min(2, 0.35*(5-2.5)) = 0.875 rad/s²
brake = min(2, 0.35*(5-2.5)) = 0.875 rad/s²
jerk = min(8, 0.35*(10-2.5)) = 2.625 rad/s³
```

したがって希望上限2 rad/s²、8 rad/s³をそのまま軌道へ適用するわけではない。
これらは軌道生成の予算式であり、実機の停止距離を保証する式ではない。
停止距離を予測して境界余裕や起動可否を判定する保護モデルは省略したままである。

## 追従・飽和からの復帰

| 設定 | 値 | 条件 |
|---|---:|---|
| `CYBERGEAR_TRACKING_ERROR_RAD` / `CYBERGEAR_TRACKING_TIMEOUT_MS` | 0.15 rad / 200 ms | 参照軌道との位置誤差が継続 |
| `CYBERGEAR_SATURATION_TIMEOUT_MS` | 1000 ms | 最終電流の振幅制限または変化率制限が継続 |
| `CYBERGEAR_TRACKING_RECOVERY_ENABLED` / `CYBERGEAR_SATURATION_RECOVERY_ENABLED` | 1 / 1 | 追従・飽和からの復帰を有効化 |
| `CYBERGEAR_RECOVERY_ZERO_MS` | 200 ms | 復帰中に0 Aを指令する時間 |

条件成立時は検出した周期から0 Aを指令し、200 ms保持する。
比較用OPERATIONモードでは同じ期間、Kp・Kd・トルクを0とする。
0 Aへの切替には電流変化率制限を適用せず、再開時は0 Aから制限を適用する。
b0と外乱推定を初期値へ戻し、補償ランプをやり直す。
位置・速度推定、原点、最終目標は保持し、原点探索やモーターの再初期化は行わない。
準備済み軌道を破棄し、ゼロ保持終了時の実測位置から最終目標へ再計画する。
計画待ちはその位置を保持する参照を使う。
実測位置がソフト範囲の外でもハード境界内なら、再計画の出発点まで軌道の許容範囲を広げる。
外部目標のソフト範囲は変更せず、内側の目標へ戻す。棄却された位置測定を出発点には使わない。

復帰時には追従・飽和・停滞の監視タイマーを解除し、復帰後に監視し直す。
復帰回数に上限はなく、異常条件が続くと復帰を繰り返し得る。
該当する復帰設定を0にすると追従・飽和異常は停止になる。

## 起動と残る保護

ENABLE後は、新しいRun応答の実測位置・速度からADRCへ引き継ぐ。
この段階で静止継続を待ち直さない。STOP確認には静止速度0.03 rad/s、継続100 msを使う。

| 監視 | 現在の値・動作 |
|---|---|
| 目標・参照位置 | −6.28～+6.28 rad |
| 実測位置 | −12.5～+12.5 rad |
| 実測速度・温度 | 26 rad/s / 85 ℃で保護 |
| フィードバック鮮度 | 100 ms以下は有効。100 msを超える継続受信断は停止 |
| 制御周期 | 10 ms。早すぎる呼出しをスキップし、遅延は実経過時間で処理 |
| 停滞 | 位置誤差0.15 rad超、指令電流4 A以上で進捗0.003 rad未満が1000 ms継続した場合に停止 |
| 起動待ち・STOP応答待ち | 3000 ms / 1000 ms |
| STOP再試行 | 50 ms間隔。確認待ち1000 ms／20回の後も未確認なら再送を継続 |
| 計画先行時間・復帰期限 | 初期50 ms、計算時間に応じて先行時間を延長。500 ms超の未反映は停止せず復帰・再計画 |
| CANバス状態・エラーカウンター | 診断用に収集。これらの値による自動停止判定はしない |
| CAN送信キュー投入失敗 | 単発は診断記録のみ、次周期の最新指令を再送。100 ms継続して失敗する場合は停止 |
| モーター故障応答 | 停止判定を維持。type-21のFW固有warning定義は未確定のため非ゼロ通知の扱いは変更しない |

復帰中も位置・速度・温度・通信などの基本保護は継続する。
通常停止は `fault=CG_FAULT_NONE` のまま `STOPPING` から `STOPPED` へ進む。
停止未確認だけで通常停止を故障へ変更しない。実故障の理由は停止処理中も保持する。
通常4軸運転では、実故障・起動失敗でモーターを停止してもmainとCAN受付を継続する。
全軸のSTOP確認後、新しいCAN ID `0x500` の0→1指令で原点探索・初期化をやり直せる。
停止処理中・初期化中に受け取った開始指令は後から再生しない。
クロック・制御タイマー・CAN基盤の初期化異常は引き続き `Error_Handler` の対象。
診断UARTの初期化失敗ではUART出力を無効化して継続する。
CANのバス状態を監視することと、通信が成立していることは別に扱う。
`cybergear_controller_commit_queued()` はCAN送信キュー投入に成功した電流を記録する。
これは実電流やモーターへの適用完了の測定ではない。
新しい有効な受信sequenceだけで観測器を補正し、受信がない周期も制御の実経過時間で予測する。
周期ずれだけでは停止しない。公称周期からずれた場合は観測器ゲインもその時間に合わせる。
電流の変化幅は最大でも公称1周期分とし、遅れを理由に大きな電流段差を作らない。
長時間の中断後に新しい有効測定があれば、観測器を実測位置・速度推定0から再同期する。
同一時刻の重複呼出しでは観測器を二重更新しない。

`CYBERGEAR_HARDWARE_CONFIRMED=1` は設定の自動校正や安全性の証明を意味しない。
原点探索は別の速度制御で、初期探索1 rad/s、再探索0.4 rad/sを使う。
原点探索の一時的な送信失敗は100 msの範囲で再送する。受信断や実機異常では工程を中止する。
試験結果と運転条件を保存したうえで、[取込み手順](cybergear_promotion.md)に従って変更する。

## 入力と計画の回復

有限な外部目標はソフト範囲に制限し、NaN/Infの目標は前の有効目標に置き換える。
これらを `TARGET` 故障にしない。`ignored_targets` に制限・棄却回数を記録する。
電流・外乱補償・電流変化率の上限到達も、従来どおり制限を適用して処理する。

候補軌道が成立しない場合、または目標が500 ms超にわたり未反映の場合は、
追従・飽和と共通の200 ms復帰処理から、実測位置を基に再計画する。
計算が採用予定時刻を過ぎた候補は破棄し、計算に必要だった時間を基に先行時間を延長する。
既に有効な軌道は更新まで継続する。実行中の軌道データの非有限値や不正設定は軽微な計画失敗と区別する。

単発の位置飛び・古い／順序不正の測定は棄却し、最後の有効測定が新鮮な間は制御を継続する。
モード不一致は100 msの確認期間を設け、ゼロ出力のプローブで応答を再確認する。
新しい有効測定が来ないまま期限を過ぎた場合や、モード不一致が継続した場合は停止する。

`recovery_reason` と `recovery_count` は復帰の理由・回数であり、`fault` とは別に記録する。
`planner_attempts`、`planner_failures`、`planner_deadline_misses`、`planner_compute_ms`、
`planner_max_compute_ms`、`planner_lead_ms` で計画の経過を確認できる。
リングログは直近32件を保持し、満杯時は最古の記録を置き換える。
最初の実故障時の記録は `fault_log` に保持し、停止再送のログで上書きしない。
