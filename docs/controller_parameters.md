# CyberGear 制御器・入力ゲインモデルの調整資料

実装は `Core/Inc/cybergear_controller.h`、`Core/Src/cybergear_controller.c`、`Core/Inc/cybergear_dynamics.h`、`Core/Src/cybergear_dynamics.c`。制御器・モデルは HAL 非依存で、ドライバから設定と時刻を渡す。角度、速度、加速度は同じ CyberGear 出力軸座標で統一する。

ここで示す制御器既定値は `cybergear_controller_default_config()` の初期値で、実機の安全値・校正結果ではない。製品側の `cybergear_config_defaults()` は `Core/Inc/cybergear_config.h` の共通設定で周期、b0 の初期値・上下限、電流上限を上書きする。現在のファイルは `CYBERGEAR_HARDWARE_CONFIRMED=1`、`b0=1`（許可範囲 `0.35～3`）、電流上限 `6 A` を指定している。確認済みフラグだけでは設定の成立も実機での校正も証明しない。ホスト試験では試験専用の数値を明示設定する。既定関数の `b0=10` や `10 A` を機体の確認済み値として転記しない。

## 停止距離予測の省略と残る設定検査

2026-09-10のユーザー指示により、停止距離の予測保護モデル全体をいったん省略した。停止距離の計算、運転中の境界余裕判定、同じ式を用いた起動時の整合性検査、専用の制動・外向き加速度・輸送遅延・停止余裕パラメーターを削除している。

通常6 A・単体試験3 Aの設定を、停止距離見積りが保護範囲を超えることを理由に拒否する判定はない。角度・速度・電流・温度・通信・追従・拘束の保護、STOP再試行と停止確認、ADRC・軌道・慣性モデルは維持する。軌道生成に使う減速電流予算と減速度上限も引き続き設定する。

設定検査は、機械確認フラグ、数値の有限性、角度範囲、速度・電流・b0の整合性、制御周期、モデル・軌道制約、各期限などを引き続き確認する。実機の許容値や停止位置を、ホスト試験の合成条件から保証するものではない。

静止待機時間も `1～29999 ms` の範囲かつ `startup_timeout_ms > 2*stationary_dwell_ms` が必要。整数オーバーフローで巨大な待機時間が検査を通り、必ず起動タイムアウトする設定を拒否する。

## 制御則と調整順序

```text
i_track = {ad + wc²*(qd-z1) + 2*zeta*wc*(vd-z2)} / b0
i_dist_requested = -gamma*z3 / b0
i_dist = 外乱補償の振幅制限・変化率制限(i_dist_requested)
i_requested = i_track + i_dist
i_amplitude = clamp(i_requested, -current_limit, +current_limit)
i_command = clamp(i_amplitude, last_queued - fall*dt, last_queued + rise*dt)
```

生成位置だけでなく参照速度と参照加速度を使うことで、移動中に速度ゼロを追って制動する構造を修正する。まず軌道の速度・加速度・jerk を低めに設定し、電流飽和と実測の停止挙動を確認する。移動時間を短くする調整は軌道側から始め、wc と wo を同時に大きくしない。b0 の姿勢補正を使う場合も、wc・zeta・wo を慣性に比例して増加させる方式は入れていない。

外乱推定 z3 は摩擦、負荷、軸間干渉、公称入力ゲイン誤差を含む総外乱加速度。独立した摩擦 FF や軸間トルク FF は加えていない。同じ総外乱を推定したまま既知負荷 FF だけを追加すると二重補償になり得るため、追加時は観測器の予測モデルと残差外乱の定義も変更する必要がある。

## `CyberGearControllerConfig`

すべての浮動小数点設定は有限値が必須。表の値は制御器単体の既定値を示す。設定は停止中に変更して再初期化する。固定周期・wo から決まる ZOH 観測器ゲインを初期化時に保持しており、運転中に構造体の設定値だけを書き換えない。

| 項目 | 単位・単体既定値 | 有効範囲・作用・調整の影響 |
|---|---|---|
| `period_ms` | ms、10 | 10 または 5 のみ。100 Hz / 200 Hz の固定計算周期。製品側は `CYBERGEAR_USE_200_HZ` で上書きする。5 ms 化は受信鮮度、CAN 負荷、ISR 最悪時間の評価と一緒に行う。 |
| `timing_tolerance_ms` | ms、1 | 実呼出間隔と period の差の許容。`period_ms/2` 未満。許容内でも演算 dt は固定値を使う。大きくして長い ISR 遅延を隠さない。 |
| `feedback_timeout_ms` | ms、100 | 新しい測定を受け取らずに許す最長時間。period 以上、2³¹ 未満。長くすると通信欠落後の継続時間が増す。実測の通信間隔・遅延と機体が許容する検出時間から決める。 |
| `bandwidth_rad_s` | rad/s、4 | wc、正値。位置誤差ゲインは wc²、速度誤差ゲインは 2*zeta*wc。増加で誤差修正は強くなるが電流要求、ノイズ・遅延・柔軟振動への感度も増す。移動速度の上限ではなく、Hz 表記でもない。 |
| `damping_ratio` | 無次元、2 | zeta、正値。大きいほど速度誤差の抑制が強く、過減衰では収束が遅くなる。軌道 FF と電流制限の影響を分けて調整する。 |
| `observer_rad_s` | rad/s、10 | wo、正値。ESO の離散極を `exp(-wo*dt)` で配置。上げると測定への追従が速くなる一方、角度量子化・受信遅延・ノイズの影響が増す。単純な差分速度への置換は行わない。 |
| `b0_initial` | rad/s²/A、10 | 初期公称入力ゲイン。正値で min～max 内。製品側は `CYBERGEAR_B0_FIXED` で上書き。小さくすると同じ要求加速度に必要とみなす電流が増す。正方向・電流符号・座標を実測確認する。 |
| `b0_min` | rad/s²/A、10 | 許可下限、正値。製品側は `CYBERGEAR_B0_MIN`。モデル側と同じ値を使う。単体既定の min=max=10 は可変慣性の校正範囲を意味しない。 |
| `b0_max` | rad/s²/A、10 | 許可上限、min 以上。製品側は `CYBERGEAR_B0_MAX`。範囲外の更新を拒否するため、校正点・退避値をすべて範囲内に置く。 |
| `current_limit_a` | A、10 | 出力電流絶対値上限、正値。製品側は `CYBERGEAR_CURRENT_LIMIT_A`。定格、安全な連続・ピーク電流、配線、電源、温度条件から決める。飽和が続く状態をゲイン増加で補わない。 |
| `current_rise_a_s` | A/s、5 | 電流が数値として増える向きの変化率、正値。−5 A から 0 A への変化もこちら。上げると正方向電流への移行は速くなるが電気・機械的変化も急になる。 |
| `current_fall_a_s` | A/s、5 | 電流が数値として減る向きの変化率、正値。+5 A から負方向制動電流へ移る際にも適用。振幅上限とは別であり、過小な値は制動を遅らせる。 |
| `disturbance_limit_a` | A、0.5 | 外乱補償電流の絶対値上限。0～current_limit。大きくすると持続負荷を補いやすいが追従・制動の電流余裕を消費する。モデル側 reserve_current はこれ以上にする。0 で補償出力を無効化できる。 |
| `disturbance_slew_a_s` | A/s、1 | 外乱補償だけの変化率、正値。最終電流スルーとは別段。小さくすると補償変動が穏やかになるが、負荷変化や符号反転への追従が遅くなる。 |
| `compensation_gain` | 無次元、1 | gamma の到達値、0～1。0 でも ESO 推定は継続する。補償有無の比較に使える。調整時は振幅・スルー制限で実際の補償が制限されているかを見る。 |
| `compensation_delay_ms` | ms、100 | 初期化から gamma=0 のまま待つ時間。0～2³⁰−1。初期状態が整う前の強い補償を避ける。目標角変更ではこの待機を再始動しない。 |
| `compensation_ramp_ms` | ms、500 | 待機後に gamma を smoothstep で上げる時間。1～2³⁰−1。長いほど補償導入が緩やか。進行カウンタは終端で飽和し、長時間運転の tick wrap でも再始動しない。 |
| `leak_mode` | 列挙値、`CYBERGEAR_LEAK_LEGACY_ERROR` | 誤差依存、固定、なしの 3 方式。既定は比較のため旧方式を保持。補償が弱い理由を調べる際は、リーク変更とゲイン変更を分離する。 |
| `leak_fixed_s` | 1/s、0.03 | 固定方式のリーク率、0 以上。z3 に `exp(-rate*dt)` を掛ける。大きいほど古い推定を速く忘れるが、持続負荷の補償も弱まる。 |
| `leak_near_s` | 1/s、0.5 | 誤差依存方式で小誤差側のリーク率、0 以上。大きくすると停止付近の残留推定が消えやすい一方、静止保持電流が減り得る。 |
| `leak_far_s` | 1/s、0.03 | 大誤差側のリーク率、0 以上。near と far の間で線形補間。小さいほど移動中の外乱推定を長く保持する。 |
| `leak_near_rad` | rad、0.003 | 小誤差側しきい値、0 以上。誤差は生成参照 qd と最新実測位置の差の絶対値。最終目標との差ではない。 |
| `leak_far_rad` | rad、0.015 | 大誤差側しきい値、near より大きい値。二つのしきい値の距離でリーク率の変化の緩やかさが決まる。 |

## 固定周期・受信・電流履歴の扱い

各制御呼出で nominal period の予測を 1 回行い、その時点の最新受信位置を現在の制御時刻の測定として近似配置する。測定は新しい `rx_sequence` が来た場合だけ最大 1 回補正し、同じ測定を繰り返し積算しない。受信が抜けた間も固定周期で予測する。受信タイムスタンプは鮮度・順序確認に使うが、モーター内部の測定時刻ではない。CAN 輸送遅延、測定から受信までの遅延、ZOH 入力の正確な適用開始時刻は補償していない。時刻付きの厳密な遅延補正観測器や入力履歴積分として扱わない。

`cybergear_controller_commit_queued()` は HAL キュー投入成功後に限って呼ぶ。最終制限後の値を `last_queued_current_a` と `applied_current_estimate_a` に保存し、次の予測入力に使う。キュー投入成功は送信完了でもモーターへの適用完了でもなく、この値は実電流の測定ではない。`applied_current_valid` は、予測に使えるキュー投入履歴をソフトウェアが持っているかを示す。初期化直後は false で、ZERO_COMMAND の投入成功も明示的に commit する必要がある。送信失敗・停止・制御所有権の喪失時は invalidate し、その後の制御出力を継続しない。

公称 b0 を更新するときは、まず旧区間を旧 b0 と投入済み電流推定で予測・補正する。その後、`z3_new = z3_old + (b_old-b_new)*i_applied_estimate` と再表現し、予測加速度 `b*i+z3` を維持する。これは物理状態の連続性のための整合処理で、gamma≠1・補償制限・電流飽和がある場合の出力電流連続性を単独で保証しない。最終電流スルー制限と合わせて検証する。

入力データは `CyberGearControllerMeasurement` の `position_rad` [rad]、フレームごとの `rx_sequence` [uint32]、`timestamp_ms` [受信 ms]、`valid` で渡す。sequence と時刻の wrap は uint32 差で扱う。同じ sequence で時刻だけを更新して鮮度を延長してはならない。`CyberGearControllerReference` は軌道の位置 [rad]・速度 [rad/s]・加速度 [rad/s²] を同時に渡す。

観察する出力は、制限前後の `tracking_current_a`、`disturbance_current_a`、`requested_current_a`、`amplitude_current_a`、最終 `current_a`、各制限フラグ、測定を補正した `measurement_corrected`、測定残差 `innovation_rad` [rad]、`gamma`、`b0`、再表現前後の `z3_before_reexpression` / `z3_after_reexpression` [rad/s²]。制限フラグを伴わない電流値だけで調整の良否を判断しない。

## `CyberGearDynamicsConfig`

このモデルは、校正済みの実姿勢スカラーから b0 を補間する小さなモデル。リンク長・質量・減速比・荷物を推測して幾何学モデルを作っていない。Ktau の確認がないため、J_hat や J_hi の慣性数値を出力したと主張しない。b0 の校正だけで入力ゲインを設定することはできるが、b0 から物理的な慣性を分離するには同じ座標のトルク／電流係数が必要になる。

モデル単体には既定値生成関数がない。表の「製品側初期値」は `cybergear_config_defaults()` が設定する値を示す。未確定な電流・b0 を参照する項目は、そのままでは有効な設定にならない。

| 項目 | 単位・製品側初期値 | 有効範囲・作用・調整の影響 |
|---|---|---|
| `schedule_enabled` | bool、false | 校正テーブルと実姿勢スナップショット API の接続確認後に true とする。false では fixed_b0 を使い、姿勢スナップショットを必要としない。 |
| `fixed_b0` | rad/s²/A、制御器 b0_initial | スケジュール無効時と姿勢情報失効時の退避値。正値で min～max 内。制御器初期 b0 と一致させる。 |
| `b0_min` | rad/s²/A、制御器 b0_min | 到達可能な全姿勢、最大想定荷物、誤差を含む保守的な入力ゲイン下限。正値。Ktau/J_hi 相当の役割で、現在姿勢の中央推定値とは別。これを大きく見積もると軌道能力を過大評価する。 |
| `b0_max` | rad/s²/A、制御器 b0_max | 校正済み上限、min 以上。全校正点と fixed_b0 を含む。制御器と一致させる。 |
| `b0_rate_limit` | (rad/s²/A)/s、1 | 公称 b0 の最大変化率、正値。大きくすると姿勢変化への追従が速くなる一方、FF 電流変化率の余裕を消費する。失効時の fixed_b0 への復帰にも適用する。 |
| `posture_timeout_ms` | ms、100 | 実姿勢情報の許容 age。1～2³¹−1。長くすると古い姿勢を長く使う。機構変化速度・通信遅延から決め、失効状態で新規高速移動を許さない。 |
| `point_count` | 点、0 | 最大 8 点。有効時は 2～8 点、無効時は 0 点可。少数点で任意の多軸姿勢を表せるとは限らない。 |
| `points[].posture_index` | 機構で定める単位、未設定 | 校正済み実姿勢のスカラー座標。有限値、厳密な昇順。単位・定義・他軸原点の確定を文書化する。目標角や原点未確定値を代用しない。 |
| `points[].b0` | rad/s²/A、未設定 | その姿勢・荷物条件で校正した公称入力ゲイン。min～max 内。隣接点間で線形補間し、校正範囲外へ外挿しない。 |
| `acceleration_current_a` | A、制御器 current_limit | 加速に使える電流の絶対上限、正値。温度・速度・電源の想定範囲を含める。製品側では最終電流上限以下を要求する。 |
| `braking_current_a` | A、制御器 current_limit | 減速に使える電流の絶対上限、正値。加速と別に確認し、回生電力・電源電圧も考慮する。最終電流上限以下。 |
| `reserve_current_a` | A、current_limit の 50% | 摩擦、軸間干渉、PD 修正、外乱補償に残す電流、0 以上かつ加速・減速電流の両方未満。製品側では disturbance_limit 以上も要求する。大きくすると軌道の加減速能力が下がる。 |
| `current_slew_a_s` | A/s、rise/fall の小さい方 | 実制御器の正負スルーの小さい方以下、正値。FF に利用可能な変化率予算の出発点。 |
| `reserve_slew_a_s` | A/s、2.5 | PD 修正と外乱補償のため FF に使わず残す変化率、0 以上。大きくすると許容 jerk が下がる。b0 変化分も引いた残りが正でなければ設定を拒否する。 |
| `acceleration_cap_rad_s2` | rad/s²、軌道加速設定、現在 1 | 機械的な加速度上限、正値。電流予算からの上限との小さい方を採用する。高くしても電流予算の上限は越えない。 |
| `braking_cap_rad_s2` | rad/s²、軌道減速設定、現在 1 | 軌道生成に使う機械的な減速度上限、正値。電流予算の上限との小さい方。停止距離予測の省略後も維持する。 |
| `jerk_cap_rad_s3` | rad/s³、軌道 jerk 設定、現在 5 | 機械的な jerk 上限、正値。FF 電流変化率予算からの上限との小さい方。柔軟振動を見ながら調整する。 |

## 全姿勢の制約と失効時の扱い

```text
A_accel = min(acceleration_cap, b0_min*(acceleration_current-reserve_current))
A_brake = min(braking_cap, b0_min*(braking_current-reserve_current))
B_rate = schedule_enabled ? b0_rate_limit : 0
S_schedule = max(A_accel,A_brake)*B_rate/b0_min²
S_remaining = current_slew-reserve_slew-S_schedule
Jerk_max = min(jerk_cap, b0_min*S_remaining)
```

`i_ff=ad/b0` の微分は `jerk/b0 - ad*b0_dot/b0²` なので、b0 更新に必要なスルーも先に予約する。S_remaining が正でなければ設定不正とする。これらの制約は現在姿勢の b0 ではなく全姿勢の b0_min に固定するため、移動開始時だけ軽い姿勢を仮定して途中で能力不足になることや、モデル失効により加速度参照を突然クリップすることを避ける。現実の全姿勢下限が正しく設定されていることが前提で、現在姿勢だけの確認では足りない。

`CyberGearPostureSnapshot` は実姿勢スカラー `posture_index`、受信・取得時刻 `timestamp_ms`、有効性 `valid`。スケジュール有効時、古い・欠けた情報は `CYBERGEAR_MODEL_STALE`、非有限または校正範囲外の値は `CYBERGEAR_MODEL_INVALID` とする。モデルは両状態でも b0 を変化率制限付きで fixed_b0 に戻した数値を返すが、それは運転継続許可ではない。製品ドライバは STALE/INVALID をモデル故障として停止へ移す。復帰値を作れたことを理由に自動再 arm しない。

`CyberGearDynamicsOutput` は `b0` [rad/s²/A]、実際の `b0_rate` [(rad/s²/A)/s]、電流予算を反映した `acceleration_rad_s2` / `braking_rad_s2` [rad/s²]、`jerk_rad_s3` [rad/s³]、加速・減速の小さい方の予約後電流 `tracking_current_budget_a` [A]、モデル `status` を返す。`FIXED` は固定モデル、`VALID` は新鮮な校正範囲内の補間状態を表す。

軌道モジュールは加速・減速上限の小さい方を全区間に適用する。モデルの電流予算式は FF の保守的な上限で、実電流が常に一致すること、電流モード内部ループの遅延、摩擦変動、機体の停止性能まで保証する式ではない。実機が使えない間の試験結果は数値・通信模擬・モデル条件内の確認として記録し、校正値の代わりにしない。
