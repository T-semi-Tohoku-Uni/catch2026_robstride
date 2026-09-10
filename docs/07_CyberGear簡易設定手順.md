# CyberGear 簡易設定手順

設定入口：[Core/Inc/cybergear_config.h](../Core/Inc/cybergear_config.h)。
設定の組立て：[Core/Src/cybergear.c](../Core/Src/cybergear.c) の `cybergear_config_defaults()`。

停止距離の予測保護モデルはユーザー指示によりいったん省略しており、専用パラメーターの設定は不要。
角度・速度・電流・温度・通信・追従・拘束の保護とSTOP処理、ADRC・軌道・慣性モデルは維持する。

**1. 初期モードを維持する**

実機未確認の構成では `CYBERGEAR_HARDWARE_CONFIRMED=0` にして準備する。現在の格納値は1。
`CYBERGEAR_USE_200_HZ=0`、`CYBERGEAR_COMPARE_OPERATION_MODE=0` のまま、100 Hz・電流モードで準備する。

**2. 機械固有の値を埋める**

次の機械値を、図面・定格・実測記録と照合して設定する。未確認の項目は `NAN` にする。
表では共通の接頭辞 `CYBERGEAR_` を省略。

| 順番 | 設定項目 | 内容・単位 |
|---|---|---|
| ① 可動域 | `SOFT_MIN_RAD` / `SOFT_MAX_RAD` | 原点設定後の目標・軌道の範囲 [rad] |
| | `HARD_MIN_RAD` / `HARD_MAX_RAD` | 実測位置の停止境界 [rad]。機械端より内側に置く |
| ② 許容値 | `CURRENT_LIMIT_A` | 許容する電流の絶対上限 [A]。旧10 Aを定格とみなさない |
| | `SPEED_TRIP_RAD_S` / `TEMPERATURE_TRIP_C` | 速度 [rad/s]・温度 [℃] の停止しきい値 |
| ③ 入力ゲイン | `B0_FIXED` / `B0_MIN` / `B0_MAX` | 電流に対する加速度の代表値・全姿勢/荷物の下限・上限 [rad/s²/A] |

**3. 値の組合せを確認する**

```text
-12.5 ≤ HARD_MIN < SOFT_MIN < SOFT_MAX < HARD_MAX ≤ 12.5
0 < B0_MIN ≤ B0_FIXED ≤ B0_MAX
SPEED_TRIP_RAD_S > TRAJECTORY_SPEED_RAD_S
```

電流上限を **1 A未満** にする場合は、`cybergear_config_defaults()` の
`c->controller.current_limit_a` 設定直後で、`c->controller.disturbance_limit_a` も調整する。
既定の補償上限は0.5 A、予約電流は電流上限の半分なので、次を満たすよう補償上限を下げる。

```text
0 ≤ disturbance_limit_a ≤ 0.5 × CURRENT_LIMIT_A
```

初回は制御帯域 `wc=4`、減衰比 `zeta=2`、観測帯域 `wo=10` を維持する。
軌道の既定値 V=0.4 rad/s、A/B=1 rad/s²、J=5 rad/s³は開始候補であり、機体の許容値に合わせて下げる。
比較PD用の `operation_kp/kd/torque_limit` は電流モードでは `NAN` のままでよい。
停止距離式による起動拒否は行わない。軌道用の減速度・減速電流予算と電流変化率制限は引き続き使う。

**4. 実機なしで確認する**

CMake/Ninja・ネイティブGCC・Python、ファームウェア用Arm GCCを用意し、リポジトリ直下で実行する。

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_host_tests.ps1
cmake --preset Debug
cmake --build --preset Debug
```

ホスト試験は専用の仮想設定を使うため、入力した機械値の妥当性は保証しない。
`HARDWARE_CONFIRMED=0` のままビルド可能。実機への書込み・駆動は行わない。

**5. 実機確認後に有効化する**

原点・正負方向・b0・停止経路と機械許容値を確認してから、`CYBERGEAR_HARDWARE_CONFIRMED=1` にして再ビルドする。
通常制御の起動にはモード読戻しと新鮮な静止feedbackも必要。
低速で追従と停止を確認後、V → A/J → 必要な場合のみ帯域の順に、一項目ずつ調整する。
STOP送信は能動制動・機械的静止の保証ではない。

詳細：[全設定・調整方法](05_CyberGear実装と調整.md) ／ [空試験結果](06_空試験結果.md)
