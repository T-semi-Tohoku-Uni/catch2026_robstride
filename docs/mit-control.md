# RobStride MIT制御

左右RS03とEL05はprivate protocolの運控モード（`run_mode=0`）を使用する。
位置・速度・Kp・Kd・フィードフォワードトルクを送るMIT型制御で、CANは従来の29-bit拡張ID・1 Mbps。
11-bit標準IDの「MIT通信プロトコル」への変更や永続設定は行わず、モーター側はprivate protocolを前提とする。
CyberGearは電流モード＋STM32上の位置ADRCを継続する。
各軸の送信周期は10 ms（100 Hz）、上位基板の通信形式と座標変換も従来どおり。

## 指令とゲイン

上位基板は位置だけを指定する。変換済み目標位置、速度0 rad/s、軸別Kp/Kd、フィードフォワードトルク0 Nmを送る。

```text
torque_ref = Kp * (target_position - measured_position)
           + Kd * (0 - measured_velocity) + torque_ff
```

`Core/Inc/motor_config.h`で軸別に設定する。

| 軸 | Kpの初期仮値 | Kdの初期仮値 |
| --- | --- | --- |
| 右RS03 | 5 | 1 |
| 左RS03 | 5 | 1 |
| EL05 | 1 | 0.1 |

これらは実機で調整していない仮値で、負荷の保持や振動の確認が必要。
`torque_ff=0`でも位置・速度誤差からトルクが発生し、トルク上限の設定にはならない。
以前のPP設定（最大速度10 rad/s、加速度1 rad/s²、位置・速度モード用電流上限10 A）はMIT制御には適用しない。
今回の実装には軌道補間や追加のトルク制限を設けていないため、目標の段差に対する応答はPPと異なる。

## CAN形式と機種別換算

通常指令は通信タイプ1。IDのbits23..8にトルクの16-bit値、bits7..0にモーターIDを格納する。
データ8バイトは位置・速度・Kp・Kdを各16-bitビッグエンディアンで格納する。
各値の換算は既存ヘルパーの最近傍丸め（0〜65535）を使う。

| 機種 | 位置 rad | 速度 rad/s | トルク Nm | Kp | Kd |
| --- | --- | --- | --- | --- | --- |
| RS03 | ±12.57 | ±20 | ±60 | 0〜5000 | 0〜100 |
| EL05 | ±12.57 | ±50 | ±6 | 0〜500 | 0〜5 |

これは通信値の換算範囲であり、機構の許容可動範囲ではない。
位置は従来の受信換算・メーカーサンプルに合わせて±12.57 radを使用する（仕様表では±4π）。
ハンドラーの`model`で機種を設定し、受信の速度・トルクも同じ機種別範囲で復号する。
従来EL05にも使っていたRS03の速度・トルク換算は修正した。

根拠資料：

- [RS03メーカー説明書260713](https://github.com/RobStride/Product_Information/blob/main/Product%20Literature/RS03/RS03User%20Manual260713.pdf)：通信タイプ1/2とrun_modeの表。
- [EL05メーカー説明書251112（Seeed配布）](https://files.seeedstudio.com/products/RobStride/Product%20Literature/EL05/EL05User%20Manual251112.pdf)：PDFの19〜20ページの通信タイプ1/2の表。サンプルコードの±5.5 Nmと不一致があるため、EL05の仕様表に明記された±6 Nmを採用した。

## 起動・失敗時の処理

起動とEL05の一度だけの起動時再試行は、ともに次の順で送信する。

```text
STOP → 10 ms → run_mode=0 → 10 ms → Enable → 10 ms
→ position/velocity/Kp/Kd/torque_ff がすべて0の運控指令
→ 周期処理で目標位置と設定ゲインを送信
```

初期指令はKp/Kdが0なので位置0への保持力は指令しない。符号付き16-bit量子化では0付近に丸め誤差がある。
モード設定が送信FIFOに登録されたときだけハンドラーの`run_mode`を更新する。
初期指令の送信登録失敗時はSTOPを試みて失敗を返す。それより前の送信失敗では、その時点で後続を中止する。
送信FIFOへの登録成功は、モーター側の受信・実行の確認ではない。

MIT送信APIは未設定機種、MIT以外のモード、非有限値、通信範囲外の値を拒否する。
通常周期でAPIが失敗した場合、対象モーターへSTOPを試みる。
STOPの到達確認や全軸停止は行わない。通常周期はEnableを再送しないため、STOP後の自動再開は保証しない。
EL05の起動時再試行条件と、全モーター共通の通信監視は従来どおり。

## 検証

```sh
cmake --preset Debug -B build/mit-Debug
cmake --build build/mit-Debug
cmake --preset Release -B build/mit-Release
cmake --build build/mit-Release
cmake -S tests -B build/mit-host -G "Visual Studio 17 2022" -A x64
cmake --build build/mit-host --config Debug
ctest --test-dir build/mit-host -C Debug --output-on-failure
```

`robstride_mit`は機種別の既知バイト列、最小・最大値、NaN/Inf・範囲外拒否、起動・再試行の送信順、各送信段階の失敗、受信換算と宛先照合、割り込みマスクの復元を確認する。
2026-09-09にWindows / Arm GNU 13.3.1 / MSVC 19.44で確認した結果：

| 検証 | 結果 |
| --- | --- |
| Debug | 警告なしで成功。FLASH 49,088 B / RAM 3,280 B |
| Release | 警告なしで成功。FLASH 35,448 B / RAM 3,280 B |
| ホストテスト | board_protocol、homing、robstride_mitの3/3成功（/W4 /WX） |

ホストテストでは実際のモーター応答、CAN負荷、機構の保持力・振動は検証できない。
書き込み・実機駆動は実施していない。
