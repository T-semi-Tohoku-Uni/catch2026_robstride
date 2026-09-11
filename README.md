# catch2026_robstride

STM32G474RBTxで、CyberGear 1台とRobStride系3台（右RS03・左RS03・EL05）を制御するファームウェアです。
上位基板とはFDCAN1、モーターとはFDCAN3で通信します。

このブランチは**左右RS03の原点設定専用**です。標準ビルドでは電源投入後に自動で開始します。

1. left（ID 4）を反時計回りに低速回転し、PC1がHighになった位置を0°に設定して保持します。
2. leftを保持したままright（ID 3）を時計回りに低速回転し、PC0がHighになった位置を0°に設定します。
3. 両方を0°で保持し続けます。CyberGear・EL05の起動や上位CAN指令による通常運転は行いません。

探索速度は左右とも0.2 rad/s、電流上限は2 Aです。GPIOは既存のプルなし入力を読み取り、外部回路でHigh/Lowを確定させます。
開始時点で入力がHighなら、その軸の探索回転を省略します。検知後は速度0で200 ms待ってから停止・ゼロ設定します。
設定値・ピン割り当ては `Core/Inc/rs03_homing_config.h` にまとめています。回転方向の符号は出力軸側から見た方向を想定しています。
各軸15秒の探索期限、通信途絶、モーターfault、送信失敗で両方を停止し、MCUリセットまで再探索しません。
実機での回転方向、入力極性、保持動作、電源再投入後のゼロ位置の保持は未検証です。

従来の4軸制御をビルドする場合は `cmake --preset Debug -DRS03_HOMING_ONLY=OFF` を指定します。
原点設定専用に戻す場合は `-DRS03_HOMING_ONLY=ON` を指定してください。以下の詳細仕様と末尾のCAN起動条件は従来の4軸制御用です。

- [詳細仕様](docs/firmware-spec.md)：通信形式、起動、原点探索、制御周期、ADRC、異常時動作、未実装事項。
- [変更内容・検証](docs/refactoring.md)：分割方針、互換性、ビルドとテストの手順。

Arm GNU Toolchain、CMake、NinjaをPATHに設定して実行します。

```sh
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release
```

生成物は`build/Debug/catch2026_robstride.elf`または`build/Release/catch2026_robstride.elf`です。
電源投入後はCAN指令待ちとなり、標準ID `0x500`の先頭int32値が `0 → 1` に変化すると初期化・原点探索を開始します。初回受信が `1` の場合は開始せず、先に `0` を受信する必要があります。
