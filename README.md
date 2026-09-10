# catch2026_robstride

STM32G474RBTxで、CyberGear 1台とRobStride系3台（右RS03・左RS03・EL05）を制御するファームウェアです。
上位基板とはFDCAN1、モーターとはFDCAN3で通信します。

- [詳細仕様](docs/firmware-spec.md)：通信形式、起動、原点探索、制御周期、ADRC、異常時動作、未実装事項。
- [変更内容・検証](docs/refactoring.md)：分割方針、互換性、ビルドとテストの手順。
- [RobStride MIT制御](docs/mit-control.md)：運控モードへの変更、機種別換算、Kp/Kdの調整と検証。

Arm GNU Toolchain、CMake、NinjaをPATHに設定して実行します。

```sh
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release
```

生成物は`build/Debug/catch2026_robstride.elf`または`build/Release/catch2026_robstride.elf`です。
電源投入後はCAN指令待ちとなり、標準ID `0x500`の先頭int32値が `0 → 1` に変化すると初期化・原点探索を開始します。初回受信が `1` の場合は開始せず、先に `0` を受信する必要があります。
