# CyberGear の実機なし検証

すべて PC 上で実行する。HAL スタブはメモリ上の CAN フレーム・時刻・故障を扱う。
ST-LINK、CAN アダプタ、実モーターへの接続・書込みは行わない。

## Windows / PowerShell

リポジトリ直下から実行する。CMake、Ninja、Python 3、ネイティブ C コンパイラが必要。
`run_host_tests.ps1` は PATH の GCC、次に CLion 同梱 MinGW を探索し、Python は `py -3` で取得する。

```powershell
./tests/run_host_tests.ps1
```

コンパイラを指定する場合:

```powershell
./tests/run_host_tests.ps1 -Compiler C:/path/to/gcc.exe -BuildDirectory build/host-tests
```

ARM 用 `arm-none-eabi-gcc` はホストテストには使わない。ファームウェアのクロスビルドとは独立した CMake プロジェクトで、生成物は既定で `build/host-tests` に入る。

## Linux 等での直接実行

```sh
cmake -S tests -B build/host-tests -G Ninja -DCMAKE_C_COMPILER=gcc
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure
```

ASan/UBSan のランタイムがある環境では、別のビルドディレクトリを使い
`-DCYBERGEAR_SANITIZERS=ON` を追加する。Windows スクリプトでは `-Sanitizers`。
今回の CLion 同梱 MinGW GCC 15.2.0 では `libasan` / `libubsan` がなく、
リンク時に `cannot find -lasan` / `cannot find -lubsan` となることを確認したため、実行結果に sanitizer 合格を含めない。

## 検査する内容

- `cybergear_host_tests`: 五次軌道の端点・全区間制約・再計画、ESO と電流制限、可変 b0 と情報失効、通信 byte 列、起動 readback、通常送信の所有権、故障ラッチ、STOP 再試行、明示再初期化、tick wrap、更新され続ける目標の計画、128 条件の仮想プラント。
- `cybergear_real_tim6_scheduler`: `main.c` の実際の TIM6 コールバックを抽出・コンパイルする。1000 tick で CG が 100 / 200 回、RobStride 各軸と上位返信が各 100 回となること、他軸の符号・オフセット、CG 故障中の呼出し、EL05 初期化中の既存抑制を検査する。
- `cybergear_real_fdcan3_dispatch`: `main.c` の実際の FDCAN3 受信コールバックを抽出・コンパイルする。CyberGear の type 2 / 17 / 21 が届くこと、RobStride は従来の type 2 条件を維持すること、DLC 不正・対象外フレームを拒否することを検査する。
- `cybergear_host_tests` には計測用状態機械も含む。固定±90°境界、内側余裕、基準位置、電流送信commit、時間wrap、制動終了・故障を検査する。
- `cybergear_calibration_app_tests`: 実ドライバと模擬CANで、モード読戻し→ゼロ電流→起動→正負の単発試験→停止確認、原点保持、電流の所有権、通信・速度・温度・位置・ログ満杯による中止を検査する。
- `cybergear_calibration_analysis`: 合成データの入力ゲイン、停止観測、量子化、欠落・故障の拒否など23件。
- `cybergear_calibration_log_pipeline`: 実際の `cg_cal_app_dump()` が生成した正負2試行のUARTログを解析CLIへ渡し、有効な入力ゲインと静止観測が得られることを検査する。
- `cybergear_calibration_isolation`: 実際の計測版mainから原点探索・ゼロ設定・他軸起動・通常ADRC・自動リセットを排除し、TIM6の100 Hzと上位CAN指令の破棄を検査する。

GCC/Clang では `-Wall -Wextra -Wpedantic -Werror -fno-fast-math` を使い、Release 設定でも `assert` を有効にする。
仮想プラントは未同定の数理モデルであり、実機の安定性・安全電流・停止距離・共振・最悪割込み時間を保証しない。
`driver_test.c` の数値は再現可能なソフト検証用 fixture で、実機設定へコピーする値ではない。
