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
- `cybergear_real_tim6_scheduler`: `main.c` の実際の TIM6 コールバックを抽出・コンパイルする。1000 tick で CG が 100 / 200 回、RobStride 各軸と上位返信が各 100 回となること、他軸の符号・オフセット、CG 故障中の呼出し、起動前・全軸停止後の送信抑止を検査する。
- `cybergear_real_fdcan3_dispatch`: `main.c` の実際の FDCAN3 受信コールバックを抽出・コンパイルする。CyberGear の type 2 / 17 / 21 が届くこと、RobStride は従来の type 2 条件を維持すること、DLC 不正・対象外フレームを拒否することを検査する。

- `robstride_startup_*`: 実RobStrideドライバを使う19条件。遅延起動、欠落、故障、設定/Enable応答欠落、送信失敗、tick wrap、全軸待機とkeepaliveを検査する。
- `motor_startup_integration`: 実mainの起動・停止・監視関数を抽出し、CyberGearとの並行起動、全軸の起動確認、タイマー移行、STOP再試行と期限を検査する。
- `board_protocol` / `board_command_integration`: big-endian変換、0→1開始、復帰中の無視、不正floatの全フレーム拒否を検査する。
- `firmware_flash_layout_checker`: 合成ELFでFlash/RAM領域、起動ベクトル、重複セグメント等を検査する。実ビルドのELFは `python3 tests/check_flash_layout.py <firmware.elf>` で別途確認する。

ユーザー指示により停止距離の予測保護モデルを省略したため、通常6 A・単体3 Aを
停止距離見積りで起動拒否することは合格条件に含めません。角度・速度・電流・温度・通信・
追従・拘束とSTOP処理、ADRC・軌道・慣性モデルの検証を維持します。
運転テストには既定設定と合成設定を使用し、応答・運動状態はHALスタブで与えます。
詳しくは [統合修正と空試験](../docs/09_統合修正と空試験.md) を参照してください。

GCC/Clang では `-Wall -Wextra -Wpedantic -Werror -fno-fast-math` を使い、Release 設定でも `assert` を有効にする。
仮想プラントは未同定の数理モデルであり、実機の安定性・安全電流・停止距離・共振・最悪割込み時間を保証しない。
`driver_test.c` の数値は再現可能なソフト検証用 fixture で、実機設定へコピーする値ではない。
