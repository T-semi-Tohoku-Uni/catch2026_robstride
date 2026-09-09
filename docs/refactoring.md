# リファクタリングと検証

## 変更内容

ブランチ：`refactor/motor-control-spec`。基準：`fix`の`fec81e5`。

`main.c`に集まっていた起動、原点探索、受信解析、周期制御を責務ごとに分離した。
同ファイルは1,359行から441行になった。

- `motor_app.c/h`：アプリ内状態をstaticで保持。`motor_app_start()`を周辺機器初期化後に1回、`motor_app_process()`をメインループで呼ぶ。受信・タイマー処理も公開アダプター経由で接続する。
- `cybergear_homing.c/h`：原点探索を分離し、対象モーター・全体の通信監視・診断出力を`CyberGearHomingContext`で明示的に渡す。
- `board_protocol.c/h`：4軸の並び順、CAN ID、BE int32/float変換、角度の符号・オフセット変換を集約する。
- `motor_config.h`：機器ID、アプリの待機時間、PP設定などを集約する。ADRC固有の定数は引き続き`cybergear.c`で管理する。
- RobStrideの3台分のハンドラー設定をIDテーブルとループで整理する。
- 未使用のEL05ログ関数、無処理のGPIOアプリコールバック、未使用の整数エンコード関数、コメントアウトされた試行コードを整理する。
- `CMakeLists.txt`に新しいソースを登録し、READMEと詳細仕様書を追加する。

既存の初期化順序、モード設定、待機時間、ADRC計算、原点探索条件、通常送信周期、復帰時のEL05維持を引き継ぐ。
上位通信のバイト順、符号付き値、特殊floatのビット列を保持しつつ、旧変換の`uint8_t << 24`に伴う符号付きintのシフト問題を、uint32への明示キャストで解消する。
floatのビット変換には`memcpy`を使い、binary32であることをコンパイル時に確認する。
制御アルゴリズムの調整、RobStrideゼロ設定APIの修正、独立した全軸監視などは今回の変更には含まれない。

リファクタリング後の仕様変更として、運転開始前の`0x500`による初期化要求を、直前の有効受信値が`0`、今回が`1`の場合に限定した。初回受信だけでは開始しない。運転開始後の復帰条件は任意の値変化のまま維持する。

## CubeMX再生成

`main.c`のアプリ接続はUSER CODE領域内に置く。クロック、GPIO、FDCAN、TIM6、USARTの生成処理は維持する。
アプリ用ソースは再生成対象外のトップレベル`CMakeLists.txt`で登録する。
CubeMXを再実行した場合は、USER CODEの保持設定とトップレベルCMakeのソース登録を確認して再ビルドする。

## ビルド

必要なツールはCMake 3.22以上、Ninja、Arm GNU Toolchain。各実行ファイルをPATHに設定する。

```sh
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release
```

今回の環境では既存のビルド出力を保持するため、専用ディレクトリで検証した。

```sh
cmake --preset Debug -B build/refactor-Debug
cmake --build build/refactor-Debug
cmake --preset Release -B build/refactor-Release
cmake --build build/refactor-Release
```

確認環境：Windows、Arm GNU 13.3.1、CMake 4.0.1、Ninja 1.13.2。

| 対象 | 結果 | FLASH使用 | RAM使用 |
| --- | --- | --- | --- |
| 変更前Debug | 成功。未使用`el05_debug_print`の警告1件 | 47,800 B | 3,272 B |
| 変更後Debug | 成功。警告なし | 48,176 B | 3,280 B |
| 変更後Release | 成功。警告なし | 34,776 B | 3,280 B |

値はリンカのmemory usage出力。RAM使用値は実行時の最大スタック消費を計測したものではない。

## ホストテスト

HALを使わない通信モジュールと、HAL・モーター送信を置き換えた原点探索をPC上で実行する。
テスト専用CMakeはファームウェアのクロスコンパイル設定から独立している。

Windows / Visual Studio Build Tools：

```sh
cmake -S tests -B build/refactor-host -G "Visual Studio 17 2022" -A x64
cmake --build build/refactor-host --config Debug
ctest --test-dir build/refactor-host -C Debug --output-on-failure
```

GCC/Clang環境向けの実行方法（この環境での実行確認はMSVCのみ）：

```sh
cmake -S tests -B build/refactor-host -G Ninja
cmake --build build/refactor-host
ctest --test-dir build/refactor-host --output-on-failure
```

| テスト | 確認項目 | 今回の結果 |
| --- | --- | --- |
| `board_protocol` | 既知の16バイト列、符号付きint32の0/1/-1/最小/最大、負の角度、±0、±Inf、NaN、非正規化数、4軸の座標変換 | 成功 |
| `homing` | 正方向探索、60°後の逆方向探索、センサー未変化の期限、応答なしとEnable再送、fault、速度送信失敗、古い応答、HAL tick周回、成功時にSTOP後ゼロ設定 | 成功 |

MSVC 19.44で`/W4 /WX /utf-8`を使用し、2/2テスト成功。
テストはReleaseビルド等でも検査が消えないよう、`assert`ではなく明示的な失敗判定を使う。
追加の差分確認では、`main.c`のUSER CODE外が変更されていないことと、原点探索が依存オブジェクト・定数名の置換を除き元の処理と一致することを確認した。

## 検証の範囲

ビルドとホストテストは、バイナリ生成・データ互換性・原点探索の分岐の一部を確認する。
CAN接続、実センサーのチャタリング、モーター慣性、割り込み競合、ADRCの実機安定性、復帰動作全体を実機で検証したものではない。
この作業では書き込み・モーター駆動は実施していない。
実機確認項目と現在の制約は[詳細仕様](firmware-spec.md)に記載する。
