# catch2026_robstride

STM32G474RBTx 向けのモーター制御ファームウェアです。

現在の既定ビルドは **CyberGear単体試験** です。起動後にCyberGearだけを原点探索し、
原点から±30度を往復し、各端で1秒待ちます。RobStrideへの送信は無効です。
書き込み先と確認方法は[CyberGear単体試験](docs/08_CyberGear単体試験.md)を参照してください。

CyberGearの設定は[簡易設定手順](docs/07_CyberGear簡易設定手順.md)から始めてください。
改修内容、実機必須パラメーター、詳しい調整方法は
[実装・調整説明](docs/05_CyberGear実装と調整.md)を参照してください。
機械可動域・許容電流・入力ゲインなどが未設定の既定値では、CyberGearは
ホーミング開始前に起動を拒否します。実機なしの検証は
`powershell -ExecutionPolicy Bypass -File tests/run_host_tests.ps1` で実行できます。

## Git で管理するもの

- `catch2026_robstride.ioc`：ピン、周辺機能、コード生成の設定。
- `Core/Src/main.c`：`USER CODE` 領域に独自実装を含むメイン処理。
- `Core/Inc/`・`Core/Src/` の `can_init`、`cybergear`、`robstride_app`：独自の制御ライブラリ。
- ルートの `CMakeLists.txt`：独自ソースの追加と浮動小数点 `printf` のリンク設定。
- `.gitignore` とこの README。

`Drivers/`、`cmake/`、`CMakePresets.json`、起動コード、リンカスクリプト、
独自実装のない生成ファイルは `.gitignore` に個別指定しています。
IDE の個人設定、ビルド成果物、キャッシュも管理対象外です。
生成ファイルに必要な変更は `.ioc` に反映し、独自実装は追跡中のソースに置いてください。
生成ファイル自体を手で維持する必要が生じた場合は、該当する除外設定を外して追跡対象に戻してください。

## clone 後の準備とビルド

1. STM32CubeMX と STM32CubeG4 のファームウェアパッケージを用意します。
   現在の `.ioc` に記録されたバージョンは **STM32CubeMX 6.18.1** と
   **STM32Cube FW_G4 V1.6.3** です。再現時はこのバージョンに合わせてください。
2. リポジトリ直下の `catch2026_robstride.ioc` を STM32CubeMX で開きます。
   出力先をリポジトリ直下、Toolchain / IDE を **CMake** とし、
   **Keep User Code when re-generating** を有効にして **Generate Code** を実行します。
   `.ioc` は `KeepUserCode=true` ですが、`LastFirmware=true` でもあるため、
   再現時に別バージョンへの自動移行を受け入れないよう確認してください。
3. `Drivers/`、`cmake/`、`CMakePresets.json` と `.gitignore` に列挙した
   起動・リンカ・Core の生成ファイルができたことを確認します。
   追跡中のルート `CMakeLists.txt` はそのまま使用します。
   再生成後は `git diff` で `main.c` や `.ioc` などの変更を確認してください。
4. CMake 3.22 以上、Ninja、Arm GNU Toolchain (`arm-none-eabi-gcc`) を PATH に用意し、
   リポジトリ直下で実行します。STM32CubeCLT の同梱ツールも利用できます。

   ```sh
   cmake --preset Debug
   cmake --build --preset Debug
   ```

Release ビルドは両方のコマンドで `Debug` を `Release` に置き換えます。
IDE やデバッガの設定は各自の環境で作成してください。

## STM32G474RB の Flash 配置

本プロジェクトは実機で確認した `DBANK=1` を前提にします。128 KB品のFlashは
Bank 1 `0x08000000–0x0800FFFF`、Bank 2 `0x08040000–0x0804FFFF` の各64 KBです。
途中のアドレスは使用できません。CubeMX生成の連続128 KB設定では、Debug版が
64 KBを超えると書込み時に `Operation exceeds memory limits` になります。

ルートCMakeは生成ツールチェーンのリンカ指定を置き換え、Git管理する
`linker/STM32G474RB_dualbank.ld` を使用します。CyberGearモジュールのコードと定数を
Bank 2、起動コード・その他のコード・RAM初期値をBank 1へ配置します。
各バンクの64 KB超過はリンク時にエラーになります。制御定数やDBANK自体は変更しません。
CubeMX再生成後も、このスクリプトを使用するルートCMakeを維持してください。

ビルド後の配置確認（実機接続不要）:

```powershell
python tests/check_flash_layout.py build/Debug/catch2026_robstride.elf build/Release/catch2026_robstride.elf
```

Bank 2の末尾は16バイト境界まで埋めます。CubeProgrammer 2.23.0の実機ログでは、
27,896バイトの転送を13,948バイトずつに分割し、Bank 2の書込みで失敗していました。
この分割長はG4の8バイト書込み単位に合わないため、半分に分割しても8バイト単位に
なるよう配置しています。2026-09-09に実機で、修正後の27,904バイトが13,952バイトずつに
分割され、両バンクの書込み、`Download verified successfully`、`main`への到達を確認しました。
確認条件はCubeProgrammer 2.23.0、ST-LINK GDB server 7.14.0、SWD 1,000 kHz、Under Resetです。

書込みにはアドレス情報を保持する **ELFまたはHEX** を使用します。
DBANK=0の別基板へ同じイメージを流用しないでください。
レイアウトの根拠は [STの説明とRM0440への参照](https://community.st.com/stm32cubeide-for-visual-studio-code-mcus-133/stm32g474rb-can-t-debug-program-over-64-kb-159628) を参照。

## 追跡解除について

この整理では過去のコミットを変更しません。既存ファイルは `git rm --cached` で
インデックスからのみ外し、作業ディレクトリには残します。
次のコミットには追跡解除の削除差分が入り、その後の生成物の変更は含まれません。
既存の別 clone でこの変更を取り込んだ場合も、上記手順で生成ファイルを用意してください。
