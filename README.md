# catch2026_robstride

STM32G474向けのファームウェアです。ビルドにはCMake 3.22以上、Ninja、
標準Cライブラリ（newlib / newlib-nano）を含むArm GNUツールチェーンが必要です。

## CLionでのビルド

1. このディレクトリをCMakeプロジェクトとして開きます。
2. CMakeプロファイルで`Debug`プリセットを選択します。
3. CMakeオプションに、使用するコンパイラのパスを指定します。
   このMacのSTM32CubeCLTでは次の値を使用します。

   ```text
   -DTOOLCHAIN_PREFIX=/opt/ST/STM32CubeCLT_1.21.0/GNU-tools-for-STM32/bin/arm-none-eabi-
   ```

   別のPCではインストール先に合わせて変更してください。
   `TOOLCHAIN_PREFIX`はディレクトリだけでなく、末尾の`arm-none-eabi-`まで含めます。
   この値はローカルのCMakeオプションに保存し、共有プリセットへの絶対パスの追加は不要です。
4. コンパイラを変更した場合は、CMakeキャッシュをリセットして再読み込みします。
5. ターゲット`catch2026_robstride`をビルドします。

生成物は`build/Debug/catch2026_robstride.elf`です。実機への書き込み・デバッグは別途設定が必要です。

## ターミナルでのビルド

```sh
cmake --preset Debug -DTOOLCHAIN_PREFIX=/opt/ST/STM32CubeCLT_1.21.0/GNU-tools-for-STM32/bin/arm-none-eabi-
cmake --build --preset Debug
```

`Release`ビルドでは両方のコマンドの`Debug`を`Release`に置き換えます。

`stdint.h`や`errno.h`が見つからない場合は、ビルドログに表示されるコンパイラを確認してください。
このMacでは、PATH上のHomebrew版GCCでこれらのヘッダーを見つけられず、
インストール済みのSTM32CubeCLT版に切り替えることでビルドできました。
CLionのツールチェーン設定に加え、上記のCMakeオプションで選択する必要があります。

CubeMXで再生成した後は、ルートの`CMakeLists.txt`に`can_init.c`、`cybergear.c`、
`robstride_app.c`の登録と、浮動小数点のprintf用オプション`-Wl,-u,_printf_float`が残っていることを確認してください。
