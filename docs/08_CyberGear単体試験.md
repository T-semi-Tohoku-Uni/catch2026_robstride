# CyberGear単体試験

現在の既定ビルドは `Core/Inc/app_mode.h` の
`APP_CYBERGEAR_STANDALONE_TEST=1` により単体試験になります。

## 書き込むファイル

`build/cybergear-swing30-3a/cybergear_swing30_v4_3a.elf`

起動時と定期ログに `SWING30_V4_3A` が表示されることを確認してください。
`then hold zero` と出るログは原点保持版のものです。CubeProgrammerの「ブラウザ」で
上の別名ファイルを選び直し、検証付きで書き込んでから実行してください。

CubeProgrammerでこのELFを選び、「プログラミング検証」と
「プログラミング後に実行」を有効にしてダウンロードします。
実行を選ばない場合は、書き込み後に基板をリセットします。
外部基板のCAN開始指令やデバッガ操作は不要です。

この版は位置制御の電流上限を3 Aに変更したものです。ホストテストでは、3 A設定で
停止距離保護 `fault=16` が原点付近でも作動する条件を再現しています。
電流上限の変更だけでは、この停止原因は解消しません。実機の往復完了は未確認です。

## 起動後の動作

1. CAN3上のCyberGear（ID `0x7f`、ホストID `0xfe`）の応答を待ちます。
2. 既存の原点探索を1回実行します。PA0のセンサーを探し、初回探索では
   60度進んでも切り替わらなければ一度反転します。低速で両方向の境界を測定し、
   その中央へ移動して原点を設定します。
3. CyberGearの位置制御を開始し、+30度に向かいます。
4. +30度 → -30度 → +30度を繰り返します（全幅60度）。軌道の目標速度上限は0.3 rad/s
   （約17.2度/s）です。実際の速度は加減速と電流予算でも制限されます。
   軌道完了・実測位置が目標の±0.5度以内・推定速度が
   0.03 rad/s以下の状態を1秒維持してから反転します。

位置制御中の電流上限は3 Aです。原点探索の速度制御は従来のままです。
1区間20秒で到達・待機が完了しない場合、実測位置が原点から±33度を超えた場合、
またはフィードバック/制御異常時にはSTOPを要求し、再始動しません。
±33度は停止指令を出す判定境界であり、停止完了位置を保証するものではありません。

RobStrideのID 3・4・5には有効化、位置・速度・電流、ゼロ設定などの
フレームを送信しません。タイマーのRobStride処理と起動処理を通らず、
送信関数でも全RobStrideフレームを拒否します。
RobStrideへのSTOP送信も行わないため、開始前にそれらが停止している状態で使います。
試験中は外部基板の目標値・開始・復帰指令を無視し、CAN1への周期送信もしません。

## 結果の確認

USART2 TX（PA2）とGNDをUSB-UARTに接続し、115200 bps・8N1・フロー制御なしで確認します。
USB-UARTのTXをUSART2 RX（PA3）にも接続すると、往復中に `x` または `X` を送って
STOPを要求できます。この入力は原点探索中には処理されません。
ST-LINKで接続しただけでは、UART配線がなければログは表示されません。

```text
CG TEST: SWING30_V4_3A; homing then +/-30 deg; RobStride TX disabled
CG TEST: homing started
CG TEST: homing complete; +/-30 deg, dwell=1000 ms
CG TEST: ADRC current<=3.0 A, speed<=0.3 rad/s; x=stop
CG TEST SWING30_V4_3A state=6 fault=0 online=1 age=...
CG TEST q=... v=... target=0.5236 limit=...
CG TEST i_cmd=... halted=0 legs=...
CG TEST: new target=-0.5236 rad
```

`state=6` は位置制御実行中です。原点探索後、0.5秒ごとに状態・位置・速度・
センサー入力を表示し、基板LED（PD2）を反転します。
`target` が +0.5236 / -0.5236 radへ切り替わることを確認します。
`halted reason=2` は制御/受信異常またはユーザー停止、3は往復範囲からの逸脱、
4は到達タイムアウトです。電流不足で到達しない場合も、自動的に電流を上げません。
位置制御の開始失敗は状態・faultでも確認してください。
原点探索などに失敗すると `CG TEST FAILED` を表示し、STOPの再試行後に待機します。
運転中の異常は既存の制御保護で停止状態に移行します。通信途絶による自動リセットや
自動再原点探索は行いません。再試験には明示的な基板リセットが必要です。

## 設定と再ビルド

機械パラメーターは `Core/Inc/cybergear_config.h` を使用します。
起動時の設定検査は維持し、位置保護境界は通信表現範囲内の±12.5 radとしています。
ホストテストは設定の整合性を確認し、実機の機械許容値を検証するものではありません。

往復の振幅・待機時間・試験用上限は `Core/Inc/cybergear_test_motion.h` にあります。
位置制御開始前に電流上限と軌道用の加減速電流予算を3 Aに設定し、補償電流上限も整合させます。
`fault=16` は `CG_FAULT_STOP_MARGIN` です。制動電流、5 A/sの電流変化制限、
外向き加速度16 rad/s²、保証制動加速度1.5 rad/s²などから停止距離を見積もります。
3 A設定では原点付近でも位置限界までの距離を超える条件があり、ホストテストで再現しています。
停止距離の保護処理とこれらの加速度・電流変化制限は変更していません。
既存の制動・加速度の設定値が実機で成立することを、この変更で検証したわけではありません。

CubeMX生成済みファイルとCMake・Ninja・Arm GCCがPATHにある環境で実行します。

```powershell
cmake -S . -B build/cybergear-swing30-3a -G Ninja '-DCMAKE_BUILD_TYPE=Release' '-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake' '-DCATCH_FIRMWARE_BASENAME=cybergear_swing30_v4_3a'
cmake --build build/cybergear-swing30-3a
```

通常の全軸動作へ戻す場合は `APP_CYBERGEAR_STANDALONE_TEST` を0に変更し、再ビルドします。
