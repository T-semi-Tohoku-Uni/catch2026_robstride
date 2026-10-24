# 学祭3軸アーム MCU（gakusai）

ROSとの通信はNode 2のCANRPCです。`target` Subject `0x200`と`current` Subject `0x210`はlittle-endian `float[3]`（12 byte）です。q0はCyberGear、q1/q2は左右Robstrideで、第4軸EL05は使用しません。モータ側の既存ID・単位・極性・offsetは保持しています。

基板間FDCAN1は1 Mbps nominal・2 Mbps data、Extended CAN FD/BRS OFFです。モータ側FDCAN3は既存Classic CAN/BRS OFFです。自動再送は無効です。通信定義は[共通ヘッダ](Core/Inc/canrpc.h)と[ROS bridgeのREADME](../../../catch-robo-2026/src/canrpc_bridge/README.md)を参照してください。

## 初期化時のリセット

BOOT通知のcapabilities bit0を有効にし、ROS Node 1から世代・期限等を検証したSYSTEM_RESETを受け付けます。受付後は新規目標を拒否します。

q0はCURRENT modeでMCU内ADRCが位置を保持するため、周期指令を単純に止めません。固定目標のADRCを継続しながら既存native STOP命令を1回送信し、専用TX event markerで実送信を確認した後にADRCを停止します。q1/q2は新規指令を止め、既存native PP目標を保持します。q0 STOPの送信確認前にはCANRPC ACCEPTEDを送らず、ACCEPTEDも実送信を確認してからMCUを再起動します。

受付から200 ms以内にq0 STOPを確認できなければ安全化失敗、ACCEPTEDを送信確認できなければ実行失敗としてFAULTを維持し、再起動しません。遅延STOPの送信完了はADRC停止だけに反映し、再起動成功へ戻したり自動enable/resumeしたりしません。通信故障時の位置保持や、モータの速度ゼロ・機械的停止を保証するものではありません。

再起動後は既存のenable・base homing・ADRC初期化を実行します。homing完了待ちには既存どおりMCU側timeoutがなく、hostの起動timeoutは機械動作停止を意味しません。

## ビルド

`arm-none-eabi-gcc`・CMake・Ninjaを用意して実行します。

```bash
cmake --preset Debug
cmake --build --preset Debug
```

ローカルcross buildとHAL mockを確認しています。実CAN、TX event、モータ停止・再起動・homingは実機未検証です。
