# 学祭3軸アーム MCU（gakusai）

ROSとの通信はCANRPC Node 2です。q0はCyberGear、q1は左RS03、q2は右RS03で、第4軸EL05は使用しません。既存motor ID・単位・極性・座標offsetを維持しています。

| 機能 | 既定wire | 意味 |
|---|---|---|
| arm_ready | Service 0、request 0 byte / response 48 byte | 初期化・各軸の診断照会。再homeしない |
| arm_target | Service 1、float32 LE×3 request 12 byte / response 20 byte | 目標適用後の各軸native TX/status確認。到達ではない |
| current | Subject 0x210、float32 LE×3、12 byte | 全3軸の新規feedbackを確認した実測PUBLISH |

旧target Subject 0x200のPUBLISH指令は受け付けません。基板間FDCAN1は既定1 Mbps nominal・2 Mbps data、Extended CAN FD/BRS OFF。native FDCAN3はClassic CAN/BRS OFFです。設定の正本はROS package `canrpc_bridge/config/gakusai.yaml`です。

## 初期化と診断

有限boot FSMで既存設定値を送り、各native命令の実TXと、投入前baseline以降の同軸type2 statusを確認します。mode変更前にSTOPを入れ、停止中の設定は運転状態0、enable/運転中設定は状態2、faultは0を要求します。メーカーstatusに要求Tagやparameter indexのechoがないため、同軸statusの時間的対応を確認するもので、遅延した旧statusとの厳密な因果相関やparameter readbackを保証しません。

`arm_ready`応答はu32 LE result（0 READY/1 NOT_READY/2 FAULT）、u32 flags、stage/reason/failure-axis/version各u8、3軸×12byteです。軸順はq0 CyberGear/q1左RS03/q2右RS03。各軸はmode、fault、validity、motor ID各u8、age_ms u32、last_command/response_state各u8、reserved u16=0。未受信はmode/fault=255、age=0xFFFFFFFFです。

flagsはbit0 boot完了、bit1 全3軸fresh、bit2 全3軸運転状態2かつfault0、bit3 全初期化TX確認、bit4 home switch到達。全0x1fでだけREADYです。validityはbit0 everseen/bit1 fresh/bit2 faultfree/bit3 responseconfirmed。FAULT確定時は安全STOPより先にstage/reason/各軸recordとageを固定保存し、validity bit7=FAULT_SNAPSHOTとして返します。後のSTOP返信で原因を上書きしません。

stage 0..28はboot FSM段階、29はREADY、30は通常制御のACK待ちです。reasonは0 NONE/1 ENQUEUE_TIMEOUT/2 TX_TIMEOUT/3 RESPONSE_TIMEOUT/4 MOTOR_FAULT/5 MODE_MISMATCH/6 HOME_TIMEOUT/7 FINAL_FEEDBACK_TIMEOUT/8 CONTROL_FAILURE。response_stateは0 NONE/1 QUEUE_WAIT/2 TX_WAIT/3 RESPONSE_WAIT/4 CONFIRMED/5 FAILEDです。完全なwire定義とcommand番号はROS packageの`docs/protocol.md`を参照してください。

有限待ちの既定値は命令200ms、homing10秒、最終feedback1秒、fresh age100ms、命令間10ms、home停止後100msです。実機適合は未確認です。原点精度や機械停止を確認する値ではありません。

## 位置要求とnative制御

`arm_target`応答はu32 result（0 APPLIED/1 INVALID/2 NOT_READY/3 FAULT）、要求目標のecho12byte、u32 confirmed-axis maskです。0かつmask7だけが全3軸適用確認です。位置到達を意味しません。

native各軸は最大1件inflightで、前のTXと同軸statusが確認できるまで次のnative指令を発行しません。10msの制御周期を維持し、BUSY中は新たなnative指令を間引きます。q0 ADRCは新feedback分だけobserverを更新し、currentはenqueue値ではなくnative TXeventで確認した値を保持します。native enqueue時にtarget RPC tokenを固定するので、旧目標のinflight応答で新RPCを成功にしません。

native ACK期限の既定は40msです。失敗・通常RPC期限切れでは診断snapshotを保存し、指令gateをlatchし、q0専用STOPを試みます。自動enableや古い目標の再実行は行いません。q1/q2の既存native PP目標や機械停止を解除・保証するものではありません。FAULT解除のためにreadonly初期化照会だけで再home/enableは行いません。現在のSYSTEM_RESETはREADYでのみ新規受付するため、FAULTからは手動電源再投入等で新READY世代へ復帰させる必要があります。

## SYSTEM_RESETの安全境界

世代・権限・期限を検証し、q0専用STOPの実TX確認後にADRCを止め、ACCEPTEDも実TX確認してからMCUを再起動します。この専用安全化STOPは既存のTX確認契約であり、通常init STOPのtype2応答確認とは区別します。物理速度ゼロの保証ではありません。

通常native eventと専用STOP markerを別々に保持します。RESET中の旧targetはUNKNOWNとして破棄し、STOP確認後の旧currentに運転状態2を要求して偽FAULTにしたり、旧targetを成功完了したりしません。200ms期限を越えればリセットせず、停止未確認を成功扱いしません。再起動後は既存enable/homingを有限FSMで実行します。

## 共通coreとビルド

wire/peer/RPCは[canrpc_core.h](Core/Inc/canrpc_core.h)、STM32処理は[canrpc_stm32.h](Core/Inc/canrpc_stm32.h)、生成設定は[canrpc_project.h](Core/Inc/canrpc_project.h)、アーム固有は[arm_boot.h](Core/Inc/arm_boot.h)と[arm_control.h](Core/Inc/arm_control.h)です。

ROSワークスペースの`src/canrpc_bridge/tools/canrpc_profile.py PROFILE --mcu-root MCU_DIRECTORY`でcore/adapter/設定/.iocを同期し、`--check`で一致を確認してください。同期後の単独ビルドにROS/Python/兄弟checkoutは不要です。

```bash
cmake --preset Debug
cmake --build --preset Debug
```

ローカルcross build・HAL mockを確認しています。実CAN/TXevent/status・実モータ停止/原点/再起動は未検証です。

native受信はtype2 statusの6bit faultを対象とし、type21 FAULT_REPORTのfull32bit値は未取得です。type21だけが返る場合は応答timeoutとなり、詳細faultを表示できません。根拠のないfault写像は追加していません。
