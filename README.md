# 学祭3軸アーム MCU（gakusai）

ROSとの通信はCANRPC Node 2です。q0はCyberGear、q1は左RS03、q2は右RS03で、第4軸EL05は使用しません。既存motor ID・単位・極性・座標offsetを維持しています。

| 機能 | 既定wire | 意味 |
|---|---|---|
| arm_ready | Service 0、request 0 byte / response 64 byte | 初期化・各軸の診断照会。再homeしない |
| arm_initialize | Service 2、request 0 byte / response 64 byte | 1boot1回の初期化開始。応答は現在診断で完了通知ではない |
| arm_target | Service 1、float32 LE×3 request 12 byte / response 20 byte | 目標適用後の各軸native TX/status確認。到達ではない |
| current | Subject 0x210、float32 LE×3、12 byte | 全3軸の新規feedbackを確認した実測PUBLISH |

旧target Subject 0x200のPUBLISH指令は受け付けません。基板間FDCAN1は既定1 Mbps nominal・2 Mbps data、Extended CAN FD/BRS OFF。native FDCAN3はClassic CAN/BRS OFFです。設定の正本はROS package `canrpc_bridge/config/gakusai.yaml`です。

## 初期化と診断

有限boot FSMで既存設定値を送り、各native命令の実TXと、投入前baseline以降の同軸type2 statusを確認します。mode変更前にSTOPを入れ、停止中の設定は運転状態0、enable/運転中設定は状態2、faultは0を要求します。メーカーstatusに要求Tagやparameter indexのechoがないため、同軸statusの時間的対応を確認するもので、遅延した旧statusとの厳密な因果相関やparameter readbackを保証しません。

`arm_ready`応答はu32 LE result（0 READY/1 NOT_READY/2 FAULT）、u32 flags、stage/reason/failure-axis/version各u8、3軸×12byte、自MCU Boot ID16byteです。versionは2です。軸順はq0 CyberGear/q1左RS03/q2右RS03。各軸はmode、fault、validity、motor ID各u8、age_ms u32、last_command/response_state各u8、reserved u16=0。未受信はmode/fault=255、age=0xFFFFFFFFです。

flagsはbit0 boot完了、bit1 全3軸fresh、bit2 全3軸運転状態2かつfault0、bit3 全初期化TX確認、bit4 home switch到達。全0x1fでだけREADYです。validityはbit0 everseen/bit1 fresh/bit2 faultfree/bit3 responseconfirmed。FAULT確定時は安全STOPより先にstage/reason/各軸recordとageを固定保存し、validity bit7=FAULT_SNAPSHOTとして返します。後のSTOP返信で原因を上書きしません。

stage 0..28はboot FSM段階、29はREADY、30は通常制御のACK待ち、31は初期化指示待ちです。reasonは0 NONE/1 ENQUEUE_TIMEOUT/2 TX_TIMEOUT/3 RESPONSE_TIMEOUT/4 MOTOR_FAULT/5 MODE_MISMATCH/6 HOME_TIMEOUT/7 FINAL_FEEDBACK_TIMEOUT/8 CONTROL_FAILURE/9 INITIALIZATION_TIMEOUT。response_stateは0 NONE/1 QUEUE_WAIT/2 TX_WAIT/3 RESPONSE_WAIT/4 CONFIRMED/5 FAILEDです。完全なwire定義とcommand番号はROS packageの`docs/protocol.md`を参照してください。

有限待ちの既定値は命令200ms、homing10秒、最終feedback1秒、fresh age100ms、命令間10ms、home停止後100msです。実機適合は未確認です。原点精度や機械停止を確認する値ではありません。

## UARTでの角度確認

USART2（115200 baud、8N1）の既存printfに、`angles_live`と各軸の`angle`行を追加しています。
200ms間隔に加え、boot完了／FAULTへの変化時にもmainから出力します。割込中に整形せず、
既存の非同期送信バッファを使います。診断のためのCAN要求は送りません。

```text
angles_live t_ms=12500 stage=28 boot_complete=1 rpc=1 target_token=0
angle axis=1 id=4 mode=2 valid=1 fresh=1 age_ms=8 motor_rad=-1.0000 joint_rad=-0.0000 target_rad=+0.0000
```

上記は書式例です。axis 0=CyberGear、1=左RS03、2=右RS03です。
`motor_rad`は最後に受信・復号したモーター角度、`joint_rad`はROSへ返す座標へ変換した角度、
`target_rad`はMCUの現在の関節目標変数（すべてrad）です。変換はq0=motor、
q1=−(motor+1.0)、q2=motor+1.884で、正規化や多回転補正を追加しません。
`target_rad`は未初期化でも表示され、送信済み・モーター側の保持目標・到達を保証しません。
`target_token=0`はまだ通常の関節目標RPCを受理していないことを示します。

未受信は`valid=0`、角度`nan`、`age_ms=4294967295`です。受信後も古い値は残り、
`fresh=0`と増加する`age_ms`で区別します。`t_ms`はMCU起動からの取得時刻です。
`angles_live`は故障後も最新の受信バッファを表示するため、`flags`のbit7が立った
既存`axis`行（故障時に固定した診断・age）とは取得時点が異なります。
出力はbest effortで、混雑／reset時には欠落し得ます。`drop`も確認してください。
UARTログは実機の原点・許容範囲・物理的な停止を保証しません。

## 位置要求とnative制御

`arm_target`応答はu32 result（0 APPLIED/1 INVALID/2 NOT_READY/3 FAULT）、要求目標のecho12byte、u32 confirmed-axis maskです。0かつmask7だけが全3軸適用確認です。位置到達を意味しません。

native各軸は最大1件inflightで、前のTXと同軸statusが確認できるまで次のnative指令を発行しません。10msの制御周期を維持し、BUSY中は新たなnative指令を間引きます。q0 ADRCは新feedback分だけobserverを更新し、currentはenqueue値ではなくnative TXeventで確認した値を保持します。native enqueue時にtarget RPC tokenを固定するので、旧目標のinflight応答で新RPCを成功にしません。

native ACK期限の既定は40msです。失敗・通常RPC期限切れでは診断snapshotを保存し、指令gateをlatchし、q0専用STOPを試みます。自動enableや古い目標の再実行は行いません。q1/q2の既存native PP目標や機械停止を解除・保証するものではありません。FAULT解除のためにreadonly初期化照会だけで再home/enableは行いません。現在のSYSTEM_RESETはREADYでのみ新規受付するため、FAULTからは手動電源再投入等で新READY世代へ復帰させる必要があります。

## SYSTEM_RESETの安全境界

世代・権限・期限を検証し、初期化開始後はq0専用STOPの実TX確認後にADRCを止め、ACCEPTEDも実TX確認してからMCUを再起動します。この専用安全化STOPは既存のTX確認契約であり、通常init STOPのtype2応答確認とは区別します。物理速度ゼロの保証ではありません。

通常native eventと専用STOP markerを別々に保持します。RESET中の旧targetはUNKNOWNとして破棄し、STOP確認後の旧currentに運転状態2を要求して偽FAULTにしたり、旧targetを成功完了したりしません。200ms期限を越えればリセットせず、停止未確認を成功扱いしません。再起動後は通信READY・アプリWAITINGへ戻り、新しいarm_initializeまでenable/homingを実行しません。

## 共通coreとビルド

wire/peer/RPCは[canrpc_core.h](Core/Inc/canrpc_core.h)、STM32処理は[canrpc_stm32.h](Core/Inc/canrpc_stm32.h)、生成設定は[canrpc_project.h](Core/Inc/canrpc_project.h)、アーム固有は[arm_boot.h](Core/Inc/arm_boot.h)と[arm_control.h](Core/Inc/arm_control.h)です。

ROSワークスペースの`src/canrpc_bridge/tools/canrpc_profile.py PROFILE --mcu-root MCU_DIRECTORY`でcore/adapter/設定/.iocを同期し、`--check`で一致を確認してください。同期後の単独ビルドにROS/PyYAML/兄弟checkoutは不要（毎buildの設定検査にPython 3標準ライブラリが必要）です。

```bash
cmake --preset Debug
cmake --build --preset Debug
```

ローカルcross build・HAL mockを確認しています。実CAN/TXevent/status・実モータ停止/原点/再起動は未検証です。

native受信はtype2 statusの6bit faultを対象とし、type21 FAULT_REPORTのfull32bit値は未取得です。type21だけが返る場合は応答timeoutとなり、詳細faultを表示できません。根拠のないfault写像は追加していません。

## CubeMXでの再生成

`catch2026_robstride.ioc`を開き、`Keep User Code when re-generating`を有効のまま
CMake向けに生成してください。アームの状態・callback・boot/control/reset処理は
`main.c`のUSER CODE領域、固有実装は`arm_boot.h`・`arm_control.h`・`arm_board.h`・`arm_debug.h`と
`can_init.c`・`cybergear.c`・`robstride_app.c`にあります。生成対象外ファイルと
最上位CMakeのユーザーsource登録・`_printf_float`リンク指定を保持してください。
`cmake/stm32cubemx/CMakeLists.txt`へ独自sourceを追加しません。

profile変更は上記同期ツールで`.ioc`と`canrpc_project.h`を更新した後、CubeMXで
生成コードへ反映し、`--check`と単独cross buildを実行してください。生成された
FDCAN1初期化は数値で保持し、生成領域へ手動で設定マクロを挿入しません。
設定不一致を二度目の`HAL_FDCAN_Init`で上書きする方法は使いません。

現アーム実装はSYSCLK/HCLK/PCLK1 80 MHz、FDCANカーネルPCLK1、TIM6の
prescaler79・period999（1 ms）を前提にしています。native FDCAN3はnominal
prescaler4・TSEG1=15・TSEG2=4（1 Mbps）、再送無効、拡張filter1を維持します。
FDCAN3 controllerのFD対応設定と、native送信headerのClassic CAN/BRSなしは
別の設定です。board FDCAN1のbitrateはprofileに従います。
`arm_board_config_valid`はboot・motor開始前にclock/native設定/周期と
FDCAN1/3 IT0/1・TIM6・EXTI0/1 IRQのenable/priority0を検査し、不一致では
`Error_Handler`へ進みます。共有board検査と合わせ、異なるclockやIRQ配置を
許可したことにはなりません。

配線はFDCAN1 PA11/PA12 AF9、FDCAN3 PA8/PA15 AF11、home入力PA0の
input/no-pull、PC0/PC1のrising EXTI/no-pullを保持してください。
UART等の表示設定を変更する場合もビルド・profile/board検査を再実行してください。
clock・制御周期・native bitrate・pin/IRQを変更して本実装をそのまま使用することは
検証対象外です。HSI48/RNGは非生成の共通adapterが直接設定・異常検査し、
HAL RNG module追加は不要です。`.ioc`のRNG周波数表示を実行中のRNG設定の
保証として扱いません。

今回の確認はローカルcross buildとHAL mockです。CubeMX GUIによる実再生成や
実機のclock/CAN/停止動作は未確認です。

### 単独buildの必須設定チェック

ビルドにはPython 3（標準ライブラリのみ）が必要です。ROS/PyYAML/兄弟checkoutは不要です。
`python3 tools/check_canrpc.py --repo . --role arm`で単独検査し、CMakeでも毎build同じcheckerを実行します。
profile同期→CubeMX再生成→profile側`--check`と単独checker→cross buildの順で確認してください。
共有`canrpc_board.h`はUSER2のHAL初期化後とCAN start後に実clock/mux/divider・handle/register・FIFO/filter/IRQを検査します。
不一致はCANRPC/app開始を拒否しますが、生成GPIO初期出力はguardより先です。周期的な設定上書きや再初期化は行いません。
実CubeMXが利用できなかったため、USER保持fixture・設定破壊負例・cross buildを検証し、実CubeMX生成は未検証です。

## 駆動電源の後投入と明示初期化

MCU起動直後は通信READY・アプリWAITING（診断stage31）です。CANRPC `arm_initialize`（service2、request0B/response64B）を受けるまでnative motor CANへ送信しません。`arm_ready`（service0）は読取専用で初期化を始めません。両応答は従来48B診断＋自MCU Boot ID16Bの64B/version3です。通信READYはモーター動作可能を意味しません。

`arm_initialize`は開始フラグを記録して直ちに現在の診断を返します。受理ACK/応答は初期化完了ではなく、呼出側はarm_readyの完了・全軸fresh等を確認します。原点復帰を含むFSMはmainで有限実行し、全体期限はprofileのinitialization_timeout_ms（既定20000ms）、超過reason9です。1MCU bootで1回だけ開始し、RUNNING・READY・FAULTで再要求しても再実行しません。失敗後は手動MCU resetが必要です。

初期化未開始のSYSTEM_RESET準備はmotor STOPを送らず成功します。開始後のSYSTEM_RESET・原点復帰・mode切替・故障時STOPとその既存確認は維持します。初期化未完了中は通常target要求を拒否し、初期移動の周期native制御をFSMが専有します。実機の電源・原点・停止性能は未検証です。

通常手順はMCU電源投入→駆動電源投入→UI INITIALIZEです。駆動電源がないままINITIALIZEすると有限時間でFAULTとなり、1bootの試行を消費します。MCUだけを手動reset/電源断し駆動電源を維持した場合、モーターが以前の指令を保持する可能性があります。新bootのWAITINGは過去の運転を検出できず、指示前STOPを送らないため、この操作の機械停止を保証しません。運転開始後のCANRPC SYSTEM_RESETは従来のSTOP実TX確認を維持しますが、外部resetや電源断とは区別してください。

## INITの初期移動と最終完了

設定・原点復帰後は既存の初期角{0,0,2}radをMCUが専有して制御し、通常arm_targetを受け付けません。全3軸の新しい実測が既存nav由来の角許容{0.05,0.01,0.01}rad内、位置差分由来の速度0.05rad/s以下で200ms継続した場合だけboot_completeを立て、arm_ready version3/flags127で最終完了を返します。設定ACKは移動完了ではありません。全体期限は既存20秒のままです。

初期角・native offset・PP速度10/加速度1/電流10・既存故障q0STOP方式は維持し、未設定機械rangeや新faultpolicyの許可gateは追加しません。INIT前nativeTX0を維持し、移動中のstale/fault/非有限角/期限超過は成功にしません。新しい全TorqueOFFは行いません。角度UART追加は保持します。
