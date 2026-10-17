# USB STATUS flags 修复：Windows 实板回归补充

日期：2026-10-11，Asia/Shanghai。本文补充已有的 `STM32_USB_STATUS_FLAGS_FIX_DELIVERY.md`，不覆盖冻结的交付包。

## 1. 板上固件身份与范围

- 用户提供的实际 CH340 启动日志确认：`0.13.2-usb-sof-snapshot-fix`。
- Build：`Oct 11 2026 09:04:30`，与冻结交付产物一致。版本和 Build 是板上日志证据，不是 Flash 全量回读校验。
- 对应交付 HEX SHA256：`50CA002B029CA58D9451BF7F77260D686EC9B0665DBDB0B49C49932D2B79F7F0`。
- 本工作区无 Git 仓库，无法提供提交号；源码快照及哈希见原交付包。
- 本次只新增主机测试脚本和实板报告，未修改、重新编译或烧录固件，未改变 CubeMX、引脚、控制参数或保护逻辑。
- Windows CDC：COM4，VID/PID 0483/5740，USB serial `338536803133`；CH340 为 COM7。
- 用户明确确认四轮全部架空、车体固定、电机供电开启、手及线缆远离轮子后执行。

## 2. 正常 STOP / 重新运动：10 轮通过

使用新增 `Tools/usb_stop_regression.py`，这是无需 ROS 的等参数探针，**不是原 Ubuntu ROS 测试脚本**。

```powershell
python Tools\usb_stop_regression.py --port COM4 --boot-log <实际启动日志路径> --wheels-lifted
```

参数固定：30 mm/s；速度命令 20 Hz；每轮运动 0.35 s；STOP 后观察 0.7 s；10 轮。
脚本核对启动版本、Build 和 CDC 身份，检查 ACK、状态、故障、flags、通信错误计数及时间连续性。
遇异常停止追加运动，退出前尝试发送 STOP 并核对停车。不会清故障、重新校准、烧录或自动重连。

实测结果：

| 项目 | 结果 |
| --- | --- |
| 完成轮数 | 10/10，退出码 0 |
| STATUS 数量 | 285 |
| 观察到的 flags | 0x0150、0x0151、0x0171，全部包含 0x0010 |
| 0x0140 / 缺失链路位 | 0 次 |
| 每轮 STOP 后重新运动 | 均收到新速度指令，进入 ACTIVE，并有轮速反馈 |
| 最终状态 | IDLE / EXPLICIT，请求速度为零，四轮速度均为零 |
| 最终 faults | runtime=0，controller=0 |
| CRC / RX overflow / TX drop | 0 / 0 / 0 |
| 测试及退出清理异常 | 无；cleanup STOP ACK 和停止核对均通过 |

四轮均有反馈，历史 RF 只有约 5 mm/s 的现象本次未复现；本测试不验收地面直行、PI 跟踪或启动超调。

原始证据目录：`build/hardware_tests/usb_stop_20261011_172518_367606/`。
其中 `events.jsonl` 保存每次发出的命令及收到的原始字节、ACK、STATUS，`summary.json` 保存判定，`boot_log.txt` 保存用户启动日志。
脚本 SHA256：`f21c7c07c1af35856386d6f1b476138f7488341671e000da42f4c5af61525cbd`。

## 3. DTR 断会话：停车已观察到，完整测试未通过

随后执行已有工具：

```powershell
python Tools\usb_cdc_test.py --port COM4 --dtr-stop-test
```

40 mm/s 短暂运动约 0.8 s 后，工具关闭 DTR、等待约 0.35 s，再重开 DTR。
实测运动时进入 ACTIVE，重开后为 IDLE / TRANSPORT_LOST，请求速度为零，四轮速度为零，fault=0/0，flags=0x0150。
但是 `tx_drop` 从 0 增至 1，工具报告 `FAIL: TX drops changed from 0 to 1`，退出码 1。
**因此不能把整个 DTR 测试记为 PASS，也不能仅凭重开后的状态宣称停车延迟已经测准。**

关键输出摘录保存为 `dtr_test_observation.txt`；后续完整只读检查保存在 `post_dtr_readonly.log`。
只读检查确认停车保持、无旧运动回放、flags 为 0x0150，tx_drop 保持 1，没有继续增长。

静态检查：当前 `tx_dropped_frames` 仅在 TX 队列满时增加；关闭 DTR 清理排队数据本身不增加这个计数。
本次一次增量表明存在队列满事件，但没有足够证据定位到哪一帧或具体时刻。
主机 DTR 测试中的暂停读取、重开会话时的遥测/应答突发可能相关，尚未证实。
这是单独的待查项，不能归因于 SOF 旧时间竞态，也没有为掩盖失败而清零计数、放宽脚本或修改固件。

## 4. 结论与剩余验证

本次实板正常启停结果支持修复有效：10 轮、285 帧中未出现旧的缺失链路位，STOP 后都能重新运动。
离线旧源码负对照已复现旧时间下溢，新代码回归通过，见原交付文档。
但本次没有在同一板上烧录旧固件做 A/B，也没有执行原 ROS 驱动的权限锁定链路，不能证明历史板上异常的唯一原因。

尚需完成：

1. 用户补充本次测试期间 COM7 的 `USB SNAP` 和启动/停车日志，进行时间关联。
2. 单独定位 DTR 测试一次 TX 队列满事件；完整 DTR 回归目前未通过。
3. 现场物理拔掉 STM32 USB DEVICE 数据线、保留 MCU/电机供电及 CH340，验证真实断链停车。此次未执行。
4. Ubuntu 原脚本与 ROS 驱动端到端回归，确认正常 STOP 后运动权限不被错误锁定；真实异常保护及显式恢复仍必须保留。

本次板上 IMU `bias=0`，启动日志没有兼容的已保存 bias；它不影响此次 USB STOP 判定。本次没有发送校准或写 Flash 命令。
电源遥测约 9.4–9.5 V 是 D24A VM 监测值，不应据此判断电池原始端电压；本次不修改电源配置。

## 5. 交付给 ROS 开发者

原冻结固件交付包仍为 `build/deliveries/STM32_USB_STATUS_FLAGS_FIX_0.13.2_20261011.zip`，内容和哈希不变。
此次补充包 `build/deliveries/STM32_USB_STATUS_FLAGS_HARDWARE_20261011.zip` 包含报告、新测试脚本、原始启停证据及 DTR 后只读日志。
请同时提供两份包，并明确说明：正常 STOP 回归通过，但 DTR 的零 TX drop 标准尚未通过，ROS 端到端及物理拔线验证仍待执行。
