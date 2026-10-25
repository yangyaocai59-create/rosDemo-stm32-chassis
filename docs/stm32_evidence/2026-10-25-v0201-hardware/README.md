# 0.20.1 四轮控制上板测试，2026-10-25

六项短时测试通过。板上固件按用户报告为 `rosDemo-0.20.1`；未复位获取启动横幅，已核对 V2 运行日志与 TRACE2 schema3/control=1。COM4 为 USB CDC，COM8 为 CH340。架空/落地条件未明确，按 unspecified 保存，不能将结果视作带载性能验收。

| 测试 | 发令 | MCU ACTIVE 时长 | 停车原因 | 结果 |
| --- | --- | ---: | --- | --- |
| 左转 | +0.40 rad/s，3 s | 2.969 s | EXPLICIT | 通过 |
| 右转 | -0.40 rad/s，3 s | 2.966 s | EXPLICIT | 通过 |
| 前进 | +40 mm/s，3 s | 2.970 s | EXPLICIT | 通过 |
| 后退 | -40 mm/s，3 s | 2.974 s | EXPLICIT | 通过 |
| 命令超时 | +40 mm/s，1 s 后不再发速度 | 1.200 s | COMMAND_TIMEOUT | 通过 |
| DTR 断开 | -40 mm/s，1 s 后撤销 DTR | 0.976 s | TRANSPORT_LOST | 通过 |

所有测试四轮计数方向正确，故障为 0/0，拒绝/CRC/溢出/丢包增量为 0。每次 STOP 后确认四轮零速，TRACE 最后 STBY=0、四轮 CCR=0；结束后串口关闭。没有清除故障、复位、修改校准或烧录。DTR 用软件撤销，未测试拔线、断电或实体急停。

电机/安全任务周期为 20/10 ms，无超期/日志丢失；累计执行最大值 motor=366 us、safety=91 us、controller=246 us。最小剩余栈 motor=812 B、safety=344 B，仅代表这些测试工况。原始日志见每次采集目录。

**性能待优化**：3 秒测试的启动后样本仍存在收敛误差。左转 RF 在排除前 800 ms 后的 109 个样本中有 25 个达到 I=50‰；当前前馈、积分余量仍需实测标定。没有根据短时数据调整 FF/PI 或轮距。IMU 保存偏置无效，参考角只扣除本次短静止基线；没有外部角度真值。

**发现的代码缺陷**：0.20.1 日志年龄偶现 4294967280/4294967277，但同步 TRACE 反馈年龄均为 1 ms。旧循环时间减更新后的反馈时间造成无符号回绕。安全年龄检查也有同类潜在误停条件，本轮硬件未实际触发；`clock_reproducer` 中相同新增回归在 201 配置断言失败、202 通过。已另生成 0.20.2 修复候选，保持 0.20.1 原件和板上版本不变。修复说明见 [0.20.2 文档](../../STM32_WHEEL_CONTROL_0.20.2_20261025.md)。

`captures/` 内每次包含 USB 原字节、逐帧 payload、发令序列、UART 原字节/文本/分块时间、summary、完整解析 trace 和 analysis。超时、断链后的最终清理会额外发送显式 STOP，因此应读取 `stop_checkpoint_status` 核对实际停车原因，不能只看最终清理后的原因 2。分析的 MCU 起止事件按测试种类分别选择 STOP_REQUEST/COMMAND_TIMEOUT/TRANSPORT_LOSS，避免将事件环中历史记录混入。

`comparison.json` 汇总六项结果；`readonly/` 为最初静止检查。`scripts/` 是本轮最终工具副本：前三项运动的导出超时为 45 秒，之后改为 90 秒以留串口打印余量，并补充停车检查点及采集尾段通信计数检查；原始采集数据未改写。各目录原 `manifest.json` 保存采集时文件哈希，全局 `capture_manifest.json` 另覆盖分析和脚本等全部证据。README 和全局清单自身不进入该清单。

离线复核（无设备访问）：

```powershell
python -B Tools/analyze_wheel_v0201_hardware.py build/hardware_tests/v0201_left_explicit_20261025_052734_560585
```

分析器适用于本次未跨 MCU uptime 回绕的记录；固件回绕安全另外由主机回归验证。
