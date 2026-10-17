# 0.13.3 实板 50 cm 前后行程与停车静置回归

日期：2026-10-12，Asia/Shanghai。
固件：`0.13.3-usb-tx-burst-fix`，Build `Oct 12 2026 08:24:27`。
本轮未修改、编译或烧录固件，仅扩展主机测试脚本。

## 条件及执行范围

用户确认落地、电机电源开启、约 80 cm 空间、串口助手关闭。上一轮短行程及两分钟静置通过；提出约 50 cm 低速前后运动后，用户回复“没有，继续吧”（按停车后没有继续爬行理解）。执行前再次告知本轮参数和异常时切断电机电源的方法。

使用 COM4 STM32 CDC（0483:5740，serial `338536803133`），同时只读捕获 COM8 CH340（1A86:7523）。先收到 CH340 数据、检查 CDC 状态，再倒计时 5 s 执行。未清故障、注入故障、自动重连或重新校准。

唯一脚本修改：`Tools/ground_stop_encoder_probe.py`。
新增显式 `--long-travel-500mm` 模式；不加该参数仍为原 30 mm/s × 0.8 s 短行程。长模式必须同时提供 CH340 捕获端口。

- 命令速度：前后分别 +40 / -40 mm/s，角速度 0，速度命令间隔 50 ms。
- 目标：四轮平均归一化增量达到 500 mm 后立即发送 STOP；使用 D65 mm、1560 counts/rev 换算。
- 每段最多 16 s；超时先 STOP，确认停稳后报告失败，不开始下一方向。
- 额外中止门限：STATUS 单轮速度超过 80 mm/s、单轮增量行程超过 650 mm、四轮增量行程差超过 150 mm、逆向增量超过 20 mm。
- 保留 ACK/STATUS/WHEEL 300 ms 超时、链路/fault/CRC/溢出/丢帧检查、CH340 采集健康检查和 finally STOP。
- 这些是基于遥测的主机保护，不保证真实地面路径或方向。编码器距离不等于实测地面距离；人工场地监护及固件原有 250 ms 命令超时仍必要。
- 每次 STOP 后观察 2 s，要求 IDLE / EXPLICIT、请求为零、四轮 cps 零；后退停稳后再静置 120 s。

执行前离线检查通过：原 STOP gate/codec、瞬态计数变化捕获、前后行程换算和门限拒绝、使用模拟时钟/传输的实际 drive 循环目标到达与 16 s 超时 STOP，以及 Python 语法检查。离线检查不连接串口。

## 实际结果

证据目录：`build/hardware_tests/ground_stop_encoder_20261012_091734_067794/`。

| 项目 | 结果 |
| --- | --- |
| 前进运行时间 | 12.391 s |
| 前进 LF/LR/RF/RR 增量 | `[3830,3830,3847,3873]` counts |
| 前进各轮估算行程 | `[501.35,501.35,503.57,506.97]` mm；平均 503.31 mm（包含停车滑行） |
| 后退运行时间 | 12.406 s |
| 后退 LF/LR/RF/RR 增量 | `[-3831,-3839,-3847,-3865]` counts |
| 后退各轮估算行程绝对值 | `[501.48,502.52,503.57,505.93]` mm；平均 503.37 mm（包含停车滑行） |
| 前进 / 后退峰值轮速 | `[55,55,56,55]` / `[55,54,57,54]` mm/s，含启动瞬态 |
| 两次 STOP 结果 | 都应答并停稳，IDLE / EXPLICIT、请求零、四轮 cps 零 |
| 全程 STATUS | 1682 帧，flags 仅 `0x0150` / `0x0171`，没有 `0x0140` |
| MCU CRC / RX overflow / TX drop | 全程均 0 / 0 / 0 |
| 主机解析 CRC | 0 |
| ACK | 567 个，result/fault 都为 0，无报告的匹配超时 |
| 静置观察 | MCU 1284009..1404009 ms；120 s，2400 帧，间隔全部 50 ms |
| 静置累计计数 | `[25,47,-44,-26]` 全程固定，四轮变化次数 0、最大绝对 cps 均 0 |
| 最终清理 | STOP 应答且确认停稳；IDLE / EXPLICIT，flags `0x0150`；串口关闭 |

## 同步串口核对

- 前进 STOP：1267666 ms；后退 STOP：1282069 ms。两次均记录 `ACTIVE -> IDLE: explicit stop; motors disabled`。
- 两次 STOP snapshot 都是 `alive=1 dtr=1 open=1`，`now=last`，age=0。
- 最终 STOP：1404059 ms，age=1。CDC 关闭：1404566 ms，DTR=0/session ended；关闭前已停稳，不能当作运动中 DTR 停车测试。
- 在相同静置窗口，左右各 600 条 ENC 日志：LF raw=25、LR raw=47、RF raw=44、RR raw=26 均固定，delta 均 0；右轮归一化 total 为负符合方向配置。
- 捕获 353485 字节；`ch340.log` 与 `ch340_raw.bin` 字节完全一致。无捕获错误、ERROR 行或新 Boot Banner。
- 未记录 USB TX 压力诊断行；零丢帧结论来自完整 STATUS 计数，而非诊断行缺失。

目录包含启动身份、USB 原始收发与解码事件 `events.jsonl`、`summary.json`、CH340 原始字节、主机时标分块与文本日志。脚本 SHA256 与 summary 中记录一致：
`685AB7AEE43E9A0ED52472A0530CC26950C2F28090FAC98C0C6EC3FC783B789A`。

## 结论与未覆盖项

本轮 50 cm 编码器估算前进→STOP→后退→STOP→120 s 静置完整通过；未复现链路位异常、TX 丢帧或停稳后的计数增长。与旧版 0.13.2 在短行程 STOP 附近 tx_drop 0→1 的记录相比，这是额外的修复有效性证据。

不能据此宣布所有问题根治：没有复现原 ROS 80 cm 历史条件，未测真实地面距离/偏航/回到起点，历史 LF 静止 +4 的根因仍未定位；ROS 端运动权限恢复尚未验收。还需重复启停、运动中 DTR/物理断链/命令超时、实际栈高水位等检查。电气 PD10/CCR 状态和编码器供电电压本轮未用仪表核验。

本轮完成后不自动开始其他运动。随后用户回复“都正常，不用重复了”，确认真实前进/后退及停车表现正常，并要求不再重复运动测试。实际地面距离/偏差仍未外部测量。

## 重跑入口（必须重新确认场地与电源）

```powershell
python Tools\ground_stop_encoder_probe.py --self-test
python Tools\ground_stop_encoder_probe.py --port COM4 --debug-port COM8 --boot-log <当前0.13.3启动日志路径> --ground-safe-confirmed --long-travel-500mm
```

端口需重新枚举确认，不要照搬 COM 编号；必须有人现场监护。只读静置监控应使用 `static_encoder_monitor.py`，不是上述运动入口。
