# 0.18.13 落地右转：自然恢复失败与整车关断验证

日期：2026-10-21，Asia/Singapore。关联固件：`0.18.13-front-recovery-failsafe`。

结论：**本次右前轮自然持续近零反馈后，新的恢复失败保护触发，软件记录显示整车输出关闭并保持故障锁存。此项验证通过；落地低速转向性能不通过，RF 停顿原因尚未解决。** 本轮没有修改、烧录、复位、校准固件，也没有清除故障或自动重试。

后续用户要求并明确授权清故障后仅重复一次，结果见[同版右转复测](repeat_134912/README.md)：RF 现场连续、没有再次触发故障，但 LF/RF 仍欠速。该复测已正常 STOP 停车；不要把本页第一轮结束时的锁存状态误当作复测后的当前状态。

## 条件与身份边界

用户确认落地、供电开启、约 80 cm 空间及线缆安全后授权执行。COM4 CDC、COM8 CH340，CH340 在打开前置 DTR/RTS 为 false，保持同一连接读取 trace。只执行右转 `vx=0, wz=-200 mrad/s`，最长 6 秒；实际收到故障约 4.829 秒即终止发送速度，不是完整运行 6 秒。

版本身份依据用户烧录报告及交付镜像/源码 manifest，本次没有捕获新启动版本行，也没有读取 Flash 来独立认证镜像。没有出现启动/复位横幅。实测软件行为与新保护一致，不用这一点冒充独立版本认证。

## 关键证据

所有 `t` 为 MCU uptime，单位 ms；UART 行前时间有些是输出时刻，而 `TRACE t=` 是历史采样时刻，不能混用。

| 事件 | MCU 时间 / 数据 |
| --- | --- |
| RF 最后一段连续原始零增量 | 674722..676282；79 个 20 ms 记录，计账 1580 ms，首尾时间差 1560 ms |
| RF 连续近零滤波反馈 | 674802..676282；75 个 20 ms 记录，计账 1500 ms，首尾时间差 1480 ms；阈值 20 cps |
| UART 锁存堵转 | 676282：`wheel_mask=0x04`，即 RF；同刻 runtime 报 health fault、motors disabled |
| 下一条完整 trace | 676302：`state=3 stop=6 fault=5 stby=0`，requested/applied 全零，四轮 PWM、FF/P/I 均零 |
| 首个 USB 故障 STATUS | 676326：FAULT/HEALTH/FAULT，controller fault=0，请求为零；轮速尚在衰减 |
| 停车观察 STATUS | 677560：仍 FAULT，四轮速度均零 |
| 最后 STATUS | 710076：仍 FAULT，flags=0x01D0，请求与四轮速度均零，CRC/RX overflow/TX drop=0 |

最后近零段在 ACTIVE 窗口右边界截断，随后转入故障，不能把原始零增量的 1580 ms 当成完整物理停车时长。第一条 FAULT trace 的 target 字段仍为上一控制采样目标 ±160 cps，但 requested/applied 已零、控制项和实际软件 PWM 已全零；不能只看残留 target 字段判定电机仍被驱动。

软件 trace 的 STBY/PWM 是控制器/驱动状态记录，不是独立示波器、电流或电机端电压测量。此次现场 RF 物理观察尚未返回，不能单靠计数断言电机卡死或编码器故障。

## 控制与负载事实

环形 trace 保留故障前 127 条完整 ACTIVE 记录及 1 条 FAULT 记录，ACTIVE 为 673762..676282，采样间隔全为 20 ms。它是末段窗口，不覆盖全部启动过程。

| 轮 | 目标 cps | 该 ACTIVE 窗口平均反馈 cps |
| --- | ---: | ---: |
| LF | +160 | +128.90 |
| LR | +160 | +158.79 |
| RF | -160 | -30.74 |
| RR | -160 | -164.58 |

RF 曾有实际编码器增量，后连续无增量；并非整个动作一直为零。窗口内 RF PWM 范围 -183..-125‰，最后维持 -133‰，FF=-124.6‰、P=-8.0‰、I=0，未出现积分堆到上限。故障前 LF/LR/RR 仍有反馈。与先前同版架空左右转正常相结合，异常与落地负载相关，但不能仅凭软件 PWM 判定力矩充足、硬件正常或机械必然有故障。

VM 遥测为 10.395..10.736 V，非电机端瞬态电压。11 条时序报告均为 motor_dt=20..20、safety_dt=10..10、overrun=0/0、log_drop=0；报告的最小 stack_free 为 motor=1272、safety=344（沿用固件日志单位）。没有证据支持把此次停顿归因于 USB 错帧、任务周期失控或积分饱和。

## 排除主机 STOP 造成停车

`summary.json` 的 `tx_commands` 保存本轮所有 CDC TX：

- 唯一 STOP (`type=0x02`) 在 `initial` 预检查阶段，运动开始后无 STOP。
- 全程没有 CLEAR_FAULT (`0x03`)。
- 速度命令共 97 条，均在 `RIGHT_motion` 阶段；收到故障后不再发速度，只用 GET_STATUS 保持会话和读取记录，CH340 `t` 只导出历史 trace。
- 接受数从 240 到 335，增加 95；拒绝数从 0 到 2。主机收到故障前仍在发的命令有 2 条被拒绝，不能把它们叫作 USB 协议错误或故障后的自动恢复尝试。
- 初次故障 STATUS 时轮速仍有惯性衰减，稍后 WHEEL/STATUS 均归零。最终没有发送 STOP，也没有清故障。

旧通用堵转规则门槛为目标 >=200 cps，此时 RF 目标固定 -160 cps；新规则允许连续近零 1500 ms 且短时恢复耗尽时报告失败。这里的轮掩码、连续近零时长、时序健康、FAULT/输出关闭和无主机 STOP 链条共同支持：**此次确实走了新增的前轮恢复失败关断路径。** 内部 `no_motion_ms/recovery_failed` 没有新增在线遥测，时序核对依据现有 trace、源码及离线集成验证，不虚构在线状态读数。

## 归档与离线复核

原始目录：`build/hardware_tests/loaded_pivot_compare_20261021_132945/`。本目录 `capture/` 保存 UART、USB 原始帧、解码、原始 summary、trace_parsed、analysis、独立 analysis_verdict 和实际采集脚本快照。原始 summary 的 `capture_result` 保持采集器原文 `COMPLETE_FIRMWARE_HEALTH_FAULT_OBSERVED_NOT_YET_ATTRIBUTED`；归因写在本说明和 `analysis_verdict.json`，不反写原始结果。

在项目根目录可运行（不接硬件、不发串口命令）：

```powershell
python docs\stm32_evidence\2026-10-21-v01813-ground-failsafe\verify_capture.py
```

该脚本复核输入文件哈希、RF 连续近零时序、故障轮、关断 trace、主机命令、锁存/零速和错误计数，并重新生成派生 `analysis_verdict.json`。`capture/loaded_pivot_compare.py` 是实际采集代码的取证快照，依赖原 `build/hardware_tests` 目录布局，不应直接在归档位置执行运动测试。

## 当前处理与下一步

本轮结束时故障保持锁存、四轮已停；未清除故障。保护验证不等于原因消失，不建议自动清故障再试或继续提高补力。先补齐用户对 RF 停车前是否实际停住的观察，再设计能区分负载/驱动与反馈的最小对照；如需接线、换轮、测电或再次运动，应先断电安排/确认安全条件。任何下一版均应另行提出范围并验证，不能用本次“安全停车通过”宣称底盘控制已全部完成。
