# 静止编码器排查：Windows 两分钟只读基线

日期：2026-10-12，Asia/Shanghai。本轮未复现；历史停车后 LF 增加 4 个计数的原因仍未确定。

## 身份及测试条件

- 用户最新启动日志：STM32F407ZGT6；版本 `0.13.2-usb-sof-snapshot-fix`；Build `Oct 11 2026 09:04:30`；IMU bias 从 Flash 恢复成功。
- USB CDC：COM4，0483:5740，serial `338536803133`。CH340 为 COM8，由用户串口助手单独记录，本工具不占用或写入 COM8。
- 用户报告 VM 开启；编码器供电电压此次没有实测核对。已告知保持车体和轮子静止、无人触碰。
- 本轮仅静置基线，无前进/后退预处理，也未发送 STOP；STATUS stop reason 为 NONE，不是历史事件中的 EXPLICIT。
- 未修改固件、CubeMX、引脚、机械参数、安全门控，未烧录、复位、校准或清故障。

## 工具及原始证据

新增主机工具 `Tools/static_encoder_monitor.py`，只发送 `CMD_GET_STATUS (0x04)`，每约 0.8 秒维持只读会话。串口会话开始时 DTR 打开，结束时随关闭端口结束会话；无速度命令或自动重连。

```powershell
python Tools\static_encoder_monitor.py --port COM4 --duration 120 --boot-log <实际启动日志路径> --stationary-confirmed
```

证据目录：`build/hardware_tests/static_encoder_20261012_061519_831293/`。

- `events.jsonl`：主机查询原始帧、接收原始字节、解析后每个协议帧及逐帧轮计数/速度、STATUS。
- `summary.json`：测试身份、条件、统计和异常结果。
- `boot_log.txt`：用户提供的完整启动日志。version/Build 是日志身份核对，不是 Flash 回读校验。

脚本 SHA256：`84c44097a52d603dc29f69659ba607b25b33154923cbef97dffcc9fe055a8743`。
启动日志 SHA256：`6a677565c210af7cd95da86facb12609b6677921ccf0a68e02ea7010e49fe9ca`。

## 实测结果

| 项目 | 结果 |
| --- | --- |
| 正式观察窗口 | 120 秒 |
| WHEEL 帧 | 2400，20 Hz，间隔最小/最大均 50 ms |
| STATUS 帧 | 1328（含查询触发的额外应答） |
| 四轮累计计数 | 全程 `[0,0,0,0]`，变化事件 0 |
| 四轮 cps | 全程 `[0,0,0,0]` |
| 四轮累计绝对计数变化 | `[0,0,0,0]`，没有观测到先增加后恢复的逐帧变化 |
| STATUS | 全程 IDLE、请求速度零、fault/controller fault 均零 |
| flags | 全程 0x0150 |
| CRC / RX overflow / TX drop | 0 / 0 / 0；主机解析 CRC 错误 0 |
| 结果 | `no_count_change_observed`，工具退出码 0 |

随后独立重解析 `rx_bytes` 原始字节，核对所有已记录查询帧均为 GET_STATUS，以及接收的 WHEEL/STATUS 条件，检查通过。统计正式观察窗口之外的初始帧时，总帧数可能与表格略有差异。

## 当前只读源码及离线核对

- 轮序 0/1/2/3=LF/LR/RF/RR；LF 定时器 TIM1，输入 PE9/PE11。四路 `.ioc`/生成代码滤波 CH1=CH2=6，GPIO_NOPULL。未实测波形，不能因 NOPULL 就认定配置错误。
- 编码器 CNT 差值处理 16/32 位回绕，应用前进极性后累加为 int64；仅初始化清累计计数，IDLE 不冻结或屏蔽小增量。
- 电机任务调用编码器更新后调用轮速估计；轮速快照包含同次估计的累计计数、采样时间及滤波 cps，USB WHEEL 从同份快照打包。四路 CNT 顺序读取，不是硬件同步锁存，不能宣称四路输入在完全同一瞬间采样。
- 正常 STOP 的控制器停止调用 MotorDriver_Disable：PD10/PD11 LOW、方向 LOW、四路 CCR=0。正常 STOP 不执行 HAL_TIM_PWM_Stop；TIM8 通道可继续运行但驱动已禁用。此为源码行为，未读实板 CCR/使能寄存器或测输出电平。
- 当前配置 13 PPR ×4 ×30=1560 counts/rev，是现有配置值，不是本轮重新测得的编码器规格；轮径 65 mm。ROS 有效轮距 0.395 m 与固件当前 0.210 m 是独立配置差异，本轮不更改。
- 六个相关生产源码文件哈希与冻结 0.13.2 源码清单一致：encoder_service、wheel_speed_estimator、motor_driver、chassis_runtime、chassis_controller、chassis_usb_protocol。
- 原生产代码提取到内存，以 native GCC `-std=c11 -Wall -Wextra -Werror` 编译离线程序，未改生产文件：72000 次模拟固定 CNT 更新（12 模拟分钟）无虚增；10 个回绕向量、方向归一化、64 位 WHEEL 极值打包检查通过。
- 模拟 LF CNT +4 可产生 -793→-789、短暂滤波 cps，200 ms 无新增计数后滤波速度清零且累计位置保留。它说明这种输入能产生类似输出，不能证明历史 CNT 确实发生变化或其电气原因。
- 原生产 MotorDriver_Disable 的 mocked HAL 测试验证 CCR/方向/使能清零及禁用后拒绝非零驱动；不代表真实电平、制动方式、定时器影子寄存器或完整 RTOS 调度已实测。

离线执行文件：`build/tests/static_encoder_audit_20261012.exe`、`build/tests/static_motor_disable_audit_20261012.exe`。测试输入从当前生产代码读入内存，硬件/RTOS 用模拟替代；没有生成新的 ARM 固件或完成新的 ARM 构建。

## 结论和下一步

本轮只验证当前静置条件下没有逐帧计数变化，不足以关闭历史异常，也不能排除 50 ms 反馈之间变化后恢复的输入瞬态。

同时间 COM8 日志已补齐，核对结果见下节；打印间隔约 200 ms，同样可能漏瞬态。
后续若用户另行同意，复现前进/后退后 STOP、静置至少两分钟，并同步保存 USB 和串口证据。
如需要判定真实微动/输入跳变/计数路径，仍需实际使能/PWM 状态及更高时间分辨率的诊断或波形；新增诊断固件、烧录及运动测试需另行确认。
不冻结 IDLE 编码器、不屏蔽小增量、不无条件清故障，不凭本轮未复现宣布根因或修复。

## 补充：COM8 同时间窗口对照完成

用户补充附件 `9192a902-db7f-43cb-b7ea-84ad86d52875/已粘贴的文本.txt` 已逐条解析，原文件复制为证据目录的 `ch340_aligned.log`。

- 共 5017 行，MCU 时间 1018401～1243801 ms，覆盖 225.4 秒，完整包含 USB 的 1059769～1179769 ms 窗口。
- USB 窗口内每轮 600 条编码器记录及 600 条速度记录。每轮编码器日志间隔最小/最大均 200 ms；左右两组日志本身有约 10 ms 打印偏移，并非硬件同步采样。
- 整份日志共有 4510 条单轮编码器记录、4508 条单轮速度记录：raw/d/total 全部为零、dir=STOP，dc/cps/rpm_x10/mmps 全部为零、stop=1。
- 结束后记录 `USB SNAP DTR-closed`：task=now=last=1179788、age=0；cfg=1、alive=1、dtr=0、open=0，随后 session ended，与主机采集结束关闭 CDC 会话相符，不是 SOF 链路误判。
- 本份附件没有 WARN/ERROR 或启动重置日志。完整同步窗口下，串口原始 CNT/累计值与 USB WHEEL 一致，未发现上报计数与原始读取不一致。

更早提交的另一份串口附件（e88e9f45…）从 1200810 ms 开始，在 USB 窗口结束后约 21.041 秒才开始，不能作为同步证据；此处没有用它替代当前对照文件。
结论仍是本轮静置基线未复现，并非历史停车后事件已修复。没有读取实际 CCR/使能寄存器、测量编码器供电/波形或执行前进后退预处理，不据此认定根因或实际电机输出。
