# 0.19.1 交接入口：校准恢复、左转重复及右转

2026-10-23，Asia/Singapore。当前固件能连续执行本次 ±0.40 rad/s 指令至少 15 秒，编码器结果接近指令积分且左转重复性良好；轮式转角与 IMU 参考仍有约 9–10% 差异，尚不能宣布导航角度精度验收通过。本轮未改生产代码、未烧录固件、未运行 ROS 建图导航。

## 三次地面结果

角度含停车后的收尾。指令积分使用 MCU 事件 START 至 STOP 请求的源时间。编码器统一使用 D=65 mm、1560 counts/rev、有效轮距 395 mm。IMU 仅为参考，没有外部真值；用户尚未设定允许误差范围。

| 测试 | MCU 连续时长 | 指令角 | 编码器角 | IMU 参考角 | 编码器减参考 / 参考幅值 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 首次左转，校准前 | 15.188 s | +348.08° | +349.63° | +351.54° | −0.54% |
| 重启后左转重复 | 15.200 s | +348.36° | +350.60° | +389.67° | −10.03% |
| 重启后右转 | 15.190 s | −348.13° | −348.98° | −383.44° | +8.99% |

首次证据见[首次左转](../2026-10-23-v0191-ground-left-400mrad-15s/README.md)。两次左转编码器角相差 0.9684°，约 0.277%；IMU 参考角相差 38.1339°。用户明确反馈地面、载荷、轮胎基本没有变化，不能把差异直接归因于换地面。仍需区分轮胎滑移/有效轮距变化、机械受力、参考测量等因素；本轮证据无法独立确定原因。

连续执行不等于每一瞬间真实车身角速度均不低于 0.40 rad/s。按含收尾角度除以 ACTIVE 时长，复测左/右编码器平均幅值约为 0.4026/0.4010 rad/s，IMU 参考约为 0.4474/0.4406 rad/s。

## IMU 校准与断电恢复

只通过现有 USB CALIBRATE_IMU 接口发送一次校准，无速度命令、无清故障、无自动重试。全程 IDLE，四轮计数不变。

- 校准前静止 Z 均值 37.052 mrad/s；校准后 500 个样本均值 0.078 mrad/s、标准差 0.8294 mrad/s。
- UART 确认 PASS_BIAS_APPLIED，一次 Flash 保存：slot=A、seq=1、地址 0x080C0000，精确偏置 milli_raw=[−49598,11398,138426]，28.28°C。
- 用户确认关闭 VM、完全断电后重上电；重启后 uptime 和编码器总数复位。只读检查 IMU READY、flags=0x07、bias=1，静止 Z 约 1 mrad/s，未再次校准。启动瞬间日志未捕获，因此不声称逐字读取了开机 Flash restore 日志或对比了恢复瞬间的精确偏置。
- 原有静止 RAM 偏置跟踪仍会微调偏置；未改 IMU、存储或滤波实现。不要再次全片擦除；若全片擦除，保存的校准会丢失。

## 连续控制、计数与停车

复测左转接受 242 条、右转接受 243 条速度命令，均 0 拒绝、无中途 STOP/START、无故障清除、无重试。STOP 均 ACK，最终 IDLE/EXPLICIT、fault=0/0、请求和四轮反馈为零，STBY=0、四轮 CCR=0，串口均关闭。

左转净计数 LF/LR/RF/RR 为 −9169/−9243/+9309/+9209；右转为 +9160/+9212/−9144/−9243。两次 TRACE2 均为正常 STOP 触发，128 帧、mask=0。末尾约 2.52 秒 ACTIVE trace 中：四轮目标幅值 603 cps，无 dc=0、无请求/实际 PWM 不一致、无积分打满，原始定时器 CNT 与累计计数差一致，bad=0、flags=0。trace 只覆盖末尾，不能据此声明全程所有 20 ms 样本都通过同一检查。

主机及设备 CRC/RX overflow/TX drop 均为零，IMU 积分窗口无源样本缺口；motor/safety 周期 20/10 ms，overrun/log_drop=0。VM 左转 10.637–11.088 V、右转 10.659–11.077 V。

独立 IMU 参考检查使用短前/后静止基线：左转约 389.652/389.982°，右转约 −383.392/−383.353°，不能用基线差解释约 35–39° 的轮式/IMU差异。右转采集在积分结束后 11 秒发生一次相同 IMU sample ID 的 Z 值 2→1 mrad/s 更新，完整记录在 yaw_exploratory.json；不在积分或前基线窗口内。离线分析现在明确记录窗口外更新，窗口内不同 payload 仍拒绝计算；原始文件保留。此修改仅涉及主机分析工具。

本轮没有重新拔线或注入故障，不能声称本轮实测覆盖了断链/所有故障停车。现有安全逻辑及先前离线回归未改。

## 版本与上位机交接

- 固件版本仍为 0.19.1-effective-track-candidate，固件源码提交 68b4597；未因测试重新编译或改版本。
- [固件包](../../../releases/0.19.1/README.md)包含原 HEX/BIN/ELF/MAP、源文件清单及离线验证记录；HEX SHA256：df84a5522ff1a2d8935600031230d1a6d151343ff52a8896ba11b54c96e381df。
- 246 项构建清单检查中，245 项字节一致，仅主机离线分析工具 Tools/assess_existing_turn_yaw.py 已按上述原因更新；所有非 Tools 项一致。delivery_verification.json 给出细节。
- 四轮独立 FF+PI，左右侧差速目标；0.40 rad/s 分配约 ±603.516 cps。原始编码器协议不变，计数没有乘有效轮距系数。
- ROS 部署源码本轮未访问。上位机需核实方向、D65 mm、1560 counts/rev、有效轮距 0.395 m 及唯一一处角速度换算，不能重复处理计数或再乘旧补偿系数。
- 先在 ROS 侧录制低速手动运行的指令、原始轮计数、轮式 odom 与 IMU，并用外部角度基准确认差异。当前可作为底盘联调候选，不能跳过上位机核对直接宣布建图/导航通过。不按一次差异盲目再拟合有效轮距，也不改 IMU 去凑轮式结果。

## 证据和复算

calibration/、reboot_readonly/、left_repeat/、right/ 分别保存完整 USB/UART 原始数据与 JSON。采集脚本在本目录，默认不执行运动；运动脚本需要显式 --execute。package_v0191_hardware_delivery.py 是当次打包来源，包含本机路径和固定采集名称，供追溯使用。capture_manifest.json 给出目录文件 SHA256。

离线复算（不访问串口）：

```powershell
python -B Tools/assess_pivot_run.py docs/stm32_evidence/2026-10-23-v0191-calibration-repeat-right/left_repeat --effective-track-m 0.395
python -B Tools/analyze_trace23_capture.py docs/stm32_evidence/2026-10-23-v0191-calibration-repeat-right/left_repeat
python -B Tools/analyze_encoder_trace.py docs/stm32_evidence/2026-10-23-v0191-calibration-repeat-right/left_repeat/ch340.log
```

切换目录名为 right 可复算右转。原始采集路径保留在 JSON 中，哈希核对请在重新生成分析结果之前执行。
