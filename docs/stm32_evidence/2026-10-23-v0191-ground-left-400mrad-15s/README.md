# 0.19.1 首次地面左转：0.40 rad/s，至少 15 秒

2026-10-23，Asia/Singapore。本记录仅描述首次测试。后续 IMU 校准、断电恢复及左右转复测见[最新交接入口](../2026-10-23-v0191-calibration-repeat-right/README.md)。后续出现约 9–10% 的轮式转角与 IMU 参考差异，因此不能把本次约 0.54% 的吻合当成稳定精度。

| 指标 | 首次左转 |
| --- | ---: |
| 主机发令时长 | 15.203 s |
| MCU START 至 STOP 请求 | 15.188 s |
| 指令积分（按 MCU 时长） | 348.0833° |
| 编码器积分 | 349.6329° |
| IMU 扣静止基线后的参考积分 | 351.5385° |
| 编码器减 IMU 参考 | −1.9055°，约 −0.5421% |
| 编码器减指令积分 | +1.5496° |

编码器使用轮径 65 mm、1560 counts/rev、有效轮距 395 mm，LF/LR/RF/RR 净计数为 −9143/−9217/+9290/+9178。原始计数未缩放或补数。角度包含停车后的收尾，不代表每一瞬间都达到 0.40 rad/s，也没有外部车身角度真值。

MCU 事件 START=62342 ms，STOP 请求=77530 ms。241 条速度命令接受，0 拒绝；没有中途 STOP/START。相同时刻的两个 STOP 事件分别记录 ACTIVE 和 IDLE，不能解读为两个主机发令周期。

STOP ACK、IDLE/EXPLICIT、fault=0/0、请求速度和四轮反馈为零均已确认。TRACE 最后 STBY=0、四轮 CCR=0；采集结束仍停止，两个串口关闭。主机/设备 CRC、RX overflow、TX drop 均为零。VM 为 10.406–10.780 V。

TRACE2 schema3 共 128 帧，reason=2、mask=0。末尾 127 个 ACTIVE 样本约覆盖 2.52 秒，并非全程每 20 ms 的 trace。四轮目标幅值显示 603 cps，末尾方向归一化平均反馈 LF/LR/RF/RR 为 580.70/620.20/614.28/597.03 cps，无 dc=0、无请求/实际 PWM 差异、无积分到限。编码器配置 bad=0、快照 flags=0，原始 CNT 差与累计计数一致。motor/safety 周期为 20/10 ms，overrun/log_drop=0。

本次 IMU 保存偏置无效，使用主机静止基线：301 个样本，Z 均值 36.44186 mrad/s，标准差 0.91178 mrad/s。独立短基线积分为 351.46345°，改用停车后基线约 350.90788°；这是参考不确定性检查，不是重新标定。后续已通过现有接口恢复校准并确认断电恢复。

capture/ 保存原始 USB、逐帧 payload、UART 原文、summary、trace 和离线分析；readonly/ 保存运动前只读检查。comparison.json 保存汇总，capture_manifest.json 保存文件哈希。目录内脚本为当次采集/分析入口；最新通用离线角度分析入口为 Tools/assess_pivot_run.py。
