# 用户报告烧录 0.18.15 后只读检查

时间：2026-10-21 14:53:27（Asia/Singapore）。原始 JSON/CH340 文本同目录。

本阶段仅打开 CDC/CH340、发送 GET_STATUS；未发 STOP、VELOCITY、CLEAR_FAULT、调试字符、复位或 IMU 校准。CH340 打开前 DTR/RTS 均为 False；CDC DTR=True、RTS=False。

- COM4：0483:5740，序列号 338536803133；COM8：CH340 1A86:7523。
- 约 4 秒采集 STATUS 47 帧、WHEEL 80 帧、IMU 405 帧。
- STATUS 源时间 21403～25348 ms，全程 IDLE / NONE、故障 0/0、flags=0x0150；速度请求及四轮速度为零，accepted/rejected=0/0，CRC/overflow/TX drop=0。
- WHEEL 最后 25342 ms，四轮计数/速度均零；UART 编码器/轮速日志也保持静止。
- 最后供电遥测 10758 mV。该值不独立证明 VM 开关状态/电机端电压，也不能识别车辆是落地还是架空。
- UART motor_dt=20..20 ms、safety_dt=10..10 ms、overrun=0/0、log_drop=0、stack_free=1380/352。
- IMU READY、flags=0x05、bias 无效；符合用户全片擦除烧录条件，不在本阶段自动校准。
- 主机解析 CRC=0、未出现串口异常。采样窗口内源时间单调，无重启横幅。

本次**没有捕获到启动版本行或复位原因**，不能通过 STATUS 认证固件版本；身份依据为用户报告烧录了已提供的 .15 文件，而非 Flash 回读。本阶段只读状态正常，不等于运动/落地流畅性通过。随后用户另行确认四轮架空安全条件，才允许后续运动测试，证据分开归档。
