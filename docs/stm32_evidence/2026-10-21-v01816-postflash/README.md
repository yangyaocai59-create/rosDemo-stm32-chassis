# 用户报告烧录 0.18.16 后只读检查

时间：2026-10-21 18:29:06（Asia/Singapore）。用户报告烧录完成，本阶段只发送GET_STATUS并被动读CH340；没有STOP、速度、清故障、调试字符、复位或校准命令。原始JSON/UART日志同目录。

- COM4：0483:5740，序列号338536803133；COM8：CH340 1A86:7523。打开CH340前DTR/RTS=False，CDC DTR=True/RTS=False。
- 约4秒采集STATUS 49帧、WHEEL 80帧、IMU 407帧。STATUS源时间13332～17298 ms，持续IDLE / NONE、故障0/0、flags=0x0150、请求/四轮速度零；主机CRC及固件CRC/overflow/TX drop均零。
- WHEEL全部四轮累计计数/速度为零；最后17282 ms仍静止。
- 最后电源遥测10703 mV。不能凭此独立识别VM开关状态、车轮是否架空，或电机端瞬态电压。
- UART motor_dt=20..20 ms、safety_dt=10..10 ms、overrun=0/0、log_drop=0，最后stack_free=1380/352。没有新版本行、复位原因或采样窗口内重启横幅。
- IMU READY、flags=0x05、bias无效，符合用户每次全片擦除烧录后的状态；本阶段不自动校准，不以此认定轮速修改引入IMU问题。

只读状态正常，不是运动/性能通过。本次没捕获启动版本行，版本身份依据为用户报告烧录了交付的.16文件，而非通过STATUS或Flash回读认证。后续任何运动都需用户重新确认架空/电源/线缆等安全条件，证据另行记录。
