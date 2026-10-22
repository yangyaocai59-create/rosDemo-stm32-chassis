# 0.18.13 用户烧录后的只读检查

日期：2026-10-21（Asia/Singapore）。用户报告已完成烧录。本次仅发送 `GET_STATUS` 并被动读取 CH340，没有发送 STOP、速度、故障清除、调试字节、复位、校准或刷写命令。

- USB CDC：COM4，VID/PID=0483:5740，序列号 338536803133。
- CH340：COM8，VID/PID=1A86:7523。打开前 DTR=False/RTS=False；CDC DTR=True/RTS=False。
- 4 秒采样末帧 STATUS t=70470 ms：IDLE、fault=0/0、requested/wheel speed=0、flags=0x0150、accepted/rejected=0/0、CRC/RX overflow/TX drop=0；主机 CRC=0。
- WHEEL t=70442 ms：累计计数 `[0,0,1,0]`、四轮 cps=0。RF 一个累计计数不等于当前轮子运动；本次未主动清零编码器。
- VM 供电遥测 10725 mV、ADC=975 mV；属于已有监测换算，未用万用表复测。
- CTRL TIMING：motor_dt=20..20 ms、safety_dt=10..10 ms、overrun=0/0、log_drop=0、stack_free=1272/352。仅是此时的运行高水位，不代替带载/故障链路栈验证。
- IMU READY、valid=1、flags=0x05、bias=0、读错误=0/0，符合用户全片擦除后未重新校准的状态；本次没有自动校准。

没有捕获新的 `Firmware version` 或 `Reset cause` 行，因此固件身份依据用户烧录报告和交付文件，不声称已独立认证板上版本；未为读版本主动复位。

本次可确认供电/停机/通信状态正常，不能确认物理架空条件，也未验证 0.18.13 新恢复失败故障的硬件效果。已向用户询问架空固定情况，确认后才进行运动。

本地原始记录：`build/hardware_tests/v01813_postflash_readonly_20261021_131944.json/.log`；本目录保留同名副本与只读采集脚本。候选制作时的 `verification.json` 和离线日志保留为当时快照，后续运动验证应另加证据，不把只读检查当作恢复失败保护验收。
