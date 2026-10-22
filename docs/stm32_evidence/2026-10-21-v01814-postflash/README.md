# 0.18.14 用户烧录后的只读检查

2026-10-21，Asia/Singapore。用户报告已烧录 0.18.14。使用 GET_STATUS-only CDC 探针及被动 CH340 捕获 4 秒；没有 STOP、速度、CLEAR_FAULT、调试口写入、复位或校准。CH340 打开前 DTR/RTS=false，CDC DTR=true/RTS=false。

- 最后 STATUS t=44371 ms，IDLE/NONE、runtime/controller fault=0/0，flags=0x0150，请求与四轮速度均零；accepted/rejected=0/0。
- 最后 WHEEL t=44361 ms，四轮累计计数/速度均零。
- VM 遥测 10.714 V，ADC 无错误；通信 CRC/RX overflow/TX drop=0，主机 CRC 错误=0。
- 控制周期报告 motor_dt=20..20、safety_dt=10..10，overrun=0/0、log_drop=0，stack_free=1280/352（沿用固件日志单位）。
- IMU READY、flags=0x05，valid=1/bias=0，无读错误，符合本次全片擦除后未显式校准的状态；未自动校准，轮控检查不以此判故障。

未捕获启动版本或 reset cause 行，本次没有 Flash readback；候选身份依据用户烧录报告及已交付 artifact，不能声称此只读状态检查独立认证了二进制版本。

原始文件 `build/hardware_tests/v01814_postflash_readonly_20261021_141421.json/.log`，本目录保存副本及实际探针脚本。只读结果正常仅允许继续安排经确认的架空检查，不能替代新候选的实车性能验证。
