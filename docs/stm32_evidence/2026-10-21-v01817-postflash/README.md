# 用户报告烧录0.18.17后的只读检查

2026-10-21 20:11:49（Asia/Singapore）。用户报告手动烧录完成。随后仅发GET_STATUS并被动读取CH340；没有STOP/速度/清故障/校准/调试写入/烧录命令。

原采集文件`build/hardware_tests/v01817_postflash_readonly_20261021_201149.{json,log}`，完整副本在本目录。COM4 CDC、COM8 CH340；打开CH340前DTR/RTS均设false，CDC DTR=true、RTS=false。

- 46个STATUS，源时间108290～112256 ms，全为IDLE、fault0/0、flags0x0150、请求及四轮速度全零。
- 80个WHEEL，计数和速度全零。
- 410个IMU样本；末次READY、flags0x05，尚无偏置校准标志，符合全片擦除清除偏置后的状态。没有自动校准，不能以此认为IMU漂移已重新验证。
- 末次供电遥测10813 mV，ADC错误0；只读期间没有测量负载电流或快速电压瞬态。
- 主机CRC=0、采集错误列表为空，末次固件CRC/RX overflow/TX drop为0。
- 4行控制时序：motor20..20 ms、safety10..10 ms、overrun0/0、log_drop0，末次stack_free1272/352。
- 没有新版本/复位横幅，未回读Flash。固件身份依据用户报告烧录交付.17，不宣称独立识别已通过。
- 本地137项文件仍与.17交付清单一致，无哈希不符。

结论：只读状态正常，可以在用户另行确认安全后进入架空检查；没有从遥测推断车辆是否架空，也没有在此阶段自行运动。
