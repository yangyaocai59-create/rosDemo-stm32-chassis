# ROS 2 RAW v2 静止测试证据（2026-10-17）

本目录保存 Jetson 对 STM32 固件 `0.16.0-imu-fractional-bias` 的只读 ROS 2
RAW v2 (`0x86`) 静止测试原始记录、分析和驱动日志。

固件参考：tag `v0.16.0`，commit `33bbef1d903e3c012018144116d01bd12061a79c`。

结果：源时间戳约 100 Hz，无源间隔超过 20 ms；600 秒累计 Z 轴偏航
`58.672°`，约 `5.867°/分钟`。轮子和里程计位置没有变化，期间没有非零
`/cmd_vel`。RAW v2 解码和 bias 单次换算通过，静止漂移未达到预设 `<=1°/分钟`
工程目标。温度补偿尚未实现。

文件说明：

- `static_10min.jsonl`：逐消息原始观测记录；
- `static_10min.json`：观测器汇总；
- `static_10min_analysis.json`：离线漂移和采样间隔分析；
- `driver.log`：ROS 驱动日志，含 mode 2 ACK、RAW v2 bias、状态和计数器；
- `manifest.json`：版本、配置、命令和证据清单；
- `static_10min.progress.json`：中途检查点；
- `status.txt`：观测器退出状态。

ROS 侧没有重复扣 bias、硬编码偏置或自动校准。该证据用于核对 STM32
校准温度、Flash 恢复值和实际 raw 残差，不代表温度补偿后的长期性能。
