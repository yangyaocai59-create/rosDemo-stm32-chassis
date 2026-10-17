# ROS 2 RAW v2 接入记录

日期：2026-10-17  
固件参考：`rosDemo-stm32-chassis` tag `v0.16.0`，commit
`33bbef1d903e3c012018144116d01bd12061a79c`。

## 已完成

- 协议层新增 `CMD_SET_IMU_STREAM (0x06)` 和 `IMU_RAW_V2 (0x86)`。
- 驱动启动后请求 stream mode `2`；收到成功 ACK 后才释放本次会话的运动门控。
- RAW v2 严格要求 52 字节、format `2`、reserved `0`、非零灵敏度和有效 CRC。
- 三轴 bias 按 signed milli-RAW 解码，ROS 只换算一次：

  ```text
  (gyro_raw - bias_milli_raw / 1000.0) / 65.5 * pi / 180
  ```

- `/imu/data_raw` 使用 RAW v2 的原始角速度和精确偏置发布；没有再次扣除偏置，也没有写入或修改固件偏置。
- diagnostics 显示 stream、format、milli-RAW bias、采样计数和健康 flags。
- 轮速 odometry 和 EKF 配置未改变，仍使用 `/wheel/odometry` 与 `/imu/data_raw`。

## 离线验证

- STM32 仓库自带 `Tools/tests/test_imu_raw_v2.py`：6 项通过。
- ROS `rosdemo_base` 新增 RAW v2、selector、会话和 SI 换算测试：
  - `test_protocol_v1`：19 项通过；
  - `test_read_only_session`：9 项通过；
  - `test_ros_messages`：11 项通过。

## Jetson 只读实机验证

使用实际 STM32 CDC 串口启动驱动约 20 秒，未发送非零速度：

- mode `2` ACK 成功；
- `/imu/data_raw` 由 RAW v2 发布，约 100 Hz；
- `gyro_scale_x10=655`，format `2`；
- 实际收到并解码 bias：`[-52438, 19020, 146592]` milli-RAW；
- `flags=0x07`、状态 `READY`、轮速计数全为零；
- CRC、版本、长度、未知消息和 MCU CRC/overflow/TX drop 均为零；
- 运动门控保持安全，测试期间没有非零速度命令。

## 尚未验收

### 10 分钟静止测试结果

证据目录：`logs/acceptance/imu_raw_v2_static_20261017_1712/`。

- 观测 600.010 秒，收到 60002 个 RAW v2 IMU 样本。
- MCU 源时间戳约 100.004 Hz，最大源间隔 11.0 ms，无源时间戳回退或超过 20 ms 的间隔。
- 轮子位置变化为零，里程计位置变化为零，`/cmd_vel` 样本为零。
- ROS 接收端偶发约 0.368 秒调度间隔，但源时间戳连续，需与 STM32 采样连续性分开记录。
- Z 轴累计角度约 `58.672°`，折算静止偏航约 **`5.867°/分钟`**；十个 1 分钟窗口约 `5.77–6.01°/分钟`。
- 结论：RAW v2 解码、精确偏置换算和采样链路通过；本次 10 分钟静止漂移未通过预设的 `≤1°/分钟` 工程目标。

因此暂不开始运动一致性、建图或重载定位验收。当前结果应先交给 STM32 工程师分析温度、校准样本与实际 raw 残差的关系；ROS 侧不再额外扣 bias、不硬编码修正值，也不自动重新校准。
