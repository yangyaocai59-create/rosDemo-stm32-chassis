# STM32 工程师：0.16.0 RAW v2 静止结果

日期：2026-10-17。Jetson 使用 STM32 CDC by-id 串口和 ROS `rosdemo_base`
驱动，默认请求 `CMD_SET_IMU_STREAM` mode `2`，实际收到 `0x86`。

固件版本：`0.16.0-imu-fractional-bias`，Git commit
`33bbef1d903e3c012018144116d01bd12061a79c`。

## 条件

- 小车落地静止，未操作键盘，未发送非零 `/cmd_vel`。
- 驱动参数 `imu.stream_mode=2`、`command.require_calibrated_imu=true`。
- 保存 bias：`[-52438, 19020, 146592]` milli-RAW。
- 观测 600.010 秒，温度结束时 diagnostics 为 28.874 °C。

## 结果

- 收到 60002 个 IMU 样本，源时间戳约 100.004 Hz。
- 源最大间隔 11.0 ms，无源时间戳回退，无超过 20 ms 的源间隔。
- RAW v2 format=2、gyro scale=655、flags=0x07、读错误和连续错误为零。
- 轮子位置、里程计位置均无变化，`/cmd_vel` 为零。
- ROS 按 `(gyro_raw - bias_milli_raw / 1000) / 65.5 * pi / 180` 换算后：
  - Z 轴累计约 `58.672°`；
  - 静止漂移约 **`5.867°/分钟`**；
  - 十个 1 分钟窗口为 `5.77–6.01°/分钟`。

例如日志中的 `gyro_z_raw=153` 与 `bias_z=146.592`，残差为
`6.408 RAW`，对应约 `5.87°/分钟`。这说明 ROS 没有重复扣除 bias；残差已经存在
于 STM32 RAW 与 milli-RAW baseline 的组合中。

## 请核对

1. 本次实际校准温度、校准样本均值与当前 raw 残差是否来自同一设备、同一轴映射。
2. 0.16.0 的 `gyro_bias_milli_raw` 是否与当前采样快照严格一致，尤其是 Flash
   恢复和运行时快照之间是否存在代际或单位问题。
3. 温度补偿前，当前结果不能按 0.375°/分钟作为 ROS 长期性能；请确认温度变化是否
   足以解释约 5.9°/分钟的差异。

证据目录：
`logs/acceptance/imu_raw_v2_static_20261017_1712/`，其中包含原始 JSONL、驱动日志、
分析 JSON 和运行 manifest。ROS 侧不再额外扣 bias、不硬编码修正、不自动重新校准。
