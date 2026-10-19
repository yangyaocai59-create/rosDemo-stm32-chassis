# 新驱动与 UDP：两次手动启停回归（2026-10-19）

**本轮两次低速前进、松键停车及停车后再次启动通过。** 用户确认两次正常、无明显顿挫、PC RViz 对齐、遥控退出。未启动 Nav2 或发送自主目标；未修改/烧录固件、校准、清锁或放宽门控。本轮不代表速度精度、制动距离、转向、重启运动、长路线或自动探索通过。

沿用[静止复查配置](JETSON_TF_STATIC_REVIEW_2026-10-19.md)：正式底盘/EKF、雷达和 SLAM Toolbox 保存地图定位，Fast DDS UDPv4。无临时计时或注入，仅增加只读记录器。

## 操作与证据

用户确认地面、前方至少 0.5 米、线缆余量和现场停车条件。PC 桌面本地终端：

```bash
source /opt/ros/humble/setup.bash
unset FASTDDS_DEFAULT_PROFILES_FILE FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
/usr/bin/python3 ~/rosdemo_rviz/pc_keyboard_teleop.py --ros-args -p linear_speed_m_s:=0.04 -p angular_speed_rad_s:=0.20
```

两次 i 约 2 秒，松键停车，中间等待，最后 Esc 退出。Jetson 使用 localization_observer.py 和独立 native 订阅器只读采样，完成反馈后正常收尾；准确命令、PID、基线日志偏移见 capture_session.json。

启动前 6 秒：激光 60、IMU 601、轮速 120 样本，扫描最大年龄 0.106 秒、停车区点数 0、轮速 0、实际 /cmd_vel 发布者 0，底盘门控 READY。

| 指标 | 第一次 | 第二次 |
|---|---:|---:|
| 非零前进指令持续 | 2.133 s | 2.336 s |
| 指令 vx / wz | 0.04 m/s / 0 | 0.04 m/s / 0 |
| 轮速里程计估算前进 | 9.19 cm | 10.36 cm |
| 轮速反馈峰值 vx | 0.0558 m/s | 0.0599 m/s |
| 零指令后轮速反馈稳定归零 | 0.233 s | 0.214 s |

每段四轮都有正向速度响应。ACTIVE 报告 requested=40 mm/s、0 mrad/s，停车后 IDLE、fault/controller_fault=NONE。运动段指令观察器最大接收间隔约 98/65 ms；该值不是 PC 源发布间隔。

总路径轮速估算 19.55 cm，未用卷尺复核。0.056–0.060 m/s 的短时反馈峰值说明本轮没有完成恒速跟踪/启动过冲验收。归零时间包含采样、滤波和传输影响，不是实测车体制动时间/距离；既有 200 ms 指令失联门控不等于物理制动耗时。

native 记录约 97.5 秒：轮速 20.00 Hz、/odom 49.95 Hz、IMU 100.00 Hz、激光 10.01 Hz；IMU 消息头最大间隔 10 ms、odom TF 最大接收间隔 40.60 ms。未据此扩大为硬件 RAW sample_count 全量验收。

底盘日志无 ERROR、STATUS 超时、输出健康锁定或 CRC/协议错误。IMU READY、bias_valid=true、calibration_result_valid=false、calibration_active=false，恢复偏置正常，未重校准。电源诊断级别 1 内容为仅监测母线、不提供 SOC，母线约 11.12 V、ADC 错误 0；未做电量或低压门限验收。

结束时 /cmd_vel 发布者 0，最后指令及轮速均为 0，/odom 唯一发布者为 EKF。停车后独立 TF GID 检查约 10 秒：odom→base_footprint 只有 EKF 一个实际来源，map→odom 只有 SLAM Toolbox 一个实际来源；SLAM 的另一个 /tf endpoint 未发变换，不依据 endpoint 数量判重复 TF。

证据：`/home/jetson/rosdemo_jetson_ws/logs/navigation/manual_restart_regression_20261019_185827`。result.json、analyze.py、motion.json/jsonl、native_inputs.csv/json、driver.log、tf_sources_after_stop.txt 均保留。

## 检查点与下一步

新停车检查点 `/home/jetson/rosdemo_checkpoints/20261019_190533` 保存当前地图估计位姿（非独立真值）、原地图副本、36 个关键文件校验、源码和日志。旧 `/home/jetson/rosdemo_checkpoints/20261019_185237` 位姿在本次运动前记录，不能直接用于当前停车点。当前只保留底盘/EKF、雷达和定位，未设置开机自启。

下一项先确认转向空间与线缆余量，做转向回归；再单独验证 ROS 节点重启后的启停，通过后才做 Nav2 真实短目标。
