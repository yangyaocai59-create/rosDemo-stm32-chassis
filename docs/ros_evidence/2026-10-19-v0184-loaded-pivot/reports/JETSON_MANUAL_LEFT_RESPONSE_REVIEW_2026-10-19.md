# 新驱动与 UDP：短左转方向与响应复查（2026-10-19）

本轮**左转方向、松键停车、PC RViz 对齐通过；转速响应尚未通过**。用户确认正常并退出遥控。数据提示同一短左转指令下响应明显不足，按逐项排障计划暂停右转、重启运动及 Nav2 实车目标。不是底盘已锁定，也不将本轮判为 IMU/SLAM 故障。

## 配置与操作

沿用正式串口/ROS 输出隔离驱动、Fast DDS UDPv4、原保存地图定位。板上版本为用户交付的 v0.18.4；只读参考仓库 tag v0.18.4，提交 `32bc6d22d8eaac9ceb6bc820a06efe1c01099976`，工作区干净，未核验板上二进制哈希。未修改/烧录固件、重校准、清锁、放宽门控或改轮距。未启动 Nav2。

用户确认地面转向空间、线缆余量及现场停车条件后，只在 PC 桌面本地终端运行：

```bash
source /opt/ros/humble/setup.bash
unset FASTDDS_DEFAULT_PROFILES_FILE FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
/usr/bin/python3 ~/rosdemo_rviz/pc_keyboard_teleop.py --ros-args -p linear_speed_m_s:=0.04 -p angular_speed_rad_s:=0.20
```

只做 j 约 2 秒、松键停车、等待、Esc 退出。Jetson 的 Python 和独立 C++ 记录器只读订阅；准确命令、PID、日志起始偏移见 capture_session.json。前检 4 秒：IMU400、轮速80、激光40样本，轮速0、实际 /cmd_vel 发布者0，门控正常。

## 数值结果

| 指标 | 本轮 |
|---|---:|
| 非零指令 | vx=0、wz=+0.20 rad/s |
| 指令持续 | 2.107 s |
| 指令角速度积分 | +24.15° |
| IMU 角速度积分 | +4.58° |
| 轮速里程计角速度积分 | +5.27° |
| EKF 角速度积分 | +4.65° |
| IMU 峰值 wz | +0.0987 rad/s |
| 有效运动指令最大观察接收间隔 | 61.3 ms |
| 非零指令期间四轮平均反馈 LF/LR/RF/RR | −33.93 / −66.64 / +52.95 / +105.81 cps |

IMU、轮速、EKF 使用相同窗口：ROS 时间 1792408481.6254466 至 1792408484.9325294，即首条非零指令前0.2秒至停车指令后1秒；插值梯形积分，见 analyze.py/result.json。所有角度是传感器估算，**没有外部量角真值**；EKF 融合 IMU，不能当成独立角度真值。24.15°是请求值积分，未扣除斜坡、打滑和负载，不应要求短动作精确等于该值，但本次差距需解释。

左轮负、右轮正，四轮都有响应。ACTIVE 的 STATUS 报告 requested=0 mm/s、200 mrad/s；未观察到插入零指令、STATUS 超时、输出健康锁定、CRC/协议错误或 ERROR。停车后 IDLE、fault/controller_fault=NONE、requested=0、四轮反馈0。轮速反馈稳定接近零出现在零指令后约73 ms（含采样/滤波/传输，不是车体物理制动时间）；最终 /cmd_vel 发布者0。

独立 C++ 全段约85.5秒：IMU100.006 Hz、轮速20.000 Hz、odom49.945 Hz、激光10.009 Hz；最大接收间隔分别约22.7/60.8/41.2/110.0 ms。odom TF 最大接收间隔41.2 ms。本段没有支持“输入链路长时间断流”的证据；map TF 的重复头时间来自 SLAM 更新行为，不等于多个发布者。

## 只读源码对照与尚缺证据

- ROS `velocity_command.cpp` 直接将角速度乘1000编码，0.20发送200 mrad/s；`geometry.track_width_m=0.395` 用于轮速里程计，未乘入下发指令。
- 固件 `chassis_kinematics.c` 用 `CHASSIS_WHEEL_TRACK_M=0.210`，轮径0.065 m、1560 counts/rev。未计斜坡/限幅的名义稳态目标为左右±0.021 m/s，约±160.43 cps。**不是本轮已抓取的实际 MCU 目标值**。
- ROS 0.395 m 是保留的有效转向系数，不是实测物理轮距。按相同轮速和该系数，理想里程计角速度约0.106 rad/s；命令几何与有效转向几何的差异可以解释部分差异，不能解释为已确认的全部根因。不能直接把0.395替换为0.210/0.215，或反向改固件系数来掩盖响应。
- 固件参考角加速度1.50 rad/s²，达到0.20的名义斜坡约0.13秒；pivot阈值0.010 rad/s，本轮满足。动态启动补偿在轮速接近静止时生效，运动后撤除。实际控制状态没有记录，不能据此断言补偿/PI缺陷。
- 交付说明指出20 mm/s处于静摩擦区，本轮名义21 mm/s相近，负载摩擦及四轮控制不一致是待查假设。母线约11.11 V、ADC错误0，未做供电能力验收。
- 2026-10-18旧记录也存在0.20指令下转角小于请求积分的现象；短启动时长、地面及负载条件不同，不能只凭本次定性为新增固件回归。

缺少当前 MCU 的 applied/profile 指令、四轮 target/measured、原始编码器增量、FF/P/I、最终 PWM 和限幅/启动补偿状态。因此先提交[下位机排查说明](STM32_LOW_SPEED_LEFT_RESPONSE_HANDOFF_2026-10-19.md)，由负责固件的工程师分析是否需要修改；本机不直接改固件、自动执行会运动的 trace 脚本或打开调试 UART。

## 证据与继续点

原始证据目录：`/home/jetson/rosdemo_jetson_ws/logs/navigation/manual_left_regression_20261019_191333`。

保存 motion.json/jsonl、native_inputs.csv/json、冻结 driver.log、precheck/capture/result/analyze.py、ROS 配置/换算源码快照及固件只读参考哈希。另生成本地 ZIP 供后续用户选择交接，**没有上传或联系工程师**。检查点另保存转向后的停车位姿；地图保持不变，当前只运行底盘/EKF、雷达和定位，遥控与 Nav2 均未运行。

下一步先核对下位机目标与反馈追踪，解释低速响应后再安排一次必要的对照。两次前进启停的既有通过结论保留；本轮只增加方向和停车通过，不扩大为转速跟踪、转向导航或整套 Nav2 验收。

## 后续离线核对

详见[离线核对与调试口事件](JETSON_OFFLINE_TURN_RESPONSE_REVIEW_2026-10-19.md)：ROS换算及参考控制逻辑通过；打开CH340时发生PIN RESET，程序在发送任何UART字节前退出，旧转向缓存丢失。尚未取得实际内部target/PWM，不自动补做运动或修改固件。
