# Jetson 静止 TF 与控制频率复查：2026-10-19

## 验收结论

**第 1 项（本轮有限负载下的静止 TF/控制计算复查）通过。新驱动及 UDP 传输的实车启停、转向和重启运动验收尚未执行。**

正式保留 Fast DDS UDPv4 传输配置；ROS RMW 仍为 rmw_fastrtps_cpp，话题 QoS 和同步发布默认值不变。原有地图、定位模式、匹配精度、TF 等待、传感器时间戳、EKF 时间偏移和全部停车/校准门控保留；固件未改、未烧录、未校准。没有创建连接实际 /cmd_vel 的预览发布者，没有新增物理运动。

用户确认车体未移动、可保持静止，并确认 PC RViz 地图、激光、TF 对齐且无红色报错。Jetson 重启后无线省电已由用户重新关闭；该设置仍是临时的。

## 诊断事实与边界

证据目录：`/home/jetson/rosdemo_jetson_ws/logs/navigation/tf_static_review_20261019`。

1. 独立 C++ 订阅器记录 DDS 源/接收时间、消息头和本机回调时间；Python 检查器的回调间隔会受到它自身图查询/调度阻塞影响，不能直接当作发布端的停顿。
2. 原共享内存/UDP 基线还记录了 32 条扫描源年龄警告，见 comparison.json，该基线不计为通过。原模式捕获 RPP 的 `map <- odom` TF 等待约 71 ms；native CSV 显示当时 odom TF 持续更新，map TF 头时间短暂落后。不能将所有 TF 错误都归为同一个坐标边问题。
3. 扫描匹配通常约 140–200 ms；定位栅格重建在基线窗口中最大约 410 ms，持有与匹配共享的 mapper 锁。静态 map_server + 关闭内部地图保存订阅确实消除了周期重建，但仍出现两次控制频率警告，且观测到约 224/270 ms 的 odom/map DDS 接收延迟。因此该重构未作为完整修复采用。
4. `minimum_time_interval=0.5` 不能单独解释 TF 更新时间：Humble 2.6.10 在间隔判断前就更新 scan_header。曾提出改为 0.2，核对源码后撤回，未执行。0.02 m 匹配网格候选也未执行。正式配置未为消除警告而降低匹配精度。
5. 仅移除共享内存传输、改用 UDPv4 的两轮候选与正式复查均未再现控制频率警告或 TF 外推错误。该结果支持本机保留此传输方案，**不足以证明 Fast DDS 内部某个锁或共享内存实现存在已定位的缺陷**，也不保证任意负载下硬实时。

官方源码/参数核对：[SLAM Toolbox 2.6.10](https://github.com/SteveMacenski/slam_toolbox/blob/2.6.10/src/slam_toolbox_common.cpp)、[Nav2 1.1.20](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_collision_monitor/src/scan.cpp)、[Fast DDS 2.6 传输描述](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/xml_configuration/transports.html)。源码副本与临时计时工具保留在日志目录，未安装到正式包。

重启中断的 match_session_02 日志存在零字节尾部，审计保存在 reboot_log_audit.json；有效片段仅作诊断，不当作完整验收。最初 Python 消息信息回调与临时库链接/未落盘构建失败亦不算有效试验。

## 实际操作与复查

本机编译：

```bash
source /opt/ros/humble/setup.bash
cd ~/rosdemo_jetson_ws
colcon build --base-paths src --packages-select rosdemo_navigation --symlink-install
```

隔离启动：`ros2 launch rosdemo_navigation navigation.launch.py enable_motion:=false`。实际命令、两次参数查询、取消结果见 `run_formal_trial.py`、`formal_udp_01/preview_timing.py` 与 `formal_udp_01/`。

| 对照 | 控制频率警告 | TF future 错误 | 查询情况 |
|---|---:|---:|---|
| UDP 候选第 1 轮 | 0 | 0 | 一次完整返回，另一次超时无完整输出 |
| UDP 候选第 2 轮 | 0 | 0 | 两次均返回 0.5 |
| 正式启动路径复查 | 0 | 0 | 两次均返回 0.5 |

正式路径 native 采样约 50 秒；动作正常取消（CANCELED），实际速度发布者 0、轮速起止 0、底盘 STATUS/输出健康锁定均为 0、激光过滤丢帧 0。

| 正式复查指标 | 实测 |
|---|---:|
| /odom 接收频率 | 49.86 Hz |
| IMU 接收频率 | 100.00 Hz |
| 激光接收频率 | 10.01 Hz |
| 控制器 DDS 源发布最大间隔 | 72.15 ms |
| 最终隔离守卫输出最大源发布间隔 | 74.36 ms |
| odom TF 最大接收间隔 | 69.61 ms |

Twist 没有 header；指令行的“源发布间隔”使用 DDS source_timestamp 计算，不能据此推定底盘制动距离。ROS/DDS 端到端采样与动作计算均为静止预览，不能替代运动验收。formal_runtime_audit.json 核对了正常安装二进制及对应 XML，未加载 LD_PRELOAD、延迟注入或异步发布环境覆盖。

## 正式变更与恢复

- `src/rosdemo_navigation/config/fastdds_udp.xml`：仅 UDPv4，不配置异步发布、非阻塞丢包、网卡白名单或话题 QoS 覆盖；PC 无需改 RMW。
- `navigation.launch.py`：默认对子进程使用安装的 XML；`dds_profile` 参数可明确选择另一个经验证配置。
- `tools/resume_saved_localization.py`：在初始化 ROS 前读取检查点记录的 XML；无该字段的旧检查点保留旧行为。配置与源码须通过 SHA256 校验，不自动绕过。
- 原 SLAM Toolbox 定位与地图发布方式保留，没有采用临时 map_server、匹配分辨率或频率候选。

检查点：`/home/jetson/rosdemo_checkpoints/20261019_185237`，关键文件 36 个，含驱动源码/安装二进制、启动脚本和 DDS XML；地图、源码、软件版本与证据均保留。只手动恢复定位，不设置开机自启、不自动发导航目标。

下一项：先由用户确认场地/线缆/现场停车条件，再做新驱动与传输组合的一次低速前进、停车、再次启动；通过后依次转向、重启恢复和 Nav2 短目标。没有标记转弯导航、绕障、雷达断流实车停车、长路线或自动探索通过。
