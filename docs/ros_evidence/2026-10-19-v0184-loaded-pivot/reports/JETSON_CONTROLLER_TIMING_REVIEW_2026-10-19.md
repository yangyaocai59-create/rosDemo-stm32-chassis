# 2026-10-19 开机恢复与控制循环静止诊断

## 范围与当前结论

本轮恢复昨天保存的地图并排查 Nav2 控制频率警告。先执行隔离静止预览，随后在用户明确确认场地后只发送一次真实低速短目标；未校准、修改或烧录 STM32。

原位开机恢复通过，用户确认 PC RViz 地图、激光和 TF 正常对齐。控制循环的主要耗时已定位到 RPP 路径变换中的 TF 等待。定位模式 `transform_timeout=0.5`、Wi-Fi 省电关闭时的最后两组静止预览以及单次实车短目标无控制频率警告或 TF 外推错误。用户确认运动平稳、自动停车、PC 对齐，据此正式保留定位参数 0.5 秒。

后续 STATUS 排查已复现主机 IMU 发布阻塞约 274 ms 导致串口线程延迟，并完成串口/输出线程隔离修复及静止保护验证，详见 [驱动修复记录](ROS_STATUS_TIMEOUT_FIX_2026-10-19.md)。本文的实车短目标在该驱动修改之前完成，新驱动的运动回归尚待安排，不能据此认定长期稳定性或转弯/长路线通过。

## 开机恢复

- 用户确认关机后车体位置、朝向未变。
- `bash ~/rosdemo_jetson_ws/tools/resume_saved_localization.sh --check`：地图与 24 个关键文件校验通过。
- `bash ~/rosdemo_jetson_ws/tools/resume_saved_localization.sh --same-pose`：底盘门控 READY，RAW v2 样本递增，无 STATUS 超时。
- 原启动日志：`/home/jetson/rosdemo_checkpoints/20261018_190100/resume_20261019_135002/`。
- 10 秒只读复查：`/odom` 50.05 Hz，`/wheel/odometry` 20.00 Hz，IMU 100.03 Hz，雷达 10.01 Hz；各话题均唯一发布者，实际 `/cmd_vel` 发布者 0，轮速最大值 0。
- 观察到 `map → odom` 和 `odom → base_footprint`；本次 Python 话题检查没有实现逐 TF 边的发布端归属分析，不据此单独声称所有 TF 发布端唯一。进程由恢复脚本统一管理，未重复启动底盘或定位进程。
- 使用地图仍为 `/home/jetson/rosdemo_maps/fresh_manual_20261018_172344`，未重建或删除地图。

## 测量方法与证据

证据目录：`/home/jetson/rosdemo_jetson_ws/logs/navigation/resume_20261019/`。

每次通过 `navigation.launch.py enable_motion:=false` 启动隔离预览，发送当前位姿前方约 0.4 米的计算目标，运行约 18 秒后取消；动作返回 CANCELED。此处的目标不连接底盘速度话题。记录 `/navigation/cmd_vel_raw` 接收时间、TF、扫描、轮速和动作状态。

`profiler/` 内的临时动态库只给预览控制器计时，调用原方法且不修改返回值，记录超过 3 ms 的方法耗时。首次尝试因 pluginlib 的局部动态链接作用域，无法通过 `RTLD_NEXT` 解析 RPP，导致隔离控制器退出；这轮无效，不作为验收。修正为从已加载插件句柄查找原方法后重测，证据在 `profiled_retry/`。首次失败日志保留于 `profiled/`。该动态库未安装到 ROS 包，也没有写入正式启动文件。

源码核对使用与安装版本一致的官方源码，副本位于证据目录：

- [Nav2 1.1.20 controller_server.cpp](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_controller/src/controller_server.cpp)
- [Nav2 1.1.20 RPP](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp)
- [SLAM Toolbox 2.6.10](https://github.com/SteveMacenski/slam_toolbox/blob/2.6.10/src/slam_toolbox_common.cpp)

SLAM Toolbox 本版本将 `map → odom` TF 时间设为扫描时间加 `transform_timeout`。它表示该变换的前向有效窗口，也用于内部 TF 等待；扩大窗口不会让扫描匹配结果变新。传感器原始时间戳和 EKF 时间偏移保持原值。

## 对照结果

| 测试 | 地图 TF 参数 | 控制频率警告 | 最大原始指令接收间隔 | TF 外推错误 |
|---|---:|---:|---:|---:|
| 原配置 baseline | 0.2 s | 24 | 0.388 s | 0 |
| 原配置加计时 profile_retry | 0.2 s | 16 | 0.311 s | 0 |
| 临时候选第一轮 tf_validity_05 | 0.5 s | 1 | 0.322 s | 0 |
| 临时候选加计时 tf05_profiled | 0.5 s | 0 | 0.059 s | 0 |
| 候选加参数查询 tf05_graph_stress | 0.5 s | 1 | 0.340 s | 1 |
| 域 67、本机通信、同样参数查询 tf05_local_only | 0.5 s | 1 | 0.218 s | 0 |
| 域 0、关闭 Wi-Fi 省电 tf05_wifi_off | 0.5 s | 0 | 0.152 s | 0 |
| 域 0、关闭省电、直接发现查询 tf05_wifi_off_direct_query | 0.5 s | 0 | 0.104 s | 0 |

计时原配置时，`canTransform(target=map, source=odom)` 最长 255.968 ms，RPP 路径变换 256.447 ms，整个控制计算 256.846 ms，说明该轮主要在等待 TF。原始 TF 历史显示地图变换的时间最差落后里程计约 0.351 秒。

域 0 参数查询对照中的外推错误来自碰撞监控的带时间补偿坐标变换，不能算作 RPP 的外推错误。该轮 RPP 路径变换也有约 291 ms 的耗时，但未记录到相应的简单 `canTransform` 等待；其具体来源未定。域 67 对照又记录到约 193 ms 的地图 TF 等待。因此不能将残余问题全部归因于 Wi-Fi，也不能因为一次无警告就宣布长期实时性通过。

表中间隔是本机观测者的接收间隔，不是硬件控制周期或严格的 DDS 源发送间隔。计时日志用于独立验证控制器内部耗时。各轮负载、进程重启和扫描匹配存在差异，这些是短时对照，不能用警告数量计算严格的性能提升比例。

详细统计：`comparison.json`；原始数据：各子目录 `preview_timing.jsonl`；版本：`versions.txt`。所有有效预览动作均接受后成功取消，未驱动车轮。

## Wi-Fi 与配置保留

本次开机后实际查询 Jetson Wi-Fi 为 `Power save: on`；此前的临时关闭设置没有跨重启保留。非交互 sudo 返回需要密码，用户在 Jetson 执行并确认：

```bash
sudo iw dev wlP1p1s0 set power_save off
iw dev wlP1p1s0 get power_save
```

结果 `Power save: off`，原始查询另存 `wifi_power_save_before.txt` / `wifi_power_save_after.txt`。这一步恢复昨天的联调条件，不足以证明无线网络是全部问题来源。本轮未修改 NetworkManager 的永久网络配置。

静止对照及实车测试通过证据目录中的 `localization_candidate.launch.py` 覆盖运行参数 0.5 秒，验证期间正式配置保持 0.2，关键源码哈希与昨天一致。用户确认实车结果后，正式将 `src/rosdemo_mapping/config/localization.yaml` 改为 0.5 秒并本机编译，将更新恢复检查点；昨天备份保留作为回退依据。控制器 0.35 秒、costmap 0.5 秒、速度上限、速度超时、碰撞区域、雷达新鲜度守卫和底盘安全门控未修改。

临时会话由 `resume_candidate.py` 管理。不要与昨天的恢复脚本同时启动，以免重复底盘。若需关机，先停止当前会话并更新实际位姿记录；地图与正式检查点保留。

## 后续验收边界

关闭省电后返回域 0，重复两轮短时静止预览，均无控制频率警告、TF 外推错误和激光过滤丢帧；动作均成功取消，实际速度发布者 0、轮速为 0。第一轮命令行查询返回 `Node not found`，只能说明查询/发现未成功，不能声称该轮服务调用负载通过；第二轮增加 `--no-daemon --spin-time 2` 直接发现，成功返回 `Double value is: 0.5`。第二轮没有超过 3 ms 的内部计时记录。

两轮最大原始指令接收间隔分别约 0.152、0.104 秒。没有 DDS 源发送时间观测，因此不能把这些接收间隔解释成相同长度的控制计算停顿，也不能据“未超过 0.20 秒”声称长期安全性或实时性验收通过。短时结果支持下一步一次有现场观察的低速直线回归；不支持认定所有延迟都由 Wi-Fi 省电引起，或任意负载下不会再现。

静止预览结束后曾保留会话 `candidate_checkpoint/resume_20261019_141828/`，因后续发现其中 STATUS 超时，实车前已经停止并另建新会话。所有预览动作的轮速和编码器位移起止均为 0。其后的单次真实目标及停车结果单列如下，不能用预览结果替代实车转弯、绕障、雷达断流停车或长期性能验收。

## 单次实车短目标与独立待办

用户明确确认地面、前方至少 60 cm 空间、线缆余量、遥控退出、PC 对齐并可现场停车后，开始实车前检查。第一段发现扫描最大年龄 0.413 s，未发送目标；保留 `real_short_goal/precheck_initial_failure.json`。等待 3 秒发现阶段后连续 6 秒观测最大扫描年龄 0.113 s、轮速 0、停车区障碍点 0，复查通过。这不能证明第一段延迟的具体来源，只能证明后续连续检查通过。

完整底盘日志另发现此前参数查询压力对照时（14:20:35）发生 STATUS 超时，USB 会话已锁定，即使后续 STATUS 显示 IDLE/NONE 也不会自动恢复权限。没有向锁定会话发送目标。保存故障证据后正常停止、仅重启一次 ROS 底盘/雷达/定位会话，不复位或重校准固件、不放宽超时。新会话 `real_regression_checkpoint/resume_20261019_142627/` 门控 READY、RAW v2 更新，启动及本次运动未再发现 STATUS 超时。

实车 Nav2 未加载计时动态库。发送前再次连续 6 秒检查：扫描 60、IMU 600、轮速 120 样本，扫描最大年龄 0.111 s、停车区点数 0、轮速 0；实际 `/cmd_vel` 唯一发布者为 `scan_cmd_vel_guard`。测试观察器额外监视扫描源时间年龄超过 0.35 s 时取消目标；正式守卫原有 0.35 s 检查是**接收间隔**，本次没有把它改成源时间年龄检查，也不能声称二者等价。

| 指标 | 本次结果 |
|---|---:|
| 目标 | 当前车头前方 0.4 m、同向，仅 1 次 |
| NavigateToPose | SUCCEEDED |
| 轮速估算本段位移 | 0.31478 m |
| 最终地图位置距目标 | 0.09666 m |
| 位置容差 | 0.10 m |
| 峰值指令 vx / 绝对 wz | 0.08 m/s / 0.07261 rad/s |
| 最终轮速、最终速度指令 | 0 |
| 最终速度话题最大接收间隔 | 0.05666 s |
| 最大扫描接收间隔 / 源时间年龄 | 0.11340 s / 0.11728 s |
| 控制频率警告 / TF 外推 / 激光过滤丢帧 | 0 / 0 / 0 |

用户确认“全部正常，运动平稳、自动停车且显示对齐”。只标记为本配置的**单次低速直线到达与停车通过**，不据此宣称任意负载实时性或长期运行通过。位移为轮速估算、目标误差为地图估计，均非新增卷尺实测。

证据：`real_short_goal/result.json`、`motion.jsonl`、`navigation_real.log`、`pre_motion_check.json`；旧会话故障位置见 `session_inhibit_before_goal.json`。实车 Nav2 已正常退出，不自动发送后续目标。

正式修改仅 `src/rosdemo_mapping/config/localization.yaml` 的 `transform_timeout: 0.2 → 0.5`（定位模式）。建图模式 mapper.yaml 仍 0.2，未据此宣称该模式相同问题已解决。EKF 时间偏移 0、控制器等待 0.35、costmap 0.5、所有底盘安全门控与速度/停车参数不变。

**独立跟进结果**：后续独立计时程序捕获 `publish_raw_v2` 墙钟等待 273.817 ms（线程 CPU 0.056 ms）、STATUS 间隔 337.593 ms，证实一次主机输出阻塞拖延串口接收的事件。已完成 ROS 串口/输出线程隔离；275 ms 静止模拟的 STATUS 最大间隔约 108 ms，350 ms 模拟则触发新增输出健康保护并写 STOP。STATUS 300 ms 等既有门控未放宽，固件未改。完整证据与验收范围见驱动修复记录。这里的短目标结果仍仅代表当时驱动版本，不能升级为新驱动运动、持续运行、转弯、绕障或自动探索验收。

## 正式配置与停车检查点

本机编译 `colcon build --packages-select rosdemo_mapping --symlink-install` 成功。新检查点 `/home/jetson/rosdemo_checkpoints/20261019_143119`，保存当前停车地图位姿与 24 个关键文件 SHA256、同一地图副本、源码及导航证据备份；active_checkpoint_path.txt 指向新目录。采集时实际速度发布者 0、轮速 0。没有新增物理运动或新地图。正式恢复路径的 0.5 秒冷启动回归尚未单独验收，不沿用昨天 0.2 秒检查点的 warm_resume_test 字段。
