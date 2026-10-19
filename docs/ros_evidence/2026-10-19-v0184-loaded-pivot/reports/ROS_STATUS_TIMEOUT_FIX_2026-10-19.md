# ROS 主机 STATUS 超时：线程隔离修复（2026-10-19）

## 结论与范围

本轮实测复现并定位了一次主机侧 STATUS 超时：串口工作线程在发布 IMU 消息时等待约 274 ms，超过原有 STATUS 接收检查的剩余时间预算，随后锁定运动。已将遥测发布和周期报告移到独立输出线程，保留原接收时刻、FIFO 顺序、协议、坐标、偏置换算和既有安全门控。

所有本轮硬件诊断均静止进行，实际 `/cmd_vel` 发布者为 0，没有发送真实导航目标。STM32 固件没有修改、烧录或重新校准。之前的实车短目标发生在本次驱动修改之前，不能代替新驱动的运动回归。

## 实际复现证据

证据根目录：`/home/jetson/rosdemo_jetson_ws/logs/navigation/status_timeout_20261019/`。

使用独立计时程序，分别记录墙钟时间、线程 CPU 时间、主动/被动上下文切换。正式源码与二进制的原始 SHA256 保存在 `accepted_files.json`，修复前副本在 `before_fix/`；计时工具本身经单独 100 ms 休眠自检。原同步模式 16 次查询及隔离规划对照未复现 300 ms 事件，但记录到 IMU 发布约 24.8 ms 等待，CPU 约 0.12 ms。

随后只给诊断驱动设置 `RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS`，仍复现：

| 项目 | 观测值 |
|---|---:|
| `publish_raw_v2` 墙钟耗时 | 273.817 ms |
| 同调用线程 CPU 时间 | 0.056 ms |
| 主动 / 被动上下文切换 | 1 / 0 |
| 遥测批次总耗时 | 273.875 ms |
| I/O 循环总耗时 | 285.161 ms |
| 触发锁定时距上次处理 STATUS | 337.593 ms |

日志：`async/checkpoint/resume_20261019_144434/base_ekf.log`，索引：`root_cause_evidence.json`。现有 300 ms STATUS 锁定逻辑按设计触发；没有向锁定会话发送运动目标。软件停止后串口释放，没有自动循环重启解锁。

证据支持本次复现的线程阻塞机制，不足以将所有历史 STATUS 超时归为同一原因，也没有定位等待发生在 DDS 内部的哪一个锁。[Humble RMW 官方说明](https://github.com/ros2/rmw_fastrtps/blob/humble/README.md#change-publication-mode)说明了两种发布模式；本机异步模式仍观察到调用等待，因此该环境变量未作为正式修复保留。

## 代码修改

- `base_driver_node.cpp` 的串口线程继续独占串口读取、解码、ACK/事务处理、STATUS 时限、速度门控、STOP 和 keepalive。
- `OutputWorkQueue` 使用单生产者/单消费者环形队列，固定容量 128 个作业；消费者执行可能等待的 ROS 发布、周期日志及诊断。执行作业期间不持有串口生产者的互斥锁。
- 事件携带原 `reception_time_ns`，时间映射、轮速积分、JointState 和 RAW v2 偏置换算仍在顺序输出路径中按原逻辑执行；没有再扣一次 bias。
- 周期报告捕获只读会话快照，由同一输出线程执行；RAW 记录写入与报告中的 flush 因而仍串行。
- 队列满、输出作业抛出异常，或输出作业年龄达到现有 STATUS 时限时，新增输出健康保护锁定本 USB 会话；不会无限缓存或自动恢复运动。新的输出保护先通过原速度门控写 STOP，再打印错误。
- 正常退出和传输异常时，先尝试串口 STOP、关闭串口，再等待当前输出调用结束；输出线程退出后才销毁映射状态或停用 ROS 发布器。

原 `command.status_timeout_ms=300`、`command.timeout_ms=200`、发送周期、校准要求、ACK 拒绝锁定及运动范围参数未改。输出健康检查复用 300 ms 界限，属于额外保护，不代替 STATUS 检查。队列容量和消费者约 1 ms 空闲轮询不构成操作系统硬实时保证。

## 软件验证

本机编译：

```bash
source /opt/ros/humble/setup.bash
cd ~/rosdemo_jetson_ws
colcon build --base-paths src --packages-select rosdemo_base --symlink-install --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select rosdemo_base --event-handlers console_direct+
colcon test-result --test-result-base build/rosdemo_base --verbose
```

最终包测试 19 个测试组通过；colcon 结果记录 257 个条目、0 错误、0 失败、28 跳过（包含工具生成的检查条目）。新增 6 个队列测试覆盖发布堵塞时生产者可继续提交、顺序及恰好一次执行、队列满后锁定、异常锁定、300 ms 边界、退出等待与丢弃未执行旧输出、参数合法性。既有协议、串口、速度门控、故障恢复、IMU 校准、轮速里程计及键盘测试均通过。新增队列测试另外在 AddressSanitizer/UndefinedBehaviorSanitizer 下通过。

日志：`final_build.log`、`final_tests.log`、`final_test_results.log`、`sanitized_tests.log`。同包原有少量格式与许可证头标注检查问题一并修正，没有改变键盘行为或许可证类型。诊断备份中的 CMake 曾被 colcon 误识别为包，已用 `logs/COLCON_IGNORE` 排除日志目录，并显式从 src 编译。

## 静止实板：受控延迟与保护

延迟注入只存在于 `fixed_probe/` 的独立测试可执行文件，通过进程环境选择，不在正式源码或安装可执行文件中。

| 测试 | 输出调用耗时 | STATUS 最大接收间隔 | 结果 |
|---|---:|---:|---|
| 模拟已观察到的等待，注入 275 ms | 275.187 ms | 107.825 ms | 无 STATUS 锁定、无输出故障，门控正常 READY |
| 超界保护，注入 350 ms | 超过 300 ms | 100.439 ms | 输出健康保护 reason=3 锁定，STOP 在错误日志前发出；没有 STATUS 超时 |

275 ms 对照表明，输出等待不再拖住 STATUS 接收。350 ms 用例是预期失败：恢复工具发现错误后退出并正常停止，不自动清锁。它验证输出路径严重失效时仍会锁定，并不表示软件出现了非预期故障。相应结果：`fixed_probe/delay_275/result.json`、`fixed_probe/delay_350/result.json`。

## 验收边界与下一步

修复后软件检查及上述静止实板检查通过。恢复正式安装驱动后继续执行同类静止查询和隔离 Nav2 计算负载，结果需单独追加。正式运行中不保留测试环境变量、计时可执行文件或异步发布覆盖。

正式驱动最终复查：`final_static/query_stress_result.json` 的约 35 秒查询负载中，没有 STATUS/输出锁定、实际速度发布者 0、轮速起止 0；四次 2 秒发现查询中两次成功、两次返回未发现节点，因此不能声称四次服务调用均成功。随后隔离规划使用两次 5 秒直接发现查询，均返回 0.5；目标接受后取消（CANCELED），轮速不变、实际速度发布者 0，底盘日志没有 STATUS/输出错误。

该隔离规划日志另有控制频率警告 1 次、TF future extrapolation 1 次、激光过滤丢帧 0 次，见 `final_static/nav_trial/`。这不影响“本次主机 STATUS 阻塞机制已修复并完成静止检查”的结论，但仍是导航链路的独立待办，必须继续排查，不据此扩大运动验收。

先处理上述压力负载下的 Nav2 TF/控制频率异常，再在现场条件确认后单独做新驱动低速前进/停车、再次启动及转向一致性回归。没有追加真实运动，不把本次静止结果升级为长路线、自动探索、真实绕障或长期稳定性通过。地图和定位参数保持现有已保存版本。

## 修复后停车检查点

新检查点 `/home/jetson/rosdemo_checkpoints/20261019_152045`；原地图、当前停车地图估计位姿、35 个关键文件校验、源码/日志备份及安装驱动二进制副本均保留。实际速度发布者 0、轮速 0。当前只保留正式底盘/EKF、雷达和定位，没有计时/注入/异步发布覆盖。新驱动运动和冷启动验收仍待进行。
