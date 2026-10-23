# 0.19.0：四轮控制结构简化候选

日期：2026-10-23。当前文件夹最新 `.25` 为基线，本地提交 `f62c66d`、标签 `baseline-v0.18.25-before-simplification`；完整字节备份见 `build/backups/v0.18.25-before-simplification.zip`。本版是结构整理和诊断候选，未烧录、未连接串口、未做新实车试验。

## 实际改动

- `wheel_speed_controller.c`：移除旧前轮自适应补偿、LF首次起步、LR/RR专用恢复与退场、补力向积分交接及 legacy 宏路径。四个独立实例复用现有统一转向FF+PI。保留NORMAL算法及诊断用的通用调用接口。
- `chassis_controller.c`：移除底盘层补力状态、RR专用计数观察。底盘只分配目标、调用单轮控制、提交输出并保持停车代次保护；每轮更新与Stop/Reset串行化。
- `fault_manager.c`：移除旧owner/前轮恢复失败查询。转向由现有PivotFeedbackGuard单独计时，NORMAL沿用其原判据。PI只报告模式，不拥有故障计时器。
- 事件诊断：16项独立RAM环保留START、显式STOP请求、断链请求、健康停车请求、零速命令和命令超时；保存源时间、事件序号、运动代次、命令序号及事件当时状态。超容量明确报告overwritten。STOP条目记录的是请求到达时刻，START在成功启用后记录，不能把请求前状态解释成停车完成状态。
- 停车后导出TRACE2时，先输出独立`RUNTIME_EVENTS`/`RUNTIME_EVENT`行，TRACE2 schema3未改。日志可识别20ms采样之间的STOP→重新启动。请求类型1=START、2=STOP、3=transport、4=health、5=zero、6=timeout。自动控制器故障及ESTOP仍由原故障日志/停车trace记录，本环不是所有ISR事件的硬件时间戳记录。
- 当前运动测试入口改用统一算法、事件、停车/故障、trace、编码器与差分对照。旧轮子专用波形测试在Git基线与历史证据中保留，不将旧96组声称为新算法覆盖。

## 控制行为与已知问题

本轮不重新猜测带载参数。所有活跃FF、P/I增益、积分±50‰、输出±300‰、斜率1000‰/s、20ms轮速采样、机械几何、USB单位/顺序保持基线。混合NORMAL/PIVOT实际C差分80,000步完全相同，含模式切换、正反、Reset/Prime与dt1..100ms；另有NORMAL对封存.21的40,000步对照。

转向保护仍为基线.25的1000ms宽限、250ms预警、2000ms持续严重欠速、100ms恢复确认；这里没有批准其物理/电气安全额定值，也没有放宽时限。NORMAL保持原1000ms宽限和750ms判定。STOP/断链/故障仍直接关断，不等待PI斜坡。结构简化不宣称解决物理顿挫。

原始编码器、估计器、定时器、电机驱动与IMU、Flash持久化实现均逐文件与本次精确ZIP基线核对。IMU未改。

当前仍需解决：固定前馈及有限积分在架空与带载之间不适配；已有原始CNT平台还不能区分真实近停与输入波形问题。不能据“PWM未打满”直接提升300‰上限，或用放宽保护代替轮速修正。

ROS f554242原始UART另包含79150ms显式STOP、79161ms重新起步，applied200→15、积分清零。该STOP来源需ROS侧核对发包/门控；本仓库没有部署中的ROS驱动，未伪造ROS侧修复。

## 验证与产物

- `powershell -NoProfile -ExecutionPolicy Bypass -File Tools/tests/run_motion_tests.ps1`：统一PI及真实runtime/fault-manager（strict、O2、fanalyzer）、8种停车、调度交错、事件11ms间隔/回绕/容量、40k NORMAL与80k混合模式差分、trace启用与关闭输出一致、导出损坏拒绝、保护边界与编码器采集诊断通过。
- USB现有20组回归通过；里程计数学自测通过。
- 完整ARM Debug clean构建及HEX/BIN校验、内存使用、受保护文件哈希与源码清单见 `build/firmware/0.19.0/verification.json`。扩展IMU原生严格配置的历史unused-function问题本轮没有重测，不声称全部IMU配置通过。
- 固件目录：`build/firmware/0.19.0/`。候选仅供后续受控实板验证；旧.21至.25产物不覆盖。

建议先离线审核本版，再做最小实板核对。参数调优作为下一独立改动，必须同时检查架空撤力余量与带载驱动力，不能只追求更少故障。没有运行车辆、清故障、烧录或推送GitHub。
