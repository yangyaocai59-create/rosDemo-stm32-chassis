# 0.18.19 离线构建与候选交付

2026-10-22，Asia/Singapore。用户批准合入LR运行恢复v2及安全消费者并编译；没有串口访问、运动、清实板故障、校准或烧录。**软件候选，非稳定发布，未实车验收。**

[完整候选说明](../../STM32_LR_RUNNING_RECOVERY_0.18.19_20261022.md)包含控制/安全范围、限制、测试结果、栈审查、镜像哈希及用户烧录后顺序。

- `v01819_motion_regression.log`：当前62组控制PASS，原始冻结.18 runtime另外保持15组通过；只限定受改动LR两处旧补偿波形断言，其他轮/模式/停车断言保留。
- `lr_test_run.log`、`lr_fault_grid.json`：新策略、边界和204组故障时序/锁存/关断审计。规定反馈下全部760ms关断，无新增延迟，不能当真实电机响应。
- `v01819_usb_regression.log`：20组PASS。`v01819_imu_python_regression.log`：19测试通过；`v01819_protocol_self_test.log`：本地协议自测。
- **`v01819_imu_quality_regression.log`：原严格扩展IMU脚本失败日志**，11组已运行断言通过，后续禁校准配置因未改的`ImuService_AbsI32`未使用而编译失败。不能称整套严格IMU回归通过。
- `v01819_imu_*known*`日志：仅对这个既有未使用函数诊断放宽error等级的独立主机补充构建，各要求唯一警告就是该函数；polling 4组、probe 3组执行通过。不修改生产IMU或原测试入口，正常固件配置不出现该warning。
- `v01819_debug_build.log`：95步clean固件构建，无编译警告/错误；FLASH179888/RAM80752/CCMRAM0 B。
- `stack_review.json`、`stack_disassembly_excerpt.txt`：实际链接调用边及应用帧小计，不是完整栈界限或.19实测空闲；旧.18硬件stack_free明确标记历史对照。
- `source_manifest.json`：149项源码/配置/测试身份，相对.18八项已有文件变化（六生产、两测试入口/断言文件）及八项新测试；旧基线与原型哈希。构建前后未漂移。
- `baseline_0.18.18/`：18个冻结旧文件，供正式隔离对照；`source_snapshot/`：本次16项变更/新增文件快照，不是完整独立编译工程。
- `package_v01819.py`：原构建/回归/镜像校验入口，拒绝覆盖现有.19交付。只编译和测试，不能用作烧录工具。其严格IMU失败/补充警告被明确记录，而非吞掉报错。
- `verification.json`：软件候选、已知扩展测试限制、镜像/地址/向量/内存校验和产物哈希，明确`flashed=false`、`hardware_tested=false`。

产物：`build/firmware/0.18.19/rosDemo-0.18.19.{hex,bin,elf,map}`。HEX SHA256：`28f17192a690fafb1c95b725dac1831cee9480ff55eb8476132f245b8f81a2bd`。BIN基址`0x08000000`。

旧.18四个归档产物不变；未创建新提交或推送。用户全片擦除烧录后IMU偏置会丢失，先只读检查，再确认静止条件后校准；运动测试另行取得现场安全确认。
