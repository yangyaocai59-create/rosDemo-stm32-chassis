# 0.18.20 离线构建与候选交付

2026-10-22，Asia/Singapore。用户批准集成已验证 RR 原型并完整编译。**软件候选，尚未烧录/实车验证，不是稳定发布。** 本轮没有串口访问、运动、复位、清实板故障或校准，没有提交或推送。

[完整说明与烧录顺序](../../STM32_RR_RUNNING_RECOVERY_0.18.20_20261022.md)。

- `v01820_motion_regression.log`：79 组正式控制 PASS。保留 .18/.19 原始完整 runtime 各 15 组；新 RR 行为单独测试，只限定两处旧立即 restart 波形断言。
- `lr_test_run.log`、`rr_test_run.log`、两份 `*_fault_grid.json`：LR/RR 各 204 组实际控制/故障联动模拟审计；新候选均 760 ms 关断，无新增延迟，非实车响应。
- `v01820_usb_regression.log`：20 组；`v01820_imu_python_regression.log`：19 测试；`v01820_protocol_self_test.log`：本地协议自测。
- **`v01820_imu_quality_regression.log` 是原严格扩展测试失败日志**：11 组已运行断言通过，关闭校准的配置因未改的 ImuService_AbsI32 未使用而编译失败，不能称整套严格 IMU 测试通过。
- `v01820_imu_*known*`：两个独立补充主机构建只放宽这个既有 unused-function 诊断，验证唯一警告，polling 4 组/probe 3 组执行通过。没有修改原 IMU 源码或严格测试入口。
- `v01820_debug_build.log`：95 步 clean 完整固件构建，零警告/错误；FLASH181732/RAM80776/CCMRAM0 B。
- `stack_review.json`、`stack_disassembly_excerpt.txt`：链接调用边和应用帧小计，不是完整栈界限。历史 .19 空闲值明确标记，不是 .20 实测。
- `source_manifest.json`：158 项源码/配置/测试，相对 .19 六生产及五已有测试/入口变化，九新测试，原型五文件 token 一致。包含冻结基线和原型哈希。
- `baseline_0.18.19/`：20 个原始旧文件，均与 .19 清单一致；`source_snapshot/`：20 项变更/新增文件，不是独立可编译工程。
- `package_v01820.py`：回归、构建、镜像校验及冻结入口，拒绝覆盖现有 .20 目录，不能用于烧录。显式保留严格 IMU 失败和补充已知警告。
- `verify_delivery.py`：核对清单/快照/旧产物/历史原型，独立从归档 ELF 重建 HEX/BIN 并对比。生成 `delivery_integrity.json`。
- `verification.json`：软件候选结果及已知限制、镜像哈希/向量/地址/内存，flashed=false、hardware_tested=false。

产物：`build/firmware/0.18.20/rosDemo-0.18.20.{hex,bin,elf,map}`。
HEX SHA256：`688610c1e502c1cef8276305643a5fb3671ff2bf5ffaed470cdb845774cea6cf`；BIN 基址 `0x08000000`。

原 .19 四个产物不变。前置 RR 原型是历史阶段记录，其“生产 .19 未改”保护使其原始 runner 不适用于已集成的当前工作树；请运行正式 `Tools/tests/run_motion_tests.ps1` 或交付校验，不重跑原型来覆盖历史证据。用户全片擦除后需重新处理 IMU 校准；运动测试仍需单独确认现场安全条件。
