# 0.18.14 离线验证归档

2026-10-21，Asia/Singapore。候选 `0.18.14-front-load-confirm`。未烧录，未打开串口、发速度或执行实板测试。

- `motion_regression.log`：35 条 PASS，真实轮控制器、driver/runtime、安全故障集成及新的连续超速确认边界。
- `usb_regression.log`：20 条 PASS，USB STOP/会话/真实断链/RAW v2/IMU 调度及日志隔离。
- `debug_build.log`：95 步完整 ARM Debug clean build，0 错误/0 警告，FLASH=176844 B、RAM=80720 B。
- `verification.json`：HEX checksum、显式 HEX/BIN 内容、向量/保留区、镜像哈希及离线状态。
- `source_manifest.json`：131 个用户源码/配置/相关测试文件，记录相对 0.18.13 的 7 个已存在文件变更及 1 个新增测试文件。生产差异仅控制器 C/H、控制配置、版本 H 四个文件。
- `package_v01814.py`：实际使用的测试/编译/校验脚本取证快照；原位置为 `build/hardware_tests/`，依赖该路径及本机 STM32 工具链，拒绝覆盖已存在版本目录，不是供用户烧录的程序。

这些 PASS 是离线逻辑与构建证据，不代表实车欠速/停顿已解决。构建期间没有修改旧 0.18.13 镜像；新的 HEX 在 `build/firmware/0.18.14/rosDemo-0.18.14.hex`。

详细规则、未改范围、风险及实板步骤见[版本说明](../../STM32_FRONT_LOAD_CONFIRM_0.18.14_20261021.md)。
