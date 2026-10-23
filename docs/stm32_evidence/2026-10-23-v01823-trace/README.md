# 0.18.23 诊断候选证据

说明入口：[设计、字段、限制与下一步](../../STM32_TRACE_CAPTURE_0.18.23_20261023.md)。

- `verification.json`：测试、内存、应用栈帧、镜像SHA256以及未烧录/未运行声明。
- `source_manifest.json` / `source_snapshot/`：相对0.18.22的变更范围及本候选源码快照。
- `logs/clean_debug_build.log`：完整清理构建输出。
- `logs/trace_capture/`：真实记录器测试、runtime开启/关闭记录的输出差分、模拟导出及解析结果。
- `logs/unified_pivot_logs/`：统一PI、安全停车、驱动、普通模式回归。
- `logs/usb_tests.log`、协议与里程计自测：接口回归。

这些都是离线输入，不是实车带载数据。相对0.18.22没有调FF、PI或保护阈值；基于0.18.22的控制候选仍未实车验收。不得用本包证明顿挫根因已唯一确定、供电/机械已经排除或ROS转角已达标。

`package_candidate.py`用于测试、清理编译和冻结新版本，不访问硬件、不提交或推送Git；目标镜像目录存在时拒绝覆盖。
