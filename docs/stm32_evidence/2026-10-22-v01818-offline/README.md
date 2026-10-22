# 0.18.18 离线验证与候选交付

2026-10-22，Asia/Singapore。用户同意合入 LF 反向首次起步原型并编译，交给用户烧录。本轮未开串口、运动、清故障、校准或烧录；没有新实车数据，不是稳定发布。

[候选完整说明](../../STM32_LF_FIRST_START_0.18.18_20261022.md)包含规则、风险、烧录步骤及所有产物 SHA-256。

- `v01818_motion_regression.log`：47 组 PASS，含 120010 次冻结 .17 隔离对照、首次起步边界和八类延长阶段安全出口。
- `v01818_usb_regression.log`：20 组 PASS，协议/会话代码未改。
- `v01818_debug_build.log`：95 步 clean 构建，零编译警告/错误，FLASH 177964/RAM 80728/CCMRAM 0 B。
- `package_v01818.py`：本次构建、回归、校验、打包入口；根目录布局运行，拒绝覆盖已有 .18 产物，不连接硬件。
- `source_manifest.json`：141 项文件身份；相对 .17 的四项已有文件变更、四项新测试；九项冻结基线、九项原型副本校验及生产代码与原型的比较结果。
- `verification.json`：内存/栈、HEX 校验/地址/向量/版本、HEX/BIN 对应、产物哈希。`flashed=false`、`hardware_tested=false` 是交付时状态，不能替代后续实车记录。
- `source_snapshot/`：三个生产文件、四个新测试和运动 runner 快照，不是完整独立编译工程。

实际产物在 `build/firmware/0.18.18/rosDemo-0.18.18.{hex,bin,elf,map}`。HEX SHA-256：`077c0fc81b6e03197146f614c8b51d33e89d0c7184cc7f8ced345d749dfb6442`。

旧 .17 四个归档产物未改。没有创建候选提交或推送 GitHub，不把旧基准提交当本次候选身份。
