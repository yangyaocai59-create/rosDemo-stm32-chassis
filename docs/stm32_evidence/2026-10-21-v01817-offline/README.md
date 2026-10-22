# 0.18.17 离线验证与交付证据

2026-10-21，Asia/Singapore。用户批准RF限定修改、回归、编译；本轮未开串口、未运动、未校准、未烧录。实车效果尚未验证。

[候选说明](../../STM32_RF_SUPPORT_COORDINATION_0.18.17_20261021.md)列出范围、行为、限制、哈希及后续验证安排。

- `v01817_motion_regression.log`：43组PASS，包括8万次与冻结.16的隔离对照、RF小积分移动欠速的旧/新差异、限幅与安全路径。
- `v01817_usb_regression.log`：20组PASS。
- `v01817_debug_build.log`：95步clean构建，无warning/error，FLASH177284/RAM80720 B。
- `package_v01817.py`：本次打包脚本快照；需在项目布局中运行，拒绝覆盖已有候选目录，不开串口/不烧录。不是归档目录直接运行的入口。
- `source_manifest.json`：137项项目文件哈希、冻结.16测试基线5项哈希、相对.16变更/新增清单。未嵌入完整vendor库/工具链。
- `verification.json`：HEX/BIN逐字节显式数据、校验和/地址范围/向量/版本/保留区、产物SHA256、静态栈。`flashed=false`/`hardware_tested=false`是打包时真实状态。
- `source_snapshot/`：本轮3个生产文件和3个关键测试快照；不是完整可单独编译工程。完整工作区依赖见清单。

实际产物`build/firmware/0.18.17/rosDemo-0.18.17.{hex,bin,elf,map}`不在此目录。HEX SHA256：`234dc0c49fb80fc32c787783b59e1ea2be724cc100f21fc563d7fce7d558a42f`。

没有新提交号或仓库推送；基准提交不能代替该候选文件的哈希身份。没有生成或伪造任何.17实车数据。
