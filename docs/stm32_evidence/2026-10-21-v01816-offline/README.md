# 0.18.16 完整构建与离线验证归档

日期：2026-10-21（Asia/Singapore）。主说明：[LF支撑协调候选](../../STM32_LF_SUPPORT_COORDINATION_0.18.16_20261021.md)。这是完整链接后的候选构建，不再只是ARM对象；仍未烧录/未实车验证。

## 文件

- `v01816_motion_regression.log`：合入生产后42个PASS输出组。
- `v01816_usb_regression.log`：20个PASS输出组。
- `v01816_debug_build.log`：95步干净Debug构建，零警告/错误。
- `package_v01816.py`：本机打包/校验脚本副本，版本目录已存在时拒绝覆盖；依赖本机Windows STM32工具链路径。
- `verification.json`：HEX记录/地址/版本/向量/BIN显式数据校验、内存、静态栈及四份产物SHA256。
- `source_manifest.json`：135个用户源码/配置/相关测试哈希，加5个冻结.15对照依赖；相对.15的7个已有文件和3个新文件列表。

日志和JSON为UTF-8。烧录HEX/BIN/ELF/MAP位于本地`build/firmware/0.18.16/`；build被Git忽略，本目录不含二进制，也未自动上传Release。

打包脚本最初对`.su`静态栈字段的分隔符解析错误，尚未创建本版归档即停止。修正为实际TAB分隔符匹配后，重新跑全部回归和干净构建，再校验打包；没有因此改变控制器或放宽栈断言。最终日志/产物来自重跑通过的构建。

## 复现

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File Tools/tests/run_motion_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File Tools/tests/run_usb_tx_tests.ps1
```

`Tools/tests/motion/run_lf_isolation.py`对冻结.15的5个参考文件先校验哈希，再用前缀公开符号编译对照，与当前生产C运行80000步逐字段对照；无串口/硬件依赖。

新`test_lf_coordination.c`将当前生产C包含进诊断翻译单元以构造人工初态，测试共享预算、运动中恢复、幅值/变化率及有限退场；不是新增硬件私有项遥测。原安全测试直接链接真实runtime/chassis/fault-manager及硬件桩，保留锁存和关断断言。

结构性原型结果见[隔离原型](../2026-10-21-lf-coordination-prototype-v1/README.md)：规定旧反馈的比较不能预测新闭环运动效果。原型目录及其验证JSON是合入前快照，其脚本检查生产仍是.15；现在请用本版生产测试入口，不强行绕过旧脚本保护。

## 验证边界

没有连接CDC/CH340、没有速度或清故障命令、没有烧录/复位/校准。没有推送或新提交号。`flashed=false`、`hardware_tested=false`是本归档生成时事实。

保持原峰值和关断逻辑，不代表卸载/恢复动态相同；增力较缓可能影响起步，撤负载较慢可能延长超调。当前只完成离线和构建校验。待用户手动烧录、重新确认安全条件后做只读→架空→单次落地检查。
