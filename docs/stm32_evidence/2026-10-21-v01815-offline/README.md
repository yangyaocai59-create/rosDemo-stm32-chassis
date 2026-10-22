# 0.18.15 离线原型、回归与构建证据

无串口、无 GPIO、无烧录、无物理运动。候选尚未上板。完整设计/限制见[版本说明](../../STM32_FRONT_SUPPORT_HANDOFF_0.18.15_20261021.md)。

## 复现入口

在仓库根目录、PATH 中有本机 GCC 时：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File Tools/tests/run_motion_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File Tools/tests/run_usb_tx_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File docs/stm32_evidence/2026-10-21-v01815-offline/run_comparison.ps1
```

`compare_support.c` 对两版使用完全相同的规定反馈：160 cps 目标，先达标并建立正向 I=20‰，近零 160 ms，再以同方向 100 cps 运行 160 ms、继续至 2160 ms。没有物理电机模型，不把旧反馈当作新固件的预测反馈。

两份 `v01814_support_compare.log` / `v01815_support_compare.log` 分别链接 .14 冻结 C 和当前 .15 C。两前轮/双方向四组一致：

|规定反馈下有向 FF+I，‰|.14 RF 反向|.15 RF 反向|
|---|---:|---:|
|近零前，已有 I=20|144.5984|144.5984|
|近零 160 ms 后|174.5984|174.5984|
|恢复至 100 cps，累计 160 ms|124.8984|174.5984|
|继续至累计 2160 ms|154.8984|174.5984|

这证明旧交接掉力在指定序列上消除、上限没有新增；**不证明**真实地面轮速、超调、流畅性已改善。实际新增支撑维持时间是需要上板复核的风险。

## 原型过程及测试诚实记录

生产 .14 未动前，复制控制器到 `build/prototypes/v01815` 做隔离修改。通过专用交接、已有前轮策略/超速确认、真实 runtime 和 fault-manager 离线测试后，再合入生产。合入 C 与通过测试的原型 SHA256 同为 `98feec8b65334641622e037ac25530e0fb54c71d0fd84db5aeecb0af8cd154d1`。

初始原型在 1 ms 步长下，I 满额时存在约 0.00038‰ 临时项残留，已通过每次转交后按剩余额度夹紧解决；不是放宽最终临时项为零断言。初始测试以“目标+100 cps”定义所有目标的超速，高目标 1600 cps 时未越过 160 cps 误差带，已修正测试输入为目标加 max(100,20%目标)，未修改生产门槛来迁就测试。

旧测试中“I 必须为零”与新交接目的冲突，因此有意改为支撑守恒/额度/最终退场断言；堵转、安全关断、时序和输出边界没有删除。新增部分负载余额不足一个 4‰ 撤减步长时，负载与正向 I 共用同一个预算的精确断言。

## 文件

- `v01815_motion_regression.log`：最终 38 个 PASS 输出组（PowerShell Tee-Object 输出，UTF-16 LE）。
- `v01815_usb_regression.log`：20 个 PASS 输出组。
- `v01815_debug_build.log`：95 步干净 Debug 编译，零警告/错误。
- `package_v01815.py`：打包脚本，已有版本目录存在时拒绝覆盖；再次复现构建应另选新目录，不删除用户固件。
- `source_manifest.json`：132 个源码/配置/相关测试哈希，相对 .14 六个已有文件变更、新增一个测试。最终测试补断言后已同步哈希。
- `verification.json`：HEX/BIN 校验、内存、版本/向量与各烧录产物哈希。未烧录/未做硬件测试。

烧录产物在本地 `build/firmware/0.18.15/`，因 build 被 Git 忽略，本证据目录不包含 HEX/BIN/ELF/MAP，也没有自动上传发布。
