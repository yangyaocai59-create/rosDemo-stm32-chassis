# 0.18.13 离线验证记录

日期：2026-10-21（Asia/Singapore）。对应 [0.18.13 前轮恢复失败保护候选](../../STM32_FRONT_RECOVERY_FAILSAFE_0.18.13_20261021.md)。本目录记录主机回归、真实 ARM 编译与候选镜像校验；尚无本版烧录或实车证据。

- `v01813_motion_regression.log`：真实生产 C 控制器/故障管理/运行状态机回归，29 条 PASS 输出；外设和反馈为测试桩，不模拟物理负载。
- `v01813_usb_regression.log`：20 条 PASS 输出，含原 STOP/断链、队列、RAW v2、时间戳及日志隔离回归。
- `v01813_front_rest_audit.log`：原诊断程序对照；五个前轮持续零反馈案例都在 1500 ms 首次锁存故障。故障后独立控制器数值不等于电机在故障后继续驱动；完整停车由运动集成测试验证。
- `v01813_debug_build.log`：95 步 ARM Debug clean build，0 errors/warnings；FLASH=176784 B、RAM=80704 B。
- `verification.json`：候选 HEX/BIN/ELF/MAP 的 SHA256、HEX 校验和及有效数据逐字节对照、地址边界、版本和向量验证。
- `source_manifest.json`：130 个用户侧源码/配置/相关测试文件的 SHA256。基线 `daecedf8e7710a53fd84e1f5d8175063d83cd26e` 加累计未提交工作区；不能用基线号代替候选发布提交。

烧录文件位于本地忽略目录 `build/firmware/0.18.13/rosDemo-0.18.13.hex`，未自动提交或推送。旧版 0.18.12 原始实车数据保留。下一阶段需要核对新版本启动、周期/栈余量、正常运动不误触发及自然带载失败后的整车停车；不得仅凭离线 PASS 标记转向能力验收完成。
