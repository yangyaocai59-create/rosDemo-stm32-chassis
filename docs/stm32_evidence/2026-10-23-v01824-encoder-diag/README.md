# 0.18.24 编码器诊断离线证据

设计与限制：[诊断版说明](../../STM32_ENCODER_DIAG_0.18.24_20261023.md)。

- verification.json：测试、资源、栈帧局部信息及SHA256。
- source_manifest.json/source_snapshot：源码封存、相对0.18.23变更范围。
- logs：原始离线测试、完整清理编译和ELF符号。
- package_candidate.py：测试/编译/封存，不连接硬件、不操作Git；已存在冻结目录时拒绝覆盖。

本版新增只读诊断，编码器累计、测速、控制参数、USB/IMU和保护判据未改。零净计数的实车原因仍未唯一确定，本包不能作为带载性能或ROS转角验收通过证明。
