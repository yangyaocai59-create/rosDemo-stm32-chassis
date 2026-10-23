# 0.19.1 实测候选固件

固件版本：0.19.1-effective-track-candidate。固件源码提交：68b4597。当前包保存已经烧录测试的原始镜像，本轮没有重新构建或烧录。

[下载固件包](rosDemo-0.19.1-existing-image.zip)，[SHA256](SHA256SUMS.txt)。包含 HEX/BIN/ELF/MAP、246 项构建时源文件清单、原始离线 verification.json。verification.json 的 OFFLINE_VALIDATED_NOT_HARDWARE_ACCEPTED 是打包前原有阶段记录，未追改；当前硬件结论请看[最新实测交接](../../docs/stm32_evidence/2026-10-23-v0191-calibration-repeat-right/README.md)。

HEX SHA256：df84a5522ff1a2d8935600031230d1a6d151343ff52a8896ba11b54c96e381df。

控制采用四轮独立 FF+PI、左右侧差速目标；有效轮距统一为 0.395 m，物理轮距 0.210 m、轮径 0.065 m、1560 counts/rev。IMU、保存偏置实现、编码器与协议未改。具体改动见[版本说明](../../docs/STM32_EFFECTIVE_TRACK_0.19.1_20261023.md)。

连续 ±0.40 rad/s、超过 15 秒及正常 STOP 已实测；编码器重复性良好，但与 IMU 参考仍约 9–10% 差异，没有外部转角真值，不是导航验收完成版。本次不需要再次烧录。今后若全片擦除，会丢失 IMU 保存校准，需按既有流程恢复。
