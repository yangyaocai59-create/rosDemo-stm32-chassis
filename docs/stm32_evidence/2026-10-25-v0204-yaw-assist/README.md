# 0.20.4 同一 C 核心的双向限时验证

2026-10-25。线缆阻挡由用户报告解除后，在板上现有 0.20.2 的轮速控制外，用 Windows DLL 运行新版相同 C 角速度核心。没有烧录、复位、改写编码器计数或传感器校准。两次各发令 10 秒，物理摆放 unspecified。

## 本轮结果

名义目标为 ±0.400 rad/s。IMU 去除每次运动前至少 3 秒的静止 RAM 零偏；以 MCU 源时间积分，各取起步首秒以后的 8 个完整 1 秒窗口，范围为 START+1 s 到 START+9 s。最后不足一秒不计入匀速评价。数值为秒均值偏差，不能代表所有瞬时采样均在界内。

| 同一 v4 核心 | 左转 | 右转 |
| --- | ---: | ---: |
| MCU START–STOP | 9.982 s | 9.970 s |
| 上述窗口平均角速度 | +0.40842 rad/s | −0.39582 rad/s |
| 1 秒窗口最大相对偏差 | 9.49% | 3.05% |
| 1 秒窗口相对偏差 RMS | 4.25% | 1.97% |
| 窗口全部在 ±10% 内 | 是，单次 | 是，单次 |
| 编码器角度，含停车后约 1 s | +233.83861° | −232.44304° |
| 相同时间窗 IMU 参考角度 | +230.76122° | −226.68368° |
| 编码器减 IMU | +3.07739° | −5.75936° |
| 用运动后零偏重算的敏感度 | 0.86444° | 0.74755° |

每次均正常停车，无健康故障、无新增拒绝、无 CRC 错误和传输丢弃。编码器配置 bad=0，输出请求与实际 PWM 一致，最终 STBY=0、四通道 CCR=0。IMU 源采样最大间隔 10 ms，没有缺样；板上原有 motor 20 ms / safety 10 ms 无超期。

这支持当前参数在这两次试验中的匀速目标，不是重复验收，也不能对比不同方向、地面、温度或电池条件宣称固定改善比例。既往未达标和阻挡失败完整保存在两个 0.20.3 证据目录。编码器与 IMU 的原始计数、角度分别保留，不用改轮距或缩放计数隐藏打滑误差。

## 算法与证据

v4 DLL SHA256：`8ce6a0e7c8d6fef752b740763a1e5dc860a3e22370010758d7aa9f701b0e1fdb`。

编译方式：

```powershell
gcc -std=c11 -Wall -Wextra -Werror -O2 -shared -DCHASSIS_CONTROL_REVISION=204 -IApplication/Chassis Tools/tests/motion/yaw_assist_host_bridge.c -lm -o build/tests/wheel_v0204_host/yaw_assist_v4.dll
```

核心参数为 KP=0.60、KI=0.30/s、80 ms 滤波、600 ms 等待、±0.10 rad/s 且不超过参考值 25% 的修正、0.12 rad/s² 变化率。失去轮速控制余量时清除积分、禁止额外增力，仍允许有效 IMU 超速时减速。纯原地转向以外不介入。

`host_v4_left` / `host_v4_right` 含原始 USB、debug 串口、控制命令、运动追踪、角度曲线与 `yaw_uniformity.json`。每次采集自带 manifest 保护原始文件，目录级 `capture_manifest.json` 另覆盖后续分析和本说明。`tested_sources` 保存所用源文件及 DLL。`analyze_uniformity.py` 可离线重算窗口，无设备访问。

上位机用 USB 遥测估计轮速余量，MCU 版本读取本地上一控制周期的完整快照并额外检查输出受限标记；调度和采样不同。整版 0.20.4 尚未烧录，仍需上板检查 RAM 零偏就绪、`YAW3` 修正、双向重复匀速、STOP/断链/超时、实时性及 180° 定角。没有把上位机试验或离线测试当作新版 MCU 验收。

## 独立固件与最终状态

strict/O2/fanalyzer、新旧 USB、191 兼容回归及 Debug/Release clean 构建全部通过。185 个受保护源文件、174 个原始产物和 372 个前序候选文件的哈希复核通过。新版 Debug RAM/FLASH 为 106072 / 189600 字节，Release 为 105216 / 81136 字节。各函数栈帧及完整验证见 `firmware_verification.json`；这是离线结果，没有替代新固件上板栈余量测量。

- 发布包：`releases/0.20.4/rosDemo-0.20.4-offline-candidate.zip`。
- ZIP SHA256：`212f4bb2a50c96c67c38d31a9ef3eba9bc9338ebcf3b0f829680a9245a0eab40`。
- Release HEX SHA256：`acd05beed6ad3bb9b6a32b7b5bbc373b352c71d088002232955a6133a4256655`。
- 0.20.3 及全部更早固件保留。

`final_readonly` 为两次 v4 测试后的 3 秒只读检查，只发 GET_STATUS。IDLE、故障 0，四轮速度与输出均零，编码器计数不变，没有复位、运动、清故障或烧录。供电末次约 10.44 V。车辆已停车，串口已关闭。
