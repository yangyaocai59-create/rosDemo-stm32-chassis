# 0.14.3 IMU RAW 实板验收记录（2026-10-16）

用户自行烧录指定 HEX 后，使用 CDC COM4、CH340 COM8，设备序列号 338536803133，完成以下测试。电机 VM 关闭、车体静止，所有窗口轮子计数未变化，无非零请求、无故障。CH340 全程只接收；助手未刷机、主动复位或操作 CH340 下载控制线。

**新 RAW 接口、旧协议兼容、显式校准保存及普通断电恢复均已实板通过。不是最终 ROS 静止漂移修复通过。**

## 身份依据

指定 HEX SHA256 为 `2ae4dbcecaffa5db311ad9a365b3ad9faab350923b51aaf54406ab8e5e04d0a4`。版本 0.14.3-imu-raw-v1、Build Oct 16 2026 09:56:40 来自对应构建文件。用户报告烧录和新增 0x06/0x85 功能实测相互印证。本轮未捕获 Boot Banner，也未回读整个实板 Flash，不能说已独立回读验证整个镜像身份。

## A. 校准前短测

证据：`build/hardware_tests/imu_raw_20261016_100648_722828/`。

- 默认旧 0x84、非法选择拒绝、切换 RAW/旧格式、会话超时恢复：PASS。
- 999 个唯一 RAW 样本，跨度 9.98 s，100.0 Hz、10..10 ms。
- 基准 [0,0,0]、flags 0x05，符合全片擦除后尚无兼容偏置的状态。
- 10 对同 sample_count 的 UART/USB RAW 精确匹配，CRC/读取错误为零。
- 无校准、运动或 Flash 写入。此时未校准的角速度不能当成校准后漂移指标。

## B. 一次显式校准及旧格式 120 s 采样

证据：`build/hardware_tests/imu_stationary_20261016_100712_748143/`。

实际 TX 仅 GET_STATUS +恰好一次校准请求；无运动、STOP 或清故障请求。

```text
IMU STATIC result=PASS_BIAS_APPLIED samples=500 duration=4991ms
IMU STATIC gyro bias raw=[-55,26,150] mdps=[-839,396,2290]
IMU STATIC gyro p2p raw=[17,17,18] temp=29.11C
IMU bias Flash saved: slot=A seq=1 address=0x080C0000 temp=29.11C
IMU STATIC gyro bias applied and persisted for future boots
```

- 12,002 个唯一旧格式样本，跨度 120.01 s，100.0 Hz、10..10 ms。
- IMU flags 0x37，读取/连续/CRC 错误零，四轮计数变化 [0,0,0,0]。
- 120 对 UART RAW/SI 换算不一致数为零。
- 本窗口旧格式角速度积分等效 +0.824847°/min，不含 ROS 残余补偿。
- 先前 bias_valid=0 与本次 seq=1 保存支持全片擦除流程，但本轮未直接扫描实板整片空白区。

## C. RAW 120 s 采样及 CDC DTR/会话恢复

证据：`build/hardware_tests/imu_raw_20261016_100928_889027/`。

- 实际 TX 仅 0x04/0x06，无校准、运动、STOP、清故障请求。
- 默认旧格式、BAD_LENGTH/BAD_VALUE、RAW 和切回旧格式：PASS。
- **11,999 个唯一 RAW 样本**，MCU 时间 177593..297573 ms，跨度 119.98 s；**100.0 Hz，10..10 ms**。
- 基准始终 **[-55,26,150]**、IMU flags 0x37、比例 655/8192，版本 1、reserved 0。
- 120 对同 sample_count 的加速度/陀螺 RAW 精确匹配，不一致零。UART RAW/SI 换算不一致零。
- CRC、IMU 读取/连续错误零；所有 STATUS 为 IDLE、零请求、两个 fault 为 NONE；轮子计数无变化。
- RAW 模式下将 CDC DTR 关闭，再开启并查询，收到旧 0x84：PASS。只切 CDC DTR，没有操作 CH340 复位线。
- 再次选择 RAW，停止请求 1.6 s 后查询，恢复旧格式：PASS。
- 没有新的校准或 Flash 保存日志，DTR/会话变化没有触发重复校准。

全速 RAW Z 均值 151.070589 LSB，减基准 150 后，无额外 mrad/s 整数量化的积分等效 **+0.980736°/min**，温度均值 29.100484°C。B 与 C 是不同窗口，二者漂移数值差异不能全部归为量化误差。

这些数据仍是“仅设备基准修正”的测量，不是 ROS corrected。不能以接近 1°/min 的单次均值宣称最终目标已完成，亦不能事后扣整段均值所得零作为独立验收。

## D. 普通断电后恢复及完整收包复核（已通过）

证据：`build/hardware_tests/imu_raw_20261016_101504_525621/`。用户确认普通断电重上电，未重新烧录；开始窗口 MCU 时间 13359 ms，上次 RAW 窗口结束为 297573 ms。

- 直接传输的基准字段始终 **[-55,26,150]**，IMU flags 始终 **0x07**；与保存值一致，本次上电未校准。
- 无 `IMU STATIC started`、无 `IMU bias Flash saved` 日志；实际发送只含 0x04/0x06，没有校准、运动、STOP 或清故障请求。
- 以完整收包记录核对，窗口 **11,999 个唯一连续 sample_count**，MCU 时间 13359..133339 ms、119.98 s、**100.0 Hz**，相邻时间 **9..11 ms**。
- 120 对同 sample_count 的 UART/USB RAW 完全一致，UART RAW/SI 换算一致。
- CRC、IMU 读取/连续错误、STATUS CRC/rx_overflow/tx_drop 均为零；四轮计数无变化，STATUS flags 始终 0x0150、IDLE/零请求/两个 fault 为 NONE。
- 切回旧格式、CDC DTR 关闭/开启后默认旧格式、主机会话超时恢复再次通过。

完整 RAW Z 均值 **152.504959 LSB**，设备基准仍为 150，等效静止积分 **+2.294390°/min**，温度均值 28.990378°C。原始零点变化仍存在，恰好说明下一步需要 ROS 的运行时残余估计；不能宣称接口改造消除了静止漂移。温度变化等因素仍未独立隔离。

### 验收脚本漏计的透明记录

原 `summary.json` 记为 11,998 个样本、一次 20 ms 间隔。核对发现被漏计的 sample_count=1285、t=13369 ms **确实存在于 usb_rx.bin 和完整 frames.jsonl**：是主机脚本 `wait()` 在一个读取批次中匹配到目标 ACK/RAW 后，未把剩余帧交给后续统计消费者。没有设备端或 USB 传输丢帧证据。

已修复脚本批次尾部缓存，并新增 `Tools/tests/test_imu_raw_capture.py`，离线重放“同批 ACK+多帧 RAW”全部保留的回归通过。没有修改固件、不需要重新烧录。

原始 summary、原始样本子集及日志全部保留，不覆盖。新增 `full_receive_analysis.json` 和 `full_receive_raw_samples.jsonl`，由 `Tools/verify_imu_raw_evidence.py` 从完整 USB 字节重解码、与完整帧日志逐帧比对后生成；本文 D 的最终数值来自该派生审计。A/C 也执行相同审计，补回样本数均为零；D 补回 1 个。最终交付状态采用审计后数值，不悄悄混用原摘要。

## 交付范围与未验收项目

本地固件 RAW 接口、显式校准保存、普通断电恢复的计划工作完成，可以冻结“固件接口已验证”交付包。最终 ZIP 名称含 INTERFACE_VERIFIED，明确不是整体漂移修复认证。

1. 本版本运动中物理断链、启停/慢转保真实板回归未专项进行；原保护源码未改、native 回归通过，mock 不替代实板结论。
2. ROS 估计器尚待实现；至少三次上电 corrected 静止漂移与最终 EKF 回归不属于本次 RAW 接口 PASS。
3. 未做实板 Flash 全量回读或捕获此次启动 Banner；未做写 Flash 时掉电、长期温漂及栈高水位专项认证。

最终包包含完整原始证据、派生统计、实际固件和源码；历史候选包与 0.14.2 交付不覆盖。
