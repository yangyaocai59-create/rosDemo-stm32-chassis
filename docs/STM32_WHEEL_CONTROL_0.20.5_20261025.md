# 0.20.5：MPU6500 初始化校验修正与故障定位

2026-10-25。用户烧录 0.20.4 后，只读检查发现 IMU 为 IO_FAULT、样本数 0；用户完整重新上电后复查仍相同。WHO_AM_I=0x70，但没有有效六轴样本，`YAW3 ready=0 valid=0`。两轮均保持静止，没有发起转向、清故障、校准或复位。本版独立构建，0.20.4 及全部旧包保留，不覆盖。

## 发现与边界

[TDK 官方 MPU6500 寄存器手册 RM-MPU-6500A-00，rev 2.1，4.33 节](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6500-Register-Map2.pdf)说明 USER_CTRL（0x6A）的 bit 4 I2C_IF_DIS 是自清零命令位。旧配置写 0x10，之后用 mask=0x70 比较是否仍为 0x10；器件正常清零后会被当作校验失败。真实 service 的寄存器模型重现了旧版进入 IO_FAULT、首次样本无法发布的路径。

这是一处有手册依据且离线可复现的问题，但现有运行版本没有逐寄存器日志，尚未确认这块板此次失败的实际寄存器和读值。不能仅靠 WHO_AM_I 或样本数 0 就断言硬件损坏，也不能将此处修复等同于已解决板上问题。

## 本版修改

只对 `CHASSIS_CONTROL_REVISION>=205` 生效：

- 仍按原顺序写全部原配置，包括 0x6A=0x10。
- 校验 0x6A 时忽略自清零 bit 4，仍检查原来受检的 bit 6 FIFO_EN 和 bit 5 I2C_MST_EN。其余寄存器掩码、身份、量程、滤波、采样频率、电源和读错误检查保留。
- 接受 0x6A 的 bit 4 已清零或仍为 1 两种读回，不跳过整套配置校验，不伪造有效样本。
- 初始化复位写失败、配置写失败、身份/寄存器读取失败和配置不符分别记录；不符时报告地址、actual、expected 和 mask。每次成功初始化输出一次 USER_CTRL 读回值，周期校验不刷屏。
- 设置表及其指纹保持不变。旧版本的配置指纹 `0x212B53A9` 有显式断言；已有持久校准格式、单位、参数、Flash 恢复和保存规则不改。

传感器生产代码只改 `Application/Sensors/mpu6500.c`，并用版本宏保留 191–204 的行为。打包器对这一处列出原哈希、新哈希和原因；其余 184 个受保护源文件仍要求与原始基线逐字节一致。没有把整个传感器目录排除出保护。

四轮速度控制、0.20.4 角速度外环参数及其禁止额外增力/保留减速的规则不改。PWM、轮速积分、20 ms 控制周期、几何、编码器计数及急停/堵转/断链/超时保护不改。此前上位机 v4 DLL 的双向匀速结果属于旧轮次，不能当作 0.20.5 上板结果。

## 验证

新增真实 driver/service 的寄存器模型：配置写入后 bit 4 自动清零；旧分支应进入 IO_FAULT 且样本为 0，新分支应 READY 并发布真实模拟样本。FIFO/I2C master 位、量程错误及读取失败仍应拒绝。测试同时覆盖 bit 4 保持和清零、原指纹、完整校准/Flash 恢复与失败路径。所有 Flash 操作仅在测试进程私有内存模型中。

完整打包包含 strict/O2/fanalyzer、新旧 MPU6500 分支、轮控/外环回归、191 兼容、新旧 USB、Debug/Release clean 构建、HEX/BIN/版本/地址/内存校验、原始及前序候选全部产物哈希检查。以包内 `verification.json` 和验证日志的实际结果为准。

独立预设为 `WheelV0205Debug` / `WheelV0205Release`，宏为 205，默认旧预设仍为 191。打包：

```powershell
python -B Tools/package_wheel_v0200.py --version 0.20.5
```

归档为 `build/firmware/0.20.5/{Debug,Release}/rosDemo-0.20.5.{hex,bin,elf,map}`，发布包为 `releases/0.20.5/rosDemo-0.20.5-offline-candidate.zip`。

## 上板待核对

0.20.5 尚未烧录。下一次启动先读取 `MPU6500 verify OK USER_CTRL=... stable_mask=0x60` 或具体失败行，再确认 READY、样本递增和 `YAW3 ready=1`。只有样本有效且静止基线就绪后，才开始短时转向、双向匀速与停车路径验证。如仍 IO_FAULT，使用新增寄存器信息继续诊断，不反复要求断电或盲目校准。

0.20.4 首次未就绪现场在 `docs/stm32_evidence/2026-10-25-v0204-first-boot-imu-fault`；本轮用户重新上电后的现场在 `docs/stm32_evidence/2026-10-25-v0204-powercycle-imu-fault`。这些记录不覆盖此前成功的上位机试验。
