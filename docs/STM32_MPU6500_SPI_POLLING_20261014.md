# MPU6500 SPI 六轴采集：0.14.1 临时测试固件

日期：2026-10-14，Asia/Shanghai。此版在 0.14.0 身份探测通过之后新增初始化、六轴和芯片温度采集。尚未烧录本版或完成实板六轴验收，不作为已经完成 ROS 联调的正式固件。

## 1. 固件身份、构建与交付

```text
Firmware version = 0.14.1-spi-mpu6500-polling
Build = Oct 14 2026 09:44:17
HEX SHA256 = 19A6D47961506ADD31B5B7F7D7735126D499F6BFA4BD133893BFC81E32326008
ELF SHA256 = 5B5BDF0AA4B562207611615A4ABFBF43E8A5B10B66C37A288C299C6D9554CC5E
```

真实 ARM Debug configure + clean build，91 步成功：Error=0，Warning=0；FLASH=151988 B / 768 KiB（19.33%），RAM=54840 B / 128 KiB（41.84%），CCMRAM=0 B / 64 KiB。工具链 GCC 14.3.1、CMake 4.3.1、Ninja 1.13.2。

烧录文件：`build/deliveries/mpu6500_spi_polling_0.14.1_20261014/rosDemo.hex`。同目录包含 ELF、本文、构建/验证日志、源码快照及 SHA256 清单。工程无 Git 元数据，没有可提供的真实提交号；用镜像/源码快照哈希追溯。旧 0.13.3 正式交接包及 0.14.0 身份探测镜像保留，不覆盖。

## 2. 已有硬件证据与接线

上一版用户提供的 33.6 秒 CH340 日志显示 WHO_AM_I=0x70，前三次连续匹配，后续保持 stable=1，reads=33、errors=0、unknown=0，任务健康正常。它证明 SPI 身份读链路正常，不证明芯片真伪、六轴量程、INT 或新固件实板通过。新模块 VCC 对模块 GND 实测 3.3 V；旧 MPU6050 已拆除。

接线不变，不需要修改 CubeMX：

| 模块端 | STM32 端 / 含义 |
| --- | --- |
| VCC | 3.3 V，不接 5 V |
| GND | STM32 GND，共地 |
| SCL / SCLK | PA5，SPI1_SCK |
| SDA / SDI | PB5，SPI1_MOSI |
| AD0 / SDO | PB4，SPI1_MISO，不能接地 |
| NCS | PA4，MPU6500_CS，初始 HIGH |
| INT | PB0，IMU_INT，保留接线，本版不启用传感器中断 |
| FSYNC | GND |
| EDA / ECL | 不接 |

沿用现有 SPI1 mode 0、8-bit、MSB-first、软件片选、prescaler=128，APB2=84 MHz 时 SCK=656.25 kHz。I2C1 PB6/PB7 生成配置及旧驱动保留，但不启动旧 IMU 采样。

## 3. 初始化及采集方式

`imuTask` 等待约 100 ms → 检查 WHO=0x70 → PWR_MGMT_1 写 0x80 复位 → 等待 100 ms → 配置 → 等待 100 ms → WHO/配置读回验证 → 进入 READY → 首次成功采样后 valid=1。等待由任务调度完成，不使用长 HAL_Delay；SPI 传输仍使用有超时的 HAL polling，并非完全异步 I/O。

| 地址 | 写值 | 目的 |
| --- | --- | --- |
| 0x6A USER_CTRL | 0x10 | 禁用主 I2C 接口、FIFO 和辅助 I2C master |
| 0x38 INT_ENABLE | 0x00 | 不启用传感器中断 |
| 0x37 INT_PIN_CFG | 0x00 | 不启用 latch/bypass/FSYNC 中断 |
| 0x23 FIFO_EN | 0x00 | 禁用各通道 FIFO |
| 0x6B PWR_MGMT_1 | 0x01 | 唤醒、温度使能、自动 PLL 时钟选择 |
| 0x6C PWR_MGMT_2 | 0x00 | 六轴全部启用 |
| 0x1A CONFIG | 0x03 | 陀螺 DLPF 41 Hz，不使用外部同步 |
| 0x1B GYRO_CONFIG | 0x08 | ±500 °/s，FCHOICE_B=0，关闭自检 |
| 0x1C ACCEL_CONFIG | 0x08 | ±4 g，关闭自检 |
| 0x1D ACCEL_CONFIG2 | 0x03 | 加速度 DLPF 41 Hz，不旁路 |
| 0x19 SMPLRT_DIV | 0x09 | 1 kHz / (1+9) = 100 Hz |
| 0x69 ACCEL_INTEL_CTRL | 0x00 | 不使用运动唤醒逻辑 |

每 10 ms polling：检查 WHO、PWR_MGMT_1，然后从 0x3B 开始在一次片选内读取 14 字节（加速度 XYZ、温度、陀螺 XYZ），显式解码有符号 16-bit。样本时间取成功读取后的 HAL_GetTick；它不是传感器硬件锁存时间。

每秒核验一次所有配置的有效位。单次传输超时参数 5 ms；任何 HAL 返回路径均释放 CS。SPI1 当前由 imuTask 独占，不能直接让其他任务或 PB0 ISR 并发访问；日后共用需另加仲裁。错误立即撤销 valid；连续三次采样失败，或一次配置核验失败，进入 IO_FAULT，约 1 s 后重试初始化。失败时保留旧快照值和累计计数，但它们无效，接收端必须看 valid/state/时间，不能将旧值当新数据。

量程换算：加速度 8192 LSB/g，陀螺 65.5 LSB/(°/s)，输出 m/s²、rad/s。MPU6500 温度使用 raw/333.87+21 °C（标称芯片温度），不能沿用 MPU6050 的 raw/340+36.53；不是精确环境温度计。

## 4. 校准、协议和安全边界

- `IMU_SPI_ID_PROBE_ONLY=0`、`IMU_USE_MPU6500_SPI=1`，`IMU_CALIBRATION_ENABLED=0`。宏检查禁止 SPI 测试复用旧校准配置。
- 不读取、扫描、恢复或写入旧 MPU6050 偏置 Flash。UART `i` 和 USB 校准入口明确拒绝；直接 SetGyroBiasRaw 也返回 false。原记录没有删除。
- USB v1 帧格式、IMU_STATE 46-byte payload、单位和命令不变。正常采样预期 `READY flags=0x05 valid=1 bias=0 cal_active=0 cal_result=0`；`bias=0` 是当前阶段的刻意限制，不是数据无效。
- 旧 USB/UART 帮助可能仍列出校准命令，但本版禁止执行，不能运行 `--imu-calibration-test` 验收。
- 仅输出未去零偏六轴和温度，无磁力计、姿态四元数、yaw 融合、DMP、FIFO、DMA 或 INT 数据就绪验证。
- 100 Hz 是配置和软件采集目标；sample_count 表示成功读次数。未使用 DRDY 判断，不能声称每次读取都是独立硬件新样本。
- 坐标映射暂时保留 identity：X 前、Y 左、Z 上。这是软件约定，新模块实际安装方向尚待静止/手动转动验证；不能只凭旧模块结论确认。
- 编码器、PWM/方向、USB SOF/TX 修复、掉线停车、ESTOP、任务健康和 IWDG 保持原样。没有新增任何电机启动行为。但旧运动入口仍存在：这不是全系统禁止运动镜像，测试必须关闭 VM，停止 ROS/键盘运动程序。

## 5. 修改文件（相对冻结 0.14.0）

新增：

- `Application/Sensors/mpu6500.c/.h`：SPI 访问、配置/读回、14-byte 采样和温度换算。
- `Tools/tests/imu_spi_probe/test_mpu6500_polling.c`、`Tools/tests/run_mpu6500_polling_tests.ps1`：实际生产代码的 native mock 回归。
- 本文及交付日志/快照。

修改：

- `CMakeLists.txt`：加入 mpu6500.c，不改 CubeMX CMake。
- `Application/Sensors/imu_service.c/.h`、`imu_config.h`：选择新后端、初始化/采集/恢复、关闭旧校准。
- `Application/Diagnostics/imu_static_test.c`：禁止 SPI 测试进入旧校准流程。
- `Application/Board/board_config.h`、`Application/System/firmware_info.h`：映射/版本说明。
- `Core/Src/freertos.c`：仅 USER CODE，启用新采样，跳过旧 Flash 恢复/静止校准处理，恢复低频 IMU 日志。
- `Tools/tests/run_imu_spi_probe_tests.ps1`、`Tools/tests/imu_spi_probe/stubs/stm32f4xx_hal.h`：显式测试旧探测模式、补 HAL 时钟 mock 声明。
- `docs/ROS2_UBUNTU_HANDOFF.md`：标明换 IMU 后最新测试状态。

与旧源码快照逐项对照，以上之外的已有生产文件、`.ioc`、外设生成配置、USB、安全、电机、Flash 存储源码与 linker 未改；freertos.c 的 USER CODE 外文本一致。

## 6. 已完成软件验证

7 个 native 测试程序成功（新六轴、旧身份探测、5 个既有 USB/SOF/TX/日志程序），采用实际生产代码 + HAL/RTOS/传感器/存储 mock。检查配置读回、有符号边界、CS/超时/错误、单位、名义温度、身份不匹配/传感器复位/量程被改、恢复、时间回绕、有效样本仍拒绝校准/Flash；旧 STOP/GET/restart、50/51 ms 真掉线保护、TX 容量和日志回归通过。

保留的 identity-only 和 MPU6050 分支均通过 ARM compiler syntax-only + Werror 检查，但不是另行交付/实板验收的镜像。链接后的 ELF 有 Mpu6500_*，没有 Mpu6050_*、ImuBiasStorage_*、ImuStaticTest_ProcessSample、HAL_FLASH_Program 或 HAL_FLASHEx_Erase。HEX 9507 条记录校验和通过，数据范围 0x08000000..0x080251B3，不覆盖 sector 10/11；交付前再次从 ELF 导出 HEX 并比对。

上述是离线软件验证，不替代实板：实际六轴、100 Hz 时序、温度合理性、轴向、栈高水位和断开恢复尚未验收。

## 7. 用户现在如何测试

1. 关闭电机 VM，保留模块 3.3 V 和 STM32 供电。不要带电重插线。关闭其他运动程序。
2. 使用之前的串口烧录流程烧录本版冻结目录的 rosDemo.hex；恢复正常 BOOT 启动。若保留旧记录，禁止全片擦除/擦除 sector 10/11（0x080C0000..0x080FFFFF），下载工具擦除设置仍需核查。
3. 打开 CH340 串口助手 115200 8-N-1，复位，确认 version/Build 与第 1 节一致。
4. 车体/模块水平静止约 20 s，保存完整启动日志。应看到以下示例关键行（数值仅示意，不是已测结果）：

```text
Firmware version = 0.14.1-spi-mpu6500-polling
Build = Oct 14 2026 09:44:17
MPU6500 SPI startup: 100Hz polling; calibration/Flash restore disabled
MPU6500 ready: WHO_AM_I=0x70 SPI1 100Hz +/-4g +/-500dps DLPF=41Hz polling; bias=0 INT disabled
IMU RAW: a=[100,150,8192] temp=1000 g=[-20,10,30] samples=100 errors=0
IMU SI: a_mmps2=[119,179,9806] w_mrads=[-5,2,7] temp_mC=23995 bias=0
```

5. CDC 口若仍是 COM4，单独执行下面只读测试，不能附加运动/校准选项；端口号以设备管理器当前枚举为准，CH340 与 CDC 不混用。

```powershell
python Tools\usb_cdc_test.py --port COM4 --imu-monitor 10
```

期望样本数持续增长、约 100 Hz、READY/valid=1、bias=0、errors=0/0。水平且 Z 向上时加速度 Z 接近 +9807 mm/s²，XY 远小于 Z；未校准陀螺不必等于零。重力大小可先用约 9–11 m/s² 排查方向/量程，不能当精度认证。

6. 先将静止串口与 Python 输出发回审核。静止正常后，保持 VM 关闭，手动缓慢绕竖直轴左转/右转车体，检查 wz 相对静止偏置分别正/负，停止后回到偏置附近。不要用电机转动代替轴向验证，不要让线缆牵扯模块。俯仰/横滚及断开重连另行安排；不用带电拔传感器。

若 WHO 错、长期 DISCONNECTED/IO_FAULT、errors 增长、数据始终全零、重力符号不符或重复 Boot Banner：保留完整日志，不提高 SPI 频率、不调 PID、不开始运动测试。

## 8. 后续未做事项

六轴与轴向实板验收后，才单独设计 MPU6500 身份绑定的校准/持久化，避免套用旧 MPU6050 记录。INT 若需要，再验证硬件数据就绪；正式 ROS 上层切换必须明确新固件的 bias=0 及校准暂时不可用，不能把此版当旧校准完整功能的替换。

寄存器与温度依据：[TDK MPU6500 register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6500-Register-Map2.pdf)、[TDK MPU6500 product specification](https://invensense.tdk.com/wp-content/uploads/2020/06/PS-MPU-6500A-01-v1.3.pdf)。
