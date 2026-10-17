# MPU6500 校准与掉电恢复交接：0.14.2

日期：2026-10-14，Asia/Shanghai。本版仅恢复新 MPU6500 的显式陀螺零偏校准和兼容 Flash 恢复，USB 线格式与运动控制不改。软件实现、真实编译及用户实板校准/保存/重启恢复已通过。本次更新仅文档和证据归档，HEX/ELF 未改变。

## 固件和构建

```text
Firmware version = 0.14.2-spi-mpu6500-bias-flash
Build = Oct 14 2026 18:26:57
HEX SHA256 = EAA3952998691BD1FFD9B2D22C28A17A13B50B30E682916AE2F20CAACC631342
ELF SHA256 = 1C1F646319184907223D877414F29F8E34D58EB1CB795959E719D7EF800C48B9
```

真实 Debug configure + clean build，91 步：Error=0，Warning=0，FLASH=158900 B / 768 KiB（20.21%），RAM=54936 B / 128 KiB（41.91%），CCMRAM=0 B。GCC 14.3.1 / CMake 4.3.1 / Ninja 1.13.2。无 Git 元数据，因此没有真实提交号，使用镜像和源码快照 SHA256 追溯。

最新已验收交付目录：`build/deliveries/mpu6500_ros_handoff_0.14.2_verified_20261014/`，包名 `STM32_MPU6500_ROS_HANDOFF_0.14.2_VERIFIED_20261014.zip`。包含冻结 HEX/ELF、最新本文、源码/协议/测试工具、构建/验证日志、两次新实板日志及用户 USB 输出节选。最初的 `mpu6500_bias_flash_0.14.2_20261014/` 为验收前历史交付，保持不变，其文档状态和烧录要求以本次更新为准。

**固定烧录条件：用户每次重新烧录都会全片擦除，不要求保留 sector10/11。重新烧录后必须手动校准一次；后续普通断电、复位会恢复记录，不需要每次重新校准。测试重启恢复时不能重新烧录。**

## 前置实板验证与硬件不变项

0.14.1 用户测试已确认：WHO=0x70、100 Hz，10 秒 USB 1001 个样本计数、9..11 ms、errors=0；静止串口段无 IMU 错误。先左转后右转分别得到正/负 wz，用户确认物理方向；抬车头 ax 由约 -700 增至 +1600..+1800 mm/s²，抬左侧 ay 由约 +40 增至 +2100..+2300 mm/s²，放平恢复。这是基础采集/轴向验证，不是姿态精度、长期温漂或所有安全机制验收。

模块 3.3 V；SCL/SCLK→PA5，SDA/SDI→PB5，AD0/SDO→PB4，NCS→PA4，INT→PB0（保留但传感器中断不启用），FSYNC→GND、共地；SPI1 mode 0 / prescaler128 / 656.25 kHz；I2C1 保留但旧 MPU6050 不采样。坐标 identity：X 前、Y 左、Z 上。

保持 SPI 100 Hz polling、±4g、±500°/s、DLPF41Hz。未加入 INT/DRDY、DMA、FIFO、DMP、磁力计、姿态融合。USART1 PA9/PA10 调试、USB PA11/PA12、ADC1_IN11 PC1、编码器/TIM8、电机极性、PD10 公共 STBY、PD11 保留 LOW、ESTOP 和 IWDG 不变。

## 启动与校准行为

- 当前 `IMU_USE_MPU6500_SPI=1`，`IMU_CALIBRATION_ENABLED=1`。identity-only 模式依旧禁止校准。
- 启动扫描 A/B journals，只接受当前型号/配置兼容且完整提交的记录。WHO=0x70、配置读回验证并进入 READY 后，由既有 imuTask 流程恢复一次偏置；不自动采集 5 秒，不自动保存。
- 无兼容记录：READY/valid=1/bias=0，等待用户/上位机显式请求。旧 MPU6050 记录不会让 bias 变成 1。
- 保留 UART `i` 和 USB CMD_CALIBRATE_IMU=0x05（空 payload）；成功采集 500 个不同样本计数，约 5 秒，以映射后的原始 XYZ 陀螺均值设置偏置，然后追加保存。
- 加速度仅用于静止/重力检查，不做加速度零偏/比例标定，也不减掉重力。当前静止 ax≈-0.7 m/s² 的安装倾斜/加速度偏差不会由陀螺校准修正。
- 运行时角速度使用 `(raw - saved_bias)/65.5 × pi/180`；采集校准均值始终使用未修正的 raw，不会对旧修正结果再次校准。
- 温度由统一 `ImuService_TemperatureRawToC()` 转换，MPU6500 为 raw/333.87+21°C，取 500 样本原始温度均值后存为四舍五入的 centi°C；不再套用 MPU6050 公式。温度只是记录元数据，无温度补偿。
- 保留原来的静止检测范围、±1000 raw 偏置边界、状态/诊断互锁、重复请求拒绝、STOP 取消、USB 会话结束取消和校准期间速度请求拒绝。另补保存尚未结束时不能启动第二轮校准的检查。
- 保存失败：新偏置可在 RAM 生效，之前的有效 Flash 记录保留，并打印 ERROR；下一次启动可能恢复的是上次保存值，不是本次 RAM 值。不要将 RAM 校准成功当成掉电保存成功。

## Flash 兼容策略

地址不变：sector10 `0x080C0000..0x080DFFFF`，sector11 `0x080E0000..0x080FFFFF`；应用 linker 仍限制 768 KiB。无运行时擦除，仅向完整擦除的 40-byte 槽追加，优先另一 journal；两区完全空时共 6552 槽，旧记录/失败槽会占用容量。用尽后拒绝保存，不能自动擦除。

40-byte 布局保持，以免更换记录长度后扫描错位：

| 偏移 | 字段 | 0.14.2 MPU6500 |
| --- | --- | --- |
| 0 | magic u32 | 0x494D5542 |
| 4 | format_version u16 | 2；旧 MPU6050 为 1，不接受 |
| 6 | record_size u16 | 40 |
| 8 | sequence u32 | 当前兼容记录序列，支持 u32 回绕 |
| 12 | imu_identity u32 | 配置描述符 CRC32，本版 0x02136185 |
| 16/20/24 | gyro_bias_raw XYZ i32 | 车体坐标原始陀螺偏置 |
| 28 | temperature i16 | 校准温度 centi°C |
| 30 | reserved/profile u16 | 0x0170：算法1、WHO0x70 |
| 32 | crc32 u32 | 前32字节 CRC32 |
| 36 | commit_marker u32 | 0x434F4D54，最后写入 |

配置描述符含型号0x6500、WHO0x70、校准算法/温度约定版本、采样周期、实际寄存器配置列表的 FNV-1a32 签名、XYZ 源轴及符号、加速度/陀螺比例 float 位模式。该描述符按本平台 little-endian 字节计算 CRC32。型号/配置不符、格式/长度错、CRC 错、未提交或偏置越界均拒绝。

`Mpu6500_ConfigurationSignature()` 是纯计算，不访问 SPI；量程/滤波等寄存器设置改变会改变记录兼容性。修改校准算法或温度约定时维护者仍必须升级相应描述符版本。

旧 format1 / identity0x60500501 记录在固件运行时不删除，完全不迁移到新 IMU；但全片擦除烧录会清除所有记录，包括新 MPU6500 记录。保留的 MPU6050 编译分支继续使用旧格式。新增空槽跳过检查，避免一个空洞后面紧跟旧/部分记录时尝试覆盖它。失败后重新扫描，部分写入槽不复用；10 个 word 的最后一个是提交标记，随后校验和完整 readback 比较。

这是型号/配置身份，不是物理传感器唯一序列号。更换另一块相同型号模块、调整模块安装方向而未改软件映射，无法自动检测：必须显式重新校准。温漂明显时也可重新校准，但禁止后台自动反复写 Flash。

## USB / ROS 开发者注意

无需改 USB 协议、Python 解码或 ROS 帧长度：v1、CMD0x05、46-byte IMU_STATE、单位/坐标/flags 完全保留。三种典型状态如下，不能与 STATUS 的 flags=0x0150 混淆：

| 情况 | IMU flags | 含义 |
| --- | --- | --- |
| 首次无兼容记录 | 0x05 | valid=1，bias=0，初始化完成 |
| 本次校准成功 | 0x37 | bias=1，cal_result=1，cal_ok=1 |
| 断电后恢复偏置 | 0x07 | valid=1，bias=1；本次没有校准结果，cal_result=0 |

`cal_result=0` 不表示已恢复的偏置无效。上位机应根据 bias_valid 决定是否需要显式校准，不要每次启动机械地重校准。没有新持久化确认 flag，因此 `--imu-calibration-test` 的 PASS 只确认 RAM 校准/flags/互锁；必须结合 CH340 的 `IMU bias Flash saved` 与真正断电恢复验收才能确认保存。

恢复的是陀螺零偏，不是姿态/yaw：本 MCU 没有新增角度积分，也没有保存上次姿态。

ROS 发布 angular_velocity 时不要重复减掉 STM32 已扣除的零偏；加速度仍含重力，不能因静止 ax 不为零就把它当陀螺校准失败。建议先只读确认 READY/valid/bias、100Hz 与帧解码，再单独安排架空低速启停回归，本次不进入 SLAM/Nav2。

## 0.14.2 实板验收结果

用户校准 UART 原始日志 `imu_calibration_uart.log`：

```text
IMU bias Flash ready: MPU6500 SPI profile=0x02136185 no compatible saved record
IMU STATIC result=PASS_BIAS_APPLIED samples=500 duration=4999ms
IMU STATIC gyro bias raw=[-38,2,68] mdps=[-580,30,1038]
IMU STATIC gyro p2p raw=[18,16,21] temp=28.40C
IMU bias Flash saved: slot=A seq=1 address=0x080C0000 temp=28.40C
IMU STATIC gyro bias applied and persisted for future boots
```

这份日志2123行、1次启动、无ERROR和IMU故障/恢复记录。用户 USB 校准命令输出：校准前flags0x05/bias0，校准后flags0x37/bias1/w=[1,0,0]mrad/s、errors0/0，脚本PASS。证明实际校准、RAM应用与Flash写入读回完成，不仅仅是native模拟。

用户随后按断电重启流程提交 `imu_reboot_restore_uart.log`：

```text
IMU bias Flash ready: MPU6500 SPI profile=0x02136185 newest=A seq=1 records=1/0
IMU gyro bias restored from Flash: raw=[-38,2,68] calibration_temp=28.40C
```

这份日志737行、1次新启动、无ERROR/IMU故障，`IMU STATIC started` 和 `IMU bias Flash saved` 均0次。偏置与上次保存完全一致。USB10秒：1000个不同样本计数、100.0Hz、10..10ms，flags0x07/valid1/bias1/cal_result0/errors0/0，角速度接近零。用户 USB 输出节选单独归档；它们是用户提交记录，不是本助手自动连接硬件运行。

本轮通过基本校准/持久化/新启动恢复验收。复位原因显示PIN RESET，不能单凭该mask独立证明所有物理供电均已切断；断电步骤依据用户执行的流程，新启动日志直接证明Flash记录恢复且没有自动校准/保存。没有做保存中途断电测试，不将此验收扩大为掉电中断或长期温漂认证。

## 修改文件

相对冻结 0.14.1：

- `Application/Sensors/imu_config.h`：启用新 SPI 校准，继续禁止 identity-only 写入。
- `Application/Sensors/imu_service.c/.h`：共享温度接口、启动说明；原启动/采样/恢复顺序不变。
- `Application/Sensors/mpu6500.c/.h`：提供实际寄存器配置签名，SPI 读写过程未改。
- `Application/Storage/imu_bias_storage.c`：新记录版本/配置校验，保留40-byte布局、只追加与提交校验，跳过占用槽。
- `Application/Diagnostics/imu_static_test.c`：正确温度、保存期间重复启动拒绝。
- `Application/System/firmware_info.h`：版本。
- `Tools/tests/imu_spi_probe/stubs/stm32f4xx_hal.h`、`test_mpu6500_polling.c`、`Tools/tests/run_mpu6500_polling_tests.ps1`：扩展 Flash mock，旧无校准测试显式禁用校准。
- 新增 `Tools/tests/imu_spi_probe/test_mpu6500_calibration.c`、`Tools/tests/run_mpu6500_calibration_tests.ps1`、本文和交付日志。
- `docs/ROS2_UBUNTU_HANDOFF.md`：更新当前模式说明。

`Core/Src/freertos.c` 本轮没有改：上一版已经保留条件编译的启动恢复/处理流程，本次宏启用它。已核对 `.ioc`、Core 全部生成文件、CMake、USB、运动/安全、linker 等均未变。

## 软件验证

8 个 native 程序通过：新 MPU6500 校准/存储整合 + 上一版7个采集、身份、USB/SOF/TX/日志测试。新测试直接使用实际 driver/service/static/storage 代码，HAL/RTOS 为 mock，Flash 是进程私有 Windows VirtualAlloc 内存，不打开串口或写真实器件。

覆盖：旧 MPU6050 拒绝、模拟配置签名变化拒绝、格式/大小/CRC/提交/越界拒绝、u32序列回绕、A/B交替、空洞/满区、全部10个word位置失败、解锁失败、readback损坏、模拟冷启动恢复、500唯一计数、单位/温度、重复/运动/owner/保存期间互锁、取消、移动/偏置越界、保存失败仅RAM有效。旧的无校准模式回归单独用 `IMU_CALIBRATION_ENABLED=0` 编译，不代表交付默认模式仍关闭校准。

三个保留模式（MPU6050、identity-only、SPI校准关闭）分别通过5个相关文件的 ARM syntax-only + Werror；不是额外交付的实板镜像。ELF 有 MPU6500/校准/偏置加载保存/HAL_FLASH_Program，未链接旧 MPU6050、identity 探测或 HAL_FLASHEx_Erase。HEX 9940 条校验和通过，范围0x08000000..0x08026CB3，与同 ELF 再次 objcopy 完全相同，不含两区校准数据。静态栈报告已检查，不替代实板栈高水位/Flash写入时序验收。

复跑（无需接硬件）：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File Tools\tests\run_mpu6500_calibration_tests.ps1
```

## 用户实板验收步骤

1. 电机 VM 关闭；保持 STM32/IMU 稳定供电。停止 ROS/键盘等其他运动发送源，避免 USB 多程序占用同一 CDC。
2. 按用户现有串口流程全片擦除并烧录交付目录 HEX，恢复正常 BOOT。本次旧校准会被清除，必须执行一次新的手动校准。无需更改既定烧录工具；不能要求跨全片擦除保留片内记录。
3. CH340 115200/8-N-1 记录启动，确认本节 version/Build。若没有新型号记录，应看到：

```text
IMU bias Flash ready: MPU6500 SPI profile=0x02136185 no compatible saved record
MPU6500 SPI startup: 100Hz polling; explicit calibration, compatible Flash restore enabled
```

4. 水平静止，让模块温度稳定约1分钟。CDC 若仍为 COM4，执行下面命令，期间约5秒完全不要动：

```powershell
python Tools\usb_cdc_test.py --port COM4 --imu-calibration-test
```

该命令会发送一个零速度互锁探测，不请求非零运动。期望 PASS、IMU flags0x37、bias=1、角速度靠近零；CH340 必须出现 `IMU bias Flash saved: ...` 及 `IMU STATIC gyro bias applied and persisted for future boots`，没有 save/verify failed。保存之后仍保留供电几秒，先把校准命令输出和CH340日志发回核对。不要在写入过程中断电或做物理故障注入。

5. 核对保存成功后，再真正断电：断开 STM32 所有供电路径（包括USB DEVICE及CH340可能的供电），确认板指示灯熄灭，再重新接入。仅断开 VM 或仅关闭串口不算断电。
6. 本次不要再运行校准命令；只读：

```powershell
python Tools\usb_cdc_test.py --port COM4 --imu-monitor 10
```

期望 CH340 `IMU gyro bias restored from Flash: raw=[...] calibration_temp=...`，数值匹配上次记录；USB READY/valid=1/bias=1/flags0x07/cal_result=0、约100Hz、errors=0，静止角速度接近零但不要求每个样本恰好0。不应自动出现新的 STATIC started 或 Flash saved。把重启启动日志和监测输出发回。

已验收：0.14.2 实际校准、保存、用户断电重启后的记录恢复、无自动重校准/重复保存、USB100Hz监测。尚未专项验收：写入中途真实掉电、Flash写入期间USB/IWDG/任务最大延迟、运行栈高水位、长期运行及不同温度下偏置变化。若Flash满/失败或bias未恢复，先保存日志；全片擦除会丢失记录，不能作为诊断首选，也不能放宽兼容检查来误加载旧型号偏置。
