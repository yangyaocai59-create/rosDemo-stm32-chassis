# SPI IMU 身份探测：0.14.0 临时测试固件

日期：2026-10-14，Asia/Shanghai。当前阶段仅读取芯片身份，未完成新 IMU 驱动，不能作为 ROS 正式运行固件。

## 身份与真实构建

```text
Firmware version = 0.14.0-spi-imu-id-probe
Build = Oct 14 2026 06:03:38
HEX SHA256 = 85046A4215E7E3D82430F329353B4A8EA819F224DB51346D1ED2A588DF1BD6BF
ELF SHA256 = 873B29ED2647C460093AF69409EC245F118D7D56F83464B2E34ABB1979D3A5EA
```

真实 ARM Debug configure + clean build 90 步成功：Error=0、Warning=0。
FLASH=150064 B / 768 KiB（19.08%），RAM=54848 B / 128 KiB（41.85%），CCMRAM=0 B / 64 KiB。
GCC 14.3.1、CMake 4.3.1、Ninja 1.13.2；尚未自动烧录或实板采集。
没有 Git 元数据，不提供虚构提交号。以镜像/源码快照 SHA256 与实际启动 version/Build 关联。

## 当前接线与配置

用户已拆掉 MPU6050，接上新模块，实测模块 VCC 对模块 GND=3.3 V。
照片丝印“9250/6500/9255”不能确定芯片型号；身份字节只表示兼容型号候选，不证明芯片真伪。

| 模块引脚 | STM32 |
| --- | --- |
| VCC | 3.3 V，未确认模块 5 V 兼容性，不接 5 V |
| GND | GND，共地 |
| SCL / SCLK | PA5，SPI1_SCK |
| SDA / SDI | PB5，SPI1_MOSI |
| AD0 / SDO | PB4，SPI1_MISO；不能接地 |
| NCS | PA4，MPU6500_CS，软件片选，初始 HIGH |
| INT | PB0，IMU_INT，已有上升沿 EXTI0 / NVIC 6 |
| FSYNC | GND，暂不使用同步 |
| EDA、ECL | 不接，辅助接口 |

CubeMX 不需修改：SPI1 master/full duplex、8-bit、MSB-first、Mode 0、软件 NSS、prescaler=128，APB2 84 MHz 下约 656.25 kHz。既有 I2C1 配置保留。
没有新增/重复定义生成的 GPIO 宏；生产代码使用 main.h 的 CS 和 INT 宏。

## 本版做什么、不做什么

- `imuTask` 启动后异步等待 100 ms（不是 HAL_Delay），每约 1 s 最多一次 SPI WHO_AM_I 读取；命令 `0x75 | 0x80 = 0xF5`，发送命令+dummy 两字节，使用第二个接收字节。
- HAL polling 超时参数 5 ms，片选覆盖整个命令/响应；HAL_OK/ERROR/BUSY/TIMEOUT 后都释放片选。当前 SPI1 由本任务独占，日后共用总线要另行设计仲裁。
- `0x70` / `0x71` / `0x73` 候选分别 MPU6500 / MPU9250 / MPU9255；需连续三次成功且同一已知 ID，日志才置 `stable=1`。失败、未知 ID、型号变化会撤销成功状态，不能把 `HAL=OK` 单独认定为识别成功。
- 前三次及状态变化时打印，随后约 5 s 输出一次诊断；HAL 状态、读次数、错误/未知次数、CS 和 INT 引脚电平均可见。
- 没有写传感器寄存器，不复位/唤醒/配置采样率，不启用数据就绪中断；INT 只是读瞬时电平，不代表中断功能或传感器 DRDY 已验收。
- 没有六轴、温度或姿态数据；`stable=1` 不是正式 `ImuService READY`。
- IMU 服务快照保持未初始化/无有效数据/无 bias；USB v1 格式仍保留，但 IMU_STATE 的 valid=0、bias=0、sample_count=0 是这版的预期行为，不能用旧 --imu-monitor 100 Hz 验收它。
- 不启动旧 I2C MPU6050 服务，不加载/保存旧 bias，不扫描 journal。原文件保留以便维护；USB/UART 校准因没有有效样本被拒绝。
- IMU 任务继续正常上报 heartbeat 和 osDelayUntil；IWDG arbiter、USB TX/SOF 修复、编码器、电机控制、安全机制均保留。
- 没有新增任何电机启动或速度指令，但原 USB/UART 运动入口仍存在，**测试时 VM 必须关闭，禁止 ROS/键盘速度程序同时连接**。这不是全系统电机锁定镜像。

## 修改文件

新增：

- `Application/Sensors/mpu_spi_probe.c`、`.h`
- `Application/Diagnostics/imu_spi_identity_test.c`、`.h`
- `Tools/tests/imu_spi_probe/`（实际生产代码 host tests 和独立模拟头文件）
- `Tools/tests/run_imu_spi_probe_tests.ps1`
- 本说明及 build 下构建、测试、校验日志。

修改现有生产文件：

- `CMakeLists.txt`：加入两项应用源文件，不改 CubeMX CMake。
- `Application/Sensors/imu_config.h`：显式 `IMU_SPI_ID_PROBE_ONLY=1`。
- `Application/Board/board_config.h`：启动映射随测试模式明确显示 SPI probe。
- `Application/System/firmware_info.h`：新测试版本标识。
- `Core/Src/freertos.c`：只在 USER CODE 区切换 imuTask 路径、停止旧 IMU 日志；保留心跳和其他业务路径。

已与冻结 0.13.3 源码逐项哈希核对，上述五项外的已有生产/硬件配置未改。
`.ioc`、SPI/GPIO/NVIC 生成代码、USB 协议及传输源码、驱动、方向/PID/几何、安全、Flash 存储源码和 linker 未改；freertos.c USER CODE 外文本一致。

## 验证结果与边界

- 新 native 回归使用实际 SPI probe/诊断状态机、实际 ImuService 快照与 ImuStaticTest 校准入口；HAL/RTOS/日志/旧传感器/存储为模拟。
- 检查准确 `F5 00` 读命令、两字节全程片选、错误路径释放、未知/00/FF/68 拒绝、连续三次识别、型号变化、断连/恢复、tick 回绕、延迟后不 burst catch-up；校准拒绝且 I2C/Flash mock 调用数为零。
- 既有 5 项 USB/SOF/TX/日志 native 回归再次通过，包括 50 轮生产 parser STOP/GET/restart 与真实 51 ms 超时/DTR 不回放模拟。
- `IMU_SPI_ID_PROBE_ONLY=0` 保留分支通过 ARM compiler syntax-only 检查；未为该分支构建/交付另一镜像。恢复生产基线请用已有冻结 0.13.3，而不是直接拿身份测试 tag 运行 ROS。
- 链接后 ELF 符号检查：有新 probe/state-machine 符号，没有旧 Mpu6050 初始化/采样、ImuBiasStorage、HAL_FLASH_Program/HAL_FLASHEx_Erase 符号。
- HEX 9388 条记录校验和通过；数据 `0x08000000..0x08024A2F`，不覆盖 sector 10/11；由同 ELF 再 objcopy 的 HEX 哈希相同。
- 本版尚未实际烧录和读取硬件 WHO_AM_I，因此不能声称 SPI 通信或 INT 实板测试通过。静态栈帧检查不是实际栈高水位验收。

复跑离线测试（不连接串口）：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File Tools\tests\run_imu_spi_probe_tests.ps1
```

## 用户现在如何测试

1. 电机 VM 关闭、无其他运动命令源。保持已确认 3.3 V 接线，禁止带电重插线。
2. 关闭占用 CH340 的串口助手，用此前串口下载流程烧录冻结目录中的 `rosDemo.hex`，不要误选 0.13.3 或旧 Release。
3. 保留校准记录时不能全片擦除或擦除 sector 10/11（`0x080C0000..0x080FFFFF`）。本镜像不写该区，但下载工具擦除设置仍需核查。
4. 回正常启动模式，打开 CH340 串口助手 115200、8-N-1，再复位一次，确认本说明的 version/Build。
5. 等约 3 s，看 `SPI IMU ID`。MPU6500 的示例（不是已采集的实板结果）：

```text
Firmware version = 0.14.0-spi-imu-id-probe
Build = Oct 14 2026 06:03:38
SPI IMU identity-only test: I2C6050 inactive; no six-axis data, calibration or Flash writes
SPI IMU ID: WHO_AM_I=0x70 candidate=MPU6500 HAL=OK streak=1/3 stable=0 reads=1 errors=0 unknown=0 CS=HIGH INT=0
SPI IMU ID: WHO_AM_I=0x70 candidate=MPU6500 HAL=OK streak=2/3 stable=0 reads=2 errors=0 unknown=0 CS=HIGH INT=0
SPI IMU ID: WHO_AM_I=0x70 candidate=MPU6500 HAL=OK streak=3/3 stable=1 reads=3 errors=0 unknown=0 CS=HIGH INT=0
```

INT=0/1 不是本轮身份判据。请发完整启动至约 10 s 的串口日志，无需转动车体或连接电机。
若 HAL=OK 但 WHO=00/FF/UNKNOWN/stable=0，先断电核对 CS/MISO/MOSI/SCK/共地及模块供电；若 HAL=TIMEOUT/BUSY/ERROR，保存完整日志继续排查，不更改 CubeMX 引脚或盲目提高频率。

身份确认后再安排六轴采样、正确温度公式/轴向、INT 数据就绪及新型号独立校准身份；它们不包含在本版内。不要复用 MPU6050 的校准记录。

## 寄存器依据（厂家原始文档）

- [TDK MPU6500 SPI 产品规格](https://invensense.tdk.com/wp-content/uploads/2020/06/PS-MPU-6500A-01-v1.3.pdf)：SPI 读位、两字节传输、CS 时序及接口限制。
- [MPU6500 register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6500-Register-Map2.pdf)：WHO_AM_I 0x75，值 0x70。
- [MPU9250 register map](https://invensense.tdk.com/wp-content/uploads/2017/11/RM-MPU-9250A-00-v1.6.pdf)：WHO_AM_I 值 0x71。
- [InvenSense MPU9255 RM-000008 原文，Stanford 存档](https://stanford.edu/class/ee267/misc/MPU-9255-Register-Map.pdf)：WHO_AM_I 值 0x73。
