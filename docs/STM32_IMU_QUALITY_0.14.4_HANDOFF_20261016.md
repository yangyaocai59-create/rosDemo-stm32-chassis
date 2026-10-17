# MPU6500 采样与校准质量升级：0.14.4

日期：2026-10-16。版本：`0.14.4-imu-drdy-quality`。

状态：**软件候选，尚未由用户烧录和完成本版本实板测试。不是零漂验收通过版。**
本轮没有打开串口、发送硬件指令、校准、复位或烧录设备。固件由用户自行烧录。
仓库没有 Git 元数据，无提交号；以交付包 SHA256 和源码快照追溯版本。

## 1. 目的与不变项

本次改善“读到新样本”和“连续静止校准”的可信度，不把成功传输 100Hz 当成静止零漂已解决。
不等待 ROS 环境即可在 Windows 通过 Python 做采样对照。

保持：当前 `.ioc`、引脚分配、GPIO 初始化、SPI1 模式/速度、量程 ±4g/±500dps、
41Hz DLPF、100Hz ODR、USB CDC、编码器、电机方向/PWM、急停、命令超时、SOF/DTR 断链停车、
Fault Manager、IWDG 策略。未修改 CubeMX 的 `cmake/stm32cubemx/CMakeLists.txt`。
`Core/Src/freertos.c` 和 `stm32f4xx_it.c` 的应用挂接仅在 USER CODE 区；不用重新生成 CubeMX。

没有加入 DMA、FIFO、DMP、姿态解算、在线零偏学习、温度补偿、自动上电校准或自动 Flash 更新。
没有把小角速度硬清零。整数基准/Flash 格式暂不升级，不能宣称整数截断误差已消除。

## 2. 采样设计

MPU6500 接线不变：VCC=3.3V，共地；PA4=NCS，PA5=SCK，PB4=MISO(模块 AD0)，
PB5=MOSI(模块 SDA)，PB0=INT。不可把模块 AD0/MISO 接地。

- `INT_ENABLE=0x01`，只启用 RAW_DATA_RDY；`INT_PIN_CFG=0`，高有效推挽脉冲、上升沿。
- 现有 PB0 EXTI0 优先级 6，FreeRTOS 最大系统调用优先级为 5，可使用 FromISR 通知。
- ISR 仅保存 HAL 毫秒时间/边沿序号并通知 imuTask；不读取 SPI、不打印、不控制电机。
- imuTask 通知唤醒，最长等待 10ms，以维护初始化、超时和心跳；没有边沿时不重复读取旧数据。
- 读取检查 WHO、供电状态、INT_STATUS 的 RAW_DATA_RDY，再进行 14 字节连续读取。
- 取中断邮箱之后重新取当前时间，防止旧 now 与新中断时间相减误判超时。
- 35ms 未见新边沿：样本无效、记录错误并进入已有重试机制；不悄悄退回轮询。
- 超过 15ms 的旧边沿不读；多边沿合并记录 missed；读取期间出现新边沿，丢弃歧义读数并记录 race。
- 每次传感器初始化/恢复递增 acquisition epoch，方便诊断区分采集阶段。

USB `sample_time_ms` 从“SPI 读完时间”改为“对应 DRDY ISR 时间”，单位和字段不变。
它是毫秒分辨率的主机 MCU 事件时间，含中断延迟，不是传感器内部精确转换时刻。
`sample_count` 仍为成功发布的软件样本计数，不是传感器内建硬件序号。
无 FIFO：IRQ 被长时间屏蔽时，EXTI 会合并边沿，missed 只能反映可检测丢失，不能证明物理绝不漏采。

## 3. 校准质量检查

仍由用户/ROS 显式请求采集 500 个样本，成功后应用固定基准并尝试保存。

开始、整个窗口、提交前检查：IDLE、无有效运动命令/控制器/电机诊断/故障/急停、
STBY 禁用、PWM 命令全零、四轮停止且累计计数不变，轮速快照不超过 30ms。
读取守卫不发出任何电机命令。允许校准本身产生的总诊断 inhibit，不允许真正的电机诊断。

采集要求：

- 样本有效且 READY；读取错误、epoch、missed、race 不得改变。
- 软件样本序号连续；相邻样本时间间隔 5..15ms，拒绝倒退/同时间/明显漏采。
- 等待新样本最长 25ms，窗口总时长不超过 6000ms；不能在故障两侧拼接数据。
- 加速度峰峰值上限 500 RAW；陀螺仪峰峰值上限从 300 改为 96 RAW。
- 陀螺仪总体标准差上限 12 RAW；加速度上限 64 RAW；原有约 0.9..1.1g 模长检查保留。
- 方差用 int64 精确分子计算，避免大 DC/1g 附近 float 相减损失精度。
- Cancel 后立即重新 Start 时，使用校准代次防止旧样本/旧完成结果污染新窗口。

阈值是本版工程初值，尚待本模块实板验证；没有照搬飞控的阈值。
标准差和轮子静止仍无法排除整车被外部匀速慢转，因此必须固定车体、无人触碰。
失败或取消保留旧基准、不写新记录。零偏范围仍为每轴 ±1000 RAW。

## 4. 保存状态与兼容

`ImuStaticTestStatus` 新增内部 persistence、已采样数量和时长。
CH340 每秒输出一行，例如：

```text
IMU QUALITY: epoch=1 drdy=1300 missed=2 race=0 timeout=0 cal=2 persist=2
```

| 字段 | 说明 |
|---|---|
| epoch | 传感器采集初始化/恢复代次 |
| drdy | ISR 观察到的边沿总数，包含初始化/维护期间边沿 |
| missed | 可检测的合并/过期事件累计数量 |
| race | 读取期间跨新边沿，丢弃歧义样本的次数 |
| timeout | 缺失 DRDY 触发恢复次数 |
| cal | 0=未执行，1=采集中，2=校准通过，3=运动/噪声失败，4=偏置范围失败，5=取消，6=数据连续性失败 |
| persist | 本次显式校准：0=未请求，1=保存中，2=保存成功，3=保存失败、仅 RAM 生效 |

普通重启恢复基准后 cal=0/persist=0 是正常的；不代表 Flash 无记录。
USB 0x84/0x85 的长度、比例、旧 flag 定义均不变，协议源码未改。
新 FAIL_DATA 在旧 USB 中表现为 result_valid=1、success/moved/range 均为 0；详细原因查看 CH340。
旧 USB calibration success **仍不等于持久化成功**；需要 CH340 `persist=2` 或明确保存成功日志确认。
此版没有添加 USB 机器可读诊断扩展命令，避免未经双方协商改变 ROS 接口。

Flash 仍为 format2、40B A/B 追加日志，不引入擦除。
启用 DRDY 改变了寄存器指纹，因此旧轮询配置的记录不兼容；不直接复用旧基准。
用户规定每次烧录全片擦除，本次刷后必须显式校准一次；以后普通断电重启不需要校准。

校准完成日志、保存等是维护操作，可能造成采样间隔/计数诊断跳变；不要把该窗口当作连续运行验收。
读取错误与恢复、饱和质量、热稳定不应仅凭 bias_valid 判断；bias_valid 只表示当前有基准。

## 5. 已完成的软件验证

- 真实 ARM GCC Debug clean build：92 步，Error=0，Warning=0。
- FLASH=163516 B / 786432 B；RAM=55304 B / 131072 B；CCM=0 B。
- 10 个 native 回归程序通过（`-Wall -Wextra -Werror`）；8 个 Python 单元测试及旧协议 self-test 通过。
- 覆盖中断缺失/邮箱时序竞态/合并/读取重叠/过期边沿/时钟回绕、硬件配置/读取故障、
  连续校准、四轮/实际控制状态守卫、STD 单独超标、取消重开、Flash 写入各阶段失败与重启恢复。
- 旧 USB RAW/legacy 布局、STOP/SOF/DTR/会话、队列背压和日志回归通过（软件模拟，不是实板认证）。
- 任务仍为 2KB IMU 栈，已将完成报告副本移出栈；GCC 单函数栈信息会随包提供。
  静态帧信息不等于最坏调用链/中断嵌套安全证明，实板仍需观察栈余量。
- HEX/BIN 与 ELF 一致性、向量表、固件版本和 Flash 地址边界由打包脚本验证。

软件通过不代表漂移已减小，不代表已烧录，也不代表电机实板安全回归完成。
0.14.3 冻结交付与原始硬件证据未覆盖。

## 6. 用户接下来怎么操作

先关闭电机 VM，保留 STM32/IMU 供电。USB CDC 与 CH340 接电脑；不要运行 ROS 或其他串口程序。
校准和测试不发出非零速度命令，但仍应保持电机 VM 关闭。

1. 用你原来的烧录工具，按原规定全片擦除，烧录本包 `firmware/rosDemo.hex`。
2. 普通启动，确认 CH340 Banner 为 `0.14.4-imu-drdy-quality`，有 `PB0 DRDY enabled`，无反复 DRDY timeout。
3. 等待启动与温度初步稳定，固定车体，保持无人触碰。显式校准一次：

```powershell
python Tools\usb_cdc_test.py --port COM4 --imu-calibration-test
```

保持 CH340 记录以确认 `gyro bias applied and persisted`，之后 QUALITY 应见 `cal=2 persist=2`。
如果校准失败，不要反复盲目重试，先保留原始日志查明 motion/data/STD 原因。

4. 关闭串口助手，让脚本独占 CH340，校准维护结束后等数秒，再做 120 秒只读采样：

```powershell
python Tools\imu_raw_stream_test.py --port COM4 --debug-port COM3 --duration 120 --vm-off-confirmed --stationary-confirmed --require-bias --require-quality
```

端口是示例；用实际设备管理器的 CDC/CH340 端口替换，不要把两者接反。
脚本只发送 GET_STATUS/SET_IMU_STREAM，无校准、故障清除、运动或刷机命令。
它会切换 CDC DTR 测试会话恢复；不主动对 CH340 发复位/下载脉冲。

5. 真正断开所有供电（包括可能回供电的 USB）后重新上电，**不要再校准**，重复上面的 120 秒只读采样。
6. 对照 RAW 基准是否恢复、读取错误、质量计数增量、温度、未量化的偏航积分结果。
   `--require-quality` 要求观测窗口内质量计数不增加、epoch 不变和 RAW 连续；启动/维护历史计数不要求为零。
   稀疏 UART 不等于无损边沿跟踪，结果仍以原始记录与完整接收分析为准。

先完成以上静止/恢复测试，再单独安排固定电机安全条件下的慢转测试；本轮不自动让车运动。

## 7. ROS 工程师须知与后续工作

RAW 0x85 仍按 `(raw - fixed_bias) / 65.5 * pi/180` 得 rad/s，double 运算。
沿用 +X 前、+Y 左、+Z 上，轴向不要二次变换。时间戳需处理 uint32 回绕及设备恢复。
不要把固定偏置恢复正确理解为残余零偏为零；不把 IMU 积分 yaw 伪装成绝对航向。

后续仍推荐 ROS 单一责任模块进行可信静止残余零偏估计，运动时冻结，保留未校正数据和校正数据，
不要两路作为独立观测重复融合。若决定改由 STM32 学习，必须先双方确认所有权和新接口，不能双重补偿。
温度补偿须有跨温度、重复冷启动的独立数据；浮点基准存储/协议精度升级须显式版本兼容设计。

参考思路而非代码移植：

- [Betaflight MPU6500 DATA READY](https://github.com/betaflight/betaflight/blob/master/src/main/drivers/accgyro/accgyro_mpu6500.c)
- [Betaflight 校准标准差与均值](https://github.com/betaflight/betaflight/blob/master/src/main/sensors/gyro.c)
- [ArduPilot Invensense 样本管理](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_InertialSensor/AP_InertialSensor_Invensense.cpp)
- [ArduPilot 温度补偿：后续参考，未在此版实现](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_InertialSensor/AP_InertialSensor_tempcal.cpp)

## 8. 修改文件

固件：`Application/Sensors/{mpu6500.c,mpu6500.h,imu_config.h,imu_service.c,imu_service.h}`；
`Application/Diagnostics/{imu_static_test.c,imu_static_test.h,imu_calibration_guard.c,imu_calibration_guard.h}`；
`Application/System/firmware_info.h`；`Core/Src/{freertos.c,stm32f4xx_it.c}`；顶层 `CMakeLists.txt`。

工具/文档：`Tools/imu_raw_stream_test.py`、`imu_quality_analysis.py`、`package_imu_quality_release.py`；
`Tools/tests/run_imu_quality_tests.ps1`、`test_imu_quality.py`；
`Tools/tests/imu_spi_probe/{test_imu_calibration_guard.c,test_imu_spi_probe.c,test_mpu6500_calibration.c,test_mpu6500_polling.c,stubs/FreeRTOS.h}`；
本说明与 `docs/ROS2_UBUNTU_HANDOFF.md` 的候选版入口。
