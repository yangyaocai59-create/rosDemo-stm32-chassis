# rosDemo 四轮差速底盘：STM32 固件与 ROS2 Ubuntu 开发交接文档

> 2026-10-17 最新软件候选：[0.16.0 小数精度偏置与 RAW v2](STM32_IMU_FRACTIONAL_BIAS_0.16.0_HANDOFF_20261017.md)。校准/运行/Flash 统一为 milli-RAW，format 3 记录；新增 mode 2 / 0x86 精确诊断，旧 RAW v1 仅保留整数兼容视图。ROS 使用 RAW 时需升级 selector/decoder；默认 SI 布局不变。包含 0.15.2 修复，不必先刷中间版本。尚未烧录/实板验证，不承诺真实零漂已降低。

> 2026-10-17 最新软件候选：[0.15.2 IMU 新样本驱动遥测](STM32_IMU_SAMPLE_TELEMETRY_0.15.2_HANDOFF_20261017.md)。修复独立 10 ms 重发时钟导致的重复/跳样机制，数据布局与偏置不变；尚待烧录和板上连续性验收，不代表零漂改善。历史 0.15.1 重启后约 30 s RAW 残差为 −0.444°/min；新增离线复核工具。整数偏置截断的精度限制需另行设计，未偷偷改成小数或强制归零。

> 2026-10-17 栈余量更新：[0.15.1 safetyTask 栈余量修正](STM32_SAFETY_STACK_0.15.1_HANDOFF_20261017.md)。针对 0.15.0 实板静态最低余量 216 B，减少安全调用链快照占栈；不修改 CubeMX 栈配置、运动参数或协议。其后实板只读窗口测得最低余量 352 B，10..10 ms、超期为零，并已验证校准保存和断电恢复；证据在 `build/hardware_tests/safety_stack_0.15.1_static_20261017_060003_603551/` 及 `cold_restore_0.15.1_20261017_060512_265678/`。冻结的原交付说明保留当时软件候选状态；静态结果不替代运动或异常路径验收。

> 2026-10-17 最新运动候选：[0.15.0 运动平顺性修正](STM32_MOTION_SMOOTHNESS_0.15.0_HANDOFF_20261017.md)。心跳时间竞态、控制日志/周期、PWM 换向与正常目标曲线已修正；新增停车后 CH340 `t` 控制轨迹。USB 帧/引脚/IMU 驱动及前馈 PI 参数不变。软件验证通过不等于实车顿挫已消除；请先看候选状态和实板验收步骤。

> 2026-10-16 新候选：[0.14.4 MPU6500 DATA READY 与校准质量升级](STM32_IMU_QUALITY_0.14.4_HANDOFF_20261016.md)。真实构建及软件回归通过，尚未烧录/实板验收；不能代替下文 0.14.3 的已验接口证据，也不表示零漂已解决。USB 帧不变，采样时间改用 DRDY ISR 时间；刷后须按全片擦除条件重新显式校准一次。新旧版本状态请勿混淆。

文档日期：2026-09-25  
本文原始基线版本：`0.13.1-stage13-imu-bias-flash`  
目标上位机：Jetson Orin Nano / Ubuntu 22.04 / ROS2 Humble  
MCU：STM32F407ZGT6  
开发板：正点原子 M144Z-M4  

> 这是一份面向新工作空间、新会话和 ROS2 开发者的自包含交接文档。
> 早期讨论中的旧引脚表、PC0 电池 ADC 和 USART2 调试口均已失效。
> **当前 `rosDemo.ioc` 与当前实际生成代码是唯一硬件依据。**

> 2026-10-12 历史更新：当时交付/板上版本为 `0.13.3-usb-tx-burst-fix`，
> Build `Oct 12 2026 08:24:27`。USB STOP 修复、实板证据、未验证项及
> ROS 接口注意事项以 [最新工程师交接](STM32_ROS_ENGINEER_FINAL_HANDOFF_20261012.md)
> 为准。本文其余章节保留硬件和原始功能基线，不代表所有最新回归均已验收。

> 2026-10-14：新增 [0.14.0 SPI 身份探测固件](STM32_SPI_IMU_ID_PROBE_20261014.md)，
> 是用户换 IMU 后的临时测试镜像，不输出有效 IMU 数据，不替代 0.13.3 ROS 稳定交付。

> 随后新增 [0.14.1 MPU6500 SPI 六轴采集测试](STM32_MPU6500_SPI_POLLING_20261014.md)：
> 当时源码默认该模式，目标 100 Hz，输出未校准六轴/温度，禁止旧偏置恢复/校准/Flash 写入。
> 当时实板 WHO=0x70 已确认，六轴/轴向待验收；后续结果见下面的0.14.2更新。

> 后续实板 0.14.1 静止/左转右转/抬车头抬左侧已验证基础采集与轴向。
> 当时源码默认 [0.14.2 MPU6500 校准及 Flash 恢复](STM32_MPU6500_BIAS_FLASH_HANDOFF_20261014.md)，
> 保留 USB v1 接口，但新记录 format2 按型号/配置校验，绝不复用 MPU6050 偏置。
> 已真实编译及离线回归，0.14.2 实板校准/保存/重启恢复与USB100Hz已通过。
> 当时板上版本为 `0.14.2-spi-mpu6500-bias-flash`，Build `Oct 14 2026 18:26:57`；当前状态见 2026-10-16 更新。
> 每次重新烧录固定全片擦除，因此烧录后校准一次；普通断电/复位不用重校准。
> 重启后IMU flags0x07、bias1、cal_result0是正常恢复，不应每次开机强制校准。
> 旧基线章节不能代替最新说明，历史镜像/源码内旧文档也不代表当前状态。

## 1. 给后续开发者或智能体的硬性约束

> 2026-10-16 当前更新：[0.14.3 高精度 IMU RAW 接口](STM32_IMU_RAW_V1_HANDOFF_20261016.md) 已真实构建、由用户烧录，并通过 RAW 长采样/兼容/DTR/会话及显式校准保存/普通断电恢复实测。保存/恢复基准 [-55,26,150]、保存 seq=1、重上电 flags=0x07。当前镜像 Build Oct 16 2026 09:56:40；镜像身份依据用户报告及接口实测，未全量回读 Flash。新增可协商 0x85，旧 0x84 默认保留；不在 STM32 做自动零偏追踪，ROS 为唯一残余估计方，估计器与 ROS 联调待完成。重上电 RAW 残差约 +2.29439°/min，**未宣称 <=1°/min 已实现**。完整记录见 [新版本实板验收](STM32_IMU_RAW_HARDWARE_VERIFICATION_20261016.md)，方案见 [统一分工](IMU_DRIFT_CORRECTION_PLAN_20261016.md)。以最终 INTERFACE_VERIFIED 包为准，历史文档不代表最新验收状态。

1. 不得根据早期讨论重新分配引脚。
2. 不得绕过 CubeMX 直接修改由 CubeMX 管理的外设配置。
3. 引脚、时钟、定时器、ADC、I2C、SPI、USART、USB 和 IWDG 配置以
   `rosDemo.ioc`、`Core/Src/*.c`、`Core/Inc/main.h` 为准。
4. 不要破坏 CubeMX 生成目录结构和 `USER CODE` 区域。
5. 应用代码放在 `Application/`，用户维护的源文件通过顶层
   `CMakeLists.txt` 加入；不要手工修改
   `cmake/stm32cubemx/CMakeLists.txt`。
6. 当前第一版有意保留 ADC、I2C、SPI、UART polling，不引入 DMA。
7. D24A 实物只有一个公共 STBY，连接 PD10。PD11 是保留输出，未连接，
   必须保持低电平。
8. STM32 与 ROS2 的正式通信接口是 USB CDC；USART1/CH340 仅用于调试、
   串口下载及人工诊断。
9. 当前协议已经过实板验证。扩展协议应优先新增消息类型，避免改变已有
   消息的长度、字段偏移和语义。
10. 电机控制、安全逻辑和看门狗属于 MCU 职责；ROS2 不能替代 MCU 的
    250 ms 命令超时、急停和故障停机。

## 2. 当前完成状态

STM32 侧已经形成可供 ROS2 集成的稳定基线：

- 四路正交编码器读取、方向归一化、回绕处理和 `int64` 累计计数；
- 四路 TB6612 电机方向与 TIM8 PWM 驱动；
- 四轮速度估计、低通滤波、前馈加 PI 闭环；
- 四轮差速运动学，输入为 `linear_x` 与 `angular_z`；
- 生产运行状态机、命令所有权和 250 ms 命令超时；
- USB CDC v1 二进制协议、CRC16、DTR 会话、SOF 物理断线检测；
- E-stop、任务心跳、轮子堵转检测、Fault Manager 和 IWDG 仲裁；
- D24A VM 电压采样与 USB 遥测；
- MPU6050 100 Hz 采样、静态陀螺仪零偏、ROS 坐标映射与 USB 遥测；
- Debug 构建、HEX 生成和 Windows 端参考测试工具。

当前可以开始 ROS2 驱动开发，但 MCU 尚不是最终发布版。剩余事项见本文
“已知限制与后续工作”。

## 3. 硬件组成与供电

### 3.1 主要器件

| 部件 | 当前型号/说明 |
|---|---|
| MCU/开发板 | STM32F407ZGT6，正点原子 M144Z-M4 |
| 电机驱动 | D24A 四路 TB6612 驱动板，一个公共 STBY |
| 电机 | 4 × WHEELTEC MG513P30-12V，30:1，霍尔 AB 编码器 |
| 轮胎 | 直径 65 mm |
| IMU | 当前 GY-521 / MPU6050；保留 SPI1 MPU6500 接口 |
| 电池 | 标称 3S1P、12 V、4500 mAh 锂电池包 |
| 电机电源 | 电池经可调 DC-DC 后接 D24A VM，当前约 11.75 V |
| 调试接口 | 板载 CH340 对应 USART1 |
| 正式上位机接口 | USB OTG FS Device / CDC ACM |

### 3.2 关键接线和电气说明

- STM32、D24A、MPU6050 和电源系统必须共地。
- D24A `VM` 是电机电源输入，目前万用表约 11.75 V。
- D24A `ADC` 是 VM 经板载约 1:11 分压后的模拟输出，连接 STM32 PC1。
- 不需要把 D24A 的 3.3 V 输出接到 STM32 3.3 V；两块板只需正确共地，
  信号电平必须兼容 3.3 V。
- M144Z-M4 的 `VREF+` 必须按板卡设计接到 3.3 V/VDDA。曾经因 VREF+
  断开导致 ADC 读数异常；当前跳线已经修正。
- MPU6050 使用 3.3 V 供电，AD0 接地，因此 7 位 I2C 地址为 `0x68`。
- 当前 MPU6050 安装方向：传感器 +X 向车前、+Y 向左、+Z 向上。
- D24A 实物只有一个 STBY，接 PD10；PD11 不接 D24A，固件始终将其置低。
- ESTOP 输入 PC13 为低有效，内部上拉，下降沿 EXTI。触发后锁存，必须
  MCU 复位才解除。

## 4. 最终引脚与外设映射

### 4.1 轮子名称

| 缩写 | 含义 |
|---|---|
| LF | Left Front，左前 |
| LR | Left Rear，左后 |
| RF | Right Front，右前 |
| RR | Right Rear，右后 |

USB 协议和所有数组中的轮子顺序固定为：`LF, LR, RF, RR`。

### 4.2 编码器

| 轮子 | 定时器 | A/CH1 | B/CH2 | 计数器宽度 | CH1/CH2 滤波 |
|---|---|---|---|---:|---:|
| LF | TIM1 | PE9 | PE11 | 16 bit | 6 / 6 |
| LR | TIM2 | PA15 | PA1 | 32 bit | 6 / 6 |
| RF | TIM3 | PA6 | PA7 | 16 bit | 6 / 6 |
| RR | TIM4 | PD12 | PD13 | 16 bit | 6 / 6 |

四个定时器均使用 `TIM_ENCODERMODE_TI12`、双通道上升沿、Prescaler=0。
固件对 16/32 位计数回绕做差分处理，并输出连续 `int64` 累计计数。

### 4.3 电机 PWM、方向和 STBY

| 轮子 | PWM | PWM 引脚 | IN1 | IN2 |
|---|---|---|---|---|
| LF | TIM8_CH1 | PC6 | PD0 `LF_IN1` | PD1 `LF_IN2` |
| LR | TIM8_CH2 | PC7 | PD2 `LR_IN1` | PD3 `LR_IN2` |
| RF | TIM8_CH3 | PC8 | PD4 `RF_IN1` | PD5 `RF_IN2` |
| RR | TIM8_CH4 | PC9 | PD6 `RR_IN1` | PD7 `RR_IN2` |

| 信号 | 引脚 | 当前语义 |
|---|---|---|
| `MOTOR_STBY_1` | PD10 | D24A 唯一公共 STBY，实际使用 |
| `MOTOR_STBY_2` | PD11 | 保留且未连接，保持 LOW |

TIM8 时钟为 168 MHz，Prescaler=0，ARR=8399，因此 PWM 为 20 kHz。
PWM1 极性高，初始 CCR 全部为 0。

### 4.4 通信、传感器和安全引脚

| 功能 | 外设/引脚 | 说明 |
|---|---|---|
| 调试 TX/RX | USART1 PA9/PA10 | CH340，115200 8-N-1，无流控 |
| USB CDC | PA11 DM / PA12 DP | USB OTG FS Device，Jetson 正式接口 |
| 电压 ADC | ADC1_IN11 / PC1 | D24A VM 约 1:11 分压 |
| MPU6050 I2C | I2C1 PB6 SCL / PB7 SDA | 100 kHz，polling |
| IMU INT | PB0 | 当前配置为上升沿 EXTI，但 MPU6050 polling 版本未使用 |
| MPU6500 CS | PA4 | 保留 SPI MPU6500，默认拉高 |
| MPU6500 SCK | PA5 / SPI1_SCK | 保留 |
| MPU6500 MISO | PB4 / SPI1_MISO | 保留 |
| MPU6500 MOSI | PB5 / SPI1_MOSI | 保留 |
| ESTOP | PC13 | 低有效、内部上拉、下降沿 EXTI、复位锁存 |

SPI1 当前为 Master、全双工、Prescaler=128，约 656.25 kbit/s；仅保留，
当前 IMU 驱动不使用 SPI1。

### 4.5 方向极性

应用层统一规定：**正数永远表示轮子物理向前**。

| 轮子 | 电机正向极性 | 编码器正向极性 |
|---|---:|---:|
| LF | +1 | +1 |
| LR | -1 | +1 |
| RF | -1 | -1 |
| RR | +1 | -1 |

LR 与 RF 的电机电气方向相对应用逻辑反向，RF 与 RR 的编码器原始计数
方向相对应用逻辑反向。不要在 ROS2 中再次按左右侧盲目取反；USB 输出的
计数和速度已经归一化为“物理向前为正”。

## 5. 时钟、CubeMX 和基础外设

- HSE：8 MHz。
- SYSCLK/HCLK：168 MHz。
- APB1：42 MHz，APB1 timer：84 MHz。
- APB2：84 MHz，APB2 timer：168 MHz。
- USB 48 MHz 时钟有效。
- ADC1：12 bit、右对齐、软件触发、单通道、144 cycles、无 DMA。
- I2C1：100 kHz Standard Mode、7 bit、无 DMA。
- UART1：115200 8-N-1、polling 日志、无 DMA。
- USB：Device Only、CDC、self-powered，VBUS sensing 关闭。
- USB 描述符仍为 ST 通用值：VID `0x0483`，PID `0x5740`；没有自定义
  VID/PID。
- STM32Cube FW_F4：V1.28.3。

IWDG 当前已经正式启用：Prescaler=64、Reload=1999、LSI 名义 32 kHz，
超时约 4 s。只允许 `safetyTask` 在所有任务健康时每 250 ms 仲裁刷新，
不存在任意任务无条件喂狗。使用断点长时间暂停程序时 MCU 可能被 IWDG
复位，这是当前预期行为。

## 6. 机械与编码器参数

| 参数 | 当前值 | 状态 |
|---|---:|---|
| 电机编码器 PPR | 13 | 型号资料/实测采用值 |
| 正交倍频 | 4 | TIM encoder TI12 |
| 减速比 | 30:1 | MG513P30 |
| 每轮一圈计数 | 13 × 4 × 30 = 1560 | 固件固定值 |
| 轮径 | 65 mm | 名义/初始测量值 |
| 轮半径 | 32.5 mm | 由轮径计算 |
| 左右轮中心距 | 210 mm | 初始实测值，仍需地面标定 |
| 前后轮中心距 | 180 mm | 结构信息，差速公式当前不使用 |

派生值：

```text
wheel_circumference = pi * 0.065 = 0.2042035 m
metres_per_count    = wheel_circumference / 1560
                    ≈ 0.0001308997 m/count
radians_per_count   = 2*pi / 1560
                    ≈ 0.00402768 rad/count
counts_per_metre    ≈ 7639.44 count/m
```

轮径和轮距是初始值。装载重量、轮胎压缩、地面滑移会导致系统误差，ROS2
实车完成后应分别做直线距离和原地旋转标定。

## 7. 底盘运动学和控制约定

ROS/固件共同坐标约定：

```text
+X：车体向前
+Y：车体向左
+Z：车体向上
+angular_z：逆时针/左转
```

四轮差速采用左右两侧等速模型：

```text
v_left  = vx - wz * track / 2
v_right = vx + wz * track / 2

LF = LR = v_left
RF = RR = v_right
```

目标轮速再通过 `counts_per_metre` 转为 counts/s。如果任一侧超过
1600 counts/s，固件按相同比例缩放两侧，保持曲率不变。

生产命令边界：

| 项目 | 限制 |
|---|---:|
| `linear_x` | -0.20 .. +0.20 m/s |
| `angular_z` | -1.50 .. +1.50 rad/s |
| 轮目标 | ±1600 counts/s |
| 速度命令超时 | 250 ms |
| PWM 闭环输出 | ±300 permille，即当前最多约 30% duty |
| 控制输出斜率 | 1500 permille/s |
| 控制周期 | 10 ms |

### 7.1 速度估计

- 编码器/速度更新周期：10 ms。
- 一阶低通时间常数：50 ms。
- 无计数变化 200 ms 后判定停止。
- 同时保存 raw cps、filtered cps、RPM、rad/s 和 m/s。

### 7.2 当前 PI 与前馈

这些参数来源于当前电机和空载/初步落地测试，后续带载仍可重新标定：

```text
Kp = 0.050 permille/(count/s)
Ki = 0.500 permille/(count/s*s)
integrator limit = ±80 permille
output limit = ±300 permille
slew = 1500 permille/s
pivot feedforward boost = ±80 permille
```

各轮正反向前馈模型：

```text
output_permille = sign * (static + gain * abs(target_cps))
```

| 轮子 | Forward static | Forward gain | Reverse static | Reverse gain |
|---|---:|---:|---:|---:|
| LF | 51.8 | 0.14600 | 46.3 | 0.14677 |
| LR | 45.0 | 0.14934 | 50.4 | 0.14667 |
| RF | 54.7 | 0.14703 | 56.8 | 0.14249 |
| RR | 58.9 | 0.15779 | 56.9 | 0.15108 |

控制器具有积分限幅、饱和/斜率 anti-windup、换向清积分、过速保护和
从诊断输出到生产闭环的受控接管。

## 8. FreeRTOS 任务结构

| 任务 | 优先级 | 栈 | 周期/行为 | 主要职责 |
|---|---|---:|---|---|
| `systemInitTask` | AboveNormal | 1024 B | 单次后退出 | 初始化 USB、标记系统初始化完成 |
| `motorTask` | High | 2048 B | 10 ms | 编码器、速度估计、底盘闭环和电机诊断 |
| `safetyTask` | High | 1024 B | 10 ms | E-stop、安全状态、Fault Manager、IWDG |
| `imuTask` | AboveNormal | 2048 B | 10 ms | MPU6050 100 Hz 采样和静态校准取样 |
| `usbRxTask` | AboveNormal | 2048 B | 2 ms | USB 流解析、命令分发和遥测发送 |
| `telemetryTask` | Normal | 2048 B | 200 ms | CH340 文本日志，不参与硬实时采样 |
| `batteryTask` | Low | 1024 B | 100 ms | ADC 轮询、电压滤波和状态 |

Stage 12D.1 已将周期 IMU UART 日志从 `imuTask` 移到低优先级
`telemetryTask`，实测 USB IMU 样本稳定为 100.0 Hz、间隔 10..10 ms。

## 9. 电机安全和故障模型

### 9.1 上电与异常安全状态

上电早期、`Error_Handler`、Cortex fault、E-stop 和关键故障路径都会：

```text
PD10 STBY = LOW
PD11 reserved = LOW
所有 IN1/IN2 = LOW
TIM8 CCR1..CCR4 = 0
TIM8 MOE disabled（紧急路径）
```

电机驱动初始化时可以启动 TIM8 四个 PWM 通道，但 CCR 保持 0，且 D24A
STBY 仍为 LOW。只有运行状态机、安全检查和 E-stop 条件全部允许时才将
PD10 拉高。

### 9.2 E-stop

- PC13 低有效，GPIO pull-up，EXTI falling。
- ISR 直接执行寄存器级电机紧急关断。
- 软件锁存后拒绝再次启用电机。
- 必须让输入恢复高电平并复位 MCU；普通 clear-fault 命令不能清除。

### 9.3 Fault Manager

启动宽限：3000 ms。监督任务超时：

| 任务 | 超时 |
|---|---:|
| motor | 250 ms |
| safety | 100 ms |
| usbRx | 250 ms |
| telemetry | 1000 ms |
| IMU | 2500 ms |
| battery | 2500 ms |

运行时直接视为关键任务故障的是 motor 和 usbRx；IWDG 刷新资格则要求
全部六个任务心跳均正常并且 systemInit 完成。

轮堵转检测只作用于生产运行，不作用于 IDLE 或调试诊断：

```text
运动启动宽限       1000 ms
连续异常确认       750 ms
最低目标速度       200 counts/s
最低控制输出       100 permille
实际/目标速度比阈值 15%
```

任一轮满足堵转条件会停止全部电机并锁存健康故障。

### 9.4 IWDG

- 名义超时约 4 s。
- `safetyTask` 是唯一刷新者。
- 刷新周期 250 ms。
- systemInit 未完成、任一任务未出现、任一任务过期或刷新失败时停止喂狗。
- 看门狗复位原因在下一次启动 Banner 中可见。

## 10. 电压监测

当前 ADC 路径：

```text
D24A VM -> D24A 10k/1k 分压输出 ADC -> STM32 PC1/ADC1_IN11
```

配置：

- 12 bit，Vref 名义 3300 mV；
- 每 100 ms 采集 16 次并平均；
- 一阶 IIR：`filtered += (sample - filtered) / 8`；
- 分压比按 11:1；
- 当前校准系数 1000/1000；
- 实测 VM 约 11.75 V，固件通常报告约 11.6 V。

**重要限制：**PC1 测到的是 DC-DC 调整后的 D24A 电机母线，不是 DC-DC
之前的 3S 电池原始电压。因此无法可靠判断电池 SOC 或过放，当前状态为
`MONITOR_ONLY`，低压/严重低压/恢复阈值均为 0，电池保护关闭。ROS2 可以
显示“motor bus voltage”，但不应把它当成真实电池百分比。

## 11. IMU

### 11.1 当前硬件和采样

- GY-521 / MPU6050，I2C 地址 `0x68`；
- I2C1 100 kHz、blocking polling；
- 采样 100 Hz；
- DLPF 44 Hz；
- 加速度量程 ±4 g，8192 LSB/g；
- 陀螺仪量程 ±500 deg/s，65.5 LSB/(deg/s)；
- INT 当前未用于数据就绪；
- 连续 3 次 I2C 失败进入恢复流程，每 1 s 重试检测/配置。

### 11.2 坐标与物理量

传感器安装坐标与 ROS `base_link` 方向一致：

```text
sensor +X = forward
sensor +Y = left
sensor +Z = up
```

固件输出已经转换为 m/s^2 和 rad/s，再在 USB 帧中以整数 milli-unit
传输。左转/逆时针时 `angular_velocity_z > 0`，右转时为负。

### 11.3 静态零偏

CH340 调试串口发送单字符 `i`，保持底盘完全静止、水平约 5 s。固件采集
500 个样本，检查重力模长、加速度峰峰值和陀螺仪峰峰值，通过后应用
本次开机的 gyro raw bias。

成功日志：

```text
IMU STATIC result=PASS_BIAS_APPLIED
IMU STATIC gyro bias applied and persisted for future boots
```

偏置会在显式校准成功后写入预留 Flash，MCU 重新上电后自动恢复。可以通过 CH340
发送 `i`，也可以通过 USB `CMD_CALIBRATE_IMU (0x05)` 请求。USB
`IMU_STATE` flag bit 1 表示偏置是否有效。ROS2 应提供显式服务触发校准，
不要在每次启动时无条件自动校准。

### 11.4 ROS 发布原则

MPU6050 没有磁力计，当前固件也没有姿态融合。ROS2 发布
`sensor_msgs/msg/Imu` 时：

- 填 `angular_velocity`，mrad/s 除以 1000；
- 填 `linear_acceleration`，mm/s^2 除以 1000；
- `orientation_covariance[0] = -1`，表示没有 orientation；
- frame 建议使用 `imu_link`，在 URDF 中给出它到 `base_link` 的静态变换；
- 当前方向旋转为 identity，但传感器安装位置的平移仍应按实物写入 URDF；
- 轮速里程计与 IMU yaw rate 可交给 `robot_localization` 融合，纯陀螺仪
  yaw 长期会漂移。

## 12. USB CDC 协议 v1

### 12.1 USB 与会话行为

- USB FS CDC ACM，self-powered。
- Linux 通常显示为 `/dev/ttyACM0`；实际编号不应写死。
- VID/PID 为 `0483:5740`，可优先使用 `/dev/serial/by-id/` 或 udev 规则。
- USB CDC 的 baud 参数不决定 USB 实际速率，但主机可统一设置 115200。
- RX ring：512 B；TX queue 深度：8；单帧最大 62 B。
- USB callbacks 只复制字节和更新标志，解析发生在 `usbRxTask`。
- SOF 超时 50 ms 用于检测自供电设备的物理主机/线缆丢失。
- 主机发送任一 CRC 正确、版本正确的协议帧会开启 1.2 s 遥测会话。
- 空闲时主机应每 0.8~1.0 s 发送 `CMD_GET_STATUS`；运动时 20 Hz 速度
  命令会自然保持会话。
- 主机应 assert DTR。DTR 关闭会结束会话、清空待发队列并立即停止运动。
- 若主机从未发 CDC control-line request，固件为兼容性仍接受协议帧。

### 12.2 通用帧格式

所有多字节整数均为 little-endian，有符号整数使用 two's complement。
USB CDC 是字节流，主机解析器必须处理拆包、粘包、噪声和重新同步。

| Offset | Size | 字段 |
|---:|---:|---|
| 0 | 1 | SOF0 = `0xA5` |
| 1 | 1 | SOF1 = `0x5A` |
| 2 | 1 | protocol version = `1` |
| 3 | 1 | message type |
| 4 | 2 | sequence |
| 6 | 2 | payload length，最大 52 |
| 8 | N | payload |
| 8+N | 2 | CRC16-CCITT-FALSE |

CRC 参数：

```text
polynomial = 0x1021
initial    = 0xFFFF
reflection = false
final xor  = 0
coverage   = version 字节到 payload 最后一个字节
```

CRC 不包含两个 SOF 和 CRC 字段本身。完整帧长度为 `10 + payload_length`。

### 12.3 Host -> STM32

#### `0x01 CMD_VELOCITY`

Payload 8 B：

| Offset | Type | 含义 |
|---:|---:|---|
| 0 | int32 | linear X，mm/s，合法范围 -200..200 |
| 4 | int32 | angular Z，mrad/s，合法范围 -1500..1500 |

主机运动期间建议每 50 ms 发送。STM32 超过 250 ms 未收到有效新命令会
停车。主机自身也应设置更短的 cmd_vel timeout，例如 200 ms。

#### `0x02 CMD_STOP`

Payload 0 B。立即停止生产运行控制。ROS Lifecycle deactivate、shutdown、
串口异常前都应尽力发送一次。

#### `0x03 CMD_CLEAR_FAULT`

Payload 0 B。只有 E-stop 未激活、诊断未占用且故障实时原因已消失时才能
成功。不能清除已经锁存的 E-stop。

#### `0x04 CMD_GET_STATUS`

Payload 0 B。返回 ACK，并立即排队 STATUS、BATTERY_STATE 和 IMU_STATE。
同时刷新 1.2 s 主机会话。

#### `0x05 CMD_CALIBRATE_IMU`

Payload 0 B。仅在底盘 IDLE、控制器和电机关闭、四轮停止、无 E-stop/关键
故障/运动诊断、IMU READY 且没有其他校准时接受。ACK `OK` 仅表示开始，
最终结果由 `IMU_STATE` bits 3..7 给出。校准期间所有 `CMD_VELOCITY` 均被
拒绝；STOP、USB断开、DTR关闭或1.2 s会话超时会取消USB发起的校准。
等待约5秒期间主机必须约每0.8秒发送有效帧保持会话。

### 12.4 STM32 -> Host

#### `0x80 ACK`，Payload 4 B

| Offset | Type | 含义 |
|---:|---:|---|
| 0 | uint8 | 被应答的请求类型 |
| 1 | uint8 | result |
| 2 | uint8 | runtime state |
| 3 | uint8 | runtime fault |

Result：`0 OK`、`1 BAD_LENGTH`、`2 BAD_VALUE`、`3 STATE_REJECTED`、
`4 UNSUPPORTED`、`5 INTERNAL`。

Runtime state：`0 INIT`、`1 IDLE`、`2 ACTIVE`、`3 FAULT`、`4 ESTOP`。

Runtime fault：`0 NONE`、`1 INVALID_COMMAND`、`2 DIAGNOSTIC_CONFLICT`、
`3 CONTROLLER_START`、`4 CONTROLLER_RUNTIME`、`5 HEALTH_MANAGER`。

#### `0x81 STATUS`，Payload 52 B，10 Hz

| Offset | Type | 含义 |
|---:|---:|---|
| 0 | uint32 | MCU uptime，ms |
| 4 | uint32 | runtime command sequence |
| 8 | uint32 | accepted commands |
| 12 | uint32 | rejected commands |
| 16 | uint16 | 最近主机 frame sequence |
| 18 | uint8 | runtime state |
| 19 | uint8 | runtime fault |
| 20 | uint8 | last stop reason |
| 21 | uint8 | controller fault |
| 22 | uint16 | flags |
| 24 | int16 | 请求 vx，mm/s |
| 26 | int16 | 请求 wz，mrad/s |
| 28 | int16 | LF 测量速度，mm/s |
| 30 | int16 | LR 测量速度，mm/s |
| 32 | int16 | RF 测量速度，mm/s |
| 34 | int16 | RR 测量速度，mm/s |
| 36 | uint32 | valid protocol frames |
| 40 | uint32 | CRC errors |
| 44 | uint32 | RX overflow packets |
| 48 | uint32 | dropped TX frames |

STATUS flags：

| Bit | 含义 |
|---:|---|
| 0 | command valid |
| 1 | diagnostics active |
| 2 | E-stop latched |
| 3 | E-stop input active |
| 4 | USB configured/alive |
| 5 | chassis controller active |
| 6 | CDC host port open / DTR |
| 7 | Fault Manager critical fault |
| 8 | IWDG refresh active and health eligible |

Stop reason：`0 NONE`、`1 ZERO_COMMAND`、`2 EXPLICIT`、
`3 COMMAND_TIMEOUT`、`4 DIAGNOSTIC`、`5 ESTOP`、`6 FAULT`、
`7 TRANSPORT_LOST`。

#### `0x82 WHEEL_STATE`，Payload 52 B，20 Hz

| Offset | Type | 含义 |
|---:|---:|---|
| 0 | uint32 | MCU sample time，ms |
| 4 | int64 | LF accumulated count |
| 12 | int64 | LR accumulated count |
| 20 | int64 | RF accumulated count |
| 28 | int64 | RR accumulated count |
| 36 | int32 | LF filtered counts/s |
| 40 | int32 | LR filtered counts/s |
| 44 | int32 | RF filtered counts/s |
| 48 | int32 | RR filtered counts/s |

全部为物理前进正，累计计数已经在 MCU 中处理定时器回绕。

#### `0x83 BATTERY_STATE`，Payload 28 B，2 Hz

| Offset | Type | 含义 |
|---:|---:|---|
| 0 | uint32 | ADC sample time，ms |
| 4 | uint32 | successful filtered sample count |
| 8 | uint32 | cumulative ADC errors |
| 12 | uint16 | latest 16-sample raw average |
| 14 | uint16 | filtered raw ADC |
| 16 | uint16 | ADC pin voltage，mV |
| 18 | uint16 | D24A VM/motor-bus voltage，mV |
| 20 | uint8 | battery state |
| 21 | uint8 | flags |
| 22 | uint16 | consecutive ADC errors |
| 24 | uint16 | low threshold，mV；当前 0 |
| 26 | uint16 | critical threshold，mV；当前 0 |

Battery state：`0 NOT_READY`、`1 MONITOR_ONLY`、`2 NORMAL`、`3 LOW`、
`4 CRITICAL`、`5 ADC_FAULT`。

Battery flags：bit0 valid、bit1 protection enabled、bit2 low、bit3 critical、
bit4 ADC fault。

#### `0x84 IMU_STATE`，Payload 46 B，100 Hz

| Offset | Type | 含义 |
|---:|---:|---|
| 0 | uint32 | MCU IMU sample time，ms |
| 4 | uint32 | successful IMU sample count |
| 8 | int32 | body acceleration X，mm/s^2 |
| 12 | int32 | body acceleration Y，mm/s^2 |
| 16 | int32 | body acceleration Z，mm/s^2 |
| 20 | int32 | body angular velocity X，mrad/s |
| 24 | int32 | body angular velocity Y，mrad/s |
| 28 | int32 | body angular velocity Z，mrad/s |
| 32 | int32 | temperature，milli-degree Celsius |
| 36 | uint32 | cumulative IMU read errors |
| 40 | uint32 | consecutive IMU read errors |
| 44 | uint8 | IMU state |
| 45 | uint8 | flags |

IMU state：`0 NOT_READY`、`1 INITIALIZING`、`2 READY`、
`3 DISCONNECTED`、`4 IO_FAULT`。

IMU flags：bit0 valid sample、bit1 gyro bias valid、bit2 initialized、bit3
calibration active、bit4 calibration result valid、bit5 last success、bit6
last moved、bit7 last bias range failure。取消表示bit4=1且bit5..7=0。失败
或取消不会清除之前已经有效的bias。

## 13. ROS2 Humble 驱动建议

### 13.1 推荐架构

第一版建议实现独立 C++ LifecycleNode，而不是立即接入 `ros2_control`：

```text
serial_transport
  - /dev/ttyACM* 打开、DTR、重连
  - RX 字节流与 TX 队列

protocol_v1
  - frame encode/decode
  - CRC16
  - little-endian 安全读写
  - parser 重同步

base_driver LifecycleNode
  - cmd_vel -> CMD_VELOCITY
  - WHEEL_STATE -> joint_states + odom
  - IMU_STATE -> imu/data_raw
  - BATTERY_STATE -> motor-bus voltage/battery_state
  - STATUS -> diagnostics
```

原因：当前 STM32 接口接收底盘 `vx/wz`，并在 MCU 内执行差速运动学和四轮
闭环；标准 `diff_drive_controller` 通常输出左右轮目标速度，再次使用会
造成控制职责重复。后续若确实需要 `ros2_control`，可以写自定义
`SystemInterface`，但必须明确由 MCU 还是 ROS 负责运动学和轮速闭环。

### 13.2 建议话题

| 方向 | 建议话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/cmd_vel` | `geometry_msgs/msg/Twist` | vx 和 wz；可配置 TwistStamped 适配层 |
| 发布 | `/joint_states` | `sensor_msgs/msg/JointState` | 四轮位置/速度 |
| 发布 | `/odom` | `nav_msgs/msg/Odometry` | 轮式里程计 |
| 可选发布 | `/tf` | odom -> base_footprint | 确保系统中只有一个发布者 |
| 发布 | `/imu/data_raw` | `sensor_msgs/msg/Imu` | 无 orientation |
| 发布 | `/imu/temperature` | `sensor_msgs/msg/Temperature` | MPU6050 温度 |
| 发布 | `/battery_state` | `sensor_msgs/msg/BatteryState` | 当前实质为 motor-bus 电压 |
| 发布 | `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 状态、故障、CRC、丢帧、偏置 |

四个 joint 名称应由参数配置，并与最终 URDF 完全一致。建议名称可为：

```text
left_front_wheel_joint
left_rear_wheel_joint
right_front_wheel_joint
right_rear_wheel_joint
```

不要在驱动中写死这些建议名称。

### 13.3 ROS 驱动状态机

推荐流程：

1. `on_configure`：解析参数、创建串口和协议对象，但不发送运动命令。
2. 打开 `/dev/serial/by-id/...` 或配置的设备路径，assert DTR。
3. 清空本地 parser，发送带新 sequence 的 `CMD_GET_STATUS`。
4. 等待匹配 ACK 和 STATUS；确认协议 v1、状态和故障。
5. `on_activate` 后才订阅/执行运动命令。
6. ACTIVE 时每 50 ms 发送最后一条仍在 host timeout 内的 cmd_vel。
7. ROS cmd_vel 超过建议 200 ms 未更新时主动发送 `CMD_STOP`。
8. `on_deactivate`、shutdown、串口异常时尽力发送 `CMD_STOP`，然后关闭 DTR。
9. 重连后绝不重放断线前的旧速度；必须等待新 cmd_vel。
10. 若收到 ESTOP/FAULT，发布 ERROR 诊断并停止发送非零速度；根据策略和
    人工确认决定是否发送 `CMD_CLEAR_FAULT`。

### 13.4 轮子位置、速度与里程计

位置：

```text
joint_position_rad = accumulated_count * 2*pi / 1560
joint_velocity_rad_s = filtered_cps * 2*pi / 1560
```

主机第一次收到 WHEEL_STATE 时应保存本地原点，避免 ROS 节点重启导致
odom 跳变；STM32 的 `int64` 计数仍可继续用于诊断。

每帧增量里程计：

```text
d_lf = delta_count_lf * metres_per_count
d_lr = delta_count_lr * metres_per_count
d_rf = delta_count_rf * metres_per_count
d_rr = delta_count_rr * metres_per_count

d_left  = (d_lf + d_lr) / 2
d_right = (d_rf + d_rr) / 2
ds      = (d_left + d_right) / 2
dtheta  = (d_right - d_left) / 0.210

x   += ds * cos(theta + dtheta/2)
y   += ds * sin(theta + dtheta/2)
yaw += dtheta
```

当前四轮固定橡胶轮属于 skid-steer，原地转弯存在侧向滑移。最终应使用
实地旋转标定得到 effective track width，而不是永久使用名义 0.210 m。

### 13.5 时间戳

- MCU timestamp 是 `uint32` 毫秒 uptime，约 49.7 天回绕。
- 主机必须执行无符号差分/扩展为 64 bit，不能直接当 Unix time。
- 第一版可用“首帧 ROS time - 首帧 MCU uptime”建立偏移。
- 设备复位可由 uptime/sample_count 明显回退、USB 重连或状态变化识别，
  识别后必须重建时间映射和 odom 本地原点策略。
- 更成熟版本可增加主机-MCU 时间同步命令；当前协议没有这一消息。

### 13.6 QoS 和诊断建议

- IMU：`SensorDataQoS`，100 Hz。
- JointState：20 Hz，可用 sensor-data/best-effort 或 reliable，按系统需求。
- Odometry：20 Hz，通常 reliable depth 5~10。
- Diagnostics/Battery：reliable。
- cmd_vel：depth 1，只保留最新命令。
- 监控 `STATUS` 中 CRC、RX overflow、TX drop 是否增长。
- 监控 `WHEEL_STATE` 20 Hz 和 `IMU_STATE` 100 Hz 的 MCU timestamp。
- 监控 IMU valid、READY、bias flag 和 read errors。
- 监控 runtime state、stop reason、E-stop、health fault、IWDG flag。

## 14. Windows/主机参考工具

仓库内可直接复用的协议参考实现：

| 文件 | 用途 |
|---|---|
| `Tools/usb_cdc_test.py` | 协议编码/解码、状态、安全、断线、IMU、轮速测试 |
| `Tools/wheel_odometry_test.py` | 轮计数到角度/里程计的参考实现 |
| `Tools/usb_cdc_soak_test.py` | 重复开关 CDC 和恢复压力测试 |

常用命令：

```powershell
python Tools\usb_cdc_test.py --self-test
python Tools\usb_cdc_test.py --port COM4
python Tools\usb_cdc_test.py --port COM4 --wheel-monitor 10
python Tools\usb_cdc_test.py --port COM4 --imu-monitor 10
python Tools\usb_cdc_test.py --port COM4 --imu-calibration-test
python Tools\usb_cdc_test.py --port COM4 --safety-test
python Tools\usb_cdc_test.py --port COM4 --drive-test
python Tools\usb_cdc_test.py --port COM4 --dtr-stop-test
python Tools\usb_cdc_test.py --port COM4 --disconnect-test
python Tools\usb_cdc_test.py --port COM4 --debug-port COM3 --health-test
python Tools\wheel_odometry_test.py --port COM4 --duration 10
python Tools\usb_cdc_soak_test.py --port COM4 --cycles 20 --open-seconds 30 --closed-seconds 2
```

COM 号只是当前 Windows 示例：历史测试中 CH340 常为 COM3、STM32 USB
CDC 常为 COM4，但重新插拔或换电脑后可能变化。

## 15. 调试 UART 命令

USART1/CH340：115200 8-N-1。主要 Debug 命令：

```text
s   立即停止/取消当前诊断
i   MPU6050 静态 5 s 校准

v   开环电机诊断模式
c   电机特性扫描
p   单轮 PI 验证
q   四轮 PI 同步验证
k   抬轮底盘运动学验证
b   地面运动验证
n   运行时所有权/超时验证
z   软件故障注入选择前缀

1/2/3/4  选择 LF/LR/RF/RR
f/r      正向/反向
```

Debug 构建还启用 motor diagnostics 和 fault injection。不要让正式机器人
运行时把 CH340 暴露为无访问控制的控制接口；发布版本应关闭这些宏或限制
调试入口。

## 16. 构建、固件和烧录

### 16.1 CMake 构建

项目使用 C11、CMake Presets 和 Ninja。通用命令：

```text
cmake --preset Debug
cmake --build --preset Debug --clean-first
```

输出：

```text
build/Debug/rosDemo.elf
build/Debug/rosDemo.hex
```

最后一次真实 Debug clean build：

```text
Error   0
Warning 0
FLASH   152780 B / 1 MB  = 14.57%
RAM      49832 B / 128KB = 38.02%
CCMRAM       0 B / 64KB
```

已使用的 STM32Cube 本地工具版本包括 CMake 4.3.1+st.1 和 GNU Tools for
STM32 14.3.1+st.2。Ubuntu 上只需兼容的 `arm-none-eabi-gcc`、CMake 和
Ninja；若仅开发 ROS2 驱动，则不要求在 Ubuntu 重编 STM32 固件。

### 16.2 当前无 ST-Link 的烧录方式

开发板通过板载 CH340 和 STM32 ROM UART bootloader 下载。通用流程：

1. 关闭占用 CH340 的串口终端。
2. 确保电机安全、底盘不会意外运动。
3. 按 M144Z-M4 板卡的 BOOT0/串口下载方式进入 System Memory bootloader。
4. 在 STM32CubeProgrammer 或板卡支持的串口下载工具中选择 CH340 COM。
5. 烧录 `build/Debug/rosDemo.hex`；Flash 起始地址为 `0x08000000`。
6. 恢复 BOOT0 为用户 Flash 启动并复位。
7. 用 CH340 查看 Boot Banner，用另一个 USB Device 口连接 CDC。

不同批次板卡的 BOOT 跳线/按键布局应以 M144Z-M4 原理图和板上丝印为准。

## 17. 已完成的实板验证

以下结果已经由当前硬件确认：

- USART1/CH340 启动日志、168 MHz 时钟和全部 FreeRTOS 任务启动正常；
- 四个编码器原始计数、方向、增量、回绕和 forward-normalized 极性正常；
- 四个电机逐轮 `f=向前`、`r=向后`，LF/RR 原始方向正常，LR/RF 由
  软件极性修正；
- 四轮闭环、差速运动学、直行、左右转和地面低速动作已运行；
- USB 速度命令、250 ms timeout、DTR 停车和物理 USB 拔线停车已验证；
- 20 轮 × 30 s CDC 打开/关闭压力测试全部 PASS，遥测恢复正常，
  当次测试 `tx_drop_total=0`；
- motorTask、usbRxTask 心跳丢失和 LF 无反馈堵转的软件故障注入全部 PASS；
- IWDG 健康仲裁、故障抑制、锁存和恢复通过；
- PC1 电压 ADC 已用万用表核对，D24A VM 约 11.75 V，遥测约 11.6 V；
- MPU6050 WHO_AM_I=`0x68`，静态校准、ROS 坐标、左右转 Z 符号通过；
- 最终 IMU USB 测试：10 s 收到 1001 个唯一样本，100.0 Hz，
  interval=10..10 ms，`READY/valid=1/bias=1/errors=0`。
- USB `0x05` IMU校准成功路径已于2026-09-25实板通过：初始flags `0x05`，
  最终flags `0x37`，Runtime保持IDLE、四轮速度为0、IMU errors为0/0；
  payload/重复请求/校准期间速度命令互锁均通过测试工具验证。
- 在D24A VM约11.44 V、同一次MCU boot内，校准后的flags `0x37`持续到后续
  10秒monitor；1001个唯一样本、100.0 Hz、interval=10..10 ms，证明bias
  跨USB会话保持。校准后`last_stop_reason`可保留为DIAGNOSTIC历史值；应结合
  IDLE、diagnostics flag=0和fault=0判断当前状态。

## 18. 已知限制和后续工作

### ROS2 开发前不阻塞，但必须记录

1. **电池保护未完成。**当前读数是调压后的电机母线，不是原始 3S 电池。
   若要低电量保护，需从原始电池安全分压到另一个 ADC 或调整硬件路径，
   再验证阈值和 BMS 行为。
2. **IMU bias 已持久化。**只有首次无有效记录或记录不兼容时，才需通过
   CH340 `i` 或 USB `0x05` 显式校准；上电不会自动执行 500 样本校准。
3. **无 orientation。**ROS `sensor_msgs/Imu` 必须将 orientation 标为
   unavailable，不能用全零四元数冒充有效姿态。
4. **机械参数仍需最终标定。**65 mm 轮径和 210 mm 轮距为初始值。
5. **PI/前馈仍是当前底盘参数。**更换电机、轮胎、驱动、电压或负载后
   需要重新扫描和调整。
6. **协议没有设备信息和时间同步消息。**当前驱动可使用固定参数和启动
   时间锚点；后续可新增 `GET_INFO/CAPABILITIES` 与时间同步，不应修改
   已有 v1 消息布局。
7. **Debug 功能仍开启。**最终 Release 应关闭电机诊断、故障注入和不必要
   的高频文本日志，并重新做安全回归。
8. **MPU6050 是开发阶段传感器。**SPI1/MPU6500 引脚已保留；将来换传感器
   时应保持 `ImuService` 和 USB `IMU_STATE` 的上层语义稳定。

### ROS2 联调后建议做的最终验收

- Nav2 连续 cmd_vel 与 MCU heartbeat 测试；
- ROS 节点崩溃、进程 kill、USB 拔线、Jetson 重启、DTR 关闭停车；
- E-stop 动态触发和复位流程；
- 30~60 min 带载行驶与 USB/任务/看门狗统计；
- 直线距离标定 effective wheel diameter；
- 原地 360 度标定 effective track width；
- 左右轮速度一致性、低速起步、反向和坡面负载；
- 轮式 odom 与 IMU 经 `robot_localization` 融合；
- 最终 Release 固件重新执行安全、断线、堵转和 IWDG 回归。

## 19. 新 Ubuntu 工作空间最小输入清单

至少把以下文件复制或提供给新工作空间中的开发者/智能体：

```text
docs/ROS2_UBUNTU_HANDOFF.md        本文，自包含系统说明
docs/usb_cdc_protocol_v1.md        协议原始规范
Tools/usb_cdc_test.py              Python 协议参考实现
Tools/wheel_odometry_test.py       里程计参考实现
```

若可用，最好保留整个 STM32 仓库作为只读参考。新 ROS2 会话应首先阅读本文，
再以 `Tools/usb_cdc_test.py` 的 CRC、stream parser 和 struct layout 作为
主机实现的可执行参考。

## 20. 推荐的下一阶段交付物

建议 ROS2 Humble 工作空间第一阶段交付：

```text
rosdemo_base/
  include/rosdemo_base/protocol_v1.hpp
  include/rosdemo_base/serial_transport.hpp
  src/protocol_v1.cpp
  src/serial_transport.cpp
  src/base_driver_node.cpp
  config/base_driver.yaml
  launch/base_driver.launch.py
  test/test_protocol_v1.cpp
  test/test_odometry.cpp
```

第一阶段先完成离线单元测试和 USB 只读接收，再依次启用：

```text
协议解析
-> STATUS/WHEEL/IMU/BATTERY 发布
-> joint_states/odom
-> cmd_vel + 20 Hz heartbeat
-> lifecycle/reconnect/safety
-> robot_localization/Nav2 集成
```

在只读接收、故障状态和停车路径没有验证前，不要直接让 Nav2 控制真实
电机。

## 21. 可直接交给 Ubuntu 新会话的开场说明

将本文和第 19 节列出的参考文件放进新的 ROS2 工作空间后，可以把下面这段
话直接交给新的开发者或智能体：

```text
请先完整阅读 docs/ROS2_UBUNTU_HANDOFF.md 和
docs/usb_cdc_protocol_v1.md，并以它们及 Tools/usb_cdc_test.py 为当前
STM32 底盘接口依据。底盘 MCU 已负责四轮编码器、速度闭环、差速运动学、
250 ms 命令超时、E-stop、Fault Manager 和 IWDG；ROS2 不要重复实现
电机级 PI，也不要更改已有 USB v1 字段语义。

目标环境是 Jetson Orin Nano、Ubuntu 22.04、ROS2 Humble。请先建立一个
C++ LifecycleNode 底盘驱动，分离 serial_transport、protocol_v1 和
base_driver 三层。第一步只实现协议单元测试和只读 STATUS/WHEEL_STATE/
BATTERY_STATE/IMU_STATE 接收；验证 CRC、拆包粘包、重同步、DTR、重连和
时间戳后，再接入 joint_states、odom、imu/data_raw、diagnostics，最后才
启用 cmd_vel。任何串口异常、节点 deactivate/shutdown 或 cmd_vel 超时都
必须进入安全停车路径；重连后不得重放旧速度命令。

开始编码前请先列出实现计划、包结构、参数表和测试矩阵。如发现文档与
实际 STM32 仓库不一致，停止假设并回到 rosDemo.ioc 与当前生成代码核对。
```
