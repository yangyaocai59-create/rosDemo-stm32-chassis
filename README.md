# rosDemo STM32 四轮差速底盘固件

STM32F407ZGT6 四轮差速移动机器人底盘控制器工程，配套 Jetson Orin Nano + Ubuntu 22.04 + ROS 2 Humble。

## 当前版本

- 固件版本：`0.18.2-wheel-speed-pivot-trim`（基于 0.17.0）
- MCU：`STM32F407ZGT6`，168 MHz
- 调试串口：USART1，PA9/PA10，115200 8-N-1
- ROS2 通信：USB CDC，PA11/PA12
- IMU：MPU6500，SPI1，100 Hz
- 编码器：TIM1=LF、TIM2=LR、TIM3=RF、TIM4=RR
- PWM：TIM8_CH1=LF、CH2=LR、CH3=RF、CH4=RR

## 开始阅读

ROS2 或上位机开发前，请先阅读：

1. `docs/STM32_IMU_FRACTIONAL_BIAS_0.16.0_HANDOFF_20261017.md`
2. `docs/STM32_IMU_STATIONARY_BIAS_TRACKING_0.17.0.md`
3. `docs/STM32_WHEEL_SPEED_CONTROL_0.18.0.md`
4. `docs/STM32_IMU_STATIONARY_BIAS_HARDWARE_20261017.md`
5. `docs/ROS2_UBUNTU_HANDOFF.md`
6. `docs/usb_cdc_protocol_v1.md`

## 重要说明

- 当前 USB IMU 精确数据使用 RAW v2，消息类型为 `0x86`。
- RAW v2 的陀螺偏置单位为 milli-RAW，不能按旧版整数 RAW 偏置解析。
- 当前 MPU6500 已完成精确偏置保存和恢复，但尚未完成温度补偿；不要把它当作长期绝对航向来源。
- ROS2 侧应融合轮速与 IMU 角速度，并通过激光、视觉或其他外部定位源修正长期航向。
- CubeMX 生成代码和当前 `.ioc` 是硬件配置的唯一依据。

## 目录

- `Application/`：底盘应用与服务模块
- `Core/`、`Drivers/`、`Middlewares/`、`USB_DEVICE/`：CubeMX 及中间件工程
- `Tools/`：USB、IMU、编码器和硬件验证工具
- `docs/`：STM32 与 ROS2 交接资料

`build/`、临时抓包、个人日志和外部 ZIP 不纳入源码提交。
