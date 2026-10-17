# STM32 IMU Bias Flash 持久化交付说明

日期：2026-09-25  
固件版本：`0.13.1-stage13-imu-bias-flash`  
目标 MCU：STM32F407ZGT6  
上位机协议：USB CDC v1，协议字段未改变

## 1. 变更范围

本次只解决一个问题：一次显式静态校准成功后，陀螺仪 bias 在 STM32
完全断电或复位后仍能恢复。

- 上电不会自动执行 500 样本静态校准。
- 首次没有有效记录时，`gyro_bias_valid` 保持 false，等待现有的
  `CMD_CALIBRATE_IMU (0x05)` 或 CH340 `i` 命令。
- 显式校准成功并应用 bias 后，固件追加一条 Flash 记录。
- 下次启动在 MPU6050 完成识别后加载最新有效记录，并继续通过现有
  `ImuService_SetGyroBiasRaw()` 应用。
- 校准失败、移动、超范围、STOP、取消或 USB 断线不写 Flash，旧记录不变。
- Flash 保存失败时，本次 RAM bias 在当前启动周期仍有效，旧 Flash 记录保留；
  CH340 输出明确错误。USB v1 没有为“保存失败”新增状态位。

本次没有修改 `.ioc`、CubeMX 引脚、IMU 量程/坐标、USB 消息编号或长度、电机
控制、运动学、E-stop、Fault Manager、IWDG、250 ms 超时和断线停车逻辑。

## 2. Flash 布局

| 区域 | 地址 | 大小 | 用途 |
|---|---|---:|---|
| 应用程序 | `0x08000000–0x080BFFFF` | 768 KB | 链接脚本允许的固件区域 |
| Sector 10 / A | `0x080C0000–0x080DFFFF` | 128 KB | IMU bias 日志 A |
| Sector 11 / B | `0x080E0000–0x080FFFFF` | 128 KB | IMU bias 日志 B |

`STM32F407xx_FLASH.ld` 已把应用区限制为 768 KB，因此以后固件增长也不能覆盖
两个记录扇区。当前 Debug 镜像最后装载地址约为 `0x080262D8`。

烧录时不要选择 **Full chip erase / 全片擦除**，否则校准记录会被清除。只擦除
应用程序实际使用的扇区可以保留 bias。需要故意恢复“首次未校准”状态时，可以
全片擦除或单独擦除 Sector 10、11。

## 3. 记录格式与掉电保护

每条记录固定 40 字节，小端布局：

| Offset | 类型 | 字段 |
|---:|---|---|
| 0 | `uint32_t` | magic = `0x494D5542` (`IMUB`) |
| 4 | `uint16_t` | format version = 1 |
| 6 | `uint16_t` | record size = 40 |
| 8 | `uint32_t` | sequence |
| 12 | `uint32_t` | IMU identity = `0x60500501` |
| 16 | `int32_t` | gyro bias X raw |
| 20 | `int32_t` | gyro bias Y raw |
| 24 | `int32_t` | gyro bias Z raw |
| 28 | `int16_t` | calibration temperature，0.01 °C |
| 30 | `uint16_t` | reserved，写零 |
| 32 | `uint32_t` | CRC-32/IEEE，覆盖 offset 0..31 |
| 36 | `uint32_t` | commit marker = `0x434F4D54` (`COMT`) |

保存时先写 offset 0..35，最后写 commit marker，再进行完整回读、CRC 和逐字节
比较。任何中途掉电产生的未提交或 CRC 错误记录都会被忽略，上一条记录仍有效。

两个扇区采用追加式 A/B 日志并优先交替写入。日常校准只写 40 字节，不执行
128 KB 扇区擦除，不会引入长时间 Flash 擦除阻塞。两个扇区合计可保存约 6552
次成功校准；写满后固件拒绝继续保存并保留最后有效记录，不会在运行期自动进行
长擦除。

## 4. 启动和 flags 行为

启动扫描并不会被视为本次启动执行了一次校准：

- 有效记录恢复成功：`IMU_STATE.flags bit1 = 1`；
- `bit3 calibration active = 0`；
- `bit4..7 calibration result = 0`；
- 空白、CRC 错误、版本错误、identity 错误或 bias 超范围：忽略记录，bit1 保持 0；
- 不因无效记录死机，不阻止 USB 或原始 IMU 数据启动，也不自动校准。

现有 USB `0x05` 校准完成状态定义完全不变。Flash 是否保存成功没有占用或重定义
现有 flags。

## 5. 构建结果

Debug clean build 已实际执行成功：

```text
Error:   0
Warning: 0
FLASH:   156376 B / 768 KB = 19.88%
RAM:      49920 B / 128 KB = 38.09%
CCMRAM:       0 B / 64 KB
```

生成文件：

```text
build/Debug/rosDemo.elf
build/Debug/rosDemo.hex
```

## 6. 实板验收

1. 烧录新 `rosDemo.hex`。首次测试建议全片擦除，确保两个记录扇区为空。
2. 上电观察 CH340，应看到：

   ```text
   IMU bias Flash ready: no compatible saved record
   MPU6050 ready: ...
   ```

3. 查询 USB 状态，确认 IMU bit1 为 0。
4. 保持底盘静止、水平，执行：

   ```powershell
   python Tools\usb_cdc_test.py --port COM4 --imu-calibration-test
   ```

5. 除原有 PASS 外，CH340 应看到：

   ```text
   IMU bias Flash saved: slot=A seq=1 address=0x080C0000 ...
   IMU STATIC gyro bias applied and persisted for future boots
   ```

6. 完全切断 STM32 电源至少 10 秒，再重新上电；不要发送校准命令。
7. CH340 应看到类似：

   ```text
   IMU bias Flash ready: newest=A seq=1 records=1/0
   IMU gyro bias restored from Flash: raw=[...] calibration_temp=...C
   ```

8. 执行：

   ```powershell
   python Tools\usb_cdc_test.py --port COM4 --imu-monitor 10
   ```

   预期 `bias=1`、`cal_active=0`、`cal_result=0`，IMU 继续约 100 Hz 发布。
9. 再显式校准一次，预期保存到 B 槽、sequence 增加；再次断电后加载更新值。
10. 回归执行 safety、DTR stop、disconnect、health 和 CDC soak 测试。

## 7. 修改文件

```text
Application/Storage/imu_bias_storage.h
Application/Storage/imu_bias_storage.c
Application/Diagnostics/imu_static_test.c
Application/Sensors/imu_service.h
Application/Sensors/imu_config.h
Application/System/firmware_info.h
Core/Src/freertos.c
STM32F407xx_FLASH.ld
CMakeLists.txt
docs/usb_cdc_protocol_v1.md
docs/ROS2_UBUNTU_HANDOFF.md
docs/STM32_IMU_BIAS_FLASH_PERSISTENCE_DELIVERY.md
```

CubeMX 生成的外设配置和 `rosDemo.ioc` 未修改。
