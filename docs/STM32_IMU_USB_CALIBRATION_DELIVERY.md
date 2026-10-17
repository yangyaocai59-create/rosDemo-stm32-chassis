# STM32 USB IMU 静态校准：ROS2 联调交付说明

> 历史说明：本文记录 `0.13.0` USB 校准功能交付时的状态。自
> `0.13.1-stage13-imu-bias-flash` 起，成功校准的 bias 已写入内部 Flash，
> 持久化行为及最新验收步骤以
> `STM32_IMU_BIAS_FLASH_PERSISTENCE_DELIVERY.md` 为准。

文档日期：2026-09-25  
固件版本：`0.13.0-stage13-usb-imu-calibration`  
协议版本：USB CDC v1（保持为 `1`）  
目标上位机：Ubuntu 22.04 / ROS2 Humble  

## 1. 交付状态

本次固件已经完成源码实现、Python协议离线自检和真实Debug clean build。
当前状态如下：

| 项目 | 结果 |
|---|---|
| C/C++编译 | PASS，0 error / 0 warning |
| Python语法检查 | PASS |
| `usb_cdc_test.py --self-test` | PASS |
| FLASH | 152780 B / 1 MB，14.57% |
| RAM | 49832 B / 128 KB，38.02% |
| CCMRAM | 0 B / 64 KB |
| 实板USB校准成功路径 | PASS（2026-09-25） |
| 同一boot内bias保持与100 Hz回归 | PASS（2026-09-25） |
| 取消/断线/移动失败回归 | 待继续执行 |

可烧录文件：

```text
build/Debug/rosDemo.hex
```

## 2. 实现范围

本次只给现有IMU静态校准增加USB入口、结果状态和安全互锁：

- 没有修改 `.ioc` 或CubeMX硬件配置；
- 没有修改引脚、MPU6050量程、DLPF、轴映射或单位；
- 没有修改电机PI、前馈、运动学、编码器和250 ms命令超时；
- 没有修改E-stop、Fault Manager或IWDG规则；
- 没有把bias写入Flash；
- 没有新增姿态融合或orientation；
- USB协议版本仍为1，已有消息编号、长度和字段偏移保持不变。

校准仍复用：

```text
Application/Diagnostics/imu_static_test.c
Application/Sensors/imu_service.c
```

算法仍采集500个新样本，约5秒，并使用原有重力模长、加速度峰峰值、
陀螺仪峰峰值和bias范围判定。

## 3. 新增主机命令

### `0x05 CMD_CALIBRATE_IMU`

| 属性 | 值 |
|---|---|
| 协议版本 | 1 |
| message type | `0x05` |
| payload length | 0 |
| ACK | 复用 `0x80 ACK`，sequence与请求一致 |

ACK结果：

| Result | 含义 |
|---|---|
| `OK` | 请求已接受，500样本采集已开始；不代表最终成功 |
| `BAD_LENGTH` | payload不是0字节 |
| `STATE_REJECTED` | 底盘/IMU/诊断/安全状态不允许，或已有校准运行 |
| `INTERNAL` | 启动时发生不能归类为状态拒绝的内部失败 |

STM32只有在以下条件全部满足时接受请求：

```text
ChassisRuntime == IDLE
没有有效速度命令
底盘控制器未运行
D24A STBY未启用
四轮电机输出均为0
四轮速度估计均为stopped
E-stop未激活且未锁存
Fault Manager无关键故障
没有电机/底盘运动诊断占用
IMU state == READY且sample valid
没有另一轮IMU校准运行
```

## 4. IMU_STATE逐字节兼容性

`0x84 IMU_STATE` 仍为46字节，所有原字段偏移不变：

| Offset | Type | 含义 |
|---:|---:|---|
| 0 | uint32 | MCU IMU sample time，ms |
| 4 | uint32 | successful sample count |
| 8 | int32 | acceleration X，mm/s² |
| 12 | int32 | acceleration Y，mm/s² |
| 16 | int32 | acceleration Z，mm/s² |
| 20 | int32 | angular velocity X，mrad/s |
| 24 | int32 | angular velocity Y，mrad/s |
| 28 | int32 | angular velocity Z，mrad/s |
| 32 | int32 | temperature，m°C |
| 36 | uint32 | cumulative read errors |
| 40 | uint32 | consecutive read errors |
| 44 | uint8 | IMU state |
| 45 | uint8 | flags |

flags定义：

| Bit | Mask | 含义 |
|---:|---:|---|
| 0 | `0x01` | valid sample，原定义不变 |
| 1 | `0x02` | gyro bias valid，原定义不变 |
| 2 | `0x04` | service initialized，原定义不变 |
| 3 | `0x08` | calibration active |
| 4 | `0x10` | calibration result valid |
| 5 | `0x20` | calibration last success |
| 6 | `0x40` | calibration last moved |
| 7 | `0x80` | calibration last bias range failure |

状态转换：

| 场景 | bits 3..7 |
|---|---|
| MCU冷启动，尚未运行 | 全0 |
| 请求被接受并采集中 | bit3=1，bit4..7=0 |
| 成功 | bit3=0，bit4=1，bit5=1 |
| 移动/重力检查失败 | bit3=0，bit4=1，bit6=1 |
| bias范围失败 | bit3=0，bit4=1，bit7=1 |
| 取消 | bit3=0，bit4=1，bit5..7=0 |

新请求开始时会清除上一轮result位。失败和取消都不会清除或覆盖此前已经
有效的bias，因此失败结果与bit1可以同时存在。例如 `bit1=1, bit4=1,
bit6=1` 表示旧bias仍有效，但最新一次校准因移动失败。

## 5. 校准期间的安全行为

- 所有 `CMD_VELOCITY` 均返回 `STATE_REJECTED`，包括零速度命令；
- Runtime保持IDLE，控制器和电机输出保持关闭；
- `CMD_STOP`始终优先，停止全部生产/Debug运动并取消校准；
- USB、IMU、motor、safety、telemetry和watchdog任务继续运行；
- STATUS、WHEEL_STATE、BATTERY_STATE和IMU_STATE继续发送；
- IWDG监督与刷新规则没有变化；
- 第500样本收尾与STOP/断线之间经过临界区保护：如果取消先发生，新bias
  不会在取消后被写入运行状态。

USB发起的未完成校准会在以下事件取消：

```text
CMD_STOP
USB物理连接/SOF丢失
CDC DTR关闭
1.2秒主机会话超时
```

因此ROS2在等待约5秒的校准结果时，必须每0.8秒左右发送一个有效协议帧，
建议使用 `CMD_GET_STATUS`。重连后不得自动恢复或重发旧校准请求。

CH340命令仍保留：

```text
i  开始UART来源的静态校准
s  停车并取消未完成校准
```

UART和USB不能同时启动两轮校准。USB断线只自动取消USB来源校准，不会因
USB端口状态误取消独立的UART调试校准。

## 6. ROS2推荐流程

建议提供一个显式服务，例如 `std_srvs/srv/Trigger`。不要默认在每次节点
启动时无条件自动校准。

推荐状态机：

1. 暂停向STM32发送 `CMD_VELOCITY`，并拒绝新的非零 `/cmd_vel`；
2. 确认STATUS为IDLE，四轮速度为0，IMU为READY/valid；
3. 发送payload为空的 `0x05`，保存request sequence；
4. 只接受同sequence、request type为`0x05`的ACK；
5. ACK `OK`后等待IMU flag bit3出现；
6. 等待期间约每0.8秒发送 `CMD_GET_STATUS`，并继续解析所有遥测；
7. 最长等待8秒；
8. bit3清零后检查bit4；
9. 只有bit4、bit5、bit1全部为1才报告服务成功；
10. bit6表示移动失败，bit7表示bias范围失败；
11. bit4=1且bit5..7=0表示取消；
12. 超时、断线、节点退出或用户取消时尽力发送 `CMD_STOP`；
13. 失败或取消后不要启用依赖已校准IMU的融合流程；如果bit1仍为1，应在
    diagnostics中明确说明它是此前bias，不是本次校准成功。

## 7. 参考测试工具

Python参考实现已经更新：

```text
Tools/usb_cdc_test.py
```

离线测试：

```powershell
python Tools\usb_cdc_test.py --self-test
```

烧录后，保持底盘静止、水平，执行：

```powershell
python Tools\usb_cdc_test.py --port COM4 --imu-calibration-test
```

该测试会验证：

- 非零payload返回BAD_LENGTH；
- 正常请求获得匹配的ACK OK；
- 执行期间出现active位；
- 重复校准请求返回STATE_REJECTED；
- 执行期间零速度命令也返回STATE_REJECTED；
- 持续发送会话keepalive；
- 8秒内出现明确的成功、移动失败、范围失败或取消结果；
- 成功时bias-valid、result-valid和last-success同时有效。

测试不会故意让电机运动。

## 8. 烧录后的回归测试

### 8.1 已通过的实板结果

2026-09-25执行：

```powershell
python Tools\usb_cdc_test.py --port COM4 --imu-calibration-test
```

结果：

```text
初始 IMU flags = 0x05
  valid=1, initialized=1, bias=0
  calibration active/result/success/moved/range均为0

最终 IMU flags = 0x37
  valid=1, initialized=1, bias=1
  calibration active=0, result valid=1, last success=1
  moved=0, range failure=0

IMU state=READY
IMU errors=0/0
Runtime始终IDLE
四轮速度始终为0
PASS: USB IMU calibration, result flags and motion interlocks verified
```

该测试同时验证了非零payload拒绝、重复请求拒绝和校准期间零速度命令
拒绝。成功路径现已完成实板验收。

当次BATTERY_STATE报告的D24A VM约为4.09 V，与此前约11.6 V基线不同。
该现象不影响IMU校准结论，但在后续任何电机动作测试前必须用万用表重新
确认D24A VM和DC-DC输出。

随后在D24A电机电源打开、同一次MCU boot内连续执行USB校准和10秒IMU
monitor，得到：

```text
D24A VM遥测约11.44 V
校准结束flags=0x37
后续monitor flags持续为0x37
bias=1, cal_result=1, cal_ok=1
IMU state=READY, errors=0/0
1001个唯一样本，100.0 Hz，interval=10..10 ms
静止角速度由校准前约[-28,-1,-23]mrad/s降至约[0,0,-1]mrad/s
MCU uptime从约68 s连续增长到86 s，没有发生复位
```

这证明运行时bias会在同一次boot内跨USB命令/端口会话保持，同时复位后仍
按设计清零，不存在意外Flash持久化。

校准结束后的STATUS可能显示 `state=IDLE, stop=DIAGNOSTIC`。这里的stop字段
是最近一次停止/所有权原因的历史记录；只要state为IDLE、STATUS diagnostics
flag bit1为0且fault为0，就不表示当前仍有诊断占用或故障。ROS2不得仅凭
`last_stop_reason=DIAGNOSTIC`判定校准失败。

### 8.2 尚需执行的回归

请按顺序执行并把终端输出保存给固件开发者：

```powershell
python Tools\usb_cdc_test.py --port COM4 --imu-calibration-test
python Tools\usb_cdc_test.py --port COM4 --imu-monitor 10
python Tools\usb_cdc_test.py --port COM4 --safety-test
python Tools\usb_cdc_test.py --port COM4 --dtr-stop-test
python Tools\usb_cdc_test.py --port COM4 --disconnect-test
python Tools\usb_cdc_soak_test.py --port COM4 --cycles 5 --open-seconds 2 --closed-seconds 1
```

另需人工验证：

- 校准过程中晃动车体，结果应为moved failure；
- 校准过程中发送STOP，结果应为cancelled且电机保持关闭；
- 校准过程中关闭DTR或拔USB，CH340应记录取消，重连后不自动重启；
- MCU复位后bias-valid重新变为0，证明没有写Flash；
- 手动逆时针旋转 `wz>0`，顺时针旋转 `wz<0`。

## 9. 修改文件

```text
Application/Communication/chassis_usb_protocol.h
Application/Communication/chassis_usb_protocol.c
Application/Diagnostics/imu_static_test.h
Application/Diagnostics/imu_static_test.c
Application/Diagnostics/motor_open_loop_test.h
Application/Diagnostics/motor_open_loop_test.c
Application/System/firmware_info.h
Core/Src/freertos.c
Tools/usb_cdc_test.py
docs/usb_cdc_protocol_v1.md
docs/ROS2_UBUNTU_HANDOFF.md
docs/STM32_IMU_USB_CALIBRATION_DELIVERY.md
```

CubeMX生成的外设配置和 `rosDemo.ioc` 未修改。
