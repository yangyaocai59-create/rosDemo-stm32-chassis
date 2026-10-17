# USB TX 突发拥塞修正交付（0.13.3）

日期：2026-10-12，Asia/Shanghai。代码和离线测试完成，真实 Debug clean build 成功；尚未烧录/实板验收。

## 1. 身份与构建产物

```text
Firmware version = 0.13.3-usb-tx-burst-fix
Build = Oct 12 2026 08:24:27
MCU = STM32F407ZGT6
HEX SHA256 = D5F7AC2C67F0FA984CDD12E92AECEC786FB50E639D8BF0C85F6E03966E7B2508
ELF SHA256 = 16CF592176C4EBEAABD7D3D568D5290D7E4D54D0D65D5A79FC2231EF6DBD8F41
```

冻结目录 `build/deliveries/usb_tx_fix_0.13.3_20261012/`。
本工作区没有 Git 元数据，无法给出提交号；不创建或伪造提交，以版本、Build、源码快照及 SHA256 关联交付。维护者可将变更导入正式仓库后补充提交号。

## 2. 问题依据和范围

上一轮双路实板记录在前进 STOP 附近 tx_drop=0→1，链路 alive=1、DTR=1，最终 STOP 成功；不是 0.13.2 已修正的 SOF 旧时间误判。
原有效 TX 队列只有 7 帧，VELOCITY/STOP/GET_STATUS 同批可生成 6 帧回应，加上已有遥测就能超过容量。原生产 parser/transport 的受控离线测试已复现此机制；实板具体积压量和丢弃消息类型未直接采集，详见 `STM32_USB_TX_BURST_REVIEW_20261012.md`。

本次不改硬件、编码器计数/滤波、PI、方向、机械参数、IMU/bias Flash、电压业务或 ROS 门控，不把 TX 修正等同于历史 LF 静止增加 4 计数已修复。
SOF 50/51 ms、DTR 关闭、真实断链、命令超时、ESTOP、Fault Manager/IWDG 停车机制保留。

## 3. 实现

1. **有界 FIFO 容量/预留**：16 个槽位有效 15 帧，普通遥测最多占 11 个排队槽，为 ACK/STATUS 留 4 个；另有有效 3 帧关键应答待发送缓存和 1 帧 in-flight 缓冲。不抢占/重排旧帧，保持设备帧 FIFO 顺序。
2. **关键应答有界重试**：ACK/STATUS 无法立即进入 FIFO 时先保存在待发送缓存；ProcessTransmit 在空间可用后按原顺序移入 FIFO。USB BUSY 或提交失败保留 FIFO 帧以便重试，不误算为直接丢帧。两层容量都耗尽时仍真实记录丢弃，不承诺无限负载下零丢帧。
3. **发送推进及 RX 预算**：入队前、处理开始、每个完整命令后及处理结束推进发送。每轮最多 128 字节/2 个完整帧，避免无限 RX 批次饿死 TX；不因等 ACK 队列而暂停命令执行，拥塞测试中 STOP 仍执行。
4. **遥测背压**：普通遥测空间不足时不入队、不消耗设备序列号，保留到期时间，在随后循环重新生成新快照；单独记录 deferred 尝试次数。不冻结/改写编码器计数，不重新发送过时采样来凑 20/100 Hz。
5. **状态查询兼容**：GET_STATUS 的 ACK/STATUS/BATTERY/IMU 语义保留；即时 BATTERY/IMU 受背压时通过到期调度重试。USB v1 所有消息 ID、CRC、长度和 STATUS flags 语义不变，ROS 无需改帧解码。
6. **原子缓冲归属**：有界复制和非阻塞 USB 提交用短临界区防止配置回调改变归属；UART 和等待不在临界区内。SOF 超时或 DTR 关闭只清未提交的 FIFO/重试数据，已经提交的缓冲仍由完成回调/撤配置释放，避免复用尚未完成的传输。
7. **USB 日志延后**：只对调用 USB Process 的任务开启日志缓存作用域；原日志接口和其他任务日志不变。telemetryTask 每次最多输出 4 条缓存记录，即使例行编码器日志被诊断测试抑制也会输出。有效缓存 15 条，格式化/拷贝有界，USB 任务不等 UART/日志互斥锁；保持日志生成时的 MCU 时间。缓存满或 UART 失败单独计 log_drop。

`USB TX` 本地诊断最多每秒在计数变化时记录：peak、retry_peak、full、drop、critical_drop、last_type、deferred、busy、submit_error、log_drop。
full 为容量/待发送策略压力事件，不等于 drop；deferred 为遥测延后尝试次数，不是丢弃帧数量；busy 与其他提交错误分别记录。它们没有加入 USB v1 STATUS payload，payload 中 tx_drop 仍表示实际入队失败后未保存的帧。

延后的日志可能在其他任务较新的日志后输出较旧 MCU 时间戳，不能单凭串口行的时间顺序判定复位；应结合 Boot Banner、USB uptime 和事件内的 snapshot 时间。

## 4. 修改文件

生产文件仅 7 个：

- `Application/Communication/usb_cdc_transport.c`、`.h`
- `Application/Communication/chassis_usb_protocol.c`
- `Application/Services/debug_log.c`、`.h`
- `Application/System/firmware_info.h`
- `Core/Src/freertos.c`：仅 USER CODE 区增加日志输出调用。

已与冻结 0.13.2 源码清单逐个哈希对照：`.ioc`、顶层/CubeMX CMake、其他 Application 文件、Drivers、Middlewares、USB_DEVICE、链接脚本及其他 Core 文件未改；freertos.c 的 USER CODE 区外文本相同。

测试/交付配套：

- 更新 `Tools/tests/usb_cdc_transport/test_usb_protocol_stop.c`，添加可启用 BATTERY/IMU 的 fixture 和日志作用域 mocks。
- 更新对应 `stubs/usbd_cdc_if.h` 与 README。
- 新增 `test_usb_tx_queue.c`、`test_usb_tx_burst.c`。
- 新增 `Tools/tests/debug_log/test_debug_log.c` 及 6 个 host-only stub headers。
- 新增 `Tools/tests/run_usb_tx_tests.ps1`，不接串口、不烧录。
- 更新 `Tools/usb_stop_regression.py` 的预期 version/Build；现有 ground probe 自动引用这份身份，旧启动日志不能通过新版本运动测试入口。
- 本交付说明、协议调度说明以及 build 下构建/测试日志和冻结产物。

## 5. 实际构建与验证

| 项目 | 结果 |
| --- | --- |
| Debug clean build | 88 步真实 ARM 构建成功 |
| Error / Warning | 0 / 0，按完整构建日志检查 |
| FLASH | 160048 B / 768 KiB，20.35% |
| RAM | 54928 B / 128 KiB，41.91% |
| CCMRAM | 0 B / 64 KiB |
| native tests | 5 个程序全部通过，编译 -Wall -Wextra -Werror |
| 主机测试工具 | 本地 self-test / Python 语法检查通过，无串口操作 |
| HEX | 10011 条记录 checksum 全部正确，数据 0x08000000..0x0802712F |
| ELF→HEX 一致性 | 再 objcopy 生成的 HEX 与交付 HEX SHA256 相同 |

测试覆盖：14 个 SOF 快照用例；50 轮真实 parser 的 STOP/GET_STATUS/重新运动；50/51 ms 与 tick 回绕；DTR/SOF/撤配置；满遥测下预留/待发送缓存与 FIFO；BUSY 重试；完整 BATTERY/IMU 突发；延后不消耗序列号；极端拥塞丢弃真实计数且 STOP 执行；日志启动前安全、任务作用域及嵌套、截断、满缓存、UART/互斥锁失败和 ISR 拒绝。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File Tools\tests\run_usb_tx_tests.ps1
```

这些是电脑上模拟 HAL/RTOS/其他服务的生产代码测试，不能替代实板中断时序、停车延迟、实际栈高水位、USB/DTR 恢复、100 Hz IMU/20 Hz WHEEL 实测。

## 6. 烧录及验收顺序

1. 先关闭电机 VM，再由用户烧录冻结目录中的 `rosDemo.hex`。不要误用旧版本或未经本次验证的 Release 文件。
2. 若要保留 bias，不能全片擦除或擦除 Flash sector 10/11 (`0x080C0000..0x080FFFFF`)；本次 HEX 没有这些地址，实际下载工具的擦除选项仍需用户核对。
3. 恢复正常启动，CH340 115200 8-N-1，确认上述 version/Build。启动日志检查 bias restore 和任务启动，无重复复位。
4. 先 `python Tools\usb_cdc_test.py --port <实际CDC端口>`，检查 IDLE、无 fault、flags 覆盖 0x0150、零 CRC/overflow/drop。检查 IMU/WHEEL 采样率和日志输出；没有运动授权时不要运行带运动参数的工具。
5. 再由用户确认四轮架空/场地安全后做 10 轮短启停及 DTR 回归，双路日志先保存再运动。DTR、物理断链及命令超时仍必须停车，恢复不能回放旧速度。
6. 短启停和通信通过后才开展用户希望的较长行程：可用 80 cm 空间内先取约 50 cm 目标，40 mm/s，保留余量，按计数估计行程加最大运行时间限制，再前后 STOP/至少两分钟静置。当前 ground probe 仍是 0.8 秒短行程，不能把它当作 50 cm 指令；长行程入口另行准备并确认后使用。

目前没有自动烧录或进行新固件实板运动。历史 LF 静止 4 计数仍未定位；本次旧硬件日志中的队列满问题也必须在新版本板上做对照后才能宣布验收。

## 7. 交给 ROS 开发者

交付包 `build/deliveries/STM32_USB_TX_BURST_FIX_0.13.3_20261012.zip` 包含冻结 HEX/ELF、说明、构建/离线测试日志、源码快照和哈希清单。
ROS 层暂不删链路位门控，不屏蔽计数/故障，不改变已有协议；注意背压可能造成遥测间隔变长，读者应用 MCU 实际采样时间，不能假定每帧固定间隔。
请回传新版本启动身份、启停/DTR/断链结果、完整 USB RX/STATUS、USB TX 诊断及同窗口 CH340 原始日志。
