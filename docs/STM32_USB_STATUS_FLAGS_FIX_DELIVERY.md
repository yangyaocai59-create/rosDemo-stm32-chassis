# STM32 STOP / USB STATUS.flags 修复交付

日期：2026-10-11。工程：`E:\DEMO\rosDemo`。
固件版本：`0.13.2-usb-sof-snapshot-fix`（Debug）。
ELF 内嵌启动 Build：`Oct 11 2026 09:04:30`。

## 1. 结论和证据边界

正常 `CMD_STOP` 不应清除 `0x0010`。它停止控制器、清零速度请求，
清除 COMMAND_VALID (`0x0001`) 和 CONTROLLER_ACTIVE (`0x0020`)，
健康运行时进入 `IDLE / EXPLICIT`，不主动改变 USB 配置、SOF 或 DTR。

确认当前固件有真实的时间采样竞态，不只是 ROS 猜测：任务先取得 now，
后续 USB SOF 中断更新 last_sof，旧 now 与新 last_sof 相减会下溢。
例如 1000 - 1001 得到 4294967295，从而误判 SOF 超过 50 ms。
STATUS 和 Process 会话判断均受影响。STOP 状态切换中的阻塞 UART 日志
可以扩大这个时间窗口；UART 发送不在 USB 快照临界区内。

新增主机侧协议回归直接编译生产协议/传输源码，并模拟 STOP 处理期间
经过 20 ms、SOF 持续更新的情形。同一回归使用交接中的旧 `rosDemo2`
源码，第一轮 STOP 后输出 `actual STATUS flags=0x0140`；新源码完成 50 轮。
因此该缺陷在固件路径上已独立复现并修正。

但历史板上那一次缺位没有 MCU 原始时间/会话快照，不能据此断言它的
唯一原因就是这个竞态。真实 SOF 间断、撤配置或 DTR 变化仍须实测排查。
本次没有烧录、打开硬件串口、驱动电机或运行 ROS 实板测试。

## 2. 实际修改

修改的固件源文件只有四个：

- `Application/Communication/usb_cdc_transport.h`
- `Application/Communication/usb_cdc_transport.c`
- `Application/Communication/chassis_usb_protocol.c`
- `Application/System/firmware_info.h`

另更新 `docs/usb_cdc_protocol_v1.md`，新增本说明和
`Tools/tests/usb_cdc_transport/` 内的两项主机测试、三个模拟头文件和 README。

新增 `UsbCdcTransport_GetLinkSnapshot()`，在一个很短的任务临界区内取得：
当前 HAL tick、last_sof、configured、SOF seen/monitor、DTR 及事件计数。
在临界区外计算年龄和链路状态；没有日志、UART/USB I/O 或等待操作。
`UsbCdcTransport_IsLinkAlive()` 不再接收调用者的旧时间。

STATUS 的位 4 和位 6 使用同一个新快照。Process 的链路/DTR 分支也使用
同一个新快照，进入处理时的会话/调度时间取该快照的当前 tick。
不再用旧任务时间与 ISR 更新时间拼接判断，既修复上报也修复会话判断。

临界区依据已核查的实际配置：OTG_FS IRQ 优先级 5、HAL TIM7 tick
优先级 15、FreeRTOS `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5`。
`taskENTER_CRITICAL()` 能屏蔽这两个更新时间的中断。调用限于任务上下文；
以后若 CubeMX 修改中断优先级或 HAL tick 来源，必须重新审计此约束。

没有修改 `.ioc`、引脚、CubeMX 生成代码/CMake、USB 描述符、协议消息
格式/长度/偏移/位号、轮子方向、控制参数、编码器、IMU 校准或 Flash 日志。
交接快照中 12 个未修改的关键文件重新比较 SHA-256，均一致，包括 runtime、
controller、FreeRTOS、USB ISR/回调和 IMU 校准代码。

## 3. 保留的安全机制与位语义

- SOF 超时仍为 50 ms：年龄 <=50 存活，51 ms 失效；没有扩大阈值。
- 撤配置、真实 SOF 超时、DTR 关闭仍清解析器/应用会话/待发送队列；
  ACTIVE 时调用 `ChassisRuntime_StopForTransportLoss()`。
- DTR 关闭路径仍丢弃未处理 RX；恢复连接不会自动重放旧运动命令。
- 250 ms 速度命令超时、ESTOP、Fault Manager 和 IWDG 逻辑未修改。
- 没有强制置位 `0x0010`，没有删除 ROS 所需位或调整 ROS 恢复策略。

| mask | 语义 |
|---|---|
| `0x0010` | legacy 名 USB_CONFIGURED，实际为 configured 且 SOF 链路存活 |
| `0x0040` | CDC host port open；控制线请求出现后按 DTR，之前保留兼容回退 |
| `0x0100` | IWDG 已 armed、任务健康且刷新资格正常 |

健康停车应覆盖 `0x0150`。运动中还有 `0x0001/0x0020` 等位，不能要求
整个 flags 始终等于 `0x0150`；应使用 `(flags & 0x0150) == 0x0150`。
真实 SOF 超时但配置和 DTR 仍在时，仍可能合法出现 `0x0140` 并触发停车。

## 4. Debug 诊断日志

本次镜像在 LINK-configured、LINK-lost、DTR-closed、STOP 和 STATUS-not-alive
事件处增加 `USB SNAP`；不逐个 SOF 打印。仅 `DEBUG` 构建且
`DEBUG_LOG_ENABLE_DEBUG=1` 时输出，不改变二进制协议。

```text
[DEBUG] USB SNAP STOP task=1000 now=1020 last=1020 age=0 sof=...
[DEBUG] USB SNAP cfg=1 mon=1 seen=1 alive=1 ctrl=1 dtr=1 open=1 events=1/0/1
```

这是格式示例，不是实板采集。`task` 是调用时间，`now` 是一致快照时间，
`last` 为 last_sof，`age` 为无符号差（支持 tick 回绕），`sof` 为累计 SOF
通知次数。`events` 顺序为配置次数/撤配置次数/控制线请求次数；`ctrl`
表示是否观察过控制线请求，`dtr` 为观察到的原始 DTR，`open` 含兼容回退。
日志在快照完成后才发送，UART 行上的时间戳不等同于快照采样时刻。
增加的事件日志会影响主机观察延迟，不应把主机 22 ms 当作 MCU SOF 间隔。

## 5. 实际构建和离线回归结果

执行了实际 `cmake --build --preset Debug --clean-first`，88 个构建步骤。
工具链 ARM GCC 14.3.1 + CMake 4.3.1 + Ninja。

| 项目 | 结果 |
|---|---|
| Debug clean build | 成功，退出码 0 |
| Error / Warning | 0 / 0（完整构建日志计数） |
| FLASH 链接占用 | 157256 B / 768 KiB，20.00% |
| RAM 链接占用 | 49928 B / 128 KiB，38.09% |
| CCMRAM | 0 B / 64 KiB |
| 新传输单元测试 | 14 个快照通过，原生产传输代码 + 模拟 tick/IRQ |
| 新协议集成回归 | 50 轮通过，原生产 parser/dispatch/STATUS/会话 + 模拟业务服务 |
| 旧源码负对照 | 第一轮 STOP 出现 0x0140，按预期断言失败 |

覆盖旧时间下溢、临界区前后 IRQ 更新、50/51 ms、uint32 tick 回绕、DTR
兼容/关闭/重开、撤配置、队列清理，以及断链恢复后必须等待新指令。
本机测试编译使用 native GCC `-Wall -Wextra -Werror`，不是 ARM 固件执行。
其他业务模块为模拟实现，不代表已验证真实电机停车时间和板上中断行为。
复跑方法见 `Tools/tests/usb_cdc_transport/README.md`。

HEX 校验和检查通过；从 ELF 再次转换出的 HEX 与交付 HEX 的 SHA-256 一致。
HEX 数据地址范围为 `0x08000000..0x08026647`，没有包含 IMU journal 区。

## 6. 镜像、源码身份和交付文件

冻结交付目录：`build/deliveries/usb_sof_fix_0.13.2_20261011/`。

- `rosDemo.hex`：本次实际 clean build 的可烧录镜像。
- `rosDemo.elf`：同次链接产物，用于调试和版本核查。
- `STM32_USB_STATUS_FLAGS_FIX_DELIVERY.md`：本说明。
- `usb_sof_fix_clean_build.log`、`usb_sof_fix_host_tests.log`：构建/离线回归证据。
- `source_snapshot.zip`：当前工程源码快照（不含 build 或历史复现附件）。
- `SOURCE_SHA256SUMS.txt`：当前源码文件清单/校验值。
- `SHA256SUMS.txt`：交付目录文件校验值。

Git 提交号：**无法提供**。此工作区及父目录没有 `.git`，也没有可用的
Git 命令。本次没有擅自创建新仓库或伪造提交；版本、完整源码快照和
SHA-256 用于关联交付。若必须有提交号，请由项目维护者在正式仓库导入
本次变更并提交，另行记录提交号与以下镜像哈希的关系。

```text
HEX SHA256 = 50CA002B029CA58D9451BF7F77260D686EC9B0665DBDB0B49C49932D2B79F7F0
ELF SHA256 = CB68E483B1BE9CB64FC1DF4F8BB5A73C19DC8BB212C8EA1A1497EBE624FA5F94
```

实际烧录身份必须由板上 CH340 启动日志确认 version 和 Build，不能只根据
源码目录名或 STATUS 推断。不要烧录旧 `build/Release` 文件；本次没有构建
或验证 Release。以后重新编译会改变 Build/哈希，应重新记录交付身份。

## 7. 烧录及实板对照（待执行）

1. 四轮架空并固定车体，断开电机 VM 电源后烧录上述冻结 HEX。
   保留 IMU bias 时不要执行全片擦除，也不要擦除 Flash sector 10/11
   (`0x080C0000..0x080FFFFF`)。具体下载工具的擦除设置由操作者核查；
   若全片擦除，已有 bias 会丢失，需要重新显式校准。
2. 返回正常启动模式，CH340 115200 8-N-1 记录完整启动日志，必须看到
   `0.13.2-usb-sof-snapshot-fix` 和 `Oct 11 2026 09:04:30`。
3. 先只读检查（COM4 应替换为实际 CDC 端口，不是 CH340）：

   ```text
   python Tools/usb_cdc_test.py --port COM4
   ```

   期望健康 IDLE、请求速度零、无 fault、flags 覆盖 0x0150、uptime 连续。
4. 上位机人员在原 Ubuntu 环境使用原 `hardware_stop_probe.py` 做同设备、
   同脚本对照，至少 10 轮：30 mm/s、20 Hz、0.35 s，然后 STOP、观察 0.7 s。
   脚本依赖原 ROS 工作区、rclpy/生命周期服务、固定 by-id 和其 Tools 路径；
   不要在当前 Windows 工程直接照搬执行。原脚本与历史证据未修改。
   独占 CDC，无其他 cmd_vel 发布者，保留原脚本 deactivate/restore 流程。
   保持 USB/DTR 连续，不应再由旧时间产生 0x0140；遇到不安全状态立刻停止
   追加运动，保留原始 events.jsonl、summary.json 和同期 CH340 日志。
5. 随后四轮仍架空，分别验证 DTR 关闭、拔掉 USB DEVICE 数据线和命令超时。
   使用原 `--dtr-stop-test`、`--disconnect-test`、`--drive-test` 工具；这些会
   发运动指令，必须由操作者现场执行。拔 USB 只拔 STM32 CDC，保留 CH340
   和 MCU/电机供电，以便证明停车而非断电；观察 TRANSPORT_LOST/停轮。
   恢复连接后应保持停车，直至收到新的速度，不应回放旧命令。
6. 最后由 ROS 人员验证端到端：正常 STOP 后新的 cmd_vel 能重新运动；
   真正异常仍触发门控，显式恢复后等待新指令。不要删除 required bit 0x0010
   或在收到一帧正常 STATUS 后无条件恢复此前速度。

请回传：烧录镜像哈希、启动 version/Build、至少 10 轮结果、出现异常时的
完整 USB SNAP/原始 STATUS，以及真实断链/DTR 停车结果。日志中若仍有
0x0140，应依据 age、cfg、seen、DTR 和事件计数继续定位，不应宣称已验收。

## 8. 本次未处理

- 历史复现中的 RF 轮最大速度仅 5 mm/s，而其余约 45–50 mm/s，需单独核查
  实际转动/编码器/驱动输出，本次不更改方向、PI 或机械参数。
- 板上启停、真实断链和完整 ROS 运动恢复均尚未验收。
- Git 提交号待正式仓库维护者补齐。

此交付仅修复 USB/SOF 采样及关联会话判断，不进入其他功能阶段。
