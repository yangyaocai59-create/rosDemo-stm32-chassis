# STM32 → ROS 工程师：USB STOP 与静止编码器排查最终交接

日期：2026-10-12，Asia/Shanghai。工程：rosDemo / STM32F407ZGT6。
这是本轮工作交接，不是整车发布验收或所有历史问题根治声明。

## 1. 先看结论

1. 正常 STOP 不应清除 USB/SOF 链路位 `0x0010`。旧固件中任务旧 now 与 ISR 新 last_sof 相减的真实竞态已在生产代码路径离线复现并修复；当前交付保留该修复。
2. 在后续 Windows 双路实板测试中又捕获 STOP 附近 tx_drop 0→1。旧有效 7 帧 TX 队列的突发拥塞机制已离线复现；0.13.3 增加有界队列/关键应答预留与重试、发送推进和遥测背压，并延后 USB 路径阻塞串口日志。
3. 当前板上 0.13.3 完成短行程和约 50 cm 编码器估算前后行程，每轮后静置 120 s；无缺链路位、CRC/溢出/TX drop 增量或停稳后计数增长。用户确认长行程实物前后运动正常，停车无继续爬行，并要求不再追加重复测试。
4. 历史后退停车约 33 s 后 LF 累计计数增加 4 的根因仍未确定。本轮未复现，不冻结 IDLE 计数、不吞小增量、不清零累计值，也不把 USB TX 修复当作编码器异常的修复。
5. 正式 ROS 驱动的运动权限恢复未在此 Windows 工作区执行；请上位机工程师在原 Ubuntu 工作区验收。不要删除 `0x0010` 门控、屏蔽 fault/计数或自动回放故障前速度。

原 `C:\Users\yang\Desktop\STM32_ENGINEER_HANDOFF.md` 与 `对话.txt` 在最终整理时已不在原路径，也未找到同名副本。因此本文依据此前已审阅的需求、保留的工程排查报告和原始测试证据整理，不声称重新读取了已缺失的原件；若原件还有未记录的附加要求，请另行补充确认。

## 2. 板上身份、构建和提交号

```text
Firmware version = 0.13.3-usb-tx-burst-fix
Build = Oct 12 2026 08:24:27
MCU = STM32F407ZGT6
HEX SHA256 = D5F7AC2C67F0FA984CDD12E92AECEC786FB50E639D8BF0C85F6E03966E7B2508
ELF SHA256 = 16CF592176C4EBEAABD7D3D568D5290D7E4D54D0D65D5A79FC2231EF6DBD8F41
```

- 同次真实 Debug clean build：88 步，Error=0、Warning=0。
- FLASH=160048 B / 768 KiB（20.35%）；RAM=54928 B / 128 KiB（41.91%）；CCMRAM=0 / 64 KiB。768 KiB 应用上限是为了保留 IMU bias journal，不能改回全 1 MiB 使用。
- 最终交接时复核上述 HEX/ELF 哈希一致；按冻结源码清单核对当前 250 个生产/硬件配置文件，全部匹配。当前脚本与报告的新增不改变板上固件。
- 最终再次执行 5 项 native 生产代码回归，全部通过。它们模拟 HAL/RTOS/硬件，不是实板端到端测试。
- Git 提交号：无法提供。工作区及父目录没有 `.git`；没有新建仓库或伪造提交。正式仓库维护者可导入本次源码并提供提交号，关联到上述镜像哈希。
- 用户已烧录，并提供匹配版本/Build 的启动日志。这是日志身份验证，不是 Flash 全量回读验证。不要用旧 Release 文件替换。

## 3. 修改范围与未修改项

0.13.3 相对冻结 0.13.2 的生产文件只有：

- `Application/Communication/usb_cdc_transport.c`、`.h`
- `Application/Communication/chassis_usb_protocol.c`
- `Application/Services/debug_log.c`、`.h`
- `Application/System/firmware_info.h`
- `Core/Src/freertos.c`：仅 USER CODE 区增加延后日志消费。

0.13.2 先前已在 transport/protocol 中修正一致 USB 时间/会话快照，0.13.3 继承它。

`.ioc`、引脚、CubeMX 结构和 USER CODE 外生成区域、CMake、USB 描述符、USB v1 帧格式/长度/消息 ID、编码器回绕与累计规则/极性/滤波、方向、PI、几何参数、ADC、IMU/校准/Flash journal、ESTOP、Fault Manager/IWDG 均未因本次 USB 修复而变更。

STOP 仍调用驱动 Disable：PD10 公共 STBY LOW、PD11 保留 LOW、四路 CCR=0、方向 LOW。正常运行后 STOP 不等于执行 `HAL_TIM_PWM_Stop`，通道可继续运行 duty=0。无主动位置保持；上述电气输出行为来自源码，未在本轮用仪表直接核验。

## 4. ROS 接口兼容与保护约束

| mask | 实际含义 |
| --- | --- |
| `0x0010` | legacy 名 USB_CONFIGURED，实际为 USB configured 且 SOF 链路存活 |
| `0x0040` | CDC host port open；收到控制线请求后按 DTR，之前保留兼容回退 |
| `0x0100` | IWDG armed、任务健康、刷新资格正常 |

健康停车应满足 `(flags & 0x0150) == 0x0150`；不要要求运行时 flags 整体等于 0x0150。本轮运动中 0x0171 正常。真实断链时 0x0140 仍可能合法出现，不能在驱动中一概忽略。

一致快照在短任务临界区同时读取新 HAL tick、SOF 时间及会话；没有扩大 50 ms SOF 门限（50 存活，51 超时）。IRQ 优先级及 HAL tick 来源如以后重新生成改变，须重新审计。

TX 采用有效 15 帧 FIFO（普通遥测最多占 11 帧），另有有效 3 帧关键应答缓存和 1 帧 in-flight；不重排已入队帧。USB BUSY 重试，遥测背压后取新快照，不伪造采样率。极端负载仍有界，真正丢弃仍真实计入 tx_drop。

USB v1 wire 格式不改。接收端使用 MCU 采样时间处理 wheel/IMU，不因调度或背压假定每个主机到达间隔固定。`USB TX` 压力诊断只在 CH340 本地输出，未加入 STATUS payload。

真 SOF 丢失、撤配置、DTR 关闭、250 ms 命令超时、急停和故障停车机制保留；恢复通信后等待新指令，不回放旧速度。ROS 显式恢复策略仍由上位机维护，不能把一帧健康 STATUS 当成自动恢复授权。

延后日志保留事件生成时间，串口输出顺序可能出现旧时间戳晚于其他任务新时间戳；是否重启应结合 Boot Banner/USB uptime 判断。

## 5. 实板证据索引

所有计数数组顺序均 LF/LR/RF/RR。

| 固件 / 条件 | 实测结果 | evidence 子目录 |
| --- | --- | --- |
| 0.13.2 静置 120 s | 2400 WHEEL，无计数变化，flags=0150，错误计数零；有同窗口 UART 对照 | `static_0.13.2` |
| 0.13.2 短前后运动后静置 120 s | 本轮未复现计数异常；当时 UART 运动窗口缺失，不用它冒充双路证据 | `short_0.13.2_usb_only` |
| 0.13.2 双路短行程 | 前进 STOP 附近 tx_drop 0→1；中止，未执行后退和静置；最后 STOP 成功 | `before_tx_fix_0.13.2` |
| 0.13.3 双路短行程 | 前后各 30 mm/s ×0.8 s；1450 STATUS、2400 静置 WHEEL，无缺位/错误/计数增长 | `short_0.13.3` |
| 0.13.3 双路长行程 | 前后各 40 mm/s、约 12.4 s；平均编码器行程 503.31/503.37 mm；1682 STATUS、2400 静置 WHEEL，无缺位/错误/计数增长 | `long_0.13.3` |
| 0.13.2 较早 10 轮架空启停 | 285 STATUS，正常 STOP 后再次运动；旧 DTR 试验停车但 tx_drop +1，完整 DTR 判定失败 | `earlier_sof_fix_0.13.2` |

新固件两轮静置窗口都有左右各 600 条 UART ENC，对应 raw/d/total 与 USB WHEEL 一致。USB原始 RX 字节、命令、ACK/STATUS/WHEEL、阶段边界保存在各目录 `events.jsonl`；判定在 `summary.json`，双路原始串口为 `ch340_raw.bin`，主机时间分块为 `ch340_chunks.jsonl`。

不要把旧版静置/架空测试混写为 0.13.3 的结果。新版本两次正常关闭端口发生在停车之后，不是运动中 DTR 停车验证。

用户最后反馈：长行程实际前进、后退均正常，停车未继续爬行；不再重复测试。地面距离、横向偏差和回到起点未外部测量，不据此验收运动学/里程计精度。

## 6. IMU 与供电补充

0.13.3 首次烧录后的启动没有兼容 bias；随后用户操作校准，最新启动日志显示 A/seq=1 并恢复 `raw=[-112,-2,-85]`，持续 bias=1、静止角速度接近零。本次未更改持久化逻辑；不要把首个日志缺记录直接认定为保存缺陷或确定为全片擦除。

若后续需重烧并保留 bias，禁止全片擦除或擦除 sector 10/11（`0x080C0000..0x080FFFFF`）；本 HEX 不包含该区域，但下载工具擦除选项仍由操作者核验。

PC1 ADC 监测的是 D24A 下游 VM（电池经过 DC-DC），不是电池原始端电压。MONITOR_ONLY 不提供已验证的低电保护/SOC；不要依据这一路直接套 3S 电池阈值。供电电压及波形/编码器真实微动在本轮未仪表核验。

## 7. 上位机工程师接下来做什么

1. 先核对本文版本/Build 和 firmware 下镜像哈希。当前板上已运行此版本，不需要因为新增报告再烧录。
2. 使用原 Ubuntu ROS 工程核对健康 STATUS、正常 STOP 后收到新 cmd_vel 能继续运动、没有错误锁定运动权限。不得有其他速度发布者争用 CDC/命令所有权。
3. 当前用户不再追加本机重复运动。0.13.3 运动中 DTR/物理拔线/命令超时实板回归、重复启停、实际栈高水位仍未完成；不得记成 PASS。另行安排时需要现场授权、安全场地和双路日志，真拔线需保留 MCU/电机电源以区别断电停车。
4. 若再出现缺位：保留 STATUS 原帧、MCU uptime、USB SNAP 的 now/last/age/cfg/seen/DTR/events、同期串口和主机连接事件；若 tx_drop 增长，再记录 USB TX peak/retry_peak/full/critical_drop/last_type/deferred/busy/submit_error/log_drop。
5. 若再出现静止计数变化：记录原始 CNT/delta/累计值与同时间 WHEEL，不清零/吞计数；补充实际轮子微动、驱动使能/PWM及编码器电气状态。当前没有证据支持直接修改极性/滤波/PI 或硬件。

### 可直接发给 ROS 工程师的摘要

> 当前 STM32 固件为 0.13.3-usb-tx-burst-fix，Build Oct 12 2026 08:24:27。它包含一致 SOF 时间/会话快照修复，以及正常 STOP 突发应答的有界 TX 保障和 USB 日志延后；协议 v1 与安全门控不变。Windows 双路短行程和约 50 cm 编码器估算前后行程回归通过，各自停车静置 120 s 未观察到计数增长，STATUS 无缺链路位或 CRC/overflow/tx_drop 增量。用户确认实车运动/停车正常，并停止追加重复测试。历史 LF +4 根因仍未确定；原 ROS 端权限恢复以及新版本运动中 DTR/物理断链等实板回归待验收，不应关闭历史问题或绕过安全位。源码、烧录文件、完整日志及校验清单见本交付包。无 Git 提交号，请正式仓库维护者导入后补齐。

## 8. 包内文件说明

- `firmware/`：冻结 HEX/ELF，不是重新编译的同名产物。
- `source/`：与镜像对应的冻结源码 zip、源文件 SHA 清单和原交付文件清单。源码快照中的早期测试说明保留原日期；本报告为最新状态。
- `build_logs/`：当时真实 clean build 与 host tests 输出。
- `reports/`：本报告、两次新实板回归、此前诊断、协议以及硬件基础文档。部分旧报告正文是当时尚未测试的状态，必须按日期结合本文读取。
- `evidence/`：上述六组原始记录，包含失败记录，不仅打包 PASS。
- `host_tools/`：最终主机工具。50 cm 入口需明确参数；不能把离线 `--self-test` 误当运动，也不能照搬 COM 编号。工具只供参考，不替代原 ROS 生命周期/权限恢复测试。
- `SHA256SUMS.txt`：本包各文件（除清单自身）的校验；解压后可由 sha256sum -c 检查。冻结源码有其独立 SOURCE_SHA256SUMS。

不包含或声称完成 Nav2/SLAM。没有更改上位机源码、发送消息给第三方或上传交付包；由用户转交工程师。
