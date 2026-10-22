# 0.18.13 前轮恢复失败保护候选

日期：2026-10-21（Asia/Singapore）。版本：`0.18.13-front-recovery-failsafe`。已完成代码、回归、ARM 编译和镜像校验，用户报告烧录完成。[只读检查](stm32_evidence/2026-10-21-v01813-postflash/README.md)正常后，经用户确认执行[架空左右各 6 秒](stm32_evidence/2026-10-21-v01813-lifted-pivot/README.md)：目标 ±160 cps，前轮末段约 162..165 cps，四轮连续、无故障或保护误触发，均安全停车；用户确认无明显异常。随后[落地右转](stm32_evidence/2026-10-21-v01813-ground-failsafe/README.md)自然重现 RF 连续近零，1500 ms 后锁存 RF 堵转，下一条 trace 四路 PWM=0、STBY=0；运动后主机未发 STOP 或清故障，最终仍故障且零速。恢复失败保护验证通过，但 RF 落地低速停顿仍未解决。未捕获新启动版本行，身份依据用户烧录报告。本版未推送 GitHub。

## 修复的问题与实际能力

[0.18.12 复核](stm32_evidence/2026-10-21-v01812-front-rest-audit/README.md)确认：前轮一次临时补力耗尽后，近零速条件仍冻结同向积分；RF 目标 -160 cps 且反馈持续为零时，PWM 可以长期停在 -133‰。原堵转检测的目标门槛 200 cps 不覆盖此工况，不能用 fault=0 判断控制/硬件正常。

本版为这个状态提供有限结束路径：保持已有有界补力，在持续无运动时锁存恢复失败，由现有安全任务关闭全部轮输出。**它修复持续失败却不停车的保护死角，不保证某个负载下电机一定起转，也未解决所有欠速或起步超调。** 机械接触、正常侧向摩擦、供电/驱动及带载反馈问题仍可能需要实测。

## 生效条件与时序

仅用于 LF/RF 的原地转向策略，沿用已有 PIVOT 判断与目标门槛：|vx|≤1 mm/s、|wz|≥10 mrad/s；该轮目标幅值至少 20 cps。

1. 近零速阈值仍为 `min(20 cps, |target|×25%)`，使用原有滤波轮速；目标 160/280 cps 时阈值均为 20 cps，目标 20 cps 时为 5 cps。
2. 新增 `no_motion_ms`，按实际有效控制步 dt（1..100 ms）连续累计近零速时间，上限 1500 ms；尚未失败时，任一超过近零阈值的反馈会清零这个计时。不是累计所有离散零计数。
3. 原先 60 ms 近零确认、最多 300 ms 主动恢复、有限渐退、load/I/recovery 力度预算继续使用。达到 1500 ms，且一次恢复已耗尽、临时量已退出，设置 `recovery_failed`。
4. 一旦设置，正常运动反馈不能在安全任务读取前擦掉这个报告，也不会再次给补力脉冲。换向、退出策略或 Stop/Reset 清理局部状态；已进入 FaultManager 的全局故障仍保持锁存。
5. `FaultManager_ObserveMotion` 在运行有效、前轮策略有效、当前目标方向匹配时消费报告，沿用 `FAULT_MANAGER_FAULT_WHEEL_STALL`（bit 2）及轮掩码。此分支不受旧 200 cps/100‰/启动宽限门槛阻挡，因为前轮已自行确认连续近零速期限。
6. safetyTask 在同一轮处理故障并调用既有 `ChassisRuntime_LatchHealthFault`，关闭共同 STBY、四路 PWM，重置轮控制器及速度命令。安全任务正常运行时，在其下一次观察处理（名义周期 10 ms）关断。1500 ms 是有效控制步累计期限，不是独立硬件定时器；初始步记账和调度会带来控制周期量级的差异。

原通用堵转规则（目标≥200 cps、输出≥100‰、1000 ms 启动宽限、750 ms 检测）保留。前轮新规则可更早处理持续近零速；后轮、NORMAL 模式、目标低于 20 cps 的工况不新增这条保护。**这不是全速度范围通用堵转检测，也不是电流保护。** 短时单帧零计数、部分欠速或反向高速不是这条连续近零规则的触发条件。

## 停车、恢复与 ROS 兼容性

- 新规则使用现有轮堵转故障位、健康故障门控和整车停止路径。USB 帧长度、消息号、单位、RAW v2、编码器与 IMU 数据处理未修改。
- 典型 STATUS 为 `state=FAULT`、`stop=FAULT`、运行故障=5（health manager）、控制器故障=0、请求速度为零；现有 HEALTH_FAULT 标志置位。其余 flags 随 USB 会话、看门狗健康状态变化，不应硬编码成单个固定值。
- CH340 沿用 `Fault manager latched wheel stall: wheel_mask=...`，LF=0x01、RF=0x04。新增启动提示 `Front pivot no-motion recovery limit=1500ms; failure latches wheel stall`。
- 故障后新速度命令被拒绝，不能反复发送 `/cmd_vel` 自动恢复。既有 `CMD_CLEAR_FAULT (0x03)` 在停止且健康条件满足后可显式清除；清除本身不会恢复旧命令，需要新的速度命令。处理现场原因后再清除。
- ESTOP、STOP、250 ms 命令超时、SOF/DTR/真实 USB 断链、任务健康与 IWDG 机制保留。轮堵转使用既有故障位，任务都健康时 IWDG 仍正常刷新，不依赖看门狗重启解除故障。
- 所有总 PWM、积分、斜率、前馈系数和补力上限与 0.18.12 相同；`chassis_controller.c`（含后轮策略）与 0.18.12 文件哈希一致。

## 源码与离线验证

相对 0.18.12，本次生产修改为：

- `Application/Chassis/wheel_speed_controller.c/.h`：连续近零计时和粘滞失败报告。
- `Application/Chassis/wheel_speed_control_config.h`：1500 ms 期限。
- `Application/Safety/fault_manager.c/.h`：在原安全观察中锁存前轮恢复失败，接口签名不变。
- `Application/System/firmware_info.h`：版本字符串。

`run_motion_tests.ps1` 已通过，保留既有核心、驱动、后轮及运行安全测试，新增/扩展：

- 两前轮/两方向，20/40/160/199/200/280 cps 与 1/10/20/100 ms dt；1500 ms 边界、计时中断、失败后迟到运动不擦报告、不重复补力、换向/模式退出/Reset 清理、低目标正常跟踪、目标边界及限幅。
- `test_front_recovery_safety.c` 链接真实轮控制器、底盘控制器、运动整形、运行状态机和 FaultManager；只有外设/反馈为桩。约 20/160/280 cps、LF/RF、左右转、tick 回绕场景均触发；验证四轮输出和 STBY 关闭、故障锁存、后续命令拒绝、显式清除后不恢复旧命令、新命令可正常启动。
- 分隔的短时近零片段不累计误报；移动前轮、低速后轮、NORMAL 隔离；旧后轮 1000+750 ms 检测仍成立。原 STOP/零命令/断链/急停/精确 250 ms 超时/时序错误/超速硬停回归通过。
- USB 回归通过：STOP/重启、SOF/DTR/真实断链逻辑、ACK/TX 队列、RAW v2 精确偏置、IMU 采样时间戳与日志隔离。

原诊断程序对照结果：0.18.12 的 RF -160/-199、LF +160 cps 在 6000 ms 仍无故障；本版这些持续零反馈场景都在第 1500 ms 锁存故障，RF -200/-280 也由新前轮规则在 1500 ms 报告。诊断程序故障后的独立 PI 打印不是实际电机继续驱动的证据；完整 runtime 集成测试验证了真实软件关断链路。主机测试不模拟真实电机动力学、地面摩擦或 GPIO 电气响应。

日志：`build/hardware_tests/v01813_motion_regression.log`、`v01813_usb_regression.log`、`v01813_front_rest_audit.log`、`v01813_debug_build.log`。

UTF-8 归档和镜像/源码校验元数据见[离线验证目录](stm32_evidence/2026-10-21-v01813-offline/README.md)。

## 编译与文件

STM32 GNU 14.3.1+st.2 / CMake Debug，95 步 clean build，0 错误、0 警告。

- FLASH 176784 B / 768 KiB；RAM 80704 B / 128 KiB；CCMRAM 0 B。
- 相对 0.18.12：FLASH +524 B、RAM +32 B。新增本地前轮状态不是 wire 格式。
- 静态栈帧参考：`FaultManager_ObserveMotion` 104 B、`StartSafetyTask` 32 B。这不是运行时栈余量验证，烧录后仍需检查任务周期和高水位。

烧录目录：`build/firmware/0.18.13/`

```text
rosDemo-0.18.13.hex   推荐按原流程烧录
rosDemo-0.18.13.bin
rosDemo-0.18.13.elf
rosDemo-0.18.13.map
verification.json
source_manifest.json
```

```text
HEX SHA256 09759895c7aa9dee66ad3ed5b5b2fbd57ad2faa244a67d8f1cd79dc8767de63a
BIN SHA256 e13047f804747a8f7db7c4df4549d220f238d9113e0e29cfd29954a1cf0f5732
```

11057 条 HEX 记录校验通过，176776 B 显式 HEX 数据与新生成的 BIN 对应地址逐字节一致；BIN 长 176784 B。八个未占用对齐字节 `0x08000188..0x0800018F` 是默认 objcopy 的零填充，HEX 不写这些间隙，不声称两种刷写方式的间隙内容一致。最高数据地址 `0x0802B28F`，低于校准保留区 `0x080C0000`；初始栈 `0x20020000`、Thumb reset vector `0x0800F439` 与版本字符串已核对。

基线提交仍为 `daecedf8e7710a53fd84e1f5d8175063d83cd26e`，工作区包含 0.18.6..0.18.13 累计未提交改动；本次没有新发布提交，不能用基线号冒充候选版本。manifest 保存源码和测试哈希。本版未推送仓库。

## 用户烧录后

后续只读代码/记录复核：[前轮欠速的撤载—积分平衡](stm32_evidence/2026-10-21-v01813-underspeed-review/README.md)在 RF 移动期确认短时快速撤载约 28‰，几乎抵消同段约 28.02‰ 正常积分修正，解释该段净支撑基本不增。提出前轮持续超速确认的窄范围候选，但尚未修改生产源码或编译下一版，不能据此宣布停顿根因全部确认。

实板补充：用户明确授权清除上一轮故障并[重复一次落地右转](stm32_evidence/2026-10-21-v01813-ground-failsafe/repeat_134912/README.md)。一次 CLEAR ACK OK/IDLE 后，120 条新速度命令全部接受；运行满 6 秒正常 STOP，RF 现场连续且未再触发故障。末段 LF/RF 平均约 +97.73/-115.06 cps，目标 ±160，仍明显欠速。当前停车状态为 IDLE/EXPLICIT；不以这次连续运动覆盖上一轮 RF 持续近零和故障事实，仍未通过带载转向性能验收。

关闭电机 VM，使用上述 HEX 按原全片擦除流程烧录，无需改 CubeMX。全片擦除会清掉 IMU Flash 校准；本版没有自动校准，交 ROS 正常使用前仍需显式校准保存。

烧录后先只读确认版本/供电/IDLE/无故障，再架空确认正常跟踪不会误触发。落地复测时保留明确时长并观察 RF：若再次持续停住，应看到新的故障锁存及整车停车，而不是运行 6 秒仍维持零反馈输出。还需验证真实恢复时不误停、直行及后轮表现。失败关断验证不应通过手挡车轮或人为加危险负载来完成；现有自然带载现象若重现即可核对。未经实车数据，不能标记转向性能问题已经解决。
