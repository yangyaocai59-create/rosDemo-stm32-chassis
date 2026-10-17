# 0.15.1 safetyTask 栈余量修正

版本：`0.15.1-safety-stack`，日期：2026-10-17。

这是基于 0.15.0 的窄范围软件候选。**未烧录、未做新的硬件测试；新的实板最低剩余栈空间仍待验证。** 用户自行烧录，不自动复位、校准或运行电机。

## 原因和已知实板证据

0.15.0 的一次 30 s 只读静态采集显示：

- motorTask/safetyTask 周期均为 10..10 ms，超期和日志丢弃为零。
- safetyTask 分配 1024 B，历史最低剩余 216 B；这是偏紧的余量，**不是已经发生栈溢出的证据**。
- IMU 约 100 Hz，采集质量计数无增长；全片擦除后 `bias=0`，尚未校准。
- 上述数据来自 `build/hardware_tests/motion_0.15.0_static_20261017_055022_192736/summary.json`，不能冒充 0.15.1 的验收结果。

## 修改方式

不手改 CubeMX 生成的线程属性或 `.ioc`，不偷偷覆盖配置的栈大小。safetyTask 仍是 1024 B，优先级和 10 ms 周期不变。

1. safetyTask 的 runtime、controller 快照及健康检查测速临时数组改为该任务独占的静态工作区，不再占据嵌套调用期间的任务栈。系统只创建一个 safetyTask；工作区指针不外传，不与其他任务或 ISR 共享。读取失败时仍按原逻辑停止运动观测，不使用旧快照判定。
2. `ChassisRuntime_SafetyProcess()` 只在同一个临界区复制 state、command_valid、last_command_ms、generation，不再复制完整 runtime 状态。
3. 新增只读 `ChassisController_GetSafetyState()`，原子复制 active/fault 两个字段，代替嵌套安全检查中不必要的完整控制器快照。原 `GetStatus()` 保留，USB 数据格式不变。
4. Stop 代际检查、超时边界、ESTOP、控制器故障传播和立即关闭输出逻辑保持不变。

没有修改 PWM/编码器配置、引脚、电机方向、速度曲线、PI/前馈、USB 协议、IMU 驱动/标定、看门狗策略或任何 CubeMX 外设配置。

## 栈分析（同一 ARM Debug 工具链、-O0）

| 函数自身栈帧 | 0.15.0 | 0.15.1 |
| --- | ---: | ---: |
| StartSafetyTask | 168 B | 32 B |
| ChassisRuntime_SafetyProcess | 144 B | 32 B |
| 新增 GetSafetyState | — | 16 B |

这是编译器 `.su` 中的函数自身栈帧，不是完整调用链上界，也不是新固件的运行时高水位。尤其不能把两行节省量直接加到旧实板的 216 B 后，当作新实板实测值。

完整状态快照转入 BSS 会小幅增加静态 RAM，准确 FLASH/RAM、构建时间及 SHA256 见交付包 `release_status.json`。打包脚本会检查上述两个栈帧均不超过 32 B，防止后续静默退化。

## 修改文件

生产代码仅五个文件：

- `Core/Src/freertos.c`：仅 USER CODE 内的 safetyTask 工作区存储位置。
- `Application/Chassis/chassis_runtime.c`：嵌套安全检查的精简快照。
- `Application/Chassis/chassis_controller.c`、`chassis_controller.h`：只读小快照接口。
- `Application/System/firmware_info.h`：版本号。

另更新 runtime 主机测试、打包脚本、文档索引和本文。相对冻结 0.15.0 的全部差异与源码校验值见交付包；旧交付包及旧实板采样不覆盖。

## 软件验证

- 真实 Debug clean build，Error=0、Warning=0；数值见包内构建日志和状态文件。
- 重跑 13 个原生主机程序、8 个 Python 测试及协议 self-test。
- 新增精简快照与完整快照的一致性、空指针拒绝、底层时序故障经精简安全检查传播并关闭输出的验证。
- 既有 STOP 抢占旧快照、命令超时、健康锁存、急停、真实 USB 断链机制的离线回归保留。
- 验证只有上述五个生产文件改变；`.ioc`、任务栈属性、生成代码 USER 区域之外及其他功能代码字节不变。

## 用户烧录与下一步

1. VM 保持关闭，由用户全片擦除并烧录本包 `firmware/rosDemo.hex`。无需修改 CubeMX，不使用旧包的 HEX。
2. 烧录后先静态采集 COM4/COM8，确认 `CTRL TIMING` 周期、超期计数、`stack_free`，同时核对启动 Banner 的 `0.15.1-safety-stack`。如启动日志已经错过，可由用户按后续协商的步骤手动复位并预先打开日志，助手不会自行发复位脉冲。
3. 要重新读取设备，请再次确认 VM 关闭、车体静置和端口未被其他程序占用。先看新的静态最小余量，不将静态通过等同异常路径已通过。
4. VM 关闭条件下，可另行批准仅静态心跳故障注入/恢复来覆盖安全日志路径；**不要直接运行完整 `--health-test`，其最后一项会驱动电机**。新的异常路径余量也应测量。
5. 全片擦除仍会清掉偏置，显式 IMU 校准放在本轮栈余量验收之后做一次。没有增加开机自动校准。
6. 如静态或故障路径余量仍不足，再由用户在 CubeMX 中将 safetyTask Stack Size 从 256 words 调至 512 words（1024→2048 B），生成后回归；不绕过 CubeMX 手改生成字段，也不声称本版已扩大分配栈。

运动平顺性、带载前馈/PI 和真实运动故障验收仍按 0.15.0 文档执行，不能用本次栈优化替代。
