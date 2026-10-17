# 0.15.0 运动平顺性固件修正与验收说明

日期：2026-10-17。版本：`0.15.0-motion-smoothness`。

状态：**软件构建和离线回归通过的 Debug 候选，尚未烧录、尚未实车验收；不能宣称实车顿挫已经消除。** 本轮没有打开 CDC/CH340、发运动命令、操作烧录器或重新校准。

## 1. 基线与范围

基于工作区 `0.14.4-imu-drdy-quality`，针对持续前进/后退及起步顿挫做固件侧修正，不把修改 ROS 键盘节点作为前置条件。

没有修改：`.ioc`、引脚、电气/编码器方向极性、TIM1～4 配置、TIM8 20 kHz 配置、USB CDC 协议与帧布局、USB SOF/DTR 真实断链判定、ADC/SPI/I2C 配置、MPU6500 驱动、Flash 标定格式/分区、IWDG 参数及前馈/PI 增益。CubeMX 生成代码修改仅在 `freertos.c` USER CODE 区内，生成的 CMake 未改。

若板上仍运行 0.14.3 或更早版本，本次升级也包含原 0.14.4 的 DRDY 采集改动：MPU6500 INT 必须已接 PB0。请同时参考 `STM32_IMU_QUALITY_0.14.4_HANDOFF_20261016.md`，不能把此前候选代码当成已完成实板验收。

本工程没有 Git 元数据，故提交号为空；用版本、完整源码快照和 SHA256 标识本交付，不编造提交号。

## 2. 已实现

### 健康检查时间竞态

原 `now_ms - heartbeat_ms` 无符号差值在“旧调用时间/新任务心跳”交错时可能下溢。改为有符号模时间年龄比较，保留严格 `>timeout`、真实超时锁存及正常 tick 回绕行为。时间比较假定监测间隔小于 2^31 ms；监督周期为毫秒级，IWDG 仍工作。

自动化验证覆盖旧 now=5000/新心跳=5001、不超时的 250 ms 边界、251 ms 真超时、锁存/清除和跨 UINT32_MAX。

### 运动与安全日志

motorTask、safetyTask、imuTask 入口注册永久非阻塞日志生产者；格式化后入有限队列，UART 发送仍只由低优先级 telemetry 消费。既有 USB 临时延迟日志机制保留。

队列 32 槽、31 条可用；队满丢日志并增加 `log_drop`，不等待 UART/mutex。三个实时生产者各有静态格式化缓冲，避免给 1 KiB safetyTask 栈额外叠加 256 字节临时记录。队列不是无限日志存储，诊断突发可能丢记录，不能将日志丢弃误当控制指令丢弃。

### 周期监控与超期恢复

motorTask、安全任务仍是 10 ms 周期，控制计算仍使用实际 dt。若下一绝对唤醒时间已过，跳过已错过的时隙，重新安排未来的唤醒点，不连续补跑旧时隙。

增加迭代周期 min/max、超期次数、错过时隙数。正常 UART 每秒新增：

```text
CTRL TIMING: motor_dt=10..10 safety_dt=10..10 overrun=0/0 log_drop=0 stack_free=.../...
```

这里是预期示例，不是实板测量结果。周期统计为启动以来的极值；`stack_free` 是运行时栈历史余量（字节），需实板确认。明显超期仍会触发既有控制时序故障，不放宽 100 ms 上限或 250 ms 命令超时。

### 电机输出和换向

- 同一方向的普通更新仅写最终 CCR，不再每 10 ms 写一次零 PWM 或反复写方向 GPIO。
- 换向先把该轮 IN1/IN2 同时拉低进入滑行，CCR 写零，至少等待 2 个 HAL tick 后才能改变方向。1 ms 时间粒度下，2 tick 保证至少约 1 ms 实际滑行，长于 20 kHz PWM 的 50 us 周期，允许零 CCR 真正锁存。
- 保护时间通过后续控制调用推进，不使用 HAL_Delay、busy wait，也不切换四轮公共 STBY 来控制单个轮子。返回状态是实际已应用输出，保护期为零。
- 生产控制器使用实际已应用输出校正斜率历史，避免保护期的请求值被当成已施加 PWM。
- PI 换向只清积分，不把历史 PWM 强行置零；正常目标过零也受输出变化率约束。安全 Stop/Reset 仍立即清零。
- 原 1500 千分比/秒 PWM 变化率、±300 千分比上限及所有前馈/PI 参数不变。单位千分比，例如 100=10% 占空比。

外围寄存器 mock 显式区分 CCR 预装载与实际锁存，验证连续 1000 次同向更新无额外零 CCR/GPIO/STBY 写入、换向先滑行、保护时间、快速重新使能和回绕。它不能代替示波器或实车波形确认。

### 正常运动目标曲线

新增纯函数 `ChassisMotionProfile_Step()`，生产 runtime 在运动学分配前使用：

| 项目 | 初版上限 |
| --- | --- |
| 线加速度 | 0.20 m/s² |
| 线减速度 | 0.30 m/s² |
| 角加速度 | 1.50 rad/s² |
| 角减速度 | 2.00 rad/s² |

这是保守工程初值，不是带载标定结果。正负换向先减到零，后续迭代再向另一方向加速。40 mm/s 从静止达到目标约需 200 ms；因此很短的测试脉冲总行程会比旧版少。

**下列操作全部绕过曲线，立即关闭驱动：显式 STOP、零速度命令、ESTOP、真实传输丢失、250 ms 命令超时、严重故障。** 这里的正常减速度只针对非零命令间的降速/换向，不把零速命令解释为缓停。

USB STATUS 的 vx/wz 仍是请求值，格式、单位和语义不变。新增的 `applied_command` 仅是内部整形目标，不是测速结果，也没有扩展 USB 帧。持续刷新相同非零命令不会重置积分或反复使能电机。

Stop 与控制迭代使用代际检查；旧快照不能在 STOP 后重新启动或用迟到的时序错误覆盖已完成的 STOP。

### 板内控制轨迹

新增只读 `motion_trace`：每 20 ms 保存一次，128 条有限环形记录，约覆盖最近 2.56 s（调度会影响间隔），停车后保留，下一次生产运动会重新记录。它不修改控制命令或输出。

在**停车且驱动未使能**后，用 CH340 串口助手发送 ASCII 小写 `t`（不是 COM4 USB 二进制端口）。telemetry 逐条导出，最多约 26 s；等待 `TRACE END`。导出期间仍处理故障和 USB，只暂停常规遥测 UART 文本；USB 二进制遥测不暂停。导出时不要安排新的运动，以免错过新的轨迹记录。

```text
TRACE t=... dt=10 req=40/0 applied=40/0 state=2 stop=0 fault=0 stby=1
TRACE LF dc=... raw=... target=... cps=... ff_x10=... p_x10=... i_x10=... pwm=...
TRACE LR ...
TRACE RF ...
TRACE RR ...
TRACE END; units req/applied=mm/s,mrad/s cps=counts/s pwm=permille
```

`dt` 是该测速样本实际时间，`dc/raw` 是这一测速样本的原始增量/未滤波 cps，环形记录本身通常隔 20 ms；不是把 20 ms 当作测速分母。P/I/前馈乘 10 后输出以保留一位小数。轨迹在 motorTask 的同一迭代采集，但各服务快照并非跨所有服务的原子硬件采样，安全抢占可发生在采集过程中。

## 3. 软件验证与构建

- 真实 ARM GCC Debug clean build，Error=0、Warning=0；具体 FLASH/RAM/CCMRAM、构建日期时间见包内 `release_status.json` 和 `verification/debug_clean_build.log`。
- 13 个原生主机测试程序：既有 IMU/USB/日志 10 个，加运动核心、电机驱动、生产 runtime 集成 3 个。
- 8 个 Python IMU 协议/采集测试和既有 USB 协议 self-test。
- 原生测试使用 `-Wall -Wextra -Werror`，包括真正超时、STOP 交错、PWM 预装载、正常曲线、输出斜率、轨迹环形边界。IMU 驱动和标定/Flash 既有回归全部重跑。
- 包内记录保护文件比对、ELF/HEX/BIN 一致性、向量表、镜像地址界限及 SHA256。

主机测试不是硬件闭环测试，不证明顿挫已消除、带载参数已最优或实际任务栈已经验收。

## 4. 修改文件

生产代码与构建：

- `Application/Safety/fault_manager.c`
- `Application/Services/debug_log.c`、`debug_log.h`
- 新增 `Application/Services/control_timing.c`、`control_timing.h`
- `Application/Chassis/wheel_speed_controller.c`
- `Application/Chassis/motor_driver.c`、`motor_driver.h`
- `Application/Chassis/chassis_controller.c`
- `Application/Chassis/chassis_runtime.c`、`chassis_runtime.h`
- 新增 `Application/Chassis/chassis_motion_profile.c`、`chassis_motion_profile.h`
- 新增 `Application/Diagnostics/motion_trace.c`、`motion_trace.h`
- `Application/Diagnostics/motor_open_loop_test.c`（只读 `t` 命令）
- `Application/System/firmware_info.h`
- `Core/Src/freertos.c`（仅 USER CODE）及顶层 `CMakeLists.txt`

测试/文档/交付：新增 `Tools/tests/motion/`、`Tools/tests/run_motion_tests.ps1`、`Tools/package_motion_release.py`；更新日志测试、Ubuntu 交接文档索引及本文。最终完整差异清单和每个源码 SHA256 在交付包内。

## 5. 用户烧录与实板验收

1. 不需要改 CubeMX 或重新接线。电机 VM 关闭，停止 ROS 和串口工具后，由用户按既有方式**全片擦除**并烧录本包 `firmware/rosDemo.hex`，不要混用旧目录 HEX。烧录基地址 0x08000000，包内同时提供 ELF/BIN 和校验值。
2. 上电先确认 Banner 为 `0.15.0-motion-smoothness`，四轮不自行转动，PD10 STBY 初态低，无循环复位、无健康故障；观察 `CTRL TIMING` 和栈余量。
3. 全片擦除会清空偏置。保持车体/轮子静止，运行既有 `--imu-calibration-test`，确认保存成功；普通断电重启后用 `--imu-monitor 10` 确认 `bias=1`、100 Hz 和错误计数正常。没有增加每次启动自动校准。
4. 四轮架空且车体固定，确认无人员/线缆接触轮子后，才开启 VM，使用既有受限 `--drive-test` 做起步/STOP 对照；先验证方向和立即停车，再安排持续的定速前进与后退。不要仅凭“没有 FAULT”判定平顺性通过。
5. 对每次产生顿挫的动作，停车后先导出 CH340 `t` 轨迹，再做下一次；记录实际 VM 电压、动作/方向/目标速度和主观顿挫时刻。轨迹窗口约 2.56 s，长动作应关注结束前的稳定段，起步另用短测试保留。
6. 在新的安全确认下回归 STOP/重新启动、命令超时、DTR 关闭/拔线停车和健康注入。`--health-test` 最后一项会转动电机，不要在本轮未确认安全时直接批量运行。
7. 若 timing 无明显超期而目标恒定、实测速度/PWM 同步周期振荡，下一步才做四轮正反向、实际 VM 下的带载前馈与 PI 标定。此候选没有为掩盖问题放宽安全超时或直接增加滤波。

保持当前可靠的供电与机械安全措施；如果新的输出时序使个别轮子起步异常或出现持续 FAULT，立即停止，不用反复加大 PWM 强行验证。

## 6. ROS 工程师说明

无需为了此升级修改 USB 解析器或消息布局。请记录固件版本并做同条件启停/恒速 A/B 对照。MCU 保留原始请求与整形目标的区分；不要把 STATUS 请求速度当成真实车速。

这次修复不将异常归咎于上位机，也不保证任何键盘输入模式都能平顺；软件验收后仍须用真实闭环轨迹确认。前馈、PI 以及负载/供电匹配仍属于后续实板标定项。
