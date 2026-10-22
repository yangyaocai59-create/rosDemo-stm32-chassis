# 0.18.10 后轮短时补力与稳态支撑分离候选

日期：2026-10-20（Asia/Tokyo）。版本：`0.18.10-rear-transient-assist`。

状态更新：用户已烧录，完成左右各 6 秒地面对照，反馈“更连续，突然加速减轻”。短时补偿退出生效，RR 左转峰值和 RMS 下降，但左转 LF/LR 长零计数及右转 LF 末段欠速等问题仍在，不能标记全车通过。详见 [0.18.10 实板证据](stm32_evidence/2026-10-20-v01810-powered-pivot/README.md)。下文保留设计与构建交付记录；离线回归不能代替实车验收。

后续同固件完成了前后直行与 pivot350 对照，见 [负载工况诊断](stm32_evidence/2026-10-20-v01810-motion-load-diagnosis/README.md)。直行连续、pivot 前轮支撑需求明显增大，下一步应优先核对转向专用负载前馈；本次诊断没有改动固件。

## 实测依据

[0.18.9 实板证据](stm32_evidence/2026-10-20-v0189-powered-pivot/README.md)中，用户感觉更连续，但 RR 左转仍有约 720 ms 连续零计数，随后滤波速度峰值 495 cps，目标为 160 cps。

源时间 83322/83342 ms，FF+I 均约 184.3‰。虽然启动状态退出、FF 下降，I 却升至 50‰，总支撑没有及时下降。提前结束启动状态本身并不等于及时撤去启动力。本次针对该结构调整，不再仅改变等待时间。

这不是机械、电流或摩擦原因已排除的证明。PWM 是控制命令，不是实测电流或有效力矩；同条件实车对照仍然必要。

## 本次行为

仅改变原地 pivot 中后轮 LR/RR **再次启动**之后的补偿退出方式：

1. 保留当前再启动之前已有的稳态残余 bias，单独记录为 `s_pivot_steady_bias`。
2. 当前总 bias 超出这份稳态支撑的部分，作为本次短时再启动补力，不再整段转入积分。
3. 达到再启动最短保持 40 ms、同目标方向滤波速度至少 50 cps 后，退出启动状态。
4. 恢复运动期间，短时部分以 200‰/s 撤去：实际 20 ms 周期每次最多 4‰，到零停止，不穿过零、不产生反向补偿。
5. 滤波速度处于 20..50 cps 中间区间时暂停撤力，避免在正在减速时继续削弱支撑；降至启动阈值时，按已有补差额规则重新启动，不叠加上一份短时补力。

先前稳态残余支撑仍沿用原来的 FF/I 平顺交接，只转移这份已有支撑。首次起步、前轮及正常直行/滚动转弯不使用本次撤力分支。

计算示意（均按目标方向换算）：

```text
temporary = max(applied_bias - steady_bias, 0)
steady_bias = 原有平顺交接(steady_bias)
if 已恢复运动:
    temporary = max(temporary - 200 * 实际控制间隔秒数, 0)
applied_bias = steady_bias + temporary
```

例如没有先前稳态残余 bias、I=29.4‰ 时，本次先补 30.6‰；转起后短时部分按 26.6、22.6…降至 0，而不是把它重新填入 I 至 50‰。这只是状态和控制量示例，不是物理轮速保证。

重要边界：**没有冻结或回退 PI 的正常误差积分**。等待转起期间累积的 I 仍保留并受 ±50‰ 限制；本次只阻止短时 bias 在退出时被额外整段搬入 I。已有稳态支撑也不保证是真正完成带载标定的最优值。不能把本次描述为所有启动积分影响均已消除。

## 不变项与风险

- 保留 0.18.9 后轮再启动补差额、80 ms 首次恢复、40 ms 后续恢复、每级 10‰、补偿上限 60‰。
- 首次起步/前轮保持原 300 ms 和原 FF/I 交接。
- 基础 KP=0.05、pivot 同向超速 KP=0.15、KI=0.25 不变。
- 积分 ±50‰、总 PWM ±300‰、输出斜率 1000‰/s 不变。200‰/s 是短时补偿自身的撤除斜率，不是总 PWM 的唯一变化来源；P/I、目标变化及总输出斜率仍照常工作。
- STOP、故障、250 ms 指令超时和真实 USB/SOF/DTR 断链停车继续禁用输出，不等待撤力过程。
- 离开 pivot、目标接近零、换向、停车及故障清理本次状态，不把旧补偿带到新方向。
- 编码器/滤波、TIM8、引脚、USB 报文、IMU、Flash 校准格式、任务周期与 ROS 几何参数不变。

风险：撤力后负载支撑可能不足，重新出现停顿；如果正常积分或原稳态支撑仍偏大，超速也可能继续存在。200‰/s 是有界候选值，不是已验证的带载标定结果。必须同时检查峰值、零计数时长和其他轮子的跟踪，不只看平均速度。

## 源码与离线回归

相对 0.18.9 的生产修改：

- `Application/Chassis/chassis_controller.c`：新增逐轮稳态 bias，后轮再启动退出时只撤短时部分；传入实际控制间隔用于撤力。
- `Application/Chassis/wheel_speed_control_config.h`：200‰/s 常量及注释。
- `Application/System/firmware_info.h`：版本。
- `Tools/tests/motion/test_runtime_motion.c`、README 和本文同步更新。

最终执行通过：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File Tools\tests\run_motion_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File Tools\tests\run_usb_tx_tests.ps1
```

新/更新覆盖（真实控制器/PI，外设为测试桩）：

- 双方向及 tick 回绕：后轮转起后每 20 ms 撤除最多 4‰，I 不被额外填入短时 bias。
- 短时部分降到零、49 cps 时暂停、50 cps 边界、撤力中再次停住后重新启动不叠加。
- 100 ms 有效延迟步按实际 dt 撤力，最多撤去剩余短时部分；无固定每调用减量错误。
- 首次启动生成的真实稳态残余支撑，在后续短时增量撤除后仍保留。
- 首次起步、前轮、补差额边界、恢复限频、换向、STOP、再次启动和超时原回归。
- 原有输出限幅/斜率、故障抢占、USB/SOF/DTR/会话、RAW v2 和日志隔离回归。

日志：`build/hardware_tests/v01810_motion_regression.log`、`v01810_usb_regression.log`。这些测试没有模拟地面摩擦、电流、电源瞬态或轮胎变形，不能替代实板验收。

## 编译及镜像

STM32Cube GNU Tools 14.3.1+st.2，CMake/Ninja，Debug clean build，95 个步骤成功，Error=0、Warning=0。日志：`build/hardware_tests/v01810_debug_build.log`。

| 区域 | 使用量 | 链接容量 | 占比 |
| --- | ---: | ---: | ---: |
| FLASH | 173780 B | 768 KiB | 22.10% |
| RAM | 80576 B | 128 KiB | 61.47% |
| CCMRAM | 0 B | 64 KiB | 0% |

- 推荐烧录：`build/firmware/0.18.10/rosDemo-0.18.10.hex`。
- BIN：`build/firmware/0.18.10/rosDemo-0.18.10.bin`，基地址 `0x08000000`。
- 同目录保留 ELF、MAP、`verification.json`；0.18.9 及此前镜像没有覆盖。

```text
HEX SHA256 a4fd86a22e01091d9dbb41cba7a2c3d962afa6c275c3e0a74f7383ecb209eaf6
BIN SHA256 979309734716c698994834657a4aca816a56b2f3b4117166e3e7b8b3c6295f31
```

验证通过：10870 条 HEX 记录校验和、HEX/BIN 一致、Debug HEX 与交付 HEX 相同、版本字符串、栈 `0x20020000`、Thumb reset vector `0x0800F439`。BIN 长 173780 B，最高数据地址 `0x0802A6D3`，无 HEX 数据触及 `0x080C0000` 后预留区。

仍是以 `daecedf` 为基线的未提交工作区，包含 0.18.6 至本版修改；不能用基线提交号代表本版完整源码，也未推送仓库。

## 下一次烧录与实板对照

由用户关闭电机 VM、按原流程烧录，恢复用户 Flash 启动后确认版本 `0.18.10-rear-transient-assist`；无需修改 CubeMX。全片擦除会清除 IMU 校准，交 ROS 使用前仍需显式校准保存，本次不自动校准。

先只读检查，再确认地面、供电与空间，采用相同 ±200 mrad/s、左右各 6 秒对照 0.18.9。每次 STOP 后读完内部 trace，保留全程 USB 计数。

重点检查 RR 左转转起前后的 FF/I/P/PWM：短时贡献应逐步消退，而不是退出时 I 突然顶到上限；同时检查零计数区间是否反而增加、是否频繁重新启动、峰值及 RMS 是否下降，以及前轮是否退化。任何一项仍明显异常都不能宣称控制通过。
