# 0.18.11 原地转向带载前馈候选

日期：2026-10-20（Asia/Tokyo）。版本：`0.18.11-pivot-load-feedforward`。

状态：真实 ARM Debug clean build、运动/USB 离线回归与镜像校验通过。用户报告烧录后，已完成 [±200 mrad/s 实测](stm32_evidence/2026-10-20-v01811-powered-pivot/README.md)及 [350 mrad/s/直行对照](stm32_evidence/2026-10-20-v01811-motion-load-diagnosis/README.md)。LF 长停顿及积分顶限改善，直行保持连续，但左转 RF 仍有 340/500 ms 零计数（分别为 200/350 目标），不能宣称问题已解决。本轮未改生产代码、CubeMX、自动烧录、提交或推送 GitHub，暂不自动生成下一版。

后续 [RF 两次同方向复核](stm32_evidence/2026-10-20-v01811-rf-left-repeat/README.md)一次再次出现 500 ms 零计数、一次 RF 连续。UART 原始计数与 USB 同时不变；用户未看清实际转动，尚未区分物理停轮和编码器信号中断。下一步需要同步独立观察，不直接追加驱动力或宣称机械问题已确认。

最新 [RF 肉眼观察复核](stm32_evidence/2026-10-20-v01811-rf-visual-check/README.md)：用户确认同次实际停轮后恢复，日志 680 ms 连续零计数、恢复峰值 474 cps，RF 积分这次触及 +50‰。三次长零计数对应的轮周计数位置接近，作为机械检查线索而非根因定论；下一步优先断开电机电源检查周期性发紧/擦碰，尚未实施新固件。

后续用户手转检查正常，[同指令架空对照](stm32_evidence/2026-10-20-v01811-rf-lifted-compare/README.md)未复现停顿，并连续经过上述名义轮位，降低了固定空载硬卡假设的优先级。但前轮架空长期超速约 59%/65%、反向修正积分全窗顶限，明确暴露固定 pivot load 在负载减小时不能有效撤减的局限。建议独立设计可撤减的负载支撑及有界重启恢复，不单纯再加固定补偿；本轮未实现或编译下一版。

## 为什么改转向支撑，而不是通用直行参数

[0.18.10 同固件负载对照](stm32_evidence/2026-10-20-v01810-motion-load-diagnosis/README.md)中，直行与 pivot 的轮速目标幅值分别约 282/280 cps：

- 直行前后四轮均能连续跟踪，末段无零计数帧，积分没有到上限。
- 匹配同一轮、同转动方向后，pivot 前轮平均 PWM 约为直行的 1.8..2.2 倍，后轮约 1.3..1.5 倍。
- LF 在 pivot 两方向多次到达积分上限，右转平均约 +237 cps，目标 +280 cps。

原架空前馈没有独立描述转向带载阻力；依赖固定积分上限和启动残余支撑可能不足。该证据不排除机械、重心、接触或电流因素，也不能直接把平均 PWM 差值当成标定常数。

## 候选参数

新增独立 pivot running-load 前馈，只有现有 `PIVOT` 模式使用。按现有判定：请求线速度幅值不超过 1 mm/s，角速度幅值至少 10 mrad/s。直行、滚动转弯和旧普通更新接口保持 NORMAL，不使用这份前馈。

| 轮子 | 车体前进方向/‰ | 车体后退方向/‰ |
| --- | ---: | ---: |
| LF | 60 | 65 |
| LR | 0 | 0 |
| RF | 60 | 45 |
| RR | 0 | 0 |

单位为 PWM 千分比，60‰ 即 6 个百分点的占空比，不是最终总 PWM。后轮偏置明确为零，以保留 0.18.10 的已测行为。

上述前轮数值参考 pivot 平均输出相对原架空前馈约 45..70‰ 的差额量级，并考虑 LF 欠速、RF 反向已有超速，取作有界候选；不是正式拟合或实车验证后的最优值，更不是直接把直行/pivot 的平均 PWM 差值原样加上去。

新增负载支撑按轮速目标渐入：

```text
abs(target) < 1 cps: load = 0
否则 load = sign(target) * 对应方向参数 * min(abs(target)/160, 1)
```

在已测低速目标约 160 cps 达到全量，280 cps 及更高保持上限；低于 160 cps 随目标减小，避免接近零目标时直接加完整支撑。它并非温度、电压、电流或地面摩擦的自适应模型。

## 如何避免新增支撑与旧 kick 重复叠加

前轮启动时的附加瞬态重新核算：

```text
temporary_budget = max(60 - abs(load), 0)
前轮 FF = 原架空 FF + load + 当前 temporary
```

目标变化时重新计算 budget。负载支撑与瞬态之和按目标方向计算，最大为 `max(60, abs(load))`，不会变成 load 再加完整 60‰。例如：

- LF 前进、RF 前进全量 load=60‰，附加瞬态为 0。
- LF 后退全量 load=65‰，附加瞬态为 0；启动前馈量级较旧 60‰ 预算多 5‰，不是完全相同的输出。
- RF 后退全量 load=45‰，附加瞬态最多 15‰，而不是另加 60‰。

前轮仍保持原 300 ms 最短启动保持及 20/50 cps 滞回。退出后，剩余前轮瞬态以共享的 200‰/s 规则撤去（20 ms 最多 4‰），不转入 I 或旧残余 bias；低速滞回区间暂停撤力，重新接近静止时可重启。运行支撑是显式 load，不再依赖把前轮 kick 整段填进积分来形成。

PI 的正常误差积分仍运行，受 ±50‰ 限制。没有冻结、回退或扩大积分，也不能保证换地面或负载后仍不触及上限。总输出还包含 P/I，不能把上述 FF 预算当成总 PWM 的唯一上限。

后轮沿用 0.18.10：首次起步旧交接、再启动补差额、80/40 ms 分级恢复、40 ms 恢复交接和短时部分逐步退出。后轮新增 load=0，离线原回归继续通过。

## 保留的限制与非修改项

- 普通前馈常量、KP=0.05、pivot 同向超速 KP=0.15、KI=0.25 不变。
- 总 PWM ±300‰、积分 ±50‰、输出斜率 1000‰/s 不变。
- STOP、故障、250 ms 指令超时及真实 SOF/DTR/USB 断链继续直接禁用输出，不等待前馈撤除。
- 换向、目标接近零、退出 pivot、停车的状态清理仍保留；实际输出受斜率和方向保护约束。
- 编码器/极性/滤波、TIM8、引脚、USB 布局、IMU/Flash 校准、任务周期和 ROS 几何参数未改。

负载前馈只在 PIVOT 模式计入，但模式切换的实际输出仍有历史积分和斜率过程；“直行参数未改”不等于承诺从任何前态切换后每个输出都完全相同。

## 源码范围

- `Application/Chassis/wheel_speed_control_config.h`：前轮分方向 load 候选表与 160 cps 渐入参数。
- `Application/Chassis/wheel_speed_controller.h/.c`：纯查表接口 `GetPivotLoadFeedforward`，只在 PIVOT 模式计入；原 NORMAL 接口不加 load。
- `Application/Chassis/chassis_controller.c`：前轮瞬态预算扣除显式 load，目标变化时防重复叠加，退出不再将前轮瞬态转入 I。
- `Application/System/firmware_info.h`：版本。
- 运动回归、README、本说明同步更新。

## 离线回归

最终执行并通过：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File Tools\tests\run_motion_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File Tools\tests\run_usb_tx_tests.ps1
```

新增/更新覆盖：

- 四轮/双方向 load 表、0/0.5/1/20/80/160/280/1600/超目标上限的渐入/封顶、非法查表输入。
- NORMAL 和旧接口仍产生原架空前馈；rear load=0。单独对齐前馈后验证 pivot 超速 P 行为，没有把新增 load 混成增益差异。
- 真实底盘控制器在双方向、不同目标、tick 回绕下：load+kick 不重复叠加，目标爬升/下降时重新扣预算。
- 前轮瞬态退出不填入 I，最终只剩显式 load；退出 pivot、进入直行、换向、STOP 清理及斜率约束。
- 0.18.10 后轮短时分离、边界/迟到/重启、原有故障/超时/急停/抢占测试。
- 原 USB 会话/SOF/DTR、队列、RAW v2、采样时间戳和日志隔离回归。

测试使用真实控制器/PI，外设为测试桩；没有模拟轮胎、地面、电流或机械耦合。因此不能用离线通过代替带载效果验收。

日志：`build/hardware_tests/v01811_motion_regression.log`、`v01811_usb_regression.log`。

## 编译和镜像

STM32Cube GNU Tools 14.3.1+st.2，CMake/Ninja，真实 Debug clean build，95 个步骤完成，Error=0、Warning=0。日志：`build/hardware_tests/v01811_debug_build.log`。

| 区域 | 使用量 | 链接容量 | 占比 |
| --- | ---: | ---: | ---: |
| FLASH | 174612 B | 768 KiB | 22.20% |
| RAM | 80576 B | 128 KiB | 61.47% |
| CCMRAM | 0 B | 64 KiB | 0% |

- 推荐烧录：`build/firmware/0.18.11/rosDemo-0.18.11.hex`。
- BIN：`build/firmware/0.18.11/rosDemo-0.18.11.bin`，基地址 `0x08000000`。
- 同目录保留 ELF、MAP、`verification.json`；0.18.10 及之前镜像未覆盖。

```text
HEX SHA256 c85564a478a62d1059f1efc14f2382f4224ee5e5226cc35a7259ec4128649a89
BIN SHA256 dad3b166e95b1c128e910ab8d262c36967cefa187bb4243427f7c0a4be2419de
```

验证通过：10922 条 HEX 记录校验和，HEX/BIN 一致，交付 HEX 与 Debug HEX 相同；版本字符串、栈 `0x20020000`、Thumb reset vector `0x0800F439` 正常。BIN 长 174612 B，最高数据地址 `0x0802AA13`，无 HEX 数据触及 `0x080C0000` 后的预留区。

仍是以 `daecedf` 为基线的未提交工作区，包含 0.18.6 至本版修改；不能用该基线提交号代表本版完整源码，尚未推送 GitHub。

## 用户烧录后验证

用户关闭电机 VM 后按原流程烧录，恢复用户 Flash 启动并确认 `0.18.11-pivot-load-feedforward`，无需修改 CubeMX。全片擦除仍会清除 IMU 校准；本次不自动校准，交 ROS 使用前须显式校准保存。

先只读检查，再确认场地和供电，优先重复 ±200 mrad/s、左右各 6 秒。保留全程 USB 与停车后完整 trace，关注 LF 是否持续触及积分上限、前轮峰值/停顿是否改善，以及 rear/模式切换是否退化。后续再以 350 mrad/s 及 ±37 mm/s 直行对照，不能以更高目标的连续性替代低速验收。

trace 的 FF 现在包含“原架空 FF + 显式 load + 瞬态”，报文布局未变。分析时需按轮子/方向/目标重建 load，不能把新增 FF 都解释成启动补偿。

如果新增支撑导致超速，或仍有明显卡滞，不继续简单堆叠补偿/扩大输出上限，应结合负载、接触和供电能力复核。只有取得实板数据后才能判定本候选改善与否。
