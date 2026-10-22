# 0.18.17：用户授权清故障一次后的落地左转复测

2026-10-21 20:31:18，Asia/Singapore。用户要求再试一次；已说明上次LF几乎不转，先断VM检查卡线/障碍/异味/异常发热。用户确认“检查无异常，安全条件满足，同意清故障并复测一次”，授权在零速、输出关闭状态仅清故障一次，再执行一次300 mrad/s左转、最长6秒，再故障不清不重试。

现场观察：用户回答LF **“基本没转或明显停住”**。与上次仅有日志、用户没看清不同，本次有实际观察支持带载起转困难，但仍未确定机械、电源、驱动、反馈和控制的唯一根因。

## 结果

再次约1.8秒后因LF堵转保护终止，**没有完成6秒、不是跟踪验收通过**。本轮不再重试。

- 清除前STATUS：FAULT / HEALTH_MANAGER / stop=FAULT、请求/四轮速度零，四轮输出关闭。
- CLEAR_FAULT只发一次，序号11；ACK `[3,0,1,0]`（OK/IDLE），随后新鲜STATUS确认IDLE、fault0，才进入3秒倒计时和单次运动。
- UART1286982 ms再次记录`wheel_mask=0x01`，runtime health-manager fault、motors disabled。
- 第一故障STATUS在主机运动相对1.813秒观察到，速度发送立即结束。之后只等待确认零速、读取trace、保持FAULT，**故障后没有主机STOP、速度或清故障命令**。
- 最终1310626 ms：FAULT、fault5/0、stop=FAULT、flags0x01D0，请求和四轮速度全零；故障保持锁存。
- 主机CRC=0，错误列表为空；这表示故障观察采集完成，不表示底盘运动成功。最终固件CRC/RX overflow/TX drop=0。

## 记录与脚本

原目录`build/hardware_tests/loaded_pivot_compare_20261021_203118`，完整副本在`capture/`。原summary保留`COMPLETE_FIRMWARE_HEALTH_FAULT_OBSERVED_NOT_YET_ATTRIBUTED`：完成的是自然故障观察与停车验证，不是完整6秒运动。

本次只调整主机采集脚本的允许测试组合，使原“观察自然故障+用户授权清除一次”的流程也支持**单次落地LEFT300**；原RIGHT200仍保留，其他组合拒绝。没有增加循环清故障，前置零速/关闭输出/新鲜轮速/ACK/IDLE检查仍在；本目录保存脚本快照，依赖原项目布局，不应从归档目录启动运动。

命令：

```powershell
python build/hardware_tests/loaded_pivot_compare.py --mode pivot --angular-mrad-s 300 --physical-setup ground --left-only --observe-recovery-fault --clear-prior-health-fault-once
```

本次总发送STOP1（运动前）、GET_STATUS65、CLEAR_FAULT1、VELOCITY36；accepted275→309（新增34）、rejected2→4（故障后拒绝2）。没有第二段运动，没有故障后的最终STOP。USB只读轮速/状态继续采集直至确认静止。

## 与第一次失败的有限对照

第一次失败：[20:19记录](../2026-10-21-v01817-ground-left-fault/README.md)。本次trace有87个完整头：86 ACTIVE（1285282～1286982 ms，每20 ms）+1 FAULT（1287002 ms），包括起步斜坡而非稳态末段；故障帧STBY0、四轮PWM/FF/P/I均0。

|项目|第一次失败|本次授权复测|
|---|---:|---:|
|ACTIVE完整帧|87|86|
|LF累计dc|-3|-3|
|LF非零dc帧|3|3|
|LF峰值PWM绝对值|183‰|183‰|
|故障前LF PWM|约-159‰|约-159‰|
|故障轮mask|0x01|0x01|
|现场LF观察|没看清，无法确认|基本没转或明显停住|

本次ACTIVE差分总和LF/LR/RF/RR为`[-3,-535,356,585]`；其余轮明显持续计数，LF缺乏运动的结果重复。最后ACTIVE中LF目标-240 cps、反馈0、FF约-146.6‰、P约-12‰、I=0、PWM-159‰。不是软件没有发出驱动命令，也不能从这些软件值推断电机端实际力矩或供电一定充足。

供电运动遥测10.560～10.703 V；依旧不是电机端快速电压/电流测量。控制时序未见超期，未见新版本/复位横幅；固件身份依据用户已烧录交付.17，没有Flash回读。137项项目文件仍与.17交付清单相符，本次生产固件和烧录产物未变、未校准。

## 下一步

两次相同目标、相同方向均在LF起步阶段失败，不再用重复清故障或提高整车速度作为处理方式。保护保持锁存，用户可关闭电机VM、保留USB。接下来建议只离线核对LF反向带载起步、速度斜坡期间恢复计时/补力权限及四轮起动时序，再决定是否需要LF限定候选或更进一步测量；本次未授权或实施新的固件修改。

不放宽堵转阈值、不删除安全门控，不将此测试写成RF或整车性能通过。
