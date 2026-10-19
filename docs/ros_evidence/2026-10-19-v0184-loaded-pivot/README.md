# ROS → STM32：v0.18.4 地面低速转向证据（2026-10-19）

**先读本页和最新实板记录。已取得实际内部 target/PWM，确认地面低速轮速跟踪不足；具体控制、驱动及机械根因待分析。此次提交只增加说明和证据，未修改或烧录固件，不要求先重做整套测试。**

证据分支：`evidence/ros-v0184-loaded-pivot-20261019`。基于交付标签 `v0.18.4`、提交 `32bc6d22d8eaac9ceb6bc820a06efe1c01099976`；分支中的固件文件保持该版本，仅作为记录所对应的参考。不是新的固件发布版本。原始证据文件名/时间戳保持Jetson主机记录。

## 阅读顺序

1. **[给下位机的最新交接说明](reports/STM32_LOADED_PIVOT_TRACE_HANDOFF_2026-10-19.md)**：已取得真实板上字段，建议离线分析项及可转发消息。
2. **[完整实板采集报告](reports/JETSON_LOADED_LEFT_TRACE_REVIEW_2026-10-19.md)**：两次ROS动作、第二次内部缓存、采集方法、验收边界及停车检查点。
3. [轮速/PWM CSV](analysis/trace_wheels.csv)、[结构化内部统计](analysis/trace_analysis.json)、[完整汇总](analysis/combined_result.json)、[127帧原始字段](analysis/trace_frames.json)。
4. [离线检查与调试口事件](reports/JETSON_OFFLINE_TURN_RESPONSE_REVIEW_2026-10-19.md)及[三线接线补充](analysis/wiring_followup.json)。
5. [最初短左转复查](reports/JETSON_MANUAL_LEFT_RESPONSE_REVIEW_2026-10-19.md)、[原排查说明](reports/STM32_LOW_SPEED_LEFT_RESPONSE_HANDOFF_2026-10-19.md)是此前阶段记录，其中“尚缺内部trace”已由本次实际采集补齐。历史结论不要覆盖最新报告。

其余reports包含ROS STATUS串口/输出隔离修复、静止TF/DDS/控制时序复查、两次前进启停及昨日v0.18.4运动验收，供核对上下文，不表示当前转速/导航验收通过。

## 已确认事实

用户实际做了两次左转并确认停车正常、遥控退出，ROS记录两次均保留。MCU在每次ACTIVE开始时清空缓存，因此127帧内部trace仅对应第二次：源77461–79981ms、20ms间隔、126帧ACTIVE和1帧停车，含TRACE END。

| 车轮 | 实际target cps | 稳态平均filtered cps | 稳态平均raw cps | 平均绝对PWM（‰） |
|---|---:|---:|---:|---:|
| LF | −160 | −24.97 | −25.89 | 134.50 |
| LR | −160 | −44.46 | −47.32 | 109.69 |
| RF | +160 | +23.63 | +25.45 | 141.07 |
| RR | +160 | +80.22 | +80.80 | 107.03 |

- requested=200mrad/s，applied约140ms达到200，之后保持；无中间STOP/fault，末帧IDLE/EXPLICIT、standby=0、PWM全0。
- 稳态112帧没有积分达到±50‰或PWM达到±300‰。FF对照基础表额外约0/±60‰，推断补偿反复切换；没有独立limited/补偿标志，不把推断写成已记录的标志。
- raw与filtered均低，ROS换算复算一致，不能仅用IMU偏航/ROS滤波解释欠速。命令几何210mm与里程计有效转向系数395mm用途不同；后续标定/模型调整需验证，不能只改ROS里程计数值掩盖欠速。
- 现有堵转监测目标门槛200cps，本次160cps低于门槛；无fault不等于速度跟踪达标，未更改保护。
- 需评估地面负载前馈、PI、启动补偿退出/保持及机械/驱动供电条件；没有高频供电/电流或外部量角真值，尚不能唯一定位根因。请用已有证据决定最小修正/必要补测。

## 调试口复位是独立问题

用户确认USB-TTL仅TX/RX/GND，未接VCC/DTR/RTS/NRST。DTR/RTS=false仍在打开串口时出现PIN RESET，旧缓存曾丢失。此次静止先打开一次、等恢复后保持同一fd，动作后只发`t`，约31.33秒完整导出；ready后至导出没有再次复位。采集脚本未发速度、复位、校准或Flash写入指令。Jetson第三方ch34x驱动还出现内核告警，但尚未证明复位电气路径，也未证明它与低速欠速同源。

## 原始数据与复算

`archives/STM32_V0184_LOADED_LEFT_TRACE_20261019.zip` 为最新完整采集：UART原始字节、分块时间、ROS JSONL、native时序、完整dump及采集/解析脚本、参考控制源码。另两个ZIP保留此前短转/离线检查和调试口事件；三线接线补充在本分支单独提供，发生在旧ZIP冻结后。

从仓库根目录离线执行，无需ROS、串口或第三方Python包：

```bash
cd docs/ros_evidence/2026-10-19-v0184-loaded-pivot
sha256sum --check SHA256SUMS
python3 analysis/analyze_saved_trace.py
```

结果生成 `analysis/recomputed_trace_analysis.json`、`recomputed_trace_frames.json`、`recomputed_trace_wheels.csv`，不覆盖发布数据。只需运行上述离线解析脚本；ZIP内其余原始采集脚本是方法留档，不是新一轮复测步骤。

SOURCE_MANIFEST.json记录来源与哈希；SHA256SUMS覆盖本目录的交付文件（不含自身及复算输出）。脚本调整仅涉及离线输出名、补齐统计及文档相对链接，已记录在来源清单。

## 工程师获取

```bash
git fetch origin
git show origin/evidence/ros-v0184-loaded-pivot-20261019:docs/ros_evidence/2026-10-19-v0184-loaded-pivot/README.md
```

也可切到证据分支浏览或下载ZIP；主工程在main/其他分支的修改不需要切换或覆盖。本证据分支不改变任何固件文件，不自动合并main，不附带烧录步骤。
