# 0.16.0：小数精度 IMU 偏置、Flash 持久化与 ROS RAW v2 交接

版本：`0.16.0-imu-fractional-bias`，日期 2026-10-17。

**状态：软件候选，尚未烧录、未实板校准、未测得新版本漂移。** 基于 0.15.2，包含其新样本驱动遥测修复和更早的运动时序/安全栈修正。用户自行全片擦除烧录；助手本轮没有打开 COM、复位、写板上 Flash、校准或驱动电机。

## 目的与范围

旧版把 500 个陀螺仪 RAW 样本的均值做整数除法，并将整数偏置保存至 Flash。65.5 LSB/(°/s) 下，1 RAW 等效 0.916°/min；截断误差幅度可接近 1 RAW。旧版实测短窗口约 −0.444°/min，并不能证明全部由截断造成。

本版消除这个**可控的数值精度限制**，不声称消除器件温漂、噪声、振动或开机重复性。没有强制静止归零、陀螺仪死区、开机自动重校准、后台偏置学习或未经标定的温补。即使非常缓慢的真实转动也继续输出。

统一表示为有符号 `milli-RAW`：

```text
1 milli-RAW = 0.001 原始计数（sensor LSB）
实际偏置 RAW = bias_milli_raw / 1000
角速度 rad/s = (gyro_raw * 1000 - bias_milli_raw)
               / (1000 * gyro_lsb_per_deg_s) * π/180
```

校准用 int64 累加后，四舍五入到最近的 milli-RAW（恰好半单位时远离零）；500 样本的任何均值都能以 0.002 RAW 步长在该表示中精确保存。一般数值舍入上界 0.0005 RAW 等效约 0.000458°/min，**仅是偏置表示的量化上界，不是硬件漂移承诺**。噪声、温度、校准误差仍可能远大于它。

## 实现与不变项

- `Application/Sensors/imu_bias_units.h`：纯整数均值转换与单位常量。
- `Application/Sensors/imu_service.[ch]`：canonical milli-RAW；先做整数差再转 SI；同一个一致快照同时包含真实原始值及精确偏置。旧整数 getter 字段/设置接口保留为兼容视图。
- `Application/Diagnostics/imu_static_test.c`：500 个连续静止样本的精确均值，用同一值更新运行偏置并保存。旧 STD、重力、时序、全轮计数、运动所有权、取消/代际检查均保留。
- `Application/Storage/imu_bias_storage.[ch]`：format 3，40 B 记录，偏置字段改用 milli-RAW；A/B 追加、CRC、回读校验与最后提交字保留。不运行时擦除，不覆盖已有记录。
- `Core/Src/freertos.c`：仅 USER CODE 内启动恢复改用 `LoadMilliRaw → SetGyroBiasMilliRaw`，不会先恢复整数再补小数；日志显示精确单位。
- `Application/Communication/chassis_usb_protocol.[ch]`：RAW v2 扩展；模式 0/1 仍支持。偏置改变也能触发新状态通报。
- `Application/System/firmware_info.h`：版本。

生产代码仅上述十个文件改动。没有修改 `.ioc`、CubeMX 生成区域、顶层或生成 CMake、GPIO、SPI 寄存器配置、DRDY 采样率、量程、DLPF、电机/编码器、任务栈属性、控制时序、急停或 USB 真实断链停车策略。

### Flash 格式与刷写条件

- 分区仍为 A `0x080C0000`、B `0x080E0000`，各 128 KiB；应用仍限制在前 768 KiB。
- format=3、size=40；X/Y/Z signed int32 milli-RAW 位于 offset 16/20/24；温度 offset 28（int16，0.01℃）；profile offset 30；CRC32 offset 32；commit offset 36。
- MPU6500 profile 字段 `0x0270`，计算的兼容性指纹 **`0x53974331`**。单位、算法、采样配置及坐标映射均纳入区分；这不是器件序列号。
- 旧 format 1/2 记录不自动迁移、不把整数误当成 milli-RAW；新旧记录即使共存也不会覆盖旧槽位或错误载入。
- 本用户每次烧录全片擦除，所以旧偏置必然清除；刷好后要显式校准一次。之后普通断电上电应直接恢复小数偏置，不必重校。整片擦除也会清除此版记录。
- 编程任一字失败、CRC/回读失败，旧已提交记录仍保留；RAM 可能已应用新偏置，`persist=FAILED` 必须与真正保存成功分开判断。

## ROS 工程师：接口兼容性与必要调整

### 默认 SI 和旧 RAW v1

默认 `IMU_STATE 0x84` 的 46 B 布局、单位、状态/flags 完全不变，其角速度现在使用精确偏置，再按旧协议量化为整数 mrad/s。原 SI 解析器无需改布局。

`CMD_SET_IMU_STREAM 0x06` 的 payload `01` 仍选择 `IMU_RAW_V1 0x85`，52 B、format=1、整数 RAW 基线字段保持原义；此时字段是**精确偏置向零截断后的兼容视图**。它不能表达小数部分，不能再要求其复算值与精确 SI 完全相等，也不能用它验收本次去量化效果。不要给旧整数字段偷偷换单位。

**正式 RAW 路径改用 payload `02` 和消息 `0x86`。** 不要把 `0x86` 交给只懂 `0x85` 的旧 decoder，也不要在 mode 2 被拒绝时静默当成“已启用精确偏置”。旧固件会返回 BAD_VALUE，可明确告警或人工回退为 coarse 模式。

### RAW v2 协议

外层仍为 protocol=1、A5 5A、既有 CRC16；仅扩展消息类型，不改变既有类型。selector `0x06 payload=02` 成功 ACK 后发送 `IMU_RAW_V2 0x86`，payload **52 B**（完整帧 62 B）：

| offset | 类型 | 含义 |
| ---: | --- | --- |
| 0 | uint32 | 采集时间 ms |
| 4 | uint32 | 采集样本计数 |
| 8 | int16 ×3 | body-frame accel RAW |
| 14 | int16 ×3 | body-frame gyro RAW，未减偏置、未缩放、未归零 |
| 20 | int32 ×3 | 实际应用的 gyro bias，单位 **milli-RAW** |
| 32 | int32 | 温度 m℃ |
| 36 | uint32 | 累计读错 |
| 40 | uint32 | 连续读错 |
| 44 | uint16 | gyro sensitivity ×10；当前 655，表示 65.5 LSB/(°/s) |
| 46 | uint16 | accel sensitivity；当前 8192 LSB/g |
| 48 | uint8 | 既有 IMU state |
| 49 | uint8 | 既有 flags |
| 50 | uint8 | format，必须为 **2** |
| 51 | uint8 | reserved，必须为 0 |

所有多字节值为 little-endian；signed 字段必须按有符号数解码。未校准时 bias-valid=0，三个偏置字段全为零。有效校准/恢复后 bias-valid=1。`cal_result=0` 表示本次上电未执行新校准，不表示恢复失败。

Python 参考（完整 framing/校验实现见本版 `Tools/usb_cdc_test.py`）：

```python
import math, struct
v = struct.unpack('<IIhhhhhhiiiiIIHHBBBB', payload)  # len(payload) == 52
assert v[18:] == (2, 0) and v[14] > 0 and v[15] > 0
gyro_scale = v[14] / 10.0
bias_raw = [b / 1000.0 for b in v[8:11]]
w_rad_s = [(g - b) / gyro_scale * math.pi / 180.0
           for g, b in zip(v[5:8], bias_raw)]
accel_m_s2 = [a / v[15] * 9.80665 for a in v[2:5]]
```

例如 `g_z=146, bias_z=145500` 代表 `146−145.500=0.500 RAW`，不是减掉 145500 RAW。归一后的 frame 仍为 ROS base_link：X 前、Y 左、Z 上；wz 正为逆时针左转。

### 时序、会话与验收

- 发布按新采集快照驱动；没有新样本时状态变化立即通报，否则每 100 ms 一次健康快照。显式 GET_STATUS 可重复当前样本。
- 按采集 `sample_count/time` 去重并用 MCU 时间积分。重复帧不是新测量；计数跳变必须报告。仍为 latest-only，不能把它当成传感器 FIFO。
- DTR 关闭、真实断链、会话 1200 ms 失活都回到默认 SI。重连后 ROS 应重新选择 mode 2，并清理自身旧积分时间基准；不能自动恢复旧非零运动命令。
- ACK echo 的是主机序号，不纳入设备遥测序号连续性检查。
- 不要另减一遍 Flash baseline：RAW v2 自带的是 MCU 已应用的那一份。若 ROS 有可信静止残差估计，必须将其作为另一层、RAM-only 的小残差，运动时冻结；不能混写回 MCU Flash。
- 本次不修改 ROS 工作空间、融合配置、SLAM/Nav2 或运动控制；ROS 开发者需应用上述 selector/decoder 改动。

## 软件验证与证据边界

真实 Debug clean build 的 Error/Warning、FLASH/RAM、Build 时间、ELF/HEX/BIN SHA256 见包内 `release_status.json` 和 `verification/`。本地没有 Git 仓库，提交号为 null，不虚构提交。

回归包含：

- 500 个交替小数均值样本经过真实校准、实际格式的虚拟 Flash、模拟冷启动恢复后，保持精确偏置；模型中残差均值为零，但每个非零慢速角速度仍输出。这是软件模型，不是新实板漂移。
- signed/边界、旧 v2 格式拒绝、校准取消/移动/数据间断/STD、Flash 全部十个编程位置失败、CRC、只读恢复无写入。
- RAW v2 字节布局、正负小数偏置、SI 复算、碎包/CRC、旧 v1 兼容、默认模式与 DTR/会话恢复。
- 独立 DRDY/USB 时钟下 SI、RAW v1/v2 无人工重复/跳样；背压游标只在入队成功后推进。
- 原有电机时序、STOP、超时、故障/急停、SOF/DTR 真实断链逻辑离线测试保留；不是新的硬件运动验收。
- 14 个原生程序、19 个 Python 测试与 legacy protocol self-test；镜像范围和冻结源码差异也检查。

## 用户烧录与后续实板步骤

**无需修改 CubeMX。跳过 0.15.2，不必多刷一次。**

1. 关闭电机 VM，用户自行全片擦除并烧录本包 `firmware/rosDemo.hex`。校准将丢失一次，这是全片擦除条件的必然结果。
2. 启动确认 `0.16.0-imu-fractional-bias`，先只读验证 SI/RAW v2 接口、健康计数及任务余量；`bias=0` 在首次校准前是预期状态。
3. 在固定安装、车体静止、VM 关闭条件下，等待温度趋稳（建议先供电数分钟），再另行授权并执行一次：

```powershell
python Tools\usb_cdc_test.py --port COM4 --imu-calibration-test
```

该现有测试含零速度互锁探针、重复校准拒绝测试，不含非零运动命令；会触发一次真正的偏置保存。不要未经授权运行完整 `--health-test`，其中有运动项目。

4. 看到 PASS 和 `precise bias milli_raw=[...]`，并确认 `persist=2` / Flash saved 后，记录三个精确数值。普通断电重启不能重新校准；检查恢复值逐位一致，bias=1、cal_active=0、cal_result=0。
5. ROS/串口助手关闭、VM 关闭且静止后，另行确认再采集至少两分钟：

```powershell
python Tools\imu_raw_stream_test.py --port COM4 --debug-port COM8 --raw-version 2 --duration 120 --vm-off-confirmed --stationary-confirmed --require-bias --require-quality
```

端口以当时枚举为准。此脚本只发送 GET_STATUS / SET_IMU_STREAM，另含 CDC DTR 会话恢复与失活测试，不写 Flash、不发运动/STOP/复位指令；CH340 只读且打开前 DTR/RTS=False。它仍是接口/采集验收，不自动给“零漂合格”结论。若源码计数有跳变，会失败并保留证据，不能忽略。

6. 用保存的 JSONL 再算不经 mrad/s 取整的 RAW v2 残差：

```powershell
python Tools\imu_raw_drift_report.py <本次证据目录>\raw_samples.jsonl
```

比较校准后、断电后及相近/不同温度下的 10 s 窗口和 2 分钟积分，记录实际每分钟漂移；不要套用旧版 0.444°/min，也不要用理论量化上界作为实测值。

本轮结束时尚存：实板新偏置保存/恢复、RAW v2 连续性、温度变化及真实漂移对照均待验证。运动平顺性验证另行进行，不在这次静止 IMU 改动中发送运动命令。

## 给 ROS 开发者交付哪些文件

推荐直接交完整本版 ZIP，先看 `README_FIRST.md` 与 `release_status.json`。只交上层所需材料时，从包内 `source_snapshot.zip` 提取：

- 本文 `docs/STM32_IMU_FRACTIONAL_BIAS_0.16.0_HANDOFF_20261017.md`。
- `Tools/usb_cdc_test.py`：包含 `decode_imu_raw_v2()`、`imu_raw_v2_to_si()`。
- `Tools/imu_raw_stream_test.py`、`imu_raw_drift_report.py`、`debug_uart_capture.py`、`imu_stationary_diagnostic.py`、`imu_quality_analysis.py`：采集、证据和残差复核依赖。
- `Tools/tests/test_imu_raw_v2.py` 及其依赖，或完整 `Tools/tests/`：离线协议对照。
- `docs/usb_cdc_protocol_v1.md`、`Application/Communication/chassis_usb_protocol.h`：消息枚举与协议索引。

如需确定哪一版实际装板，再交烧录后的 Boot Banner 与新采集 summary；软件 ZIP 的 SHA 不能代替板上版本/测试证据。
