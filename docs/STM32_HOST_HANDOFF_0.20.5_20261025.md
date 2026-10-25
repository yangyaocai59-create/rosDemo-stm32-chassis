# 0.20.5 上位机交接 — 2026-10-25

当前车辆由用户确认已烧录 0.20.5，完成一次静止校准并保存，普通断电后校准恢复以及左右各一次 180° 已验证。旧版本固件、离线包及原始测试记录全部保留。本次 Git 交付没有再次烧录、校准或执行运动，也没有缩减 RAM 缓冲区。

## 获取代码与固件

仓库：<https://github.com/yangyaocai59-create/rosDemo-stm32-chassis>

分支：`codex/four-wheel-v0200`。已有仓库请先保留自己的未提交修改，再执行：

```sh
git fetch origin
git switch codex/four-wheel-v0200
git pull --ff-only origin codex/four-wheel-v0200
```

首次获取：

```sh
git clone --branch codex/four-wheel-v0200 https://github.com/yangyaocai59-create/rosDemo-stm32-chassis.git
```

- 当前源码在仓库根目录；当前行为说明以本文和后续硬件证据为准。
- 可直接烧录的 Release HEX：[`releases/0.20.5/Release/rosDemo-0.20.5.hex`](../releases/0.20.5/Release/rosDemo-0.20.5.hex)。同目录提供 BIN、ELF、MAP 和 SHA256SUMS。
- 完整原始交付包：[`releases/0.20.5/rosDemo-0.20.5-offline-candidate.zip`](../releases/0.20.5/rosDemo-0.20.5-offline-candidate.zip)，内含 Debug/Release 固件、当时源码快照和离线验证记录。ZIP SHA256：`63cf733ce359831f56d452eb47570bd1baaf6d66a68fcf82882a02923b5247f4`。
- Release HEX SHA256：`4ea4ee72b726de2815ed3a7825133d504860a3bb94fc21fdc12adccb14c0977c`。直接提供的四个 Release 文件均与原始交付包逐字节一致，没有用编辑器流程修复后的重新构建覆盖它们。
- 原始包及其 README 中的“尚未烧录”是打包时状态，保持原始字节。后续已完成的板上验证见本文引用的硬件记录。

## 与昨天 0.20.1 的差异

1. 0.20.2 修正反馈时间比较及快照竞态，避免新鲜反馈被误判为过期，并修正反馈年龄日志回绕。
2. 0.20.3/0.20.4 增加有限幅 IMU 角速度修正。仅在线速度绝对值不超过 0.001 m/s、角速度绝对值 0.12–0.60 rad/s 的纯原地转向中启用。编码器仍负责各轮速度闭环。轮速跟踪异常或输出受限时禁止额外增力，但允许有效 IMU 反馈降低过高转速；原急停、堵转、超时和断链保护保留。
3. 0.20.5 修正 MPU6500 USER_CTRL 自清零位的读回校验，增加初始化故障定位日志。量程、采样配置、持久校准格式和 Flash 保存/恢复规则不变。
4. 配套构建流程自动生成与版本化 ELF 同名的 MAP，并可同步 VS Code 分析对象。该项是开发工具改进，不要求上位机或车辆重新烧录。

## 上位机兼容与接入注意

- USB CDC 协议 v1、现有帧布局、CRC、速度指令单位、状态及编码器/IMU 遥测布局保持兼容，无需仅为本次升级重写驱动。完整定义见 [USB 协议](usb_cdc_protocol_v1.md)。线速度命令仍为 mm/s，角速度为 mrad/s，左转为正；持续运动建议按原约定 20 Hz 刷新命令，超时仍为 250 ms。
- 上电后建议先保持静止至少 3 秒，等待 RAM 陀螺残余零偏基线；达到样本数和稳定性条件后 UART `YAW3 ready=1`。它没有新增 USB 就绪字段。基线未就绪时原轮速控制仍可工作，IMU 转向修正暂不介入。
- 底盘现在已经带角速度修正。若上位机另有角速度补偿，请评估两层配合，避免直接叠加此前试验用的主机补偿 DLL。上位机航向/定角控制仍可按现有接口发送角速度和 STOP。
- **没有新增 MCU“转 180°”命令。** 本次演示由主机按有效 IMU 源时间积分、接近目标时减速、达到目标后显式发 STOP。仅收到 ACK 不代表已经完成定角转向。
- 编码器累计值与轮式里程计仍是轮子的实际测量/推算，未改写为 IMU 角度，也未新增底盘端里程计融合。打滑时两者可能相差较大，ROS 侧不要将轮式转角当作已融合航向。IMU 积分继续按源样本计数/时间戳去重。
- 当前烧录工具固定全片擦除，升级会清除 IMU 校准。烧录后先确认 IMU 有效采样，再静止执行一次已有 `CMD_CALIBRATE_IMU` 并确认完成和 Flash 保存；校准期间用 GET_STATUS 保持会话，不能把启动 ACK 当作完成。普通断电后无需每次重新校准。详见 [全片擦除流程](STM32_FULL_ERASE_FLASH_WORKFLOW_20261025.md)。

## 当前测试结果和边界

- 普通断电重上电后，未重新校准即恢复有效偏置和原校准温度。
- 左/右各一次 180°，STOP 后约 1 秒的 IMU 参考角分别为 **+180.40° / −180.36°**，同期编码器估算 **+174.38° / −160.06°**。两次均停车且无新增故障。
- 另有左右各 10 秒转向记录，1 秒窗口最大角速度偏差约 8.8% / 11.6%，右转有一个窗口略超 ±10%。
- 上述是当前有限工况的板上结果，不是外部角度仪精度证明，也不替代不同地面、负载、长期漂移及所有停车故障路径的重复验收。

证据：[断电恢复及双向 180°](stm32_evidence/2026-10-25-v0205-reboot-180-bidirectional/README.md)、[10 秒转向与校准](stm32_evidence/2026-10-25-v0205-hardware/README.md)。

## 需要自行编译时

安装工程原有的 ARM GCC、CMake、Ninja，并确保工具在 PATH 中；编辑器同步还需 Python。

```sh
cmake --preset WheelV0205Release
cmake --build --preset WheelV0205Release
```

输出位于 `build/WheelV0205Release/`。默认 `Debug`/`Release` 仍保留旧 191 行为，**编译 0.20.5 必须选择 `WheelV0205Release` 或 `WheelV0205Debug`**。重新编译含构建时间，因此生成文件的哈希可能不同于已归档镜像。

主机离线回归入口（需要本机 GCC、Python 和 PowerShell，不操作串口或电机）：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File Tools/tests/run_wheel_v0200_tests.ps1 -ControlRevision 205
python -B Tools/compare_encoder_imu_yaw.py --self-test
```

本机生成的 `.vscode/firmware-status.json` 及界面回执不提交到 Git，避免将本机路径或车辆烧录报告带到其他工程师机器。可选的状态扩展源码与打包脚本已提交，安装步骤见 [VS Code 同步说明](VSCODE_FIRMWARE_STATUS_20261025.md)。

本次上传前已重新执行上述离线回归，205 的 strict/O2/fanalyzer、191 兼容及新旧 USB 回归全部通过，转角分析自测通过。同时复核 174 个原始产物、474 个前序候选文件及六个 0.20.x 原始 ZIP 的校验；当前两组 0.20.5 硬件证据共 111 个清单文件校验通过。所有新提交的原始证据、固件归档和差分夹具均验证 Git 暂存字节与磁盘一致。
