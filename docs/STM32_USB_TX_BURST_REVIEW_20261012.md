# STOP 附近一次 TX 队列满：只读排查

日期：2026-10-12。未修改固件、未烧录、未访问串口或发送运动命令。本轮只读检查源码及已有双路证据，并在电脑上编译运行生产代码的受控模拟。

## 实板证据

证据目录 `build/hardware_tests/ground_stop_encoder_20261012_063648_246798/`。

- 主机最后一条速度命令 seq=25、STOP seq=26、GET_STATUS seq=27 在同一记录时刻 2828.875 monotonic seconds 发出。
- 最后 tx_drop=0 STATUS tick=2354088；首次 tx_drop=1 STATUS tick=2354188。其间有前进后的正常 STOP。
- STOP/GET_STATUS ACK 均收到，最终 IDLE/EXPLICIT、请求速度及四轮速度零，fault=0/0。
- UART STOP snapshot：now=last=2354104、age=0、alive=1、dtr=1、open=1；没有链路失活或 DTR 关闭发生在故障之前的证据。
- 非 ACK 的设备帧序列从 34549（BATTERY）跳到 34551（IMU），缺失 34550；解析 CRC 为零，tx_drop 增量为 1。这个序列缺口支持实际丢掉了一次设备主动上报帧，不只是主机显示误读。
- 根据发送顺序，缺失帧很可能是 GET_STATUS 紧随 BATTERY 的 IMU 快照，但 MCU 没有记录丢弃消息类型，不能把这个推断写成已直接测得的事实。

## 生产代码中的拥塞机制

1. `usb_cdc_transport.c` TX 环形队列数组深度 8，留一个槽位判断空/满，因此有效排队容量为 7；另有一个已经提交的 in-flight 缓冲。
2. `tx_dropped_frames` 只在 `next_head == s_tx_tail` 时增加。DTR 关闭清队列、未配置拒绝、USBD_BUSY 本身不直接增加这个计数。
3. 每条 VELOCITY 和 STOP 各生成一个 ACK。GET_STATUS 一次生成 ACK、STATUS、BATTERY、IMU，后两者在本板有效，因此三条命令的回应突发合计 6 帧。
4. `ChassisUsbProtocol_Process` 先完整处理 RX 环中的命令，再生成到期遥测，最后才调用一次 `UsbCdcTransport_ProcessTransmit`。处理期间可能有完成中断，但只清 in-flight 标志；任务未推进发送，队列中的帧不会因此自动出队。
5. STOP 路径还有同步阻塞 UART 日志（runtime、诊断测试、USB SNAP），UART 全局互斥锁也可能造成等待；因此任务处理时间和已有遥测积压都影响峰值排队。
6. GET_STATUS 的额外 BATTERY/IMU 快照、100 Hz IMU 周期流和多命令突发共同消耗队列容量。长期平均带宽正常，不代表突发排队一定安全。

## 离线受控验证

读取现有 `test_usb_protocol_stop.c` 的实际 parser/dispatch/STATUS 及应用头文件作为测试基架，将 BATTERY/IMU getter 模拟为有效值，仍然链接原生产 `usb_cdc_transport.c`；未改生产源码。
native GCC `-std=c11 -Wall -Wextra -Werror -DDEBUG` 编译，保留断言，无告警/错误；没有 ARM 构建或新固件。

在模拟中预置合法 STATUS/BATTERY/IMU，再提交一帧为 in-flight，保持完成回调待发生；然后一次提供 VELOCITY、STOP、GET_STATUS，调用实际 Process。

```text
preexisting=1 (1 in flight), drops=0, link_alive=1, state=1, stop=2
preexisting=3 (1 in flight), drops=1, link_alive=1, state=1, stop=2
PASS: production parser+TX queue reproduces one drop under a bounded burst
```

第二种情况：已有 2 帧待排队，再新增 6 帧回应，容量只有 7，因此一帧无法入队，STOP 和链路依然正常。这说明当前生产逻辑确实存在可确定复现的突发队列满机制。
执行文件：`build/tests/usb_tx_burst_audit_20261012.exe`；测试输入在内存中装配，硬件/业务服务/中断时机为模拟。

它与本次实板的命令批次、有效 BATTERY/IMU、序列缺口及计数增量相符，但实板未记录入队瞬间占用、busy/in-flight 或丢弃帧类型，仍不能证明其具体积压量恰好等于模拟值，也不能认定 UART 日志或主机双路采集是唯一触发因素。
既有 USB STOP 离线测试的 BATTERY/IMU getter 返回 false，因此未覆盖本次完整回应突发；今后应增加有效遥测及积压场景，不能仅引用旧的 50 轮零丢弃结果。

## 建议修改方案（未执行，需用户确认）

- 保持 USB 协议帧格式、消息类型和硬件接口不变，保留真实断链/DTR/命令超时/故障停车机制。
- 在命令/应答处理中及时推进发送，限制每轮处理/入队的突发预算；STOP 动作仍优先执行，不因回应排队而延迟。
- 为 ACK/关键 STATUS 设计有界的应答保障及重试，避免当前 SendFrame 返回值被忽略导致关键应答静默丢失；具体容量和优先级需离线验证，不能承诺任意负载下永不丢帧。
- 将 USB 关键路径里的诊断输出延后，避免同步 UART 等待阻塞应答/发送推进，不直接扩大为全工程日志框架重构。
- 记录队列峰值、满事件消息类型、busy/in-flight 等必要诊断，保留 tx_drop 计数真实语义，不清零或屏蔽错误。
- 补充 BATTERY/IMU 有效、队列积压、连续命令、STOP/DTR/真断链的离线回归，再实际 Debug build 并由用户烧录，之后分项实板验证。

## 更长行程的后续安排（未执行）

用户报告有约 80 cm 可用空间并希望行程更长。确认修复和短距离停车保护后，可优先使用约 50 cm 目标行程、40 mm/s 低速，保留余下空间作停车余量；具体可用距离及车体边界由用户现场核对。
使用累计计数估计行程并设置最大运行时间和状态门控，而不是仅按时间硬跑满 80 cm；编码器换算及打滑误差使计数行程不能代替外部测距。
执行前后行程、STOP、至少两分钟静置，双路日志先成功保存再倒计时。该安排不是当前已授权或已完成的运动测试，需再次确认现场条件后执行。

历史 LF 静止增加 4 计数仍作为独立待查项，不把 TX 拥塞修复等同于编码器异常修复。
