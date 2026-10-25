# 用户重新上电后的复查及 0.20.5 离线修订

2026-10-25。用户明确报告已重新上电后，进行 6 秒只读检查。MCU uptime 约 20–27 秒，仍为 `IMU status=IO_FAULT WHO_AM_I=0x70 samples=0 errors=0`，`YAW3 ready=0 valid=0`。runtime 为 IDLE，输出和四轮速度均零，编码器计数在采集中不变。供电约 10.36 V，ADC 正常。

未捕获启动 Banner；板上版本依据为用户在 0.20.4 交付后确认烧录，且观察到新版 YAW3 日志，不能声称已独立读取完整 Flash 版本。

本轮仅发送 GET_STATUS，没有运动、复位、清故障、校准、传感器写命令或烧录。最后再次读取 3 秒，uptime 约 550 秒，故障仍存在，车辆保持静止，串口关闭。因此没有进行缺失角度反馈的转向验收，也没有将轮速回退当作外环就绪。

## 离线可复现的问题

查阅 [TDK 官方寄存器手册 RM-MPU-6500A-00，4.33 节](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6500-Register-Map2.pdf)，USER_CTRL 的 bit 4 I2C_IF_DIS 会自动清零，而旧驱动要求其保持为 1。官方站点当前 PDF 直开重定向到搜索页，依据为官方 PDF 的索引内容；未声称已下载或逐页查看 PDF。

新增寄存器模型模拟该行为，实际 driver/service 的旧分支会在首个样本前进入 IO_FAULT；修订分支进入 READY 并发布样本，稳定配置位、量程及读错误仍应拒绝。模型测试、完整校准与 Flash 恢复模型、新旧分支均在 strict/O2/fanalyzer 中通过。对应日志保存在本目录。

这确认代码存在此问题，尚未确认当前板上失败的具体寄存器；旧固件没有该诊断入口。为避免继续盲目断电或校准，新版同时报告失败阶段、地址、实际值、期望值及掩码，初始化成功时报告 USER_CTRL 读回。需要烧录后核对。

## 独立 0.20.5

0.20.5 仅对新增版本修正该比较，仍执行原配置写入；设置表及校准指纹 `0x212B53A9` 不变，191–204 宏分支保留原行为。传感器生产代码只修改 `mpu6500.c`，例外在验证 JSON 中逐文件记录原/新哈希，其余 184 个受保护源文件校验通过。四轮及外环控制参数不改。

完整主机回归、191 兼容、新旧 USB 和 Debug/Release clean 编译全部通过。174 个原始产物及 474 个前序候选文件的哈希复核通过。Debug RAM/FLASH 为 106072 / 190252 字节，Release 为 105216 / 81632 字节。实际检查与函数栈帧见 `v0205_offline_verification.json`。

- Release HEX：`build/firmware/0.20.5/Release/rosDemo-0.20.5.hex`。
- HEX SHA256：`4ea4ee72b726de2815ed3a7825133d504860a3bb94fc21fdc12adccb14c0977c`。
- ZIP：`releases/0.20.5/rosDemo-0.20.5-offline-candidate.zip`。
- ZIP SHA256：`63cf733ce359831f56d452eb47570bd1baaf6d66a68fcf82882a02923b5247f4`。

**0.20.5 尚未烧录，未宣称硬件故障已解决或转向验收已通过。** 下一步烧录后先读取启动诊断、IMU 样本与基线就绪，再继续短时运动和停车测试。所有前版固件与既往证据保留。目录校验见 `capture_manifest.json`。
