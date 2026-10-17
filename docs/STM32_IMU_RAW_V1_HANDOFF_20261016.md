# STM32 高精度 IMU RAW 接口交付：0.14.3

日期：2026-10-16（Asia/Shanghai）。**软件实现、真实构建及离线校验完成；用户已烧录，RAW 长采样/兼容/DTR/会话恢复、一次校准保存及普通断电恢复均已实板通过。不是“静止漂移已经修复”的结论。** 本助手未擦除、下载或主动复位设备。实测及收包脚本漏计修复见 `STM32_IMU_RAW_HARDWARE_VERIFICATION_20261016.md`。

## 固件身份与构建

```text
Firmware version = 0.14.3-imu-raw-v1
Build = Oct 16 2026 09:56:40
MCU = STM32F407ZGT6
```

Build 字符串是本机编译器时间；不以它推断 ROS 主机的时钟。用户已报告烧录指定镜像，新增接口功能已实测；本轮未捕获 Boot Banner 或回读整个 Flash，不能宣称独立回读确认了整个镜像身份。

GCC 14.3.1、CMake 4.3.1、Ninja 1.13.2，91 步真实 clean build 成功。编译 Error=0、Warning=0。

| 区域 | 链接器占用 |
| --- | --- |
| FLASH | 159,508 B / 768 KiB |
| RAM | 54,936 B / 128 KiB |
| CCMRAM | 0 B / 64 KiB |

`size` 的 text=159000、data=500、bss=54432，与链接器区域计数有对齐差异；Flash/RAM 占用以上表为准。两区 Flash journal 仍保留，链接器未链接 HAL_FLASHEx_Erase。

无 `.git` 元数据，**没有可提供的 Git 提交号**。用源码快照及 SHA256 对应这次发布，不能捏造提交号。

镜像 SHA256：

```text
rosDemo.hex  2ae4dbcecaffa5db311ad9a365b3ad9faab350923b51aaf54406ab8e5e04d0a4
rosDemo.elf  8f1f302328b81fec16ee9efc7653c12b724d96507dd94de06e6e13c8271c7ba7
rosDemo.bin  ea85aa0f521205368a703faac3e4b471563aaf0189d077dfeef9ec07487ec7d7
```

HEX 与从同一 ELF 再次 objcopy 的结果逐字节一致；全部 Intel HEX checksum 正确，加载地址范围 0x08000000..0x08026F13，不含校准区。BIN 已与 HEX 的实际加载字节比对。初始 SP=0x20020000，Reset vector=0x0800F029。

## 本次改动和明确未改内容

实际固件只修改三个文件：

- `Application/Communication/chassis_usb_protocol.[ch]`：RAW 编码和会话选择。
- `Application/System/firmware_info.h`：版本号。

其他改动是 `Tools/usb_cdc_test.py` 的 RAW 解码/转换、协议回归用例、测试脚本及本文等文档。新增 `Tools/imu_raw_stream_test.py` 用于烧录后静止验收。

对照冻结的 0.14.2 源码，`.ioc`、Core、Drivers、Middlewares、USB_DEVICE、CMake、链接脚本、硬件引脚、MPU6500 采集/量程/滤波/帧映射、显式校准及 Flash 算法、底盘运动控制和保护逻辑均未改。没有增加 DMA、传感器中断、开机自动校准、残余偏置估计或 Flash 写入。

新固件只是提供无额外整数 mrad/s 量化损失的真实数据；最终残余零偏估计由 ROS 的独立组件唯一负责。本地未开发或验证 ROS 组件，也没有宣称最终 <=1°/min 已实现。

## 兼容接口

原 USB v1 封装、SOF、CRC16、STATUS 等原有消息全部保留。默认仍为旧 0x84 IMU_STATE（46 B），旧工具无需修改。增加：

- `0x06 CMD_SET_IMU_STREAM`，payload 1 B：0=LEGACY，1=RAW_V1。
- ACK 沿用原有 ACK，0=OK；错误长度返回 BAD_LENGTH，值不是 0/1 返回 BAD_VALUE。
- RAW_V1 下 0x85（52 B、100 Hz）替代本会话 0x84，其他遥测不变。
- 新会话、DTR 关闭、真实断链和 1200 ms 主机会话超时恢复 LEGACY。
- 0x06 不调用运动/校准 API，不刷新运动命令时间戳，不取代 250 ms 运动超时，不清故障。
- 切换瞬间允许此前已入队的旧格式帧排空；消费者根据类型识别，ACK OK 加第一帧有效目标格式才算协商完成，不要把过渡帧误解为协议长度错误。DTR/断链仍按原 transport 清理队列。
- 旧固件会对 0x06 返回 UNSUPPORTED，ROS 明确降级即可，不要锁存成电机故障。

### 0x85 payload 字段（小端）

| 偏移 | 类型 | 含义 |
| --- | --- | --- |
| 0 / 4 | uint32 各一 | sample_time_ms / sample_count |
| 8 / 10 / 12 | int16 各一 | accel_raw X/Y/Z |
| 14 / 16 / 18 | int16 各一 | gyro_raw X/Y/Z，未减偏置 |
| 20 / 24 / 28 | int32 各一 | 同快照当前基准偏置 X/Y/Z |
| 32 | int32 | temperature_mC |
| 36 / 40 | uint32 各一 | 总读取错误 / 连续错误 |
| 44 | uint16 | gyro_lsb_per_dps_x10，655 表示 65.5 |
| 46 | uint16 | accel_lsb_per_g，当前 8192 |
| 48 | uint8 | IMU state |
| 49 | uint8 | IMU flags，沿用 0x84 各位 |
| 50 / 51 | uint8 各一 | raw_format_version=1 / reserved=0 |

struct=`<IIhhhhhhiiiiIIHHBBBB`，总长 52 B（含封装 62 B）。RAW 已映射到 +X 前、+Y 左、+Z 上，不要再进行相同轴变换。

基准无效时三轴基准字段发 0、bias_valid=0。不能把带 bias_valid=0 的数据当成已恢复校准；没有兼容记录时不自动校准。

```python
raw = decode_imu_raw_v1(frame)  # Tools/usb_cdc_test.py
si = imu_raw_v1_to_si(raw)      # Python double，只有基准修正
# 其中：w[i] = (float(raw_g[i])-float(bias[i])) / 65.5 * pi/180
#       a[i] = raw_a[i] / 8192.0 * 9.80665
```

无姿态解算、不伪造航向；ROS 发布与残余估计规则详见 `IMU_DRIFT_CORRECTION_PLAN_20261016.md`。原 0x84 的整数 mrad/s 单位完全没变。

## 已完成验证

- 九个 native C 测试程序（生产源码 + HAL/RTOS/USB/Flash mock）全部成功，编译使用 Wall/Wextra/Werror。
- 新 RAW 用例覆盖符号/极值、52 B 布局、单次一致性快照、无效偏置清零、校准后新基准、非法选择、旧格式兼容、遥测会话超时、RAW 队列背压和 STOP 突发；检查不刷新运动命令时间戳。
- 原生产 USB/SOF 一致快照、50 次 STOP/GET_STATUS、真实 SOF 51 ms 丢失、DTR/撤配置停车、TX buffer ownership/控制应答预留、无旧指令回放回归保留。
- 原 MPU6500 配置/采集/恢复、500 样本显式校准、Flash CRC/格式/配置隔离/序号回绕/全区/每个字编程失败/读回校验/冷启动不重复写入回归保留。
- Python RAW 四个测试用例成功（包含 63 个分片边界）、原有协议 self-test 成功。
- 额外主机采集器回归成功：同批 ACK+多帧 RAW 的后续帧不再漏计。完整实板字节与帧日志已独立重解码核对，原始证据不覆盖。
- 真实 ARM clean build、ELF/HEX/BIN 一致性、地址与向量检查完成。

上述回归属于软件/模拟测试，不是实板电机安全认证或跨上电漂移验收。随后 RAW 验收脚本已在用户刷写的新固件上通过 10 s 校准前和 120 s 校准后实测，详细范围见独立验收记录。

## 本次已完成流程，以及以后重新烧录时的复用说明

1. 电机 VM 关闭、车体固定。关闭占用 CH340/CDC 的程序。
2. 使用本交付目录的 `firmware/rosDemo.hex`，按既定流程**全片擦除后**写入。不修改 BOOT/跳线的既有正确流程，不让助手自行切换下载电平。若工具只支持 BIN，使用配套 BIN，地址明确为 0x08000000，不重复加载到其他地址。
3. 运行程序后 CH340 115200 8N1 应显示 `0.14.3-imu-raw-v1` 和上述 Build，原电机安全/USB/IMU 启动日志正常，额外出现 `IMU stream: legacy default; CMD 0x06 selects RAW v1/0x85 for this session`。
4. 全片擦除已删除 seq=4 等旧记录，这是用户要求的烧录方式所致；本固件不会自动校准。保持静止，之后明确做一次显式校准，并核对 Flash 保存日志。既有工具 `--imu-calibration-test` 会发零速度互锁探测；若由助手执行，可用 `imu_stationary_diagnostic.py --calibrate-once`，它只发一次校准请求，不发任何速度命令。
5. 本次用户烧录后，助手已实际执行一次显式校准、校准前短测、校准后 120 s RAW 及普通断电后的 120 s RAW。下面是复用命令；具体已发生的结果以独立验收记录及原始证据为准。

```powershell
# RAW 结构/模式/采样/会话超时验收；只发 0x04、0x06，不校准、不驱动电机。
python Tools/imu_raw_stream_test.py --port COM4 --debug-port COM8 --duration 120 --require-bias --vm-off-confirmed --stationary-confirmed
```

串口号以当时枚举为准。RAW 验收脚本将保留 USB 原始字节/完整 RAW 样本/CH340 日志，核对同 sample_count RAW 数据一致性，测试旧默认、非法选择、切回旧格式及会话超时。PASS_INTERFACE_ONLY 仅表示接口通过，不表示 ROS 零偏补偿已经完成。

校准保存、RAW 100 Hz/连续性、与 UART RAW 一致、DTR 重开默认旧模式、旧格式工具及普通断电后的基准恢复均已实测通过。当前保存/恢复基准 [-55,26,150]，保存 seq=1；断电后 flags=0x07，没有自动校准或写 Flash。真实运动中物理断链/运动超时/重复启停回归需另有现场安全确认。ROS 估计器、至少三次上电静止 corrected 漂移、运动慢转保真及最终 EKF 回归均待 ROS 实现后验收。

**本地固件接口工作完成，无需为了转交再次烧录或校准。** 最终包 `STM32_IMU_RAW_0.14.3_INTERFACE_VERIFIED_20261016.zip`，不要将接口通过转述为全部漂移验收通过。设备基准修正后的重上电 RAW 残差仍约 +2.29439°/min，后续由 ROS 唯一补偿方按方案实现。历史冻结候选包及 0.14.2 包不改。
