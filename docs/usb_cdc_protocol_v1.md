# Chassis USB CDC protocol v1

This document defines the byte protocol between the STM32F407 chassis and the
Jetson host. USB CDC is a byte stream: one frame may be split across USB
packets, and several frames may arrive in one packet.

Firmware 0.16.0 adds opt-in `CMD_SET_IMU_STREAM` payload `02`, message `0x86`
RAW v2: same 52-byte RAW layout, but signed int32 bias fields at offsets
20/24/28 use **milli-RAW**, and byte 50 is **2**. Gyro RAW values and sensitivity
fields are unchanged. Use `(gyro_raw - bias_milli_raw/1000.0)` for correction.
The default `0x84` SI layout is unchanged and uses the precise baseline.
Mode 1 / `0x85` remains a **coarse integer-baseline compatibility view**; it
cannot reproduce the new fractional correction exactly. See the complete
[0.16.0 ROS handoff](STM32_IMU_FRACTIONAL_BIAS_0.16.0_HANDOFF_20261017.md).

Firmware 0.15.2 scheduling note (wire layouts unchanged): IMU SI/RAW periodic
telemetry follows each new acquisition snapshot, nominally 100 Hz, rather than
an independent 10 ms resend clock. Unchanged snapshots are sent only for explicit
query/stream selection, health/calibration changes, or a 100 ms stalled-source
health heartbeat. Deduplicate by source sample counter/timestamp before
integration; a received frame is not necessarily a new acquisition. Delivery
remains latest-only under task/host stalls, not a lossless sensor FIFO. See
[0.15.2 handoff](STM32_IMU_SAMPLE_TELEMETRY_0.15.2_HANDOFF_20261017.md).

## Frame

All multi-byte integers are little-endian. Signed values use two's complement.

| Offset | Size | Field |
|---:|---:|---|
| 0 | 1 | SOF0 = `0xA5` |
| 1 | 1 | SOF1 = `0x5A` |
| 2 | 1 | protocol version = `1` |
| 3 | 1 | message type |
| 4 | 2 | sequence |
| 6 | 2 | payload length, maximum 52 |
| 8 | N | payload |
| 8+N | 2 | CRC16-CCITT-FALSE |

CRC parameters: polynomial `0x1021`, initial value `0xFFFF`, no reflection,
no final XOR. The CRC covers bytes from `version` through the final payload
byte; it does not cover the two SOF bytes or the CRC field.

The complete frame is at most 62 bytes, so it fits inside one USB FS CDC data
packet. Sequence numbers are diagnostic/correlation values; safety does not
depend on their ordering.

## Host-to-controller messages

### `0x01` CMD_VELOCITY

Payload length is 8 bytes:

| Offset | Type | Meaning |
|---:|---|---|
| 0 | int32 | linear X, mm/s; valid range -200..200 |
| 4 | int32 | angular Z, mrad/s; valid range -1500..1500 |

Positive linear X is chassis forward. Positive angular Z is counter-clockwise
(left turn), matching ROS REP-103. A non-zero command must be refreshed faster
than the 250 ms firmware timeout. The recommended host rate is 20 Hz.

### `0x02` CMD_STOP

Zero-length payload. Immediately stops production runtime control, cancels an
unfinished IMU calibration, and stops any Debug-only UART motor diagnostic.
STOP has priority over diagnostic/calibration ownership.

### `0x03` CMD_CLEAR_FAULT

Zero-length payload. Accepted only when E-stop is inactive, diagnostics do not
own the motors, and the controller is stopped. A latched E-stop still requires
an MCU reset.

### `0x04` CMD_GET_STATUS

Zero-length payload. Returns an ACK followed by STATUS, BATTERY_STATE and
IMU_STATE frames. Any valid host frame opens a 1.2 s telemetry session; while
it remains open the controller sends STATUS at 10 Hz, WHEEL_STATE at 20 Hz,
BATTERY_STATE at 2 Hz and IMU_STATE at 100 Hz. Velocity commands therefore
receive continuous telemetry without requiring extra polling.

### `0x05` CMD_CALIBRATE_IMU

Zero-length payload. Requests one non-blocking, 500-sample stationary gyro-bias
calibration using the existing STM32-side raw-sample algorithm. The matching
ACK uses the request sequence. `OK` means only that collection started; final
success or failure is reported through `IMU_STATE` flags.

The request is accepted only while the chassis is safely stopped: runtime
IDLE, no valid command or active controller, motor standby disabled and all
outputs zero, all four wheels stopped, E-stop inactive, no critical health
fault, no motion diagnostic owner, and IMU READY/valid. A second request while
calibration is active returns `STATE_REJECTED`. A non-zero payload returns
`BAD_LENGTH`.

Every `CMD_VELOCITY`, including a zero command, returns `STATE_REJECTED` while
calibration is active. `CMD_STOP`, USB physical loss, DTR close, or expiration
of the 1.2 s host session cancels an unfinished USB-originated calibration.
The host must therefore continue sending a valid keepalive such as
`CMD_GET_STATUS` every 0.8 s while waiting for the approximately 5 s result.

## Controller-to-host messages

### `0x80` ACK

Payload length is 4 bytes:

| Offset | Type | Meaning |
|---:|---|---|
| 0 | uint8 | acknowledged request type |
| 1 | uint8 | result |
| 2 | uint8 | runtime state |
| 3 | uint8 | runtime fault |

Result values: `0=OK`, `1=BAD_LENGTH`, `2=BAD_VALUE`,
`3=STATE_REJECTED`, `4=UNSUPPORTED`, `5=INTERNAL`.

Runtime states: `0=INIT`, `1=IDLE`, `2=ACTIVE`, `3=FAULT`, `4=ESTOP`.

### `0x81` STATUS

Payload length is 52 bytes:

| Offset | Type | Meaning |
|---:|---|---|
| 0 | uint32 | MCU uptime, ms |
| 4 | uint32 | runtime command sequence |
| 8 | uint32 | accepted runtime commands |
| 12 | uint32 | rejected runtime commands |
| 16 | uint16 | most recent host frame sequence |
| 18 | uint8 | runtime state |
| 19 | uint8 | runtime fault |
| 20 | uint8 | last stop reason |
| 21 | uint8 | controller fault |
| 22 | uint16 | flags |
| 24 | int16 | requested linear X, mm/s |
| 26 | int16 | requested angular Z, mrad/s |
| 28 | int16 | LF measured speed, mm/s |
| 30 | int16 | LR measured speed, mm/s |
| 32 | int16 | RF measured speed, mm/s |
| 34 | int16 | RR measured speed, mm/s |
| 36 | uint32 | valid protocol frames |
| 40 | uint32 | CRC errors |
| 44 | uint32 | RX overflow packets |
| 48 | uint32 | dropped TX frames |

Flag bits: bit 0 command valid, bit 1 diagnostics active, bit 2 E-stop
latched, bit 3 E-stop input active, bit 4 USB configured, bit 5 chassis
controller active, bit 6 host CDC port open (DTR asserted).
Bit 4 (`0x0010`, legacy name USB_CONFIGURED) means configured **and link alive**:
when SOF monitoring is enabled, the latest SOF must be at most 50 ms old.
Bit 6 (`0x0040`) follows DTR after the first control-line request; before that
request it retains the compatibility fallback for hosts without DTR support.
Normal STOP clears command-valid/controller-active (bits 0/5), not bits 4/6.
With a healthy link, open host port and eligible watchdog, stopped STATUS has
`0x0150`. While moving, additional bits are expected: check required bits with
`(flags & 0x0150) == 0x0150`, not equality against the entire flags field.
Firmware 0.13.2 samples fresh HAL time and USB/SOF/DTR state together in a short
task critical section for both STATUS and protocol link/session decisions;
it does not use an old caller timestamp to compare against a newer SOF IRQ.
Bit 7 is the latched Fault/Health Manager fault indicator. Bit 8 means the IWDG
arbiter is armed, all supervised heartbeats are current, and refresh is
eligible. The payload size and all existing offsets are unchanged.

Runtime fault values: `0=NONE`, `1=INVALID_COMMAND`,
`2=DIAGNOSTIC_CONFLICT`, `3=CONTROLLER_START`, `4=CONTROLLER_RUNTIME`,
`5=HEALTH_MANAGER`.

Stop reasons: `0=NONE`, `1=ZERO_COMMAND`, `2=EXPLICIT`,
`3=COMMAND_TIMEOUT`, `4=DIAGNOSTIC`, `5=ESTOP`, `6=FAULT`,
`7=TRANSPORT_LOST`.

### `0x82` WHEEL_STATE

Payload length is 52 bytes. The controller publishes this frame at 20 Hz while
a host protocol session is active. A valid host frame opens the session for
1.2 seconds, so an idle ROS driver should send `CMD_GET_STATUS` approximately
once per second. Regular velocity commands also keep the session active.

| Offset | Type | Meaning |
|---:|---:|---|
| 0 | uint32 | MCU sample time, ms |
| 4 | int64 | LF accumulated encoder count |
| 12 | int64 | LR accumulated encoder count |
| 20 | int64 | RF accumulated encoder count |
| 28 | int64 | RR accumulated encoder count |
| 36 | int32 | LF filtered speed, counts/s |
| 40 | int32 | LR filtered speed, counts/s |
| 44 | int32 | RF filtered speed, counts/s |
| 48 | int32 | RR filtered speed, counts/s |

Wheel order is always LF, LR, RF, RR. Counts and speeds are
polarity-normalized: positive means physical wheel-forward. Accumulated
`int64` counts are the source for host-side joint position and odometry and
remain continuous across hardware timer wrap. Speed values are provided for
joint velocity and diagnostics; they are not raw timer registers.

### `0x83` BATTERY_STATE

Payload length is 28 bytes. The controller publishes this frame at 2 Hz while
a host protocol session is active and sends one immediately after a successful
`CMD_GET_STATUS`.

| Offset | Type | Meaning |
|---:|---:|---|
| 0 | uint32 | ADC sample time, ms |
| 4 | uint32 | successful filtered sample count |
| 8 | uint32 | cumulative ADC acquisition errors |
| 12 | uint16 | latest 16-sample raw average |
| 14 | uint16 | filtered 12-bit ADC count |
| 16 | uint16 | calculated ADC pin voltage, mV |
| 18 | uint16 | calculated D24A VM/supply voltage, mV |
| 20 | uint8 | battery monitor state |
| 21 | uint8 | battery flags |
| 22 | uint16 | consecutive ADC errors |
| 24 | uint16 | configured low threshold, mV; zero if disabled |
| 26 | uint16 | configured critical threshold, mV; zero if disabled |

Battery states: `0=NOT_READY`, `1=MONITOR_ONLY`, `2=NORMAL`, `3=LOW`,
`4=CRITICAL`, `5=ADC_FAULT`. Flag bits: bit 0 valid measurement, bit 1
protection enabled, bit 2 low, bit 3 critical, bit 4 ADC fault.

The D24A nominal 10 kOhm/1 kOhm divider is represented as 1/11. Stage 11A
keeps protection disabled until the battery chemistry and a real-voltage
calibration point are confirmed; therefore normal initial operation reports
`MONITOR_ONLY`, flag `0x01`, and zero thresholds.

### `0x84` IMU_STATE

Payload length is 46 bytes. The controller publishes the most recent MPU6050
sample at 100 Hz while a host protocol session is active and sends one
immediately after a successful `CMD_GET_STATUS`.

| Offset | Type | Meaning |
|---:|---:|---|
| 0 | uint32 | MCU IMU sample time, ms |
| 4 | uint32 | successful IMU sample count |
| 8 | int32 | body-frame linear acceleration X, mm/s² |
| 12 | int32 | body-frame linear acceleration Y, mm/s² |
| 16 | int32 | body-frame linear acceleration Z, mm/s² |
| 20 | int32 | body-frame angular velocity X, mrad/s |
| 24 | int32 | body-frame angular velocity Y, mrad/s |
| 28 | int32 | body-frame angular velocity Z, mrad/s |
| 32 | int32 | sensor temperature, m°C |
| 36 | uint32 | cumulative IMU read errors |
| 40 | uint32 | consecutive IMU read errors |
| 44 | uint8 | IMU state |
| 45 | uint8 | IMU flags |

IMU states: `0=NOT_READY`, `1=INITIALIZING`, `2=READY`,
`3=DISCONNECTED`, `4=IO_FAULT`.

IMU flag bits:

| Bit | Name | Meaning |
|---:|---|---|
| 0 | valid sample | Existing definition; current sample is valid |
| 1 | gyro bias valid | Existing definition; runtime bias is applied |
| 2 | service initialized | Existing definition; IMU service initialized |
| 3 | calibration active | A 500-sample stationary collection is running |
| 4 | calibration result valid | The most recently accepted calibration ended |
| 5 | calibration last success | It passed and the new bias was applied |
| 6 | calibration last moved | Motion/gravity stationarity checks failed |
| 7 | calibration last bias range | Stationary, but bias was outside its allowed range |

Cold boot clears bits 3..7. Accepting a new request sets bit 3 and clears the
old result bits. Cancellation is represented by bit 4 set with bits 5..7
clear. Failure or cancellation never clears or overwrites a previously valid
bias, so bit 1 may remain set while bits 6 or 7 describe the latest failed
attempt. A successful explicit calibration also appends the bias to the
reserved internal-Flash journal. After a later MCU reset, a compatible valid
record is restored automatically; this sets bit 1 but leaves bits 3..7 clear.

Acceleration and angular velocity are already mapped into the ROS `base_link`
right-handed frame: +X forward, +Y left, +Z up and positive angular Z is a
counter-clockwise left turn. They are physical SI quantities expressed with
integer milli-units for deterministic transport. A ROS driver divides the
acceleration and angular-velocity fields by 1000 before filling
`sensor_msgs/Imu`. The host should inspect flag bit 1 before treating
stationary angular velocity as bias-corrected. Flash absence, corruption,
format mismatch or IMU-identity mismatch leaves bit 1 clear and does not start
an automatic calibration.

## Safety behavior

- CRC/length/version failures never refresh the chassis command heartbeat.
- Loss of velocity frames stops an active chassis within 250 ms.
- USB deconfiguration requests an immediate production-runtime stop and
  records `TRANSPORT_LOST`; loss of application heartbeats while USB remains
  configured records `COMMAND_TIMEOUT`.
- Because the final controller is self-powered and CubeMX VBUS sensing is
  disabled, a 50 ms USB SOF watchdog also detects physical cable/host loss.
  The 250 ms velocity-command watchdog remains an independent fallback.
- Out-of-range velocity commands are passed to the runtime safety boundary,
  which enters FAULT and removes motor drive.
- USB callbacks only copy bytes and update flags. Parsing and motor-facing API
  calls happen in `usbRxTask`, never in interrupt context.
- CDC DTR is monitored when the host supports it. Closing the host serial port
  immediately ends telemetry, discards queued unsent frames, and stops an
  active runtime. Hosts that do not issue the optional control-line request
  retain protocol-frame and heartbeat compatibility.

## Validation utility

`Tools/usb_cdc_test.py` is the protocol reference client. Install pyserial and
replace `COM4` with the STM32 Virtual COM Port as needed:

```text
python -m pip install pyserial
python Tools/usb_cdc_test.py --port COM4
python Tools/usb_cdc_test.py --port COM4 --wheel-monitor 10
python Tools/usb_cdc_test.py --port COM4 --imu-monitor 10
python Tools/usb_cdc_test.py --port COM4 --imu-calibration-test
python Tools/usb_cdc_test.py --port COM4 --safety-test
python Tools/usb_cdc_test.py --port COM4 --drive-test
python Tools/usb_cdc_test.py --port COM4 --dtr-stop-test
python Tools/usb_cdc_test.py --port COM4 --disconnect-test
python Tools/wheel_odometry_test.py --port COM4 --duration 10
python Tools/usb_cdc_soak_test.py --port COM4 --cycles 5
```

The safety test does not intentionally move the motors. It verifies that a
bad-CRC velocity frame is ignored, an out-of-range velocity forces FAULT, and
the clear-fault command returns the stopped runtime to IDLE. The disconnect
test moves at 40 mm/s for at most five seconds. Unplug only the STM32 USB device
cable after the utility prints `ACTIVE confirmed`, while keeping the CH340
debug cable and motor power connected. An unplug before ACTIVE is confirmed
fails the test. The UART log must show `transport lost` and the chassis must
stop immediately. If the host never detects an unplug, the utility sends STOP
at the five-second limit.

The DTR-stop test first establishes an ACTIVE runtime at 40 mm/s, closes the
logical host session, then reconnects and verifies `IDLE / TRANSPORT_LOST`, a
fault-free runtime, and an unchanged TX-drop counter. It is the automated
counterpart to the physical cable-disconnect test.

The wheel monitor is read-only: it never sends a velocity command. It verifies
that CRC-valid `WHEEL_STATE` frames arrive continuously and reports their MCU
sample rate. Its displayed order is LF, LR, RF, RR.

The IMU monitor is also read-only. It verifies CRC-valid, READY and valid
`IMU_STATE` frames, advancing sample timestamps/counters and an 80..120 Hz
effective sample rate. It prints the latest fixed-point SI sample and the gyro
bias-valid flag.

The IMU calibration test requires the chassis to remain still and level. It
checks the rejected non-zero payload, matching ACK, active/result flags,
duplicate-request rejection, and zero-velocity-command rejection. It sends a
`CMD_GET_STATUS` keepalive throughout the test and never intentionally moves
the chassis.

`Tools/wheel_odometry_test.py` is the Stage 9B host-side reference for wheel
angle and differential-drive odometry calculations. See
`docs/host_odometry_validation.md` for its geometry, conventions, and manual
straight/left/right validation procedure.

`Tools/usb_cdc_soak_test.py` repeatedly opens and closes the CDC host port and
checks DTR session termination, stale-frame isolation, telemetry timing, MCU
uptime continuity, and communication error counters. See
`docs/usb_cdc_soak_test.md`.
# 0.13.3 TX 调度补充（帧格式不变）

`0.13.3-usb-tx-burst-fix` 保持消息 ID、CRC、payload 和 STATUS flags 不变。
ACK/STATUS 使用 FIFO 预留及有界待发送缓存，不抢占旧帧顺序；WHEEL/BATTERY/IMU
背压时延后生成，保留累计计数，下一次取最新快照，可能出现非固定遥测间隔。
未入队的延后遥测不消耗 device sequence；实际丢弃仍计 tx_drop。
USB TX 本地诊断的 deferred/full 不等于丢帧，详细说明见
`STM32_USB_TX_BURST_FIX_DELIVERY_20261012.md`。

