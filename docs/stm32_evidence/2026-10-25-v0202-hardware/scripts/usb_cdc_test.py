#!/usr/bin/env python3
"""Inspect and safely validate rosDemo USB CDC protocol v1."""

from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass

SOF = b"\xA5\x5A"
VERSION = 1
MAX_PAYLOAD = 52

CMD_VELOCITY = 0x01
CMD_STOP = 0x02
CMD_CLEAR_FAULT = 0x03
CMD_GET_STATUS = 0x04
CMD_CALIBRATE_IMU = 0x05
CMD_SET_IMU_STREAM = 0x06
MSG_ACK = 0x80
MSG_STATUS = 0x81
MSG_WHEEL_STATE = 0x82
MSG_BATTERY_STATE = 0x83
MSG_IMU_STATE = 0x84
MSG_IMU_RAW_V1 = 0x85
IMU_RAW_V1_STRUCT = struct.Struct("<IIhhhhhhiiiiIIHHBBBB")
MSG_IMU_RAW_V2 = 0x86
IMU_RAW_V2_STRUCT = IMU_RAW_V1_STRUCT  # same offsets; bias units/format differ

BATTERY_STATE_NAMES = [
    "NOT_READY", "MONITOR_ONLY", "NORMAL", "LOW", "CRITICAL", "ADC_FAULT"
]
IMU_STATE_NAMES = ["NOT_READY", "INITIALIZING", "READY", "DISCONNECTED", "IO_FAULT"]

IMU_FLAG_VALID = 0x01
IMU_FLAG_GYRO_BIAS_VALID = 0x02
IMU_FLAG_INITIALIZED = 0x04
IMU_FLAG_CALIBRATION_ACTIVE = 0x08
IMU_FLAG_CALIBRATION_RESULT_VALID = 0x10
IMU_FLAG_CALIBRATION_LAST_SUCCESS = 0x20
IMU_FLAG_CALIBRATION_LAST_MOVED = 0x40
IMU_FLAG_CALIBRATION_LAST_RANGE = 0x80

STATE_NAMES = ["INIT", "IDLE", "ACTIVE", "FAULT", "ESTOP"]
RESULT_NAMES = [
    "OK", "BAD_LENGTH", "BAD_VALUE", "STATE_REJECTED", "UNSUPPORTED", "INTERNAL"
]
STOP_NAMES = [
    "NONE", "ZERO_COMMAND", "EXPLICIT", "COMMAND_TIMEOUT", "DIAGNOSTIC", "ESTOP", "FAULT",
    "TRANSPORT_LOST",
]

STATE_IDLE = 1
STATE_ACTIVE = 2
STATE_FAULT = 3
RESULT_OK = 0
RESULT_STATE_REJECTED = 3
RUNTIME_FAULT_HEALTH_MANAGER = 5
STATUS_FLAG_HEALTH_FAULT = 0x0080
STATUS_FLAG_IWDG_REFRESH_ACTIVE = 0x0100


def name(table: list[str], value: int) -> str:
    return table[value] if 0 <= value < len(table) else f"UNKNOWN({value})"


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(message_type: int, sequence: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too large")
    body = struct.pack("<BBHH", VERSION, message_type, sequence & 0xFFFF, len(payload)) + payload
    return SOF + body + struct.pack("<H", crc16_ccitt_false(body))


@dataclass
class Frame:
    message_type: int
    sequence: int
    payload: bytes


class StreamParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes) -> list[Frame]:
        self.buffer.extend(data)
        frames: list[Frame] = []
        while True:
            sof_index = self.buffer.find(SOF)
            if sof_index < 0:
                if self.buffer[-1:] == SOF[:1]:
                    self.buffer[:] = self.buffer[-1:]
                else:
                    self.buffer.clear()
                break
            if sof_index:
                del self.buffer[:sof_index]
            if len(self.buffer) < 8:
                break
            version, message_type, sequence, payload_length = struct.unpack_from("<BBHH", self.buffer, 2)
            if payload_length > MAX_PAYLOAD:
                del self.buffer[0]
                continue
            frame_length = 10 + payload_length
            if len(self.buffer) < frame_length:
                break
            candidate = bytes(self.buffer[:frame_length])
            del self.buffer[:frame_length]
            received_crc = struct.unpack_from("<H", candidate, 8 + payload_length)[0]
            if version != VERSION or received_crc != crc16_ccitt_false(candidate[2 : 8 + payload_length]):
                self.crc_errors += 1
                continue
            frames.append(Frame(message_type, sequence, candidate[8 : 8 + payload_length]))
        return frames


def print_frame(frame: Frame) -> tuple[int | None, int | None]:
    if frame.message_type == MSG_ACK and len(frame.payload) == 4:
        request, result, state, fault = struct.unpack("<BBBB", frame.payload)
        print(
            f"ACK seq={frame.sequence} request=0x{request:02X} "
            f"result={name(RESULT_NAMES, result)} state={name(STATE_NAMES, state)} fault={fault}"
        )
        return state, None
    if frame.message_type == MSG_STATUS and len(frame.payload) == 52:
        values = struct.unpack("<IIIIHBBBBHhhhhhhIIII", frame.payload)
        (
            uptime, command_sequence, accepted, rejected, last_rx_sequence,
            state, fault, stop_reason, controller_fault, flags,
            vx, wz, lf, lr, rf, rr, valid, crc_errors, rx_overflows, tx_drops,
        ) = values
        print(
            f"STATUS t={uptime}ms state={name(STATE_NAMES, state)} "
            f"stop={name(STOP_NAMES, stop_reason)} vx={vx}mm/s wz={wz}mrad/s "
            f"wheel=[{lf},{lr},{rf},{rr}]mm/s flags=0x{flags:04X} "
            f"cmd_seq={command_sequence} accepted={accepted} rejected={rejected} "
            f"last_rx={last_rx_sequence} frames={valid} crc={crc_errors} "
            f"rx_overflow={rx_overflows} tx_drop={tx_drops} "
            f"fault={fault}/{controller_fault}"
        )
        return state, stop_reason
    if frame.message_type == MSG_WHEEL_STATE and len(frame.payload) == 52:
        uptime, lf_count, lr_count, rf_count, rr_count, lf_cps, lr_cps, rf_cps, rr_cps = (
            struct.unpack("<Iqqqqiiii", frame.payload)
        )
        print(
            f"WHEEL t={uptime}ms count=[{lf_count},{lr_count},{rf_count},{rr_count}] "
            f"cps=[{lf_cps},{lr_cps},{rf_cps},{rr_cps}]"
        )
        return None, None
    if frame.message_type == MSG_BATTERY_STATE and len(frame.payload) == 28:
        (
            sample_time, sample_count, adc_errors, raw, filtered_raw,
            adc_mv, supply_mv, state, flags, consecutive_errors,
            low_mv, critical_mv,
        ) = struct.unpack("<IIIHHHHBBHHH", frame.payload)
        print(
            f"BATTERY t={sample_time}ms supply={supply_mv}mV adc={adc_mv}mV "
            f"raw={raw}/{filtered_raw} state={name(BATTERY_STATE_NAMES, state)} "
            f"flags=0x{flags:02X} samples={sample_count} adc_errors={adc_errors} "
            f"consecutive_errors={consecutive_errors} "
            f"thresholds={low_mv}/{critical_mv}mV"
        )
        return None, None
    if frame.message_type == MSG_IMU_STATE and len(frame.payload) == 46:
        (
            sample_time, sample_count,
            ax, ay, az, wx, wy, wz, temperature_mc,
            read_errors, consecutive_errors, state, flags,
        ) = struct.unpack("<IIiiiiiiiIIBB", frame.payload)
        print(
            f"IMU t={sample_time}ms sample={sample_count} "
            f"a=[{ax},{ay},{az}]mm/s2 w=[{wx},{wy},{wz}]mrad/s "
            f"temp={temperature_mc / 1000.0:.2f}C "
            f"state={name(IMU_STATE_NAMES, state)} flags=0x{flags:02X} "
            f"valid={int(bool(flags & IMU_FLAG_VALID))} "
            f"bias={int(bool(flags & IMU_FLAG_GYRO_BIAS_VALID))} "
            f"cal_active={int(bool(flags & IMU_FLAG_CALIBRATION_ACTIVE))} "
            f"cal_result={int(bool(flags & IMU_FLAG_CALIBRATION_RESULT_VALID))} "
            f"cal_ok={int(bool(flags & IMU_FLAG_CALIBRATION_LAST_SUCCESS))} "
            f"cal_moved={int(bool(flags & IMU_FLAG_CALIBRATION_LAST_MOVED))} "
            f"cal_range={int(bool(flags & IMU_FLAG_CALIBRATION_LAST_RANGE))} "
            f"errors={read_errors}/{consecutive_errors}"
        )
        return None, None
    print(f"FRAME type=0x{frame.message_type:02X} seq={frame.sequence} len={len(frame.payload)}")
    return None, None


def decode_status(frame: Frame) -> tuple[int, ...] | None:
    if frame.message_type != MSG_STATUS or len(frame.payload) != 52:
        return None
    return struct.unpack("<IIIIHBBBBHhhhhhhIIII", frame.payload)


def decode_wheel_state(frame: Frame) -> tuple[int, ...] | None:
    if frame.message_type != MSG_WHEEL_STATE or len(frame.payload) != 52:
        return None
    return struct.unpack("<Iqqqqiiii", frame.payload)


def decode_imu_state(frame: Frame) -> tuple[int, ...] | None:
    if frame.message_type != MSG_IMU_STATE or len(frame.payload) != 46:
        return None
    return struct.unpack("<IIiiiiiiiIIBB", frame.payload)


def decode_imu_raw_v1(frame: Frame) -> tuple[int, ...] | None:
    """Return exact body-frame RAW/baseline values; do not round to mrad/s.

    Layout: time,count, a[3],g[3],bias[3],temp_mC,errors,consecutive,
    gyro_scale_x10,accel_scale,state,flags,format_version,reserved.
    Raises on an unsupported or invalid RAW schema instead of mis-scaling data.
    """
    if frame.message_type != MSG_IMU_RAW_V1:
        return None
    if len(frame.payload) != IMU_RAW_V1_STRUCT.size:
        raise ValueError("IMU_RAW_V1 payload must be exactly 52 bytes")
    raw = IMU_RAW_V1_STRUCT.unpack(frame.payload)
    if raw[18:] != (1, 0) or raw[14] == 0 or raw[15] == 0:
        raise ValueError("unsupported RAW schema/reserved byte or zero scale")
    if raw[17] & IMU_FLAG_GYRO_BIAS_VALID == 0 and any(raw[8:11]):
        raise ValueError("invalid baseline flag with nonzero baseline values")
    return raw


def imu_raw_v1_to_si(raw: tuple[int, ...]) -> dict:
    """Coarse integer-baseline view. Firmware >=0.16 may apply a fractional
    correction not representable here: use RAW v2 for exact SI reconstruction.
    """
    return _imu_raw_to_si(raw, 1.0)


def decode_imu_raw_v2(frame: Frame) -> tuple[int, ...] | None:
    """Exact applied gyro baseline in signed milli-RAW at offsets 20/24/28.

    Sensor RAW values/scales remain unmodified. Message 0x86, format 2.
    """
    if frame.message_type != MSG_IMU_RAW_V2:
        return None
    if len(frame.payload) != IMU_RAW_V2_STRUCT.size:
        raise ValueError('IMU_RAW_V2 payload must be exactly 52 bytes')
    raw = IMU_RAW_V2_STRUCT.unpack(frame.payload)
    if raw[18:] != (2, 0) or raw[14] == 0 or raw[15] == 0:
        raise ValueError('unsupported RAW v2 schema/reserved byte or zero scale')
    if any(abs(b) > 1000000 for b in raw[8:11]):
        raise ValueError('milli-RAW baseline exceeds firmware bounds')
    if raw[17] & IMU_FLAG_GYRO_BIAS_VALID == 0 and any(raw[8:11]):
        raise ValueError('invalid baseline flag with nonzero baseline values')
    return raw


def imu_raw_v2_to_si(raw: tuple[int, ...]) -> dict:
    """Exact baseline correction; no static dead zone or ROS residual learning."""
    return _imu_raw_to_si(raw, 1000.0)


def _imu_raw_to_si(raw: tuple[int, ...], bias_divisor: float) -> dict:
    import math
    scale = raw[14] / 10.0
    return dict(sample_time_ms=raw[0], sample_count=raw[1],
                accel_m_s2=[v / raw[15] * 9.80665 for v in raw[2:5]],
                angular_velocity_rad_s=[(float(v)-float(b)/bias_divisor) / scale * math.pi / 180.0
                                        for v, b in zip(raw[5:8], raw[8:11])],
                temperature_c=raw[11]/1000.0, read_errors=raw[12],
                consecutive_errors=raw[13], state=raw[16], flags=raw[17])


def latest_status(frames: list[Frame]) -> tuple[int, ...] | None:
    statuses = [decoded for frame in frames if (decoded := decode_status(frame)) is not None]
    return statuses[-1] if statuses else None


def latest_imu_state(frames: list[Frame]) -> tuple[int, ...] | None:
    samples = [decoded for frame in frames if (decoded := decode_imu_state(frame)) is not None]
    return samples[-1] if samples else None


def has_ack(frames: list[Frame], sequence: int, result: int, state: int | None = None) -> bool:
    for frame in frames:
        if frame.message_type != MSG_ACK or frame.sequence != sequence or len(frame.payload) != 4:
            continue
        _, received_result, received_state, _ = struct.unpack("<BBBB", frame.payload)
        if received_result == result and (state is None or received_state == state):
            return True
    return False


def has_request_ack(frames: list[Frame], sequence: int, request: int, result: int) -> bool:
    for frame in frames:
        if frame.message_type != MSG_ACK or frame.sequence != sequence or len(frame.payload) != 4:
            continue
        received_request, received_result, _, _ = struct.unpack("<BBBB", frame.payload)
        if received_request == request and received_result == result:
            return True
    return False


def synchronize_session(port, parser: StreamParser, sequence: int) -> bool:
    """Discard stale CDC data and wait for this process's GET_STATUS ACK."""
    port.reset_input_buffer()
    port.write(encode_frame(CMD_GET_STATUS, sequence))
    deadline = time.monotonic() + 2.0

    while time.monotonic() < deadline:
        data = port.read(port.in_waiting or 1)
        if not data:
            continue
        for frame in parser.feed(data):
            if (
                frame.message_type == MSG_ACK
                and frame.sequence == sequence
                and len(frame.payload) == 4
                and frame.payload[0] == CMD_GET_STATUS
                and frame.payload[1] == 0
            ):
                return True
    return False


def read_for(
    port, parser: StreamParser, duration: float, *, echo: bool = True, echo_imu: bool = False
) -> list[Frame]:
    deadline = time.monotonic() + duration
    frames: list[Frame] = []
    while time.monotonic() < deadline:
        data = port.read(port.in_waiting or 1)
        if data:
            new_frames = parser.feed(data)
            frames.extend(new_frames)
            if echo:
                for frame in new_frames:
                    if echo_imu or frame.message_type != MSG_IMU_STATE:
                        print_frame(frame)
    return frames


def wait_for_status(
    port,
    parser: StreamParser,
    duration: float,
    predicate,
) -> tuple[int, ...] | None:
    """Return and print the first STATUS matching predicate."""
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        frames = read_for(port, parser, min(0.1, deadline - time.monotonic()), echo=False)
        for frame in frames:
            status = decode_status(frame)
            if status is not None and predicate(status):
                print_frame(frame)
                return status
    return None


def send_debug_selector(debug_port, selector: str) -> None:
    """Send one deterministic two-character CH340 fault-injection command."""
    # Do not query, purge, or flush the port here. Some Windows CH340/CH34C
    # drivers intermittently reject ClearCommError/PurgeComm even though TX
    # remains fully usable. write() is blocking on this backend, and the short
    # delay keeps the two single-byte commands separated on the wire.
    debug_port.write(b"z")
    time.sleep(0.05)
    debug_port.write(selector.encode("ascii"))
    time.sleep(0.1)


def status_is_health_fault(status: tuple[int, ...]) -> bool:
    return (
        status[5] == STATE_FAULT
        and status[6] == RUNTIME_FAULT_HEALTH_MANAGER
        and (status[9] & STATUS_FLAG_HEALTH_FAULT) != 0
    )


def run_self_test() -> None:
    payload = struct.pack("<ii", 40, -250)
    encoded = encode_frame(CMD_VELOCITY, 0x1234, payload)
    parser = StreamParser()
    frames = parser.feed(b"noise" + encoded[:3]) + parser.feed(encoded[3:])
    assert len(frames) == 1
    assert frames[0] == Frame(CMD_VELOCITY, 0x1234, payload)
    assert len(encode_frame(MSG_STATUS, 1, bytes(52))) == 62
    wheel_payload = struct.pack("<Iqqqqiiii", 1234, -1, 2, -3, 4, -5, 6, -7, 8)
    assert decode_wheel_state(Frame(MSG_WHEEL_STATE, 2, wheel_payload)) == (
        1234, -1, 2, -3, 4, -5, 6, -7, 8
    )
    assert len(encode_frame(MSG_WHEEL_STATE, 2, wheel_payload)) == 62
    battery_payload = struct.pack(
        "<IIIHHHHBBHHH", 1234, 12, 0, 1354, 1350, 1088, 11968, 1, 1, 0, 0, 0
    )
    assert len(battery_payload) == 28
    assert len(encode_frame(MSG_BATTERY_STATE, 3, battery_payload)) == 38
    imu_payload = struct.pack(
        "<IIiiiiiiiIIBB",
        1234, 100, 10, -20, 9807, 1, -2, 3, 28000, 0, 0, 2,
        IMU_FLAG_VALID | IMU_FLAG_GYRO_BIAS_VALID | IMU_FLAG_INITIALIZED,
    )
    assert len(imu_payload) == 46
    assert decode_imu_state(Frame(MSG_IMU_STATE, 4, imu_payload)) == (
        1234, 100, 10, -20, 9807, 1, -2, 3, 28000, 0, 0, 2, 7
    )
    assert len(encode_frame(MSG_IMU_STATE, 4, imu_payload)) == 56
    assert len(encode_frame(CMD_CALIBRATE_IMU, 5)) == 10
    assert (
        IMU_FLAG_CALIBRATION_ACTIVE
        | IMU_FLAG_CALIBRATION_RESULT_VALID
        | IMU_FLAG_CALIBRATION_LAST_SUCCESS
        | IMU_FLAG_CALIBRATION_LAST_MOVED
        | IMU_FLAG_CALIBRATION_LAST_RANGE
    ) == 0xF8
    corrupted = bytearray(encoded)
    corrupted[-1] ^= 0x01
    assert not StreamParser().feed(corrupted)
    health_status = (
        1000, 0, 0, 0, 0,
        STATE_FAULT, RUNTIME_FAULT_HEALTH_MANAGER, 6, 0,
        STATUS_FLAG_HEALTH_FAULT,
        0, 0, 0, 0, 0, 0,
        1, 0, 0, 0,
    )
    assert status_is_health_fault(health_status)
    print("Protocol self-test passed")


def main() -> int:
    argument_parser = argparse.ArgumentParser(description=__doc__)
    argument_parser.add_argument("--port", help="USB CDC port, for example COM4 or /dev/ttyACM0")
    test_group = argument_parser.add_mutually_exclusive_group()
    test_group.add_argument("--drive-test", action="store_true", help="drive forward at 40 mm/s for 1 second")
    test_group.add_argument(
        "--safety-test",
        action="store_true",
        help="non-motion bad-CRC, out-of-range, fault-clear and recovery test",
    )
    test_group.add_argument(
        "--disconnect-test",
        action="store_true",
        help="drive at 40 mm/s while waiting for a manual USB-device cable unplug",
    )
    test_group.add_argument(
        "--dtr-stop-test",
        action="store_true",
        help="drive briefly, deassert DTR, reconnect and verify a transport-loss stop",
    )
    test_group.add_argument(
        "--wheel-monitor",
        type=float,
        metavar="SECONDS",
        help="read and validate 20 Hz wheel-state telemetry without commanding motion",
    )
    test_group.add_argument(
        "--imu-monitor",
        type=float,
        metavar="SECONDS",
        help="read and validate 100 Hz SI-unit IMU telemetry without commanding motion",
    )
    test_group.add_argument(
        "--imu-calibration-test",
        action="store_true",
        help="request USB IMU static calibration and validate flags/interlocks",
    )
    test_group.add_argument(
        "--health-test",
        action="store_true",
        help="Debug-only heartbeat and LF no-feedback fault-injection regression",
    )
    argument_parser.add_argument(
        "--debug-port",
        help="CH340 USART1 port required by --health-test, for example COM3",
    )
    argument_parser.add_argument("--self-test", action="store_true", help="test framing locally without a serial port")
    args = argument_parser.parse_args()

    if args.self_test:
        run_self_test()
        return 0
    if not args.port:
        argument_parser.error("--port is required unless --self-test is used")
    if args.health_test and not args.debug_port:
        argument_parser.error("--debug-port is required with --health-test")
    if args.health_test and args.debug_port.casefold() == args.port.casefold():
        argument_parser.error("--debug-port must be the CH340 port, not the USB CDC port")

    try:
        import serial
    except ImportError:
        print("pyserial is required: python -m pip install pyserial", file=sys.stderr)
        return 2

    parser = StreamParser()
    sequence = 1
    with serial.Serial(args.port, 115200, timeout=0.02, write_timeout=0.2) as port:
        port.reset_input_buffer()
        port.write(encode_frame(CMD_GET_STATUS, sequence))
        sequence += 1
        initial = read_for(port, parser, 0.6)
        initial_status = latest_status(initial)
        if initial_status is None:
            print("No STATUS received; check that this is the STM32 USB CDC port", file=sys.stderr)
            return 3
        initial_imu = latest_imu_state(initial)
        if initial_imu is not None:
            print_frame(
                Frame(
                    MSG_IMU_STATE,
                    0,
                    struct.pack("<IIiiiiiiiIIBB", *initial_imu),
                )
            )

        if args.wheel_monitor is not None:
            if args.wheel_monitor <= 0.0:
                argument_parser.error("--wheel-monitor must be greater than zero")
            samples: list[tuple[int, ...]] = []
            monitor_start = time.monotonic()
            port.write(encode_frame(CMD_GET_STATUS, sequence))
            sequence += 1
            next_keepalive = monitor_start + 0.8
            while time.monotonic() - monitor_start < args.wheel_monitor:
                now = time.monotonic()
                if now >= next_keepalive:
                    port.write(encode_frame(CMD_GET_STATUS, sequence))
                    sequence += 1
                    next_keepalive += 0.8
                frames = read_for(port, parser, 0.05)
                samples.extend(
                    decoded
                    for frame in frames
                    if (decoded := decode_wheel_state(frame)) is not None
                )

            if len(samples) < 2:
                print("FAIL: fewer than two WHEEL_STATE samples received", file=sys.stderr)
                return 11
            intervals = [
                (current[0] - previous[0]) & 0xFFFFFFFF
                for previous, current in zip(samples, samples[1:])
            ]
            invalid_intervals = [interval for interval in intervals if interval == 0 or interval > 200]
            total_interval = sum(intervals)
            measured_hz = 1000.0 * len(intervals) / total_interval if total_interval else 0.0
            if invalid_intervals:
                print(
                    f"FAIL: invalid MCU telemetry intervals: {invalid_intervals[:8]}",
                    file=sys.stderr,
                )
                return 12
            print(
                f"PASS: {len(samples)} WHEEL_STATE samples, MCU rate={measured_hz:.1f} Hz, "
                f"interval={min(intervals)}..{max(intervals)} ms"
            )
            print("Wheel order is LF, LR, RF, RR; positive count/speed means physical forward.")
            return 0

        if args.imu_monitor is not None:
            if args.imu_monitor <= 0.0:
                argument_parser.error("--imu-monitor must be greater than zero")
            samples: list[tuple[int, ...]] = []
            monitor_start = time.monotonic()
            port.write(encode_frame(CMD_GET_STATUS, sequence))
            sequence += 1
            next_keepalive = monitor_start + 0.8
            while time.monotonic() - monitor_start < args.imu_monitor:
                now = time.monotonic()
                if now >= next_keepalive:
                    port.write(encode_frame(CMD_GET_STATUS, sequence))
                    sequence += 1
                    next_keepalive += 0.8
                frames = read_for(port, parser, 0.02, echo=False)
                samples.extend(
                    decoded
                    for frame in frames
                    if (decoded := decode_imu_state(frame)) is not None
                )

            valid_samples = [
                sample for sample in samples
                if sample[11] == 2 and (sample[12] & IMU_FLAG_VALID) != 0
            ]
            if len(valid_samples) < 2:
                print("FAIL: fewer than two valid READY IMU_STATE samples received", file=sys.stderr)
                return 13
            unique_samples = [valid_samples[0]]
            for sample in valid_samples[1:]:
                if sample[1] != unique_samples[-1][1]:
                    unique_samples.append(sample)
            if len(unique_samples) < 2:
                print("FAIL: IMU sample counter did not advance", file=sys.stderr)
                return 14
            intervals = [
                (current[0] - previous[0]) & 0xFFFFFFFF
                for previous, current in zip(unique_samples, unique_samples[1:])
            ]
            invalid_intervals = [interval for interval in intervals if interval == 0 or interval > 30]
            count_jumps = [
                (current[1] - previous[1]) & 0xFFFFFFFF
                for previous, current in zip(unique_samples, unique_samples[1:])
                if ((current[1] - previous[1]) & 0xFFFFFFFF) > 3
            ]
            if invalid_intervals or count_jumps:
                print(
                    f"FAIL: invalid IMU progression intervals={invalid_intervals[:8]} "
                    f"count_steps={count_jumps[:8]}",
                    file=sys.stderr,
                )
                return 15
            total_interval = sum(intervals)
            measured_hz = 1000.0 * len(intervals) / total_interval if total_interval else 0.0
            print_frame(
                Frame(
                    MSG_IMU_STATE,
                    0,
                    struct.pack("<IIiiiiiiiIIBB", *unique_samples[-1]),
                )
            )
            if not 80.0 <= measured_hz <= 120.0:
                print(
                    f"FAIL: IMU sample rate {measured_hz:.1f} Hz is outside 80..120 Hz",
                    file=sys.stderr,
                )
                return 16
            print(
                f"PASS: {len(unique_samples)} unique IMU_STATE samples, "
                f"MCU rate={measured_hz:.1f} Hz, interval={min(intervals)}..{max(intervals)} ms"
            )
            print("IMU frame is ROS base_link: +X forward, +Y left, +Z up; +wz is CCW-left.")
            return 0

        if not (
            args.drive_test
            or args.safety_test
            or args.disconnect_test
            or args.dtr_stop_test
            or args.health_test
            or args.imu_calibration_test
        ):
            print(
                "Read-only status check complete. Add --safety-test, --drive-test, "
                "--wheel-monitor, --imu-monitor, --dtr-stop-test, --disconnect-test, "
                "--imu-calibration-test, or --health-test."
            )
            return 0

        if initial_status[5] != 1:
            print(
                f"Test refused: runtime is {name(STATE_NAMES, initial_status[5])}, not IDLE",
                file=sys.stderr,
            )
            return 4

        if args.imu_calibration_test:
            print("Keep the chassis completely still and level for about 5 seconds.")
            print("This test sends only a zero-velocity interlock probe; it must not move motors.")

            bad_length_sequence = sequence
            sequence += 1
            port.write(encode_frame(CMD_CALIBRATE_IMU, bad_length_sequence, b"\x00"))
            bad_length_frames = read_for(port, parser, 0.3, echo=False)
            if not has_request_ack(bad_length_frames, bad_length_sequence, CMD_CALIBRATE_IMU, 1):
                print("FAIL: non-zero calibration payload did not return BAD_LENGTH", file=sys.stderr)
                return 31

            calibration_sequence = sequence
            sequence += 1
            port.write(encode_frame(CMD_CALIBRATE_IMU, calibration_sequence))

            start = time.monotonic()
            next_keepalive = start + 0.8
            accepted = False
            active_seen = False
            duplicate_sent = False
            duplicate_rejected = False
            velocity_probe_rejected = False
            duplicate_sequence = -1
            velocity_sequence = -1
            final_imu: tuple[int, ...] | None = None

            while time.monotonic() - start < 8.0:
                now = time.monotonic()
                if now >= next_keepalive:
                    port.write(encode_frame(CMD_GET_STATUS, sequence))
                    sequence += 1
                    next_keepalive += 0.8

                frames = read_for(port, parser, 0.02, echo=False)
                if has_request_ack(frames, calibration_sequence, CMD_CALIBRATE_IMU, RESULT_OK):
                    accepted = True
                if duplicate_sequence >= 0 and has_request_ack(
                    frames, duplicate_sequence, CMD_CALIBRATE_IMU, RESULT_STATE_REJECTED
                ):
                    duplicate_rejected = True
                if velocity_sequence >= 0 and has_request_ack(
                    frames, velocity_sequence, CMD_VELOCITY, RESULT_STATE_REJECTED
                ):
                    velocity_probe_rejected = True

                for frame in frames:
                    sample = decode_imu_state(frame)
                    if sample is None:
                        continue
                    flags = sample[12]
                    if flags & IMU_FLAG_CALIBRATION_ACTIVE:
                        active_seen = True
                        if not duplicate_sent:
                            duplicate_sequence = sequence
                            sequence += 1
                            velocity_sequence = sequence
                            sequence += 1
                            port.write(encode_frame(CMD_CALIBRATE_IMU, duplicate_sequence))
                            port.write(
                                encode_frame(CMD_VELOCITY, velocity_sequence, struct.pack("<ii", 0, 0))
                            )
                            duplicate_sent = True
                    elif active_seen and (flags & IMU_FLAG_CALIBRATION_RESULT_VALID):
                        final_imu = sample
                        break
                if final_imu is not None:
                    break

            if not accepted:
                print("FAIL: calibration request did not receive ACK OK", file=sys.stderr)
                return 32
            if not active_seen:
                print("FAIL: calibration active flag was never observed", file=sys.stderr)
                return 33
            if not duplicate_rejected:
                print("FAIL: duplicate calibration request was not rejected", file=sys.stderr)
                return 34
            if not velocity_probe_rejected:
                print("FAIL: velocity command was not rejected during calibration", file=sys.stderr)
                return 35
            if final_imu is None:
                print("FAIL: calibration did not finish within 8 seconds", file=sys.stderr)
                return 36

            final_flags = final_imu[12]
            print_frame(
                Frame(MSG_IMU_STATE, 0, struct.pack("<IIiiiiiiiIIBB", *final_imu))
            )
            if final_flags & IMU_FLAG_CALIBRATION_LAST_MOVED:
                print("FAIL: calibration detected chassis movement; keep it still and retry", file=sys.stderr)
                return 37
            if final_flags & IMU_FLAG_CALIBRATION_LAST_RANGE:
                print("FAIL: measured gyro bias exceeded the allowed range", file=sys.stderr)
                return 38
            required = (
                IMU_FLAG_GYRO_BIAS_VALID
                | IMU_FLAG_CALIBRATION_RESULT_VALID
                | IMU_FLAG_CALIBRATION_LAST_SUCCESS
            )
            if (final_flags & required) != required:
                print(f"FAIL: unexpected final calibration flags 0x{final_flags:02X}", file=sys.stderr)
                return 39
            print("PASS: USB IMU calibration, result flags and motion interlocks verified")
            return 0

        if args.health_test:
            if (initial_status[9] & STATUS_FLAG_IWDG_REFRESH_ACTIVE) == 0:
                print(
                    "FAIL: IWDG refresh arbiter is not active in initial IDLE status",
                    file=sys.stderr,
                )
                return 28
            print("WARNING: close every CH340 serial terminal before this test.")
            print("The first two cases do not move the chassis.")
            print("The final case drives forward at 40 mm/s; keep at least 0.5 m clear.")

            def recover_health_fault(debug_port, current_sequence: int) -> tuple[bool, int]:
                send_debug_selector(debug_port, "0")
                time.sleep(0.35)
                clear_sequence = current_sequence
                port.write(encode_frame(CMD_CLEAR_FAULT, clear_sequence))
                current_sequence += 1
                clear_frames = read_for(port, parser, 0.4, echo=False)
                if not has_ack(clear_frames, clear_sequence, RESULT_OK, STATE_IDLE):
                    print("FAIL: health fault did not clear after injection was disabled", file=sys.stderr)
                    return False, current_sequence

                port.write(encode_frame(CMD_GET_STATUS, current_sequence))
                current_sequence += 1
                recovered = wait_for_status(
                    port,
                    parser,
                    0.6,
                    lambda status: status[5] == STATE_IDLE,
                )
                if (
                    recovered is None
                    or recovered[6] != 0
                    or (recovered[9] & STATUS_FLAG_HEALTH_FAULT) != 0
                    or (recovered[9] & STATUS_FLAG_IWDG_REFRESH_ACTIVE) == 0
                ):
                    print("FAIL: health recovery did not return fault-free IDLE", file=sys.stderr)
                    return False, current_sequence
                return True, current_sequence

            def run_heartbeat_injection(
                debug_port, selector: str, label: str, current_sequence: int
            ) -> tuple[bool, int]:
                print(f"Injecting {label} heartbeat loss...")
                send_debug_selector(debug_port, selector)
                fault_status = wait_for_status(
                    port,
                    parser,
                    1.5,
                    lambda status: status_is_health_fault(status)
                    and (status[9] & STATUS_FLAG_IWDG_REFRESH_ACTIVE) == 0,
                )
                if fault_status is None:
                    print(f"FAIL: {label} heartbeat loss did not latch health fault", file=sys.stderr)
                    send_debug_selector(debug_port, "0")
                    return False, current_sequence

                rejected_sequence = current_sequence
                port.write(
                    encode_frame(
                        CMD_VELOCITY,
                        rejected_sequence,
                        struct.pack("<ii", 40, 0),
                    )
                )
                current_sequence += 1
                rejected_frames = read_for(port, parser, 0.3, echo=False)
                if not has_ack(
                    rejected_frames,
                    rejected_sequence,
                    RESULT_STATE_REJECTED,
                    STATE_FAULT,
                ):
                    print("FAIL: velocity command was not rejected in health FAULT", file=sys.stderr)
                    send_debug_selector(debug_port, "0")
                    return False, current_sequence

                recovered, current_sequence = recover_health_fault(
                    debug_port, current_sequence
                )
                if recovered:
                    print(f"PASS: {label} timeout, inhibit, latch and recovery verified")
                return recovered, current_sequence

            def disable_injection_best_effort() -> None:
                try:
                    with serial.Serial(
                        args.debug_port,
                        115200,
                        timeout=0.02,
                        write_timeout=0.2,
                        dsrdtr=False,
                        rtscts=False,
                    ) as cleanup_debug_port:
                        send_debug_selector(cleanup_debug_port, "0")
                except (serial.SerialException, OSError):
                    pass

            try:
                with serial.Serial(
                    args.debug_port,
                    115200,
                    timeout=0.02,
                    write_timeout=0.2,
                    dsrdtr=False,
                    rtscts=False,
                ) as debug_port:
                    send_debug_selector(debug_port, "0")

                    passed, sequence = run_heartbeat_injection(
                        debug_port, "m", "motorTask", sequence
                    )
                    if not passed:
                        return 21
                    passed, sequence = run_heartbeat_injection(
                        debug_port, "u", "usbRxTask", sequence
                    )
                    if not passed:
                        return 22

                    print("LF no-feedback injection starts in 3 seconds...")
                    for remaining in range(3, 0, -1):
                        print(f"Starting in {remaining}...", flush=True)
                        time.sleep(1.0)
                    send_debug_selector(debug_port, "1")

                    active_observed = False
                    fault_status = None
                    drive_start = time.monotonic()
                    next_command = drive_start
                    while time.monotonic() - drive_start < 4.0:
                        now = time.monotonic()
                        if now >= next_command:
                            port.write(
                                encode_frame(
                                    CMD_VELOCITY,
                                    sequence,
                                    struct.pack("<ii", 40, 0),
                                )
                            )
                            sequence += 1
                            next_command += 0.05
                        frames = read_for(
                            port,
                            parser,
                            min(0.02, max(0.0, next_command - time.monotonic())),
                            echo=False,
                        )
                        for frame in frames:
                            status = decode_status(frame)
                            if status is None:
                                continue
                            active_observed |= status[5] == STATE_ACTIVE
                            if (
                                status_is_health_fault(status)
                                and (
                                    status[9] & STATUS_FLAG_IWDG_REFRESH_ACTIVE
                                ) != 0
                            ):
                                print_frame(frame)
                                fault_status = status
                                break
                        if fault_status is not None:
                            break

                    if not active_observed or fault_status is None:
                        port.write(encode_frame(CMD_STOP, sequence))
                        sequence += 1
                        send_debug_selector(debug_port, "0")
                        print(
                            "FAIL: LF no-feedback injection did not produce ACTIVE -> health FAULT",
                            file=sys.stderr,
                        )
                        return 23

                    stopped = wait_for_status(
                        port,
                        parser,
                        1.0,
                        lambda status: status_is_health_fault(status)
                        and (status[9] & STATUS_FLAG_IWDG_REFRESH_ACTIVE) != 0
                        and max(abs(value) for value in status[12:16]) <= 5,
                    )
                    if stopped is None:
                        send_debug_selector(debug_port, "0")
                        print("FAIL: wheel speeds did not decay after health FAULT", file=sys.stderr)
                        return 24

                    rejected_sequence = sequence
                    port.write(
                        encode_frame(
                            CMD_VELOCITY,
                            rejected_sequence,
                            struct.pack("<ii", 40, 0),
                        )
                    )
                    sequence += 1
                    rejected_frames = read_for(port, parser, 0.3, echo=False)
                    if not has_ack(
                        rejected_frames,
                        rejected_sequence,
                        RESULT_STATE_REJECTED,
                        STATE_FAULT,
                    ):
                        send_debug_selector(debug_port, "0")
                        print("FAIL: post-stall velocity command was not rejected", file=sys.stderr)
                        return 25

                    recovered, sequence = recover_health_fault(debug_port, sequence)
                    if not recovered:
                        return 26
                    print("PASS: LF no-feedback stall detection stopped all wheels")
                    print("PASS: Stage 10B software fault-injection regression complete")
                    return 0
            except (serial.SerialException, OSError) as error:
                try:
                    port.write(encode_frame(CMD_STOP, sequence))
                except (serial.SerialException, OSError):
                    pass
                disable_injection_best_effort()
                print(f"FAIL: cannot use CH340 debug port: {error}", file=sys.stderr)
                return 27
            except KeyboardInterrupt:
                try:
                    port.write(encode_frame(CMD_STOP, sequence))
                except (serial.SerialException, OSError):
                    pass
                disable_injection_best_effort()
                print("ABORTED: STOP sent and fault injection disabled", file=sys.stderr)
                return 130

        if args.safety_test:
            baseline_accepted = initial_status[2]
            baseline_crc_errors = initial_status[17]

            bad_crc_sequence = sequence
            corrupted = bytearray(
                encode_frame(CMD_VELOCITY, bad_crc_sequence, struct.pack("<ii", 40, 0))
            )
            corrupted[-1] ^= 0x01
            port.write(corrupted)
            sequence += 1
            read_for(port, parser, 0.25)

            status_sequence = sequence
            port.write(encode_frame(CMD_GET_STATUS, status_sequence))
            sequence += 1
            after_crc_frames = read_for(port, parser, 0.5)
            after_crc = latest_status(after_crc_frames)
            if (
                after_crc is None
                or after_crc[5] != 1
                or after_crc[2] != baseline_accepted
                or after_crc[17] < baseline_crc_errors + 1
                or has_ack(after_crc_frames, bad_crc_sequence, 0)
            ):
                print("FAIL: corrupted velocity frame was not safely rejected", file=sys.stderr)
                return 6
            print("PASS: bad CRC rejected, no command accepted, runtime remained IDLE")

            range_sequence = sequence
            port.write(
                encode_frame(CMD_VELOCITY, range_sequence, struct.pack("<ii", 201, 0))
            )
            sequence += 1
            fault_frames = read_for(port, parser, 0.5)
            if not has_ack(fault_frames, range_sequence, 2, 3):
                print("FAIL: out-of-range command did not produce BAD_VALUE/FAULT", file=sys.stderr)
                return 7
            print("PASS: out-of-range velocity forced FAULT with motor drive disabled")

            clear_sequence = sequence
            port.write(encode_frame(CMD_CLEAR_FAULT, clear_sequence))
            sequence += 1
            clear_frames = read_for(port, parser, 0.35)
            if not has_ack(clear_frames, clear_sequence, 0, 1):
                print("FAIL: software fault did not clear back to IDLE", file=sys.stderr)
                return 8

            verify_sequence = sequence
            port.write(encode_frame(CMD_GET_STATUS, verify_sequence))
            sequence += 1
            recovery_frames = read_for(port, parser, 0.4)
            recovered = latest_status(recovery_frames)
            if recovered is None or recovered[5] != 1 or recovered[6] != 0:
                print("FAIL: runtime did not recover to fault-free IDLE", file=sys.stderr)
                return 9
            print("PASS: FAULT clear and USB command recovery verified")
            print("PASS: Stage 8C automatic protocol safety regression complete")
            return 0

        if args.dtr_stop_test:
            baseline_tx_drops = initial_status[19]
            print("WARNING: keep at least 0.5 m clear around the chassis.")
            print("In 3 seconds the chassis will drive at 40 mm/s for about 0.8 seconds.")
            print("The test will then deassert DTR and verify an immediate transport-loss stop.")
            for remaining in range(3, 0, -1):
                print(f"Starting in {remaining}...", flush=True)
                time.sleep(1.0)

            active_observed = False
            start = time.monotonic()
            next_command = start
            while time.monotonic() - start < 0.8:
                now = time.monotonic()
                if now >= next_command:
                    port.write(encode_frame(CMD_VELOCITY, sequence, struct.pack("<ii", 40, 0)))
                    sequence += 1
                    next_command += 0.05
                for frame in read_for(
                    port,
                    parser,
                    min(0.01, max(0.0, next_command - time.monotonic())),
                ):
                    status = decode_status(frame)
                    active_observed |= status is not None and status[5] == 2

            if not active_observed:
                port.write(encode_frame(CMD_STOP, sequence))
                read_for(port, parser, 0.2)
                print("FAIL: runtime never reported ACTIVE; STOP sent", file=sys.stderr)
                return 13

            port.dtr = False
            time.sleep(0.35)
            port.dtr = True
            recovery_sequence = sequence
            sequence += 1
            if not synchronize_session(port, parser, recovery_sequence):
                print("FAIL: no session ACK after DTR reconnect", file=sys.stderr)
                return 14

            port.write(encode_frame(CMD_GET_STATUS, sequence))
            sequence += 1
            recovery_frames = read_for(port, parser, 0.5)
            recovered = latest_status(recovery_frames)
            if recovered is None:
                print("FAIL: no STATUS after DTR reconnect", file=sys.stderr)
                return 15
            if recovered[5] != 1 or recovered[7] != 7:
                print(
                    f"FAIL: expected IDLE/TRANSPORT_LOST, got "
                    f"{name(STATE_NAMES, recovered[5])}/{name(STOP_NAMES, recovered[7])}",
                    file=sys.stderr,
                )
                return 16
            if recovered[6] != 0 or recovered[8] != 0:
                print("FAIL: DTR stop left a runtime/controller fault", file=sys.stderr)
                return 17
            if recovered[19] != baseline_tx_drops:
                print(
                    f"FAIL: TX drops changed from {baseline_tx_drops} to {recovered[19]}",
                    file=sys.stderr,
                )
                return 18
            print("PASS: ACTIVE -> IDLE/TRANSPORT_LOST on DTR close")
            print("PASS: DTR reconnect, fault-free status, and TX-drop stability verified")
            return 0

        if args.disconnect_test:
            print("WARNING: keep at least 1 m clear in front of the chassis.")
            print("In 3 seconds the chassis will move at 40 mm/s.")
            print("Wait for the 'ACTIVE confirmed' prompt before unplugging anything.")
            for remaining in range(3, 0, -1):
                print(f"Starting in {remaining}...", flush=True)
                time.sleep(1.0)

            disconnect_detected = False
            active_observed = False
            activation_start = time.monotonic()
            next_command = activation_start
            try:
                while time.monotonic() - activation_start < 2.0:
                    now = time.monotonic()
                    if now >= next_command:
                        port.write(
                            encode_frame(CMD_VELOCITY, sequence, struct.pack("<ii", 40, 0))
                        )
                        sequence += 1
                        next_command += 0.05
                    for frame in read_for(
                        port,
                        parser,
                        min(0.01, max(0.0, next_command - time.monotonic())),
                    ):
                        status = decode_status(frame)
                        if status is not None and status[5] == 2:
                            active_observed = True
                    if active_observed:
                        break
            except (serial.SerialException, OSError) as error:
                print(f"FAIL: USB disconnected before ACTIVE was confirmed: {error}", file=sys.stderr)
                return 19

            if not active_observed:
                port.write(encode_frame(CMD_STOP, sequence))
                read_for(port, parser, 0.2)
                print("FAIL: runtime did not enter ACTIVE within 2 seconds; STOP sent", file=sys.stderr)
                return 20

            print("ACTIVE confirmed: unplug ONLY the STM32 USB DEVICE cable NOW.", flush=True)
            print("Keep the CH340 debug cable and motor power connected.", flush=True)
            disconnect_start = time.monotonic()
            next_command = disconnect_start
            try:
                while time.monotonic() - disconnect_start < 5.0:
                    now = time.monotonic()
                    if now >= next_command:
                        port.write(
                            encode_frame(CMD_VELOCITY, sequence, struct.pack("<ii", 40, 0))
                        )
                        sequence += 1
                        next_command += 0.05
                    read_for(
                        port,
                        parser,
                        min(0.01, max(0.0, next_command - time.monotonic())),
                    )
            except (serial.SerialException, OSError) as error:
                disconnect_detected = True
                print(f"Host detected USB disconnect after ACTIVE: {error}")

            if not disconnect_detected:
                port.write(encode_frame(CMD_STOP, sequence))
                read_for(port, parser, 0.2)
                print("FAIL: no USB disconnect detected during the 5-second window; STOP sent", file=sys.stderr)
                return 10
            print("PASS: physical USB disconnect detected after ACTIVE was confirmed.")
            print("Verify CH340 reports TRANSPORT_LOST and immediate stop.")
            return 0

        print("WARNING: driving forward at 40 mm/s for 1 second; keep 0.5 m clear.")
        start = time.monotonic()
        next_command = start
        while time.monotonic() - start < 1.0:
            now = time.monotonic()
            if now >= next_command:
                port.write(encode_frame(CMD_VELOCITY, sequence, struct.pack("<ii", 40, 0)))
                sequence += 1
                next_command += 0.05
            read_for(port, parser, min(0.01, max(0.0, next_command - time.monotonic())))

        print("Velocity heartbeat stopped intentionally; waiting for 250 ms firmware timeout.")
        observed_timeout = False
        for frame in read_for(port, parser, 1.0):
            if (status := decode_status(frame)) is not None:
                if status[5] == 1 and status[7] == 3:
                    observed_timeout = True

        port.write(encode_frame(CMD_STOP, sequence))
        read_for(port, parser, 0.2)
        if not observed_timeout:
            print("FAIL: ACTIVE -> IDLE COMMAND_TIMEOUT was not observed", file=sys.stderr)
            return 5
        print("PASS: USB velocity control and 250 ms heartbeat stop verified")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
