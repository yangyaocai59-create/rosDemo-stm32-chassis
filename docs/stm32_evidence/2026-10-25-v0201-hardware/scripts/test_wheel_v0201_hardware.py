"""Bounded 0.20.1 hardware checks with raw evidence and verified STOP.

Requires --execute. Never flashes, resets, clears faults, or retries motion.
Physical setup is recorded as unspecified unless supplied by the operator.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import statistics
import struct
import time

import serial

from debug_uart_capture import DebugUartCapture
from parse_motion_trace_v2 import parse
import usb_cdc_test as wire

CASES = {"forward": (40, 0), "reverse": (-40, 0),
         "left": (0, 400), "right": (0, -400)}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--case", choices=CASES, required=True)
    ap.add_argument("--seconds", type=float, default=3.)
    ap.add_argument("--setup", choices=("unspecified", "ground", "lifted"), default="unspecified")
    ap.add_argument("--stop-mode", choices=("explicit", "timeout", "dtr"), default="explicit")
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--debug-port", default="COM8")
    args = ap.parse_args()
    if not args.execute or not 1 <= args.seconds <= 15:
        ap.error("--execute required; seconds must be 1..15")
    folder = Path(__file__).resolve().parents[1] / "build/hardware_tests" / (
        "v0201_" + args.case + "_" + args.stop_mode + "_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    folder.mkdir(parents=True, exist_ok=False)
    capture = DebugUartCapture(args.debug_port, folder)
    parser = wire.StreamParser()
    frames, commands, raw = [], [], bytearray()
    usb = None
    status = wheel = battery = None
    status_time = wheel_time = 0.
    zero_since = None
    pending = {}
    sequence = 0
    counters = None
    phase = "baseline"
    summary = dict(firmware_user_report="0.20.1", boot_banner_verified=False,
                   setup=args.setup, case=args.case, planned_seconds=args.seconds,
                   stop_mode=args.stop_mode, motion_attempts=0, fault_cleared=False,
                   reset_requested=False, error=None, trace_error=None,
                   final_stopped_verified=False, folder=str(folder))

    def send(kind, payload=b""):
        nonlocal sequence
        sequence = (sequence + 1) & 65535
        encoded = wire.encode_frame(kind, sequence, payload)
        now = time.monotonic()
        if usb.write(encoded) != len(encoded):
            raise RuntimeError("partial USB write")
        pending[sequence] = (kind, now)
        commands.append(dict(host_monotonic=now, phase=phase, type=kind,
                             sequence=sequence, payload_hex=payload.hex()))

    def pump(seconds, enforce=True):
        nonlocal status, wheel, battery, status_time, wheel_time, zero_since, counters
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            capture.check()
            data = usb.read(8192)
            raw.extend(data)
            for frame in parser.feed(data):
                now = time.monotonic()
                frames.append(dict(host_monotonic=now, phase=phase, type=frame.message_type,
                                   sequence=frame.sequence, payload_hex=frame.payload.hex()))
                if frame.message_type == wire.MSG_ACK and len(frame.payload) == 4:
                    request, result, _, fault = frame.payload
                    request_info = pending.pop(frame.sequence, None)
                    if enforce and request_info and (request != request_info[0] or result or fault):
                        raise RuntimeError("rejected/mismatched ACK: " + str(tuple(frame.payload)))
                decoded = wire.decode_status(frame)
                if decoded is not None:
                    if status is not None and ((decoded[0] - status[0]) & 0xffffffff) > 1000:
                        raise RuntimeError("MCU reset or STATUS time gap >1s")
                    status, status_time = decoded, now
                    if counters is None:
                        counters = status[17:20]
                    if enforce:
                        if status[5] not in (1, 2) or status[6] or status[8] or status[9] & 0x008e:
                            raise RuntimeError("runtime/health fault: " + str(status))
                        if status[9] & 0x150 != 0x150:
                            raise RuntimeError("missing ready/link/watchdog flags: " + str(status))
                        if status[17:20] != counters or status[3]:
                            raise RuntimeError("device transport error/rejected command: " + str(status))
                decoded = wire.decode_wheel_state(frame)
                if decoded is not None:
                    wheel, wheel_time = decoded, now
                    if any(wheel[5:]):
                        zero_since = None
                    elif zero_since is None:
                        zero_since = now
                if frame.message_type == wire.MSG_BATTERY_STATE and len(frame.payload) == 28:
                    battery = struct.unpack("<IIIHHHHBBHHH", frame.payload)
                    if enforce and (not 9000 <= battery[6] <= 16000 or battery[2] or battery[9]):
                        raise RuntimeError("supply/ADC outside test bounds: " + str(battery))
            if enforce:
                now = time.monotonic()
                if parser.crc_errors:
                    raise RuntimeError("host CRC errors")
                if any(now - sent > .4 for _, sent in pending.values()):
                    raise RuntimeError("ACK timeout >400ms")
                if status_time and now - status_time > .4:
                    raise RuntimeError("STATUS stale >400ms")
                if wheel_time and now - wheel_time > .3:
                    raise RuntimeError("wheel telemetry stale >300ms")

    def stopped(require_healthy=True):
        return bool(status and status[5] in ((1,) if require_healthy else (1, 3, 4))
                    and (not require_healthy or not (status[6] or status[8]))
                    and not status[9] & 0x20 and not any(status[10:16])
                    and wheel and not any(wheel[5:]) and zero_since is not None
                    and time.monotonic() - zero_since >= .3
                    and time.monotonic() - status_time < .4
                    and time.monotonic() - wheel_time < .3)

    def dump_trace():
        nonlocal phase
        if not stopped(False):
            raise RuntimeError("cannot dump trace before verified inactive/zero speed")
        phase = "trace"
        capture.port.write(b"t")
        end, next_query = time.monotonic() + 90, 0.
        while time.monotonic() < end:
            if time.monotonic() >= next_query:
                send(wire.CMD_GET_STATUS)
                next_query = time.monotonic() + .2
            pump(.02, False)
            text = (folder / "ch340_raw.bin").read_bytes().decode("utf-8", errors="replace")
            if "TRACE END;" in text:
                decoded = parse(text)
                if decoded[-1]["header"].get("schema") != 3 or decoded[-1]["header"].get("control") != 1:
                    raise RuntimeError("expected schema3 with V2 control diagnostics")
                (folder / "trace_v2.json").write_text(json.dumps(decoded, indent=2) + "\n", encoding="utf-8")
                summary["trace_header"] = decoded[-1]["header"]
                return
        raise RuntimeError("trace incomplete after 90s")

    try:
        capture.start()
        usb = serial.Serial(port=None, baudrate=115200, timeout=.005, write_timeout=.2,
                            dsrdtr=False, rtscts=False)
        usb.dtr = True
        usb.rts = False
        usb.port = args.port
        usb.open()
        send(wire.CMD_GET_STATUS)
        pump(1.2)
        if not stopped() or battery is None:
            raise RuntimeError("baseline not healthy IDLE/stationary with supply telemetry")
        if b"CTRL V2" not in (folder / "ch340_raw.bin").read_bytes():
            raise RuntimeError("missing V2 control diagnostics")
        summary.update(initial_status=status, initial_wheel=wheel, initial_battery=battery)
        phase = "motion"
        vx, wz = CASES[args.case]
        payload = struct.pack("<ii", vx, wz)
        summary["command_mm_s_mrad_s"] = [vx, wz]
        print(f"START {args.case}: vx={vx}mm/s wz={wz}mrad/s max={args.seconds}s; setup={args.setup}", flush=True)
        started, next_send, next_query = time.monotonic(), 0., 0.
        summary["motion_attempts"] = 1
        while time.monotonic() - started < args.seconds:
            pump(.005)
            now = time.monotonic()
            if now >= next_send:
                send(wire.CMD_VELOCITY, payload)
                next_send = now + .05
            if now >= next_query:
                send(wire.CMD_GET_STATUS)
                next_query = now + .1
        summary["motion_command_seconds"] = time.monotonic() - started
        phase = "stop"
        stop_start = time.monotonic()
        if args.stop_mode == "explicit":
            send(wire.CMD_STOP)
        elif args.stop_mode == "dtr":
            usb.dtr = False
            # No velocity or queries while observing device-side transport-loss stop.
            pump(.8, False)
            usb.dtr = True
            pending.clear()
        while time.monotonic() - stop_start < 2.5:
            send(wire.CMD_GET_STATUS)
            pump(.1)
            if stopped():
                break
        summary["stop_verified_after_s"] = time.monotonic() - stop_start
        if not stopped():
            raise RuntimeError("STOP failed to reach healthy IDLE and all four zero")
        expected_reason = {"explicit": 2, "timeout": 3, "dtr": 7}[args.stop_mode]
        if status[7] != expected_reason:
            raise RuntimeError("unexpected stop reason: " + str(status))
        summary["stop_checkpoint_status"] = status
        summary["stop_checkpoint_wheel"] = wheel
        if pending:
            pump(.1)
        summary["final_stopped_verified"] = True
        summary["motion_end_wheel"] = wheel
        dump_trace()
    except Exception as error:
        summary["error"] = str(error)
        print("ABORT:", error, flush=True)
    finally:
        if usb is not None and usb.is_open:
            try:
                phase = "final_stop"
                usb.dtr = True
                send(wire.CMD_STOP)
                pump(.8, False)
                send(wire.CMD_GET_STATUS)
                pump(.3, False)
                summary["final_stopped_verified"] = stopped(False)
                if summary["motion_attempts"] and "trace_header" not in summary:
                    try:
                        dump_trace()
                    except Exception as error:
                        summary["trace_error"] = str(error)
            except Exception as error:
                summary["cleanup_error"] = str(error)
            usb.close()
        capture.close()
        summary.update(final_status=status, final_wheel=wheel, final_battery=battery,
                       host_crc_errors=parser.crc_errors, capture_error=capture.error, ports_closed=True)
        if status and counters is not None and (status[17:20] != counters or status[3]):
            summary["error"] = summary["error"] or "device transport/rejection counters changed during capture"
        if parser.crc_errors:
            summary["error"] = summary["error"] or "host CRC errors during capture"
        (folder / "usb_raw.bin").write_bytes(raw)
        (folder / "usb_frames.json").write_text(json.dumps(frames, indent=2) + "\n", encoding="utf-8")
        (folder / "commands.json").write_text(json.dumps(commands, indent=2) + "\n", encoding="utf-8")
        if summary["motion_attempts"]:
            observed = [wire.decode_wheel_state(wire.Frame(f["type"], f["sequence"], bytes.fromhex(f["payload_hex"])))
                        for f in frames if f["phase"] == "motion" and f["type"] == wire.MSG_WHEEL_STATE]
            summary["motion_wheel_frames"] = len(observed)
            if summary.get("motion_end_wheel") and summary.get("initial_wheel"):
                summary["net_counts"] = [b - a for a, b in zip(summary["initial_wheel"][1:5], summary["motion_end_wheel"][1:5])]
                vx, wz = CASES[args.case]
                signs = ([1 if vx > 0 else -1] * 4) if vx else ([-1, -1, 1, 1] if wz > 0 else [1, 1, -1, -1])
                if any(n * sign < 10 for n, sign in zip(summary["net_counts"], signs)):
                    summary["error"] = summary["error"] or "wheel count direction/no motion check failed"
            if observed:
                summary["mean_motion_cps_includes_startup"] = [statistics.mean(r[5 + i] for r in observed) for i in range(4)]
        summary["result"] = "PASS_BOUNDED_HARDWARE_CHECK" if (
            not summary["error"] and not summary["trace_error"] and not capture.error
            and summary["final_stopped_verified"]) else "FAIL"
        (folder / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        manifest = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in folder.iterdir() if p.is_file()}
        (folder / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(summary, indent=2), flush=True)
    return int(summary["result"] == "FAIL")


if __name__ == "__main__":
    raise SystemExit(main())
