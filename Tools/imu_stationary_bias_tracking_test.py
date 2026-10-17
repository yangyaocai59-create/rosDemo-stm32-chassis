#!/usr/bin/env python3
"""Read-only hardware check for firmware 0.17 RAM-only stationary tracking."""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import datetime, timezone, timedelta
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "Tools"))
import serial
from serial.tools import list_ports
import usb_cdc_test as wire


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", required=True)
    ap.add_argument("--duration", type=int, default=600)
    ap.add_argument("--stationary-confirmed", action="store_true")
    ap.add_argument("--vm-off-confirmed", action="store_true")
    args = ap.parse_args()
    if not args.stationary_confirmed or not args.vm_off_confirmed:
        ap.error("confirm stationary chassis and motor VM off")
    if not 10 <= args.duration <= 600:
        ap.error("duration must be 10..600 seconds")

    ports = {p.device.casefold(): p for p in list_ports.comports()}
    device = ports.get(args.port.casefold())
    if device is None or (device.vid, device.pid) != (0x0483, 0x5740):
        ap.error("port must be the STM32 CDC 0483:5740")
    stamp = datetime.now(timezone(timedelta(hours=8))).strftime("%Y%m%d_%H%M%S_%f")
    folder = ROOT / "build" / "hardware_tests" / ("imu_stationary_bias_" + stamp)
    folder.mkdir(parents=True, exist_ok=False)
    summary = {
        "result": "INCOMPLETE",
        "firmware_expected": "0.17.0-imu-stationary-bias",
        "duration_requested_s": args.duration,
        "stationary_confirmed": True,
        "vm_off_confirmed": True,
        "commands": [],
        "errors": [],
    }
    parser = wire.StreamParser()
    rows = []
    unique = []
    baseline_changes = []
    last_count = None
    last_time = None
    last_status_host = 0.0
    last_wheel_host = 0.0
    sequence = 0
    next_query = 0.0
    next_progress = 30
    port = serial.Serial(port=args.port, baudrate=115200, timeout=.01,
                         write_timeout=.2, dsrdtr=False, rtscts=False)

    def send(message_type: int, payload: bytes = b"") -> int:
        nonlocal sequence
        sequence = (sequence + 1) & 0xFFFF
        packet = wire.encode_frame(message_type, sequence, payload)
        port.write(packet)
        summary["commands"].append({"type": message_type, "sequence": sequence,
                                    "payload": payload.hex()})
        return sequence

    try:
        ack_seq = send(wire.CMD_SET_IMU_STREAM, b"\x02")
        deadline = time.monotonic() + 3.0
        got_ack = False
        while time.monotonic() < deadline and not got_ack:
            for frame in parser.feed(port.read(8192)):
                if frame.message_type == wire.MSG_ACK and frame.sequence == ack_seq:
                    if frame.payload[:2] != bytes([wire.CMD_SET_IMU_STREAM, wire.RESULT_OK]):
                        raise RuntimeError("RAW v2 selector was rejected")
                    got_ack = True
                    break
        if not got_ack:
            raise RuntimeError("RAW v2 selector ACK timeout")

        start = time.monotonic()
        while time.monotonic() - start < args.duration:
            now = time.monotonic()
            if now >= next_query:
                send(wire.CMD_GET_STATUS)
                next_query = now + .8
            for frame in parser.feed(port.read(8192)):
                status = wire.decode_status(frame)
                if status is not None:
                    last_status_host = time.monotonic()
                    if status[5] != wire.STATE_IDLE or status[6] or status[8] or any(status[10:16]):
                        raise RuntimeError("device is not fault-free IDLE with zero motion")
                wheel = wire.decode_wheel_state(frame)
                if wheel is not None:
                    last_wheel_host = time.monotonic()
                    if any(wheel[5:9]):
                        raise RuntimeError("invalid wheel telemetry")
                raw = wire.decode_imu_raw_v2(frame)
                if raw is None:
                    continue
                if raw[16] != wire.IMU_STATE_NAMES.index("READY") or (raw[17] & 7) != 7 or raw[17] & 8:
                    raise RuntimeError(
                        "RAW v2 sample is not READY/valid/not-calibrating: "
                        f"state={raw[16]} flags=0x{raw[17]:02X} "
                        f"bias_milli_raw={list(raw[8:11])}")
                row = list(raw)
                rows.append(row)
                if last_count is None or raw[1] != last_count:
                    if last_count is not None:
                        step = (raw[1] - last_count) & 0xFFFFFFFF
                        dt = (raw[0] - last_time) & 0xFFFFFFFF
                        if step != 1 or not 5 <= dt <= 15:
                            raise RuntimeError(f"source sample gap step={step} dt_ms={dt}")
                    last_count, last_time = raw[1], raw[0]
                    unique.append(raw)
                    baseline = tuple(raw[8:11])
                    if not baseline_changes or baseline_changes[-1]["bias_milli_raw"] != list(baseline):
                        baseline_changes.append({"elapsed_s": time.monotonic() - start,
                                                 "bias_milli_raw": list(baseline),
                                                 "temperature_c": raw[11] / 1000.0})
            if last_status_host and now - last_status_host > .8:
                raise RuntimeError("STATUS telemetry stale")
            if last_wheel_host and now - last_wheel_host > .8:
                raise RuntimeError("WHEEL telemetry stale")
            if now - start >= next_progress:
                print(f"{next_progress}s unique={len(unique)} bias={baseline_changes[-1]['bias_milli_raw']} temp={unique[-1][11]/1000:.3f}C", flush=True)
                next_progress += 30

        if len(unique) < args.duration * 95:
            raise RuntimeError(f"source sample rate too low: {len(unique)} unique")
        send(wire.CMD_SET_IMU_STREAM, b"\x00")
        summary["result"] = "PASS_STATIONARY_TRACKING_CAPTURE"
    except Exception as error:
        summary["errors"].append(str(error))
    finally:
        try:
            port.close()
        except Exception as error:
            summary["errors"].append("close: " + str(error))
        if unique:
            z_residual = [(r[7] - r[10] / 1000.0) / (r[14] / 10.0) for r in unique]
            summary.update(
                raw_frames=len(rows), unique_samples=len(unique),
                source_rate_hz=(len(unique) - 1) / ((unique[-1][0] - unique[0][0]) / 1000.0),
                bias_changes=baseline_changes,
                first_bias_milli_raw=list(unique[0][8:11]),
                last_bias_milli_raw=list(unique[-1][8:11]),
                temperature_c_range=[min(r[11] for r in unique) / 1000.0,
                                     max(r[11] for r in unique) / 1000.0],
                mean_z_deg_min=sum(z_residual) / len(z_residual) * 60.0,
                source_time_ms_range=[unique[0][0], unique[-1][0]],
            )
            with (folder / "raw_samples.jsonl").open("w", encoding="utf-8") as out:
                for row in rows:
                    out.write(json.dumps(row) + "\n")
        (folder / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(summary, indent=2), flush=True)
    return 0 if not summary["errors"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
