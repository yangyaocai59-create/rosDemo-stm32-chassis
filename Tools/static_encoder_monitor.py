#!/usr/bin/env python3
"""Read-only stationary encoder capture. Only CMD_GET_STATUS is transmitted."""
from __future__ import annotations

import argparse
from datetime import datetime, timedelta, timezone
import hashlib
import json
from pathlib import Path
import struct
import sys
import time

import usb_cdc_test as protocol


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", required=True)
    ap.add_argument("--duration", type=float, default=120.0)
    ap.add_argument("--boot-log", type=Path, required=True)
    ap.add_argument("--stationary-confirmed", action="store_true")
    args = ap.parse_args()
    if not args.stationary_confirmed or not 120 <= args.duration <= 3600:
        ap.error("confirm stationary observation and duration 120..3600 seconds")
    import serial
    from serial.tools import list_ports
    info = next((p for p in list_ports.comports()
                 if p.device.casefold() == args.port.casefold()), None)
    if info is None or (info.vid, info.pid) != (0x0483, 0x5740):
        ap.error("selected port is not the enumerated STM32 USB CDC device")
    boot = args.boot_log.read_bytes()
    stamp = datetime.now(timezone(timedelta(hours=8))).strftime("%Y%m%d_%H%M%S_%f")
    out = Path(__file__).resolve().parents[1] / "build" / "hardware_tests" / ("static_encoder_" + stamp)
    out.mkdir(parents=True, exist_ok=False)
    (out / "boot_log.txt").write_bytes(boot)
    summary = dict(test="read-only stationary USB capture", timezone="Asia/Shanghai",
                   port=args.port, usb_serial=info.serial_number, duration_requested_s=args.duration,
                   boot_log_sha256=hashlib.sha256(boot).hexdigest(),
                   script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                   host_tx_allowed=["CMD_GET_STATUS (0x04)"], motion_commands_sent=0,
                   vm_condition="user reports VM on", encoder_supply_measured=False,
                   debug_port="COM8 captured separately by user serial terminal",
                   firmware_modified=False, wheel_frames=0, status_frames=0,
                   count_change_events=0, net_count_change=[0]*4,
                   cumulative_absolute_count_change=[0]*4, max_abs_cps=[0]*4,
                   interval_min_ms=None, interval_max_ms=None, anomalies=[], observed_flags=[])
    parser = protocol.StreamParser()
    sequence = 1
    pending = {}
    wheel = None
    status = None
    wheel_seen = status_seen = None
    baseline = previous = None
    counter_baseline = None
    t0 = None
    ticks = {}
    flags = set()
    started = time.monotonic()
    next_query = started
    next_progress = 30.0
    print(f"Read-only capture: {out}", flush=True)
    print("Only GET_STATUS queries; no velocity, STOP, fault-clear, calibration or CH340 writes.", flush=True)
    result = 1
    try:
        with (out / "events.jsonl").open("w", encoding="utf-8") as events:
            def emit(kind, **values):
                events.write(json.dumps(dict(host_monotonic_s=time.monotonic(), kind=kind, **values)) + "\n")

            with serial.Serial(args.port, 115200, timeout=0.01, write_timeout=0.2) as port:
                port.dtr = True
                while True:
                    now = time.monotonic()
                    if t0 is not None and now - t0 >= args.duration:
                        summary["duration_observed_s"] = now - t0
                        break
                    if now >= next_query:
                        frame = protocol.encode_frame(protocol.CMD_GET_STATUS, sequence)
                        if port.write(frame) != len(frame):
                            raise RuntimeError("partial GET_STATUS write")
                        emit("host_tx", message_type=protocol.CMD_GET_STATUS, sequence=sequence, frame_hex=frame.hex())
                        pending[sequence] = now
                        sequence = (sequence + 1) & 0xFFFF
                        next_query = now + 0.8
                    data = port.read(min(max(port.in_waiting, 1), 8192))
                    if data:
                        emit("rx_bytes", data_hex=data.hex())
                        frames = parser.feed(data)
                        if parser.crc_errors:
                            raise RuntimeError("RX CRC/protocol-version error")
                        for frame in frames:
                            emit("frame", message_type=frame.message_type, sequence=frame.sequence, payload_hex=frame.payload.hex())
                            if frame.message_type == protocol.MSG_ACK:
                                if len(frame.payload) != 4:
                                    raise RuntimeError("invalid ACK length")
                                request, ack, state, fault = frame.payload
                                if frame.sequence in pending:
                                    if request != protocol.CMD_GET_STATUS or ack != protocol.RESULT_OK:
                                        raise RuntimeError("GET_STATUS rejected")
                                    del pending[frame.sequence]
                            elif frame.message_type == protocol.MSG_STATUS:
                                status = protocol.decode_status(frame)
                                if status is None:
                                    raise RuntimeError("invalid STATUS length")
                                status_seen = time.monotonic()
                                flags.add(status[9])
                                if (status[5] != protocol.STATE_IDLE or status[6] or status[8]
                                        or status[10:12] != (0, 0) or status[9] & 0x008E
                                        or status[9] & 0x0150 != 0x0150):
                                    raise RuntimeError(f"non-idle/unhealthy STATUS: {status}")
                                counters = status[17:20]
                                if counter_baseline is None:
                                    counter_baseline = counters
                                if counters != counter_baseline:
                                    raise RuntimeError(f"communication counters changed: {counter_baseline} -> {counters}")
                                summary["last_status"] = list(status)
                                if t0 is not None:
                                    summary["status_frames"] += 1
                                emit("status", values=list(status))
                            elif frame.message_type == protocol.MSG_WHEEL_STATE:
                                wheel = protocol.decode_wheel_state(frame)
                                if wheel is None:
                                    raise RuntimeError("invalid WHEEL_STATE length")
                                wheel_seen = time.monotonic()
                                emit("wheel", sample_time_ms=wheel[0], counts=list(wheel[1:5]), cps=list(wheel[5:9]))
                                if t0 is not None:
                                    summary["wheel_frames"] += 1
                                    dt = (wheel[0] - previous[0]) & 0xFFFFFFFF
                                    if dt == 0 or dt > 200:
                                        raise RuntimeError(f"invalid wheel sample interval: {dt} ms")
                                    summary["interval_min_ms"] = min(summary["interval_min_ms"] or dt, dt)
                                    summary["interval_max_ms"] = max(summary["interval_max_ms"] or dt, dt)
                                    delta = [wheel[i+1] - previous[i+1] for i in range(4)]
                                    if any(delta):
                                        summary["count_change_events"] += 1
                                        emit("count_change", sample_time_ms=wheel[0], delta=delta, counts=list(wheel[1:5]), cps=list(wheel[5:9]))
                                    for i in range(4):
                                        summary["net_count_change"][i] = wheel[i+1] - baseline[i+1]
                                        summary["cumulative_absolute_count_change"][i] += abs(delta[i])
                                        summary["max_abs_cps"][i] = max(summary["max_abs_cps"][i], abs(wheel[i+5]))
                                    previous = wheel
                                    summary["last_wheel"] = list(wheel)
                            # Check monotonic time independently for STATUS and WHEEL.
                            if frame.message_type in (protocol.MSG_STATUS, protocol.MSG_WHEEL_STATE):
                                tick = struct.unpack_from("<I", frame.payload)[0]
                                old = ticks.get(frame.message_type)
                                if old is not None and ((tick-old) & 0xFFFFFFFF) > 5000:
                                    raise RuntimeError("MCU timestamp reset/discontinuity")
                                ticks[frame.message_type] = tick
                    now = time.monotonic()
                    if pending and now - min(pending.values()) > 1.0:
                        raise RuntimeError("GET_STATUS matching ACK timeout")
                    if t0 is None:
                        if status is not None and wheel is not None:
                            if any(wheel[5:9]):
                                raise RuntimeError("initial wheel cps is nonzero; stationary baseline not established")
                            t0 = now
                            baseline = previous = wheel
                            summary["first_wheel"] = list(wheel)
                            summary["first_status"] = list(status)
                            emit("observation_start", wheel=list(wheel), status=list(status))
                            print("Baseline established. Keep chassis/wheels untouched for 120 seconds.", flush=True)
                        elif now - started > 3.0:
                            raise RuntimeError("missing initial STATUS/WHEEL telemetry")
                    elif now - wheel_seen > 0.5 or now - status_seen > 0.5:
                        raise RuntimeError("STATUS/WHEEL stream stale")
                    elif now - t0 >= next_progress:
                        print(f"{now-t0:.1f}s: wheels={summary['wheel_frames']} changes={summary['count_change_events']} net={summary['net_count_change']} max_cps={summary['max_abs_cps']}", flush=True)
                        events.flush()
                        next_progress += 30.0
                emit("observation_end")
                # Context exit closes DTR; no motion/STOP commands and no reconnect.
        quiet = not summary["count_change_events"] and not any(summary["max_abs_cps"])
        if summary["wheel_frames"] < 2000 or summary["status_frames"] < 500:
            raise RuntimeError("insufficient telemetry for requested observation")
        summary["result"] = "no_count_change_observed" if quiet else "count_or_cps_change_observed"
        result = 0 if quiet else 3
    except (Exception, KeyboardInterrupt) as error:
        summary["anomalies"].append(str(error) or "operator interrupted")
        summary["result"] = "incomplete_or_unhealthy_capture"
        print(f"Capture stopped: {error}. No movement commands were sent.", flush=True)
    finally:
        summary["parser_crc_errors"] = parser.crc_errors
        summary["observed_flags"] = [f"0x{x:04X}" for x in sorted(flags)]
        summary["communication_counter_baseline"] = list(counter_baseline) if counter_baseline else None
        summary["host_session_closed_at_end"] = True
        (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(summary, indent=2), flush=True)
    return result


if __name__ == "__main__":
    sys.exit(main())
