#!/usr/bin/env python3
"""Bounded ground forward/reverse then stationary capture; no fault clear/reset."""
from __future__ import annotations

import argparse
from datetime import datetime, timedelta, timezone
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
import time

import usb_cdc_test as protocol
import usb_stop_regression as regression
from debug_uart_capture import DebugUartCapture

SPEED = 30
DRIVE_SECONDS = 0.8
SETTLE_SECONDS = 2.0
OBSERVE_SECONDS = 120.0
MAX_OBSERVED_MM_S = 120
LONG_DISTANCE_MM = 500.0
LONG_SPEED_MM_S = 40
LONG_MAX_SECONDS = 16.0
LONG_MAX_WHEEL_MM = 650.0
LONG_MAX_SPREAD_MM = 150.0
LONG_MAX_MM_S = 80
MM_PER_COUNT = math.pi * 65.0 / 1560.0


def travel_mm(before, current, sign):
    """Incremental, forward-normalized wheel travel; not physical ground position."""
    if sign not in (-1, 1):
        raise ValueError("direction must be +1 or -1")
    return [sign * (current[i+1]-before[i+1]) * MM_PER_COUNT for i in range(4)]


def validate_long_travel(wheels):
    if min(wheels) < -20.0:
        raise regression.TestFailure("wheel moved against requested direction (>20mm)")
    if max(wheels) > LONG_MAX_WHEEL_MM:
        raise regression.TestFailure("individual wheel travel exceeded 650mm")
    if max(wheels)-min(wheels) > LONG_MAX_SPREAD_MM:
        raise regression.TestFailure("wheel travel spread exceeded 150mm")


class GroundProbe(regression.Probe):
    def __init__(self, port, events, summary, debug_capture=None, long_travel=False):
        super().__init__(port, events, summary)
        self.debug_capture = debug_capture
        self.long_travel = long_travel
        self.wheel_parser = protocol.StreamParser()
        self.wheel = None
        self.wheel_seen = None
        self.stationary_previous = None
        self.stationary_baseline = None
        self.stationary_frames = 0
        self.change_events = 0
        self.absolute_changes = [0]*4
        self.max_cps = [0]*4
        self.intervals = []

    def emit(self, kind, **data):
        super().emit(kind, **data)
        if kind == "rx_bytes":
            for frame in self.wheel_parser.feed(bytes.fromhex(data["data_hex"])):
                if frame.message_type != protocol.MSG_WHEEL_STATE:
                    continue
                wheel = protocol.decode_wheel_state(frame)
                if wheel is None:
                    raise regression.TestFailure("invalid WHEEL_STATE length")
                if self.wheel is not None:
                    dt = (wheel[0]-self.wheel[0]) & 0xFFFFFFFF
                    if dt == 0 or dt > 200:
                        raise regression.TestFailure(f"invalid WHEEL interval {dt}ms")
                self.wheel = wheel
                self.wheel_seen = time.monotonic()
                self.summary["wheel_frames"] += 1
                super().emit("wheel", sample_time_ms=wheel[0], counts=list(wheel[1:5]), cps=list(wheel[5:9]))
                if self.phase == "stationary":
                    if self.stationary_previous is None:
                        self.stationary_baseline = wheel
                    else:
                        self.intervals.append((wheel[0]-self.stationary_previous[0]) & 0xFFFFFFFF)
                        delta = [wheel[i+1]-self.stationary_previous[i+1] for i in range(4)]
                        if any(delta):
                            self.change_events += 1
                            super().emit("stationary_count_change", sample_time_ms=wheel[0], delta=delta,
                                         counts=list(wheel[1:5]), cps=list(wheel[5:9]))
                            print(f"COUNT CHANGE t={wheel[0]} delta={delta}", flush=True)
                        self.absolute_changes = [a+abs(d) for a, d in zip(self.absolute_changes, delta)]
                    self.max_cps = [max(a, abs(v)) for a, v in zip(self.max_cps, wheel[5:9])]
                    self.stationary_previous = wheel
                    self.stationary_frames += 1

    def pump(self, *, stationary=False, enforce=True):
        self.receive(enforce)
        now = time.monotonic()
        if enforce:
            if self.debug_capture is not None:
                self.debug_capture.check()
            if any(now-sent > 0.3 for _, sent in self.pending.values()):
                raise regression.TestFailure("matching ACK timeout")
            if self.latest_time is None or now-self.latest_time > 0.3:
                raise regression.TestFailure("STATUS stale")
            if self.wheel_seen is None or now-self.wheel_seen > 0.3:
                raise regression.TestFailure("WHEEL stale")
            speed_limit = LONG_MAX_MM_S if self.long_travel else MAX_OBSERVED_MM_S
            if max(abs(v) for v in self.latest[12:16]) > speed_limit:
                raise regression.TestFailure("host wheel-speed safety limit exceeded")
            if stationary and not regression.stopped(self.latest, explicit=True):
                raise regression.TestFailure("stationary phase left IDLE/EXPLICIT/zero request")

    def wait(self, seconds, *, stationary=False, enforce=True):
        start = time.monotonic()
        next_query = start
        progress = 30
        while time.monotonic()-start < seconds:
            self.pump(stationary=stationary, enforce=enforce)
            now = time.monotonic()
            if now >= next_query:
                self.send(protocol.CMD_GET_STATUS)
                next_query = now + 0.8
            if stationary and now-start >= progress:
                print(f"{now-start:.1f}s after settling: frames={self.stationary_frames} count_changes={self.change_events} max_cps={self.max_cps}", flush=True)
                self.events.flush()
                progress += 30

    def drive(self, sign):
        if not regression.stopped(self.latest) or any(self.wheel[5:9]):
            raise regression.TestFailure("not stationary before motion")
        self.phase = "forward" if sign > 0 else "reverse"
        before = self.wheel
        self.peaks = [0]*4
        start = time.monotonic()
        next_velocity = start
        active = False
        max_seconds = LONG_MAX_SECONDS if self.long_travel else DRIVE_SECONDS
        speed = LONG_SPEED_MM_S if self.long_travel else SPEED
        distance_reached = False
        progress = 2.0
        while time.monotonic()-start < max_seconds:
            self.pump()
            now = time.monotonic()
            active |= self.latest[5] == protocol.STATE_ACTIVE
            if self.long_travel:
                wheels = travel_mm(before, self.wheel, sign)
                validate_long_travel(wheels)
                mean_mm = sum(wheels)/4
                if mean_mm >= LONG_DISTANCE_MM:
                    distance_reached = True
                    break  # Do not send another velocity after reaching target.
                if now-start >= progress:
                    print(f"{self.phase}: elapsed={now-start:.1f}s wheel_mm={[round(v,1) for v in wheels]} mean={mean_mm:.1f}mm", flush=True)
                    progress += 2.0
            if now >= next_velocity:
                self.send(protocol.CMD_VELOCITY, struct.pack("<ii", sign*speed, 0))
                next_velocity = now + 0.05
        drive_actual_seconds = time.monotonic()-start
        motion_phase = self.phase
        self.phase += "_stop"
        stop_sequence = self.send(protocol.CMD_STOP)
        self.emit("stop_requested", sequence=stop_sequence, last_status_mcu_time_ms=self.latest[0])
        self.wait(SETTLE_SECONDS)
        if stop_sequence in self.pending or not regression.stopped(self.latest, explicit=True) or any(self.wheel[5:9]):
            raise regression.TestFailure("STOP ACK or settled IDLE/EXPLICIT/zero-cps check failed")
        delta = [self.wheel[i+1]-before[i+1] for i in range(4)]
        settled_travel = travel_mm(before, self.wheel, sign)
        if self.long_travel:
            validate_long_travel(settled_travel)
            if not distance_reached:
                raise regression.TestFailure("500mm target not reached before 16s limit; STOP confirmed")
        if not active or not all(sign*d > 0 for d in delta):
            raise regression.TestFailure(f"motion/encoder signs not verified: {delta}")
        self.summary["motions"].append(dict(phase=motion_phase, delta_counts=delta,
                                            drive_actual_seconds=drive_actual_seconds,
                                            distance_target_reached=distance_reached if self.long_travel else None,
                                            settled_wheel_travel_mm=settled_travel,
                                            peak_mm_s=self.peaks.copy(), settled_wheel=list(self.wheel),
                                            settled_status=list(self.latest)))
        print(f"{motion_phase} complete: delta={delta}; STOP confirmed, four cps=0", flush=True)


def self_test():
    regression.self_test()
    import io
    summary = dict(status_frames=0, wheel_frames=0, anomalies=[])
    probe = GroundProbe(None, io.StringIO(), summary)
    probe.phase = "stationary"
    def feed(t, count):
        frame = protocol.encode_frame(protocol.MSG_WHEEL_STATE, t,
                                      struct.pack("<Iqqqqiiii", t, count, 0, 0, 0, 0, 0, 0, 0))
        probe.emit("rx_bytes", data_hex=frame.hex())
    feed(100, -793)
    feed(150, -789)
    feed(200, -793)
    assert probe.change_events == 2 and probe.absolute_changes == [8,0,0,0]
    assert probe.stationary_frames == 3 and probe.stationary_previous[1] == -793
    print("PASS: count-change detector preserves transient return-to-baseline; no COM port opened")
    initial = (0, 100, -200, 300, -400, 0, 0, 0, 0)
    for sign in (-1, 1):
        current = (50, *(c+sign*3820 for c in initial[1:5]), 0, 0, 0, 0)
        wheels = travel_mm(initial, current, sign)
        assert 499.9 < sum(wheels)/4 < 500.1
        validate_long_travel(wheels)
    for invalid in ([-21,0,0,0], [651]*4, [0,0,0,151]):
        try:
            validate_long_travel(invalid)
        except regression.TestFailure:
            pass
        else:
            raise AssertionError("unsafe long travel accepted")
    print("PASS: long-travel forward/reverse geometry and guards; no COM port opened")
    # Exercise the actual drive loop without a port, clock delay or real motors.
    from unittest.mock import patch
    from contextlib import redirect_stdout
    for sign, advance in ((1, 40), (-1, 40), (1, 0)):
        result = dict(status_frames=0, wheel_frames=0, anomalies=[], motions=[])
        trial = GroundProbe(None, io.StringIO(), result, long_travel=True)
        trial.latest = (0,0,0,0,0,1,0,2,0,0x0150,0,0,0,0,0,0,0,0,0,0)
        trial.wheel = initial
        clock = [0.0]
        sent = []
        def fake_pump():
            clock[0] += 0.1
            trial.wheel = (int(clock[0]*1000), *(c+sign*advance for c in trial.wheel[1:5]), 0,0,0,0)
            status = list(trial.latest)
            status[5] = protocol.STATE_ACTIVE
            trial.latest = tuple(status)
        def fake_send(kind, payload=b""):
            sent.append((kind, payload))
            return len(sent)
        def fake_wait(seconds):
            status = list(trial.latest)
            status[5], status[7], status[10], status[11] = 1,2,0,0
            trial.latest = tuple(status)
        trial.pump, trial.send, trial.wait = fake_pump, fake_send, fake_wait
        with patch.object(time, "monotonic", side_effect=lambda: clock[0]), redirect_stdout(io.StringIO()):
            if advance:
                trial.drive(sign)
                assert result["motions"][0]["distance_target_reached"]
                assert LONG_DISTANCE_MM <= sum(result["motions"][0]["settled_wheel_travel_mm"])/4 < 510
                assert sent[-1][0] == protocol.CMD_STOP
                assert all(struct.unpack("<ii", payload) == (sign*40,0)
                           for kind,payload in sent if kind == protocol.CMD_VELOCITY)
            else:
                try:
                    trial.drive(sign)
                except regression.TestFailure as error:
                    assert "16s limit" in str(error)
                else:
                    raise AssertionError("timeout did not abort")
                assert sent[-1][0] == protocol.CMD_STOP
                assert not result["motions"]
        assert clock[0] <= LONG_MAX_SECONDS+0.11
    print("PASS: actual long-drive loop target/reverse/16s timeout and STOP; mocked clock/transport only")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port")
    ap.add_argument("--debug-port", help="optional CH340 port; must capture successfully before motion")
    ap.add_argument("--boot-log", type=Path)
    ap.add_argument("--ground-safe-confirmed", action="store_true")
    ap.add_argument("--long-travel-500mm", action="store_true",
                    help="explicit 500mm encoder target per direction, 40mm/s, max 16s; requires 80cm clear space")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        self_test()
        return 0
    if not args.port or not args.boot_log or not args.ground_safe_confirmed:
        ap.error("motion requires --port, --boot-log and --ground-safe-confirmed")
    if args.long_travel_500mm and not args.debug_port:
        ap.error("long travel requires synchronous --debug-port capture")
    boot = args.boot_log.read_bytes()
    text = boot.decode("utf-8-sig")
    if f"Firmware version = {regression.VERSION}" not in text or f"Build = {regression.BUILD}" not in text:
        ap.error("startup version/Build does not match validated firmware")
    import serial
    from serial.tools import list_ports
    info = next((p for p in list_ports.comports() if p.device.casefold() == args.port.casefold()), None)
    if info is None or (info.vid, info.pid) != (0x0483, 0x5740):
        ap.error("port is not the enumerated STM32 USB CDC")
    if args.debug_port:
        debug_info = next((p for p in list_ports.comports() if p.device.casefold() == args.debug_port.casefold()), None)
        if debug_info is None or (debug_info.vid, debug_info.pid) != (0x1A86, 0x7523):
            ap.error("debug port is not the enumerated CH340; no motion permitted")
    stamp = datetime.now(timezone(timedelta(hours=8))).strftime("%Y%m%d_%H%M%S_%f")
    out = Path(__file__).resolve().parents[1] / "build/hardware_tests" / ("ground_stop_encoder_"+stamp)
    out.mkdir(parents=True, exist_ok=False)
    (out / "boot_log.txt").write_bytes(boot)
    summary = dict(firmware=regression.VERSION, build=regression.BUILD, usb_serial=info.serial_number,
                   port=args.port, timezone="Asia/Shanghai", firmware_modified=False,
                   script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                   boot_log_sha256=hashlib.sha256(boot).hexdigest(), ground_test=True,
                   speed_mm_s=LONG_SPEED_MM_S if args.long_travel_500mm else SPEED,
                   drive_seconds=None if args.long_travel_500mm else DRIVE_SECONDS,
                   encoder_target_mm=LONG_DISTANCE_MM if args.long_travel_500mm else None,
                   drive_max_seconds=LONG_MAX_SECONDS if args.long_travel_500mm else DRIVE_SECONDS,
                   wheel_diameter_mm=65, counts_per_wheel_rev=1560,
                   distance_is_encoder_estimate_not_ground_measurement=True,
                   settle_seconds=SETTLE_SECONDS,
                   observation_seconds=OBSERVE_SECONDS, historical_80cm_test_reproduced=False,
                   status_frames=0, wheel_frames=0, anomalies=[], motions=[],
                   debug_log=(args.debug_port+" captured read-only" if args.debug_port else "recorded separately by user"),
                   helper_sha256=hashlib.sha256(Path(__file__).with_name("debug_uart_capture.py").read_bytes()).hexdigest(),
                   encoder_supply_measured=False)
    print(f"Ground test evidence: {out}", flush=True)
    result = 1
    probe = None
    debug_capture = DebugUartCapture(args.debug_port, out) if args.debug_port else None
    try:
        if debug_capture is not None:
            debug_capture.start()
            print(f"{args.debug_port} raw capture running; debug bytes received before any motion.", flush=True)
        with (out / "events.jsonl").open("w", encoding="utf-8") as events:
            with serial.Serial(args.port, 115200, timeout=0.005, write_timeout=0.1) as port:
                probe = GroundProbe(port, events, summary, debug_capture, args.long_travel_500mm)
                try:
                    probe.send(protocol.CMD_GET_STATUS)
                    start = time.monotonic()
                    while probe.latest is None or probe.wheel is None:
                        probe.receive()
                        if time.monotonic()-start > 2:
                            raise regression.TestFailure("initial STATUS/WHEEL missing")
                    if not regression.stopped(probe.latest) or any(probe.wheel[5:9]):
                        raise regression.TestFailure("initial state not stopped")
                    for countdown in range(5, 0, -1):
                        print(f"Ground motion in {countdown}... keep clear; Ctrl+C cancels", flush=True)
                        probe.wait(1)
                    probe.drive(1)
                    probe.drive(-1)
                    probe.phase = "stationary"
                    probe.stationary_baseline = probe.stationary_previous = probe.wheel
                    summary["stationary_first_wheel"] = list(probe.wheel)
                    probe.emit("stationary_start", wheel=list(probe.wheel), status=list(probe.latest))
                    print("Both motions stopped. Do not touch chassis/wheels for 120 seconds.", flush=True)
                    start = time.monotonic()
                    probe.wait(OBSERVE_SECONDS, stationary=True)
                    summary["observation_actual_s"] = time.monotonic()-start
                    if probe.stationary_frames < 2000:
                        raise regression.TestFailure("insufficient stationary telemetry")
                    summary["result"] = "no_post_stop_count_change_observed" if not probe.change_events and not any(probe.max_cps) else "post_stop_count_or_cps_change_observed"
                    result = 0 if summary["result"] == "no_post_stop_count_change_observed" else 3
                finally:
                    probe.phase = "cleanup"
                    try:
                        stop_seq = probe.send(protocol.CMD_STOP)
                        probe.wait(0.5, enforce=False)
                        summary["cleanup_stop_ack"] = stop_seq not in probe.pending
                        summary["cleanup_stopped"] = regression.stopped(probe.latest, explicit=True)
                    except Exception as error:
                        summary["cleanup_error"] = str(error)
                    if not summary.get("cleanup_stop_ack") or not summary.get("cleanup_stopped"):
                        raise regression.TestFailure("final STOP could not be verified; operator must remove motor power")
                    if summary["anomalies"]:
                        raise regression.TestFailure("unsafe STATUS/counters observed during capture or final cleanup")
    except (Exception, KeyboardInterrupt) as error:
        summary["result"] = "incomplete_or_aborted"
        summary["anomalies"].append(str(error) or "operator interrupted")
        print(f"ABORT: {error}; no more velocity commands. Keep clear; remove VM if stop unconfirmed.", flush=True)
        result = 1
    finally:
        if debug_capture is not None:
            debug_capture.close()
            summary["debug_bytes_received"] = debug_capture.bytes_received
            summary["debug_capture_error"] = debug_capture.error
            if debug_capture.error:
                summary["result"] = "incomplete_or_aborted"
                summary["anomalies"].append(debug_capture.error)
                result = 1
        if probe is not None:
            summary.update(stationary_frames=probe.stationary_frames,
                           count_change_events=probe.change_events,
                           absolute_count_changes=probe.absolute_changes, max_abs_cps=probe.max_cps,
                           interval_min_ms=min(probe.intervals) if probe.intervals else None,
                           interval_max_ms=max(probe.intervals) if probe.intervals else None,
                           stationary_last_wheel=list(probe.stationary_previous) if probe.stationary_previous else None,
                           stationary_net_counts=[probe.stationary_previous[i+1]-probe.stationary_baseline[i+1] for i in range(4)] if probe.stationary_previous else None,
                           parser_crc_errors=probe.parser.crc_errors,
                           observed_flags=[f"0x{x:04X}" for x in sorted(probe.flags)],
                           final_status=list(probe.latest) if probe.latest else None)
        (out / "summary.json").write_text(json.dumps(summary, indent=2)+"\n", encoding="utf-8")
        print(json.dumps(summary, indent=2), flush=True)
    return result


if __name__ == "__main__":
    sys.exit(main())
