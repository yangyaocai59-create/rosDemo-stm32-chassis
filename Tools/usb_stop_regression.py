#!/usr/bin/env python3
"""Bounded, lifted-wheel STOP regression without ROS (Windows/Linux).

No fault clear, firmware flash, reset or automatic reconnect is performed.
Motion requires explicit lifted-wheel consent and a matching board boot log.
"""
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

VERSION = "0.13.3-usb-tx-burst-fix"
BUILD = "Oct 12 2026 08:24:27"
REQUIRED_FLAGS = 0x0150
UNSAFE_FLAGS = 0x008E
SPEED_MM_S = 30
DRIVE_SECONDS = 0.35
STOP_SECONDS = 0.7
CYCLES = 10


class TestFailure(Exception):
    pass


def validate_status(status: tuple[int, ...]) -> None:
    if (status[5] not in (1, 2) or status[6] != 0 or status[8] != 0
            or status[9] & UNSAFE_FLAGS
            or (status[9] & REQUIRED_FLAGS) != REQUIRED_FLAGS):
        raise TestFailure(
            f"unsafe STATUS state={status[5]} fault={status[6]}/{status[8]} "
            f"flags=0x{status[9]:04X} missing=0x{REQUIRED_FLAGS & ~status[9]:04X}"
        )


def stopped(status: tuple[int, ...] | None, *, explicit: bool = False) -> bool:
    return (status is not None and status[5] == 1 and status[10:12] == (0, 0)
            and max(abs(v) for v in status[12:16]) <= 5
            and (not explicit or status[7] == 2))


class Probe:
    def __init__(self, port, events, summary):
        self.port = port
        self.events = events
        self.summary = summary
        self.parser = protocol.StreamParser()
        self.sequence = 1
        self.phase = "setup"
        self.latest = None
        self.latest_time = None
        self.pending = {}
        self.counters = None
        self.flags = set()
        self.peaks = [0, 0, 0, 0]

    def emit(self, kind, **data):
        self.events.write(json.dumps(dict(host_monotonic_s=time.monotonic(),
                                        phase=self.phase, kind=kind, **data)) + "\n")

    def send(self, kind, payload=b""):
        sequence = self.sequence
        frame = protocol.encode_frame(kind, sequence, payload)
        if self.port.write(frame) != len(frame):
            raise TestFailure("partial serial write")
        self.sequence = (sequence + 1) & 0xFFFF
        self.pending[sequence] = (kind, time.monotonic())
        self.emit("command", message_type=kind, sequence=sequence,
                  payload_hex=payload.hex(), frame_hex=frame.hex())
        return sequence

    def receive(self, enforce=True):
        data = self.port.read(max(1, min(self.port.in_waiting, 8192)))
        if not data:
            return
        self.emit("rx_bytes", data_hex=data.hex())
        frames = self.parser.feed(data)
        if enforce and self.parser.crc_errors:
            raise TestFailure("CRC/version error in received stream")
        for frame in frames:
            if frame.message_type == protocol.MSG_ACK and len(frame.payload) == 4:
                request, result, state, fault = frame.payload
                pending = self.pending.pop(frame.sequence, None)
                self.emit("ack", sequence=frame.sequence, request=request,
                          result=result, state=state, fault=fault)
                if enforce and pending is not None:
                    if request != pending[0] or result != 0 or fault != 0:
                        raise TestFailure("command ACK rejected or mismatched")
            status = protocol.decode_status(frame)
            if status is None:
                continue
            if self.latest is not None and enforce:
                delta = (status[0] - self.latest[0]) & 0xFFFFFFFF
                if delta > 5000:
                    raise TestFailure("MCU uptime reset/backwards or long gap")
            self.latest = status
            self.latest_time = time.monotonic()
            self.summary["status_frames"] += 1
            self.flags.add(status[9])
            self.peaks = [max(old, abs(v)) for old, v in zip(self.peaks, status[12:16])]
            detail = dict(mcu_time_ms=status[0], state=status[5], fault=status[6],
                          stop=status[7], controller_fault=status[8],
                          flags=f"0x{status[9]:04X}",
                          missing_ready_flags=f"0x{REQUIRED_FLAGS & ~status[9]:04X}",
                          requested_mm_s=status[10], requested_mrad_s=status[11],
                          wheels_mm_s=list(status[12:16]),
                          mcu_crc=status[17], rx_overflow=status[18], tx_drop=status[19])
            self.emit("status", **detail)
            try:
                validate_status(status)
                if self.counters is None:
                    self.counters = status[17:20]
                elif status[17:20] != self.counters:
                    raise TestFailure("MCU CRC/overflow/TX-drop counters increased")
            except TestFailure:
                self.summary["anomalies"].append(dict(phase=self.phase, **detail))
                if enforce:
                    raise

    def observe(self, seconds, *, drive=False, enforce=True):
        started = time.monotonic()
        next_velocity = started
        next_status = started
        active = False
        moving = False
        while time.monotonic() - started < seconds:
            # Consume unsafe replies before issuing another velocity command.
            self.receive(enforce)
            now = time.monotonic()
            if enforce:
                if any(now - sent > 0.3 for _, sent in self.pending.values()):
                    raise TestFailure("matching ACK timeout (>300ms)")
                if ((drive or now - started > 0.3)
                        and (self.latest_time is None or now - self.latest_time > 0.3)):
                    raise TestFailure("STATUS stale (>300ms)")
            if drive and now >= next_velocity:
                self.send(protocol.CMD_VELOCITY, struct.pack("<ii", SPEED_MM_S, 0))
                next_velocity = now + 0.05
            if now >= next_status:
                self.send(protocol.CMD_GET_STATUS)
                next_status = now + 0.1
            if self.latest is not None:
                active |= self.latest[5] == 2
                moving |= max(abs(v) for v in self.latest[12:16]) > 5
        return active, moving


def self_test():
    good = (1000, 0, 0, 0, 1, 1, 0, 2, 0, 0x0150, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0)
    validate_status(good)
    assert stopped(good, explicit=True)
    active = list(good)
    active[5], active[9], active[10], active[12] = 2, 0x0171, 30, 30
    validate_status(tuple(active))
    assert not stopped(tuple(active))
    for index, value in [(9, 0x0140), (9, 0x00D0), (9, 0x0152),
                         (5, 3), (6, 5), (8, 1)]:
        bad = list(good)
        bad[index] = value
        try:
            validate_status(tuple(bad))
        except TestFailure:
            pass
        else:
            raise AssertionError("unsafe status accepted")
    payload = struct.pack("<IIIIHBBBBHhhhhhhIIII", *good)
    frame = protocol.encode_frame(protocol.MSG_STATUS, 123, payload)
    parser = protocol.StreamParser()
    assert not parser.feed(frame[:5])
    assert protocol.decode_status(parser.feed(frame[5:])[0]) == good
    assert parser.crc_errors == 0
    print("PASS: offline STOP-gate/codec self-test; no serial port opened")


def main():
    args_parser = argparse.ArgumentParser(description=__doc__)
    args_parser.add_argument("--port", help="STM32 CDC port, NOT CH340")
    args_parser.add_argument("--boot-log", type=Path, help="board startup log confirming version/Build")
    args_parser.add_argument("--wheels-lifted", action="store_true",
                             help="confirm wheels lifted, chassis secured and motor power on")
    args_parser.add_argument("--self-test", action="store_true")
    args = args_parser.parse_args()
    if args.self_test:
        self_test()
        return 0
    if not args.port or not args.boot_log or not args.wheels_lifted:
        args_parser.error("motion requires --port, --boot-log and --wheels-lifted")
    boot_data = args.boot_log.read_bytes()
    boot_text = boot_data.decode("utf-8-sig")
    if f"Firmware version = {VERSION}" not in boot_text or f"Build = {BUILD}" not in boot_text:
        args_parser.error("boot log does not match the delivered firmware version/Build")
    import serial
    from serial.tools import list_ports
    devices = [p for p in list_ports.comports() if p.device.casefold() == args.port.casefold()]
    if len(devices) != 1 or (devices[0].vid, devices[0].pid) != (0x0483, 0x5740):
        args_parser.error("selected port is not the enumerated STM32 CDC 0483:5740")
    serial_number = devices[0].serial_number
    stamp = datetime.now(timezone(timedelta(hours=8))).strftime("%Y%m%d_%H%M%S_%f")
    output = Path(__file__).resolve().parents[1] / "build" / "hardware_tests" / ("usb_stop_" + stamp)
    output.mkdir(parents=True)
    (output / "boot_log.txt").write_bytes(boot_data)
    summary = dict(test="Windows/Linux equivalent-parameters probe; not original ROS script",
                   firmware_version_from_boot=VERSION, build_from_boot=BUILD,
                   boot_log_sha256=hashlib.sha256(boot_data).hexdigest(),
                   script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                   port=args.port, usb_serial=serial_number, timezone="Asia/Shanghai",
                   command_mm_s=SPEED_MM_S, velocity_period_s=0.05,
                   drive_seconds=DRIVE_SECONDS, stop_seconds=STOP_SECONDS,
                   max_cycles=CYCLES, cycles_completed=0, status_frames=0,
                   anomalies=[], cycle_results=[], firmware_modified=False,
                   ros_end_to_end_tested=False, result="not_started")
    probe = None
    port = None
    with (output / "events.jsonl").open("w", encoding="utf-8", buffering=1) as events:
        try:
            port = serial.Serial(args.port, 115200, timeout=0.005, write_timeout=0.2,
                                 dsrdtr=False, rtscts=False, exclusive=True)
            port.dtr = True
            port.reset_input_buffer()
            probe = Probe(port, events, summary)
            probe.phase = "stationary_baseline"
            probe.observe(1.0)
            if not stopped(probe.latest):
                raise TestFailure("baseline is not measured stopped IDLE")
            print("WARNING: 10 lifted-wheel pulses, 30 mm/s for 0.35 s each. Keep clear.", flush=True)
            probe.phase = "countdown"
            for remaining in (3, 2, 1):
                print(f"Starting in {remaining}...", flush=True)
                probe.observe(1.0)
                if not stopped(probe.latest):
                    raise TestFailure("chassis moved or left IDLE during countdown")
            for cycle in range(1, CYCLES + 1):
                probe.phase = f"cycle_{cycle}_drive"
                probe.peaks = [0, 0, 0, 0]
                active, moving = probe.observe(DRIVE_SECONDS, drive=True)
                if not active or not moving:
                    raise TestFailure("drive did not produce ACTIVE and measured wheel motion")
                peaks = list(probe.peaks)
                probe.phase = f"cycle_{cycle}_stop"
                stop_sequence = probe.send(protocol.CMD_STOP)
                probe.observe(STOP_SECONDS)
                if stop_sequence in probe.pending or not stopped(probe.latest, explicit=True):
                    raise TestFailure("STOP did not ACK and produce measured stopped IDLE/EXPLICIT")
                summary["cycles_completed"] = cycle
                summary["cycle_results"].append(dict(cycle=cycle, peaks_mm_s=peaks,
                                                      stop_flags=f"0x{probe.latest[9]:04X}"))
                print(f"PASS cycle {cycle}/10: stopped flags=0x{probe.latest[9]:04X} peaks={peaks}mm/s", flush=True)
            summary["result"] = "bounded_cycles_passed"
        except (Exception, KeyboardInterrupt) as error:
            summary["result"] = "stopped_on_failure"
            summary["error"] = str(error) or "interrupted"
            print("FAIL:", summary["error"], flush=True)
            if probe is not None:
                probe.emit("failure", error=summary["error"])
        finally:
            if probe is not None:
                probe.phase = "cleanup"
                try:
                    stop_sequence = probe.send(protocol.CMD_STOP)
                    probe.observe(0.5, enforce=False)
                    summary["cleanup_stop_ack"] = stop_sequence not in probe.pending
                    summary["cleanup_stopped"] = stopped(probe.latest, explicit=True)
                    if not summary["cleanup_stop_ack"] or not summary["cleanup_stopped"]:
                        summary["result"] = "stopped_on_failure"
                        summary["cleanup_error"] = "final STOP not acknowledged or no stopped STATUS"
                except Exception as error:
                    summary["cleanup_error"] = str(error)
                    summary["result"] = "stopped_on_failure"
                summary["observed_flags"] = [f"0x{v:04X}" for v in sorted(probe.flags)]
                summary["parser_crc_errors"] = probe.parser.crc_errors
                if summary["anomalies"] or probe.parser.crc_errors:
                    summary["result"] = "stopped_on_failure"
                if probe.latest is not None:
                    status = probe.latest
                    summary["final_status"] = dict(uptime_ms=status[0], state=status[5],
                        fault=status[6], controller_fault=status[8], stop=status[7],
                        flags=f"0x{status[9]:04X}", requested=list(status[10:12]),
                        wheels_mm_s=list(status[12:16]), crc=status[17],
                        rx_overflow=status[18], tx_drop=status[19])
            if port is not None:
                port.close()
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2), flush=True)
    print("Artifacts:", output, flush=True)
    return 0 if summary["result"] == "bounded_cycles_passed" else 1


if __name__ == "__main__":
    sys.exit(main())
