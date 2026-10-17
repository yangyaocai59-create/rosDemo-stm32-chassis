#!/usr/bin/env python3
"""Direct STM32 low-speed pivot test and read-only CH340 motion trace dump."""
from __future__ import annotations

import argparse
import json
import struct
import threading
import time
from datetime import datetime, timezone, timedelta
from pathlib import Path

import serial
import usb_cdc_test as wire


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--usb-port", default="COM4")
    ap.add_argument("--debug-port", default="COM8")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--direction", choices=("both", "left", "right"), default="both")
    ap.add_argument("--mode", choices=("pivot", "straight"), default="pivot")
    ap.add_argument("--linear-mm-s", type=int, default=40)
    args = ap.parse_args()
    if not 3.0 <= args.seconds <= 20.0:
        ap.error("seconds must be 3..20")
    if not 1 <= abs(args.linear_mm_s) <= 100:
        ap.error("linear-mm-s magnitude must be 1..100")

    stamp = datetime.now(timezone(timedelta(hours=8))).strftime("%Y%m%d_%H%M%S_%f")
    folder = Path(__file__).resolve().parents[1] / "build" / "hardware_tests" / (
        "stm32_pivot_trace_" + stamp
    )
    folder.mkdir(parents=True, exist_ok=False)
    usb = serial.Serial(port=None, baudrate=115200, timeout=.01, write_timeout=.2,
                        dsrdtr=False, rtscts=False)
    usb.dtr = True
    usb.rts = False
    usb.port = args.usb_port
    debug = serial.Serial(port=None, baudrate=115200, timeout=.02, write_timeout=.2,
                          dsrdtr=False, rtscts=False)
    debug.dtr = False
    debug.rts = False
    debug.port = args.debug_port
    parser = wire.StreamParser()
    sequence = 0
    debug_stop = threading.Event()
    debug_thread = None
    debug_error = []

    def send(msg: int, payload: bytes = b"") -> None:
        nonlocal sequence
        sequence = (sequence + 1) & 0xFFFF
        usb.write(wire.encode_frame(msg, sequence, payload))

    def read_debug() -> None:
        try:
            with (folder / "ch340.log").open("w", encoding="utf-8", newline="") as out:
                while not debug_stop.is_set():
                    data = debug.read(512)
                    if data:
                        out.write(data.decode("utf-8", errors="replace"))
                        out.flush()
        except Exception as error:
            debug_error.append(str(error))

    summary = {"result": "INCOMPLETE", "directions": [], "errors": []}

    def drain(seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            data = usb.read(8192)
            if data:
                parser.feed(data)
            time.sleep(.002)

    def run_direction(label: str, vx_mm_s: int, wz_mrad_s: int) -> None:
        started = time.monotonic()
        end = started + args.seconds
        command_count = 0
        while time.monotonic() < end:
            send(wire.CMD_VELOCITY, struct.pack("<ii", vx_mm_s, wz_mrad_s))
            command_count += 1
            drain(.045)
        send(wire.CMD_STOP)
        drain(1.0)
        debug.write(b"t")
        drain_debug_until(5.0)
        summary["directions"].append({"direction": label, "vx_mm_s": vx_mm_s,
                                       "wz_mrad_s": wz_mrad_s,
                                       "duration_s": args.seconds,
                                       "velocity_commands": command_count})

    def drain_debug_until(seconds: float) -> None:
        time.sleep(seconds)

    try:
        usb.open()
        debug.open()
        debug_thread = threading.Thread(target=read_debug, daemon=True)
        debug_thread.start()
        send(wire.CMD_GET_STATUS)
        drain(.8)
        if args.mode == "pivot":
            if args.direction in ("both", "left"):
                run_direction("LEFT", 0, 200)
            if args.direction in ("both", "right"):
                run_direction("RIGHT", 0, -200)
        else:
            if args.direction in ("both", "left"):
                run_direction("FORWARD", abs(args.linear_mm_s), 0)
            if args.direction in ("both", "right"):
                run_direction("REVERSE", -abs(args.linear_mm_s), 0)
        summary["result"] = "PASS_DIRECT_STM32_MOTION_TRACE"
    except Exception as error:
        summary["errors"].append(str(error))
    finally:
        try:
            usb.write(wire.encode_frame(wire.CMD_STOP, (sequence + 1) & 0xFFFF))
        except Exception:
            pass
        try:
            usb.close()
        except Exception:
            pass
        debug_stop.set()
        if debug_thread is not None:
            debug_thread.join(timeout=2.0)
        try:
            debug.close()
        except Exception:
            pass
        if debug_error:
            summary["errors"].extend("CH340: " + e for e in debug_error)
        (folder / "summary.json").write_text(json.dumps(summary, indent=2) + "\n",
                                              encoding="utf-8")
        print(json.dumps({**summary, "evidence": str(folder)}, indent=2), flush=True)
    return 0 if not summary["errors"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
