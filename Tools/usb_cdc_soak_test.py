#!/usr/bin/env python3
"""Repeatedly open/close STM32 USB CDC and verify telemetry recovery."""

from __future__ import annotations

import argparse
import sys
import time

from usb_cdc_test import (
    CMD_GET_STATUS,
    StreamParser,
    decode_status,
    decode_wheel_state,
    encode_frame,
    synchronize_session,
)

HOST_PORT_OPEN_FLAG = 1 << 6


def main() -> int:
    argument_parser = argparse.ArgumentParser(description=__doc__)
    argument_parser.add_argument("--port", required=True, help="STM32 USB CDC port, for example COM4")
    argument_parser.add_argument("--cycles", type=int, default=5, help="number of open/close cycles")
    argument_parser.add_argument("--open-seconds", type=float, default=2.0, help="telemetry time per cycle")
    argument_parser.add_argument("--closed-seconds", type=float, default=1.0, help="delay with DTR deasserted")
    args = argument_parser.parse_args()

    if args.cycles < 2:
        argument_parser.error("--cycles must be at least 2")
    if args.open_seconds < 1.0:
        argument_parser.error("--open-seconds must be at least 1.0")
    if args.closed_seconds < 0.2:
        argument_parser.error("--closed-seconds must be at least 0.2")

    try:
        import serial
    except ImportError:
        print("pyserial is required: python -m pip install pyserial", file=sys.stderr)
        return 2

    previous_tx_drops: int | None = None
    previous_uptime: int | None = None
    sequence = int(time.monotonic_ns()) & 0xFFFF

    print(
        f"USB CDC soak: cycles={args.cycles} open={args.open_seconds:.1f}s "
        f"closed={args.closed_seconds:.1f}s; no velocity commands will be sent."
    )

    for cycle in range(1, args.cycles + 1):
        stream_parser = StreamParser()
        samples: list[tuple[int, ...]] = []
        statuses: list[tuple[int, ...]] = []

        try:
            with serial.Serial(args.port, 115200, timeout=0.02, write_timeout=0.2) as port:
                port.dtr = True
                if not synchronize_session(port, stream_parser, sequence):
                    print(f"FAIL cycle {cycle}: session ACK timeout", file=sys.stderr)
                    return 3
                sequence += 1
                start = time.monotonic()
                next_keepalive = start + 0.8

                while time.monotonic() - start < args.open_seconds:
                    now = time.monotonic()
                    if now >= next_keepalive:
                        port.write(encode_frame(CMD_GET_STATUS, sequence))
                        sequence += 1
                        next_keepalive += 0.8

                    data = port.read(port.in_waiting or 1)
                    if not data:
                        continue
                    for frame in stream_parser.feed(data):
                        if (status := decode_status(frame)) is not None:
                            statuses.append(status)
                        elif (wheel := decode_wheel_state(frame)) is not None:
                            samples.append(wheel)

                port.dtr = False
                time.sleep(0.10)
        except (serial.SerialException, OSError) as error:
            print(f"FAIL cycle {cycle}: serial error: {error}", file=sys.stderr)
            return 4

        if len(samples) < 2 or not statuses:
            print(
                f"FAIL cycle {cycle}: samples={len(samples)} statuses={len(statuses)}",
                file=sys.stderr,
            )
            return 5

        timestamps = [sample[0] for sample in samples]
        intervals = [
            (current - previous) & 0xFFFFFFFF
            for previous, current in zip(timestamps, timestamps[1:])
        ]
        invalid_intervals = [value for value in intervals if value == 0 or value > 200]
        interval_sum = sum(intervals)
        rate_hz = 1000.0 * len(intervals) / interval_sum if interval_sum else 0.0
        first_status = statuses[0]
        last_status = statuses[-1]
        protocol_crc_delta = last_status[17] - first_status[17]
        overflow_delta = last_status[18] - first_status[18]
        tx_drop_delta = last_status[19] - first_status[19]

        if previous_tx_drops is not None and first_status[19] != previous_tx_drops:
            print(
                f"FAIL cycle {cycle}: TX drops changed while port was closed "
                f"({previous_tx_drops} -> {first_status[19]})",
                file=sys.stderr,
            )
            return 6
        if previous_uptime is not None and first_status[0] < previous_uptime:
            print(
                f"FAIL cycle {cycle}: MCU uptime moved backwards "
                f"({previous_uptime} -> {first_status[0]}), possible reset",
                file=sys.stderr,
            )
            return 7
        if (first_status[9] & HOST_PORT_OPEN_FLAG) == 0:
            print(f"FAIL cycle {cycle}: STATUS does not report DTR open", file=sys.stderr)
            return 8
        if invalid_intervals:
            print(
                f"FAIL cycle {cycle}: invalid telemetry intervals {invalid_intervals[:8]}",
                file=sys.stderr,
            )
            return 9
        if stream_parser.crc_errors or protocol_crc_delta or overflow_delta or tx_drop_delta:
            print(
                f"FAIL cycle {cycle}: parser_crc={stream_parser.crc_errors} "
                f"protocol_crc={protocol_crc_delta} overflow={overflow_delta} "
                f"tx_drop={tx_drop_delta}",
                file=sys.stderr,
            )
            return 10

        print(
            f"PASS cycle {cycle}/{args.cycles}: samples={len(samples)} rate={rate_hz:.1f}Hz "
            f"interval={min(intervals)}..{max(intervals)}ms uptime={last_status[0]}ms "
            f"tx_drop_total={last_status[19]}"
        )
        previous_tx_drops = last_status[19]
        previous_uptime = last_status[0]
        if cycle < args.cycles:
            time.sleep(args.closed_seconds)

    print("PASS: Stage 9C repeated CDC open/close and telemetry recovery complete")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
