#!/usr/bin/env python3
"""Read STM32 wheel telemetry and validate host-side differential odometry."""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass

from usb_cdc_test import (
    CMD_GET_STATUS,
    StreamParser,
    decode_status,
    decode_wheel_state,
    encode_frame,
    synchronize_session,
)

COUNTS_PER_WHEEL_REV = 1560.0
WHEEL_DIAMETER_M = 0.065
TRACK_WIDTH_M = 0.210
METRES_PER_COUNT = math.pi * WHEEL_DIAMETER_M / COUNTS_PER_WHEEL_REV
RADIANS_PER_COUNT = 2.0 * math.pi / COUNTS_PER_WHEEL_REV

MIN_EXPECTED_LINEAR_M = 0.03
MAX_STRAIGHT_YAW_RAD = math.radians(15.0)
MIN_EXPECTED_TURN_RAD = math.radians(15.0)


@dataclass
class OdometryState:
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0
    linear_m_s: float = 0.0
    angular_rad_s: float = 0.0
    left_distance_m: float = 0.0
    right_distance_m: float = 0.0


class WheelOdometry:
    def __init__(self) -> None:
        self.state = OdometryState()
        self.origin_counts: tuple[int, int, int, int] | None = None
        self.previous_counts: tuple[int, int, int, int] | None = None
        self.latest_counts: tuple[int, int, int, int] | None = None
        self.latest_cps: tuple[int, int, int, int] = (0, 0, 0, 0)

    def update(self, wheel_sample: tuple[int, ...]) -> None:
        counts = tuple(wheel_sample[1:5])
        cps = tuple(wheel_sample[5:9])
        if self.previous_counts is None:
            self.origin_counts = counts
            self.previous_counts = counts
            self.latest_counts = counts
            self.latest_cps = cps
            return

        deltas = tuple(current - previous for current, previous in zip(counts, self.previous_counts))
        left_delta_m = 0.5 * (deltas[0] + deltas[1]) * METRES_PER_COUNT
        right_delta_m = 0.5 * (deltas[2] + deltas[3]) * METRES_PER_COUNT
        distance_delta_m = 0.5 * (left_delta_m + right_delta_m)
        yaw_delta_rad = (right_delta_m - left_delta_m) / TRACK_WIDTH_M
        old_yaw = self.state.yaw_rad

        if abs(yaw_delta_rad) < 1.0e-12:
            self.state.x_m += distance_delta_m * math.cos(old_yaw)
            self.state.y_m += distance_delta_m * math.sin(old_yaw)
        else:
            turn_radius_m = distance_delta_m / yaw_delta_rad
            new_yaw = old_yaw + yaw_delta_rad
            self.state.x_m += turn_radius_m * (math.sin(new_yaw) - math.sin(old_yaw))
            self.state.y_m -= turn_radius_m * (math.cos(new_yaw) - math.cos(old_yaw))

        self.state.yaw_rad += yaw_delta_rad
        self.state.left_distance_m += left_delta_m
        self.state.right_distance_m += right_delta_m

        left_cps = 0.5 * (cps[0] + cps[1])
        right_cps = 0.5 * (cps[2] + cps[3])
        left_m_s = left_cps * METRES_PER_COUNT
        right_m_s = right_cps * METRES_PER_COUNT
        self.state.linear_m_s = 0.5 * (left_m_s + right_m_s)
        self.state.angular_rad_s = (right_m_s - left_m_s) / TRACK_WIDTH_M
        self.previous_counts = counts
        self.latest_counts = counts
        self.latest_cps = cps

    def relative_counts(self) -> tuple[int, int, int, int]:
        if self.origin_counts is None or self.latest_counts is None:
            return (0, 0, 0, 0)
        return tuple(current - origin for current, origin in zip(self.latest_counts, self.origin_counts))

    def relative_joint_radians(self) -> tuple[float, float, float, float]:
        return tuple(value * RADIANS_PER_COUNT for value in self.relative_counts())


def validate_expected_motion(expectation: str, odometry: WheelOdometry) -> tuple[bool, str]:
    state = odometry.state
    counts = odometry.relative_counts()
    left_counts = 0.5 * (counts[0] + counts[1])
    right_counts = 0.5 * (counts[2] + counts[3])
    if expectation == "none":
        return True, "no motion-direction assertion requested"
    if expectation == "straight":
        if any(value <= 0 for value in counts):
            return False, f"straight-forward test requires all four counts positive, got {counts}"
        if state.x_m < MIN_EXPECTED_LINEAR_M:
            return False, f"forward displacement {state.x_m:.3f} m is below {MIN_EXPECTED_LINEAR_M:.3f} m"
        if abs(state.yaw_rad) > MAX_STRAIGHT_YAW_RAD:
            return False, f"absolute yaw {math.degrees(abs(state.yaw_rad)):.1f} deg exceeds 15 deg"
        return True, "positive straight displacement with limited yaw"
    if expectation == "left":
        if left_counts >= 0.0 or right_counts <= 0.0:
            return False, f"left pivot requires left counts negative and right counts positive, got {counts}"
        if state.yaw_rad < MIN_EXPECTED_TURN_RAD:
            return False, f"left yaw {math.degrees(state.yaw_rad):.1f} deg is below 15 deg"
        return True, "positive yaw confirms ROS counter-clockwise/left convention"
    if expectation == "right":
        if left_counts <= 0.0 or right_counts >= 0.0:
            return False, f"right pivot requires left counts positive and right counts negative, got {counts}"
        if state.yaw_rad > -MIN_EXPECTED_TURN_RAD:
            return False, f"right yaw {math.degrees(state.yaw_rad):.1f} deg is not below -15 deg"
        return True, "negative yaw confirms ROS clockwise/right convention"
    raise ValueError(f"unsupported expectation: {expectation}")


def run_self_test() -> None:
    circumference = math.pi * WHEEL_DIAMETER_M

    straight = WheelOdometry()
    straight.update((0, 0, 0, 0, 0, 0, 0, 0, 0))
    straight.update((1000, 1560, 1560, 1560, 1560, 0, 0, 0, 0))
    assert math.isclose(straight.state.x_m, circumference, rel_tol=1.0e-12)
    assert math.isclose(straight.state.y_m, 0.0, abs_tol=1.0e-12)
    assert math.isclose(straight.state.yaw_rad, 0.0, abs_tol=1.0e-12)

    left = WheelOdometry()
    left.update((0, 0, 0, 0, 0, 0, 0, 0, 0))
    left.update((1000, -780, -780, 780, 780, 0, 0, 0, 0))
    assert math.isclose(left.state.x_m, 0.0, abs_tol=1.0e-12)
    assert left.state.yaw_rad > 0.0
    assert validate_expected_motion("left", left)[0]

    right = WheelOdometry()
    right.update((0, 0, 0, 0, 0, 0, 0, 0, 0))
    right.update((1000, 780, 780, -780, -780, 0, 0, 0, 0))
    assert right.state.yaw_rad < 0.0
    assert validate_expected_motion("right", right)[0]
    print("Wheel odometry self-test passed")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="STM32 USB CDC port, for example COM4 or /dev/ttyACM0")
    parser.add_argument("--duration", type=float, default=10.0, help="monitor duration in seconds")
    parser.add_argument(
        "--expect",
        choices=("none", "straight", "left", "right"),
        default="none",
        help="optional manually-produced motion to validate",
    )
    parser.add_argument("--self-test", action="store_true", help="test odometry math without hardware")
    args = parser.parse_args()

    if args.self_test:
        run_self_test()
        return 0
    if not args.port:
        parser.error("--port is required unless --self-test is used")
    if args.duration <= 0.0:
        parser.error("--duration must be greater than zero")

    try:
        import serial
    except ImportError:
        print("pyserial is required: python -m pip install pyserial", file=sys.stderr)
        return 2

    stream_parser = StreamParser()
    odometry = WheelOdometry()
    sequence = int(time.monotonic_ns()) & 0xFFFF
    samples = 0
    timestamps: list[int] = []
    first_status: tuple[int, ...] | None = None
    last_status: tuple[int, ...] | None = None
    print(
        f"Read-only odometry monitor: {args.duration:.1f} s, expectation={args.expect}. "
        "No velocity command will be sent."
    )
    with serial.Serial(args.port, 115200, timeout=0.02, write_timeout=0.2) as port:
        if not synchronize_session(port, stream_parser, sequence):
            print("FAIL: timed out waiting for the current session ACK", file=sys.stderr)
            return 3
        print(f"Session synchronized with ACK sequence={sequence}; old buffered frames discarded.")
        sequence += 1
        start = time.monotonic()
        next_keepalive = start + 0.8
        next_report = start

        while time.monotonic() - start < args.duration:
            now = time.monotonic()
            if now >= next_keepalive:
                port.write(encode_frame(CMD_GET_STATUS, sequence))
                sequence += 1
                next_keepalive += 0.8

            data = port.read(port.in_waiting or 1)
            if not data:
                continue
            for frame in stream_parser.feed(data):
                status = decode_status(frame)
                if status is not None:
                    if first_status is None:
                        first_status = status
                    last_status = status
                    continue

                wheel_sample = decode_wheel_state(frame)
                if wheel_sample is None:
                    continue
                odometry.update(wheel_sample)
                timestamps.append(wheel_sample[0])
                samples += 1

                if now >= next_report:
                    joint = odometry.relative_joint_radians()
                    state = odometry.state
                    print(
                        f"ODOM t={wheel_sample[0]}ms x={state.x_m:+.3f}m y={state.y_m:+.3f}m "
                        f"yaw={math.degrees(state.yaw_rad):+.1f}deg "
                        f"vx={state.linear_m_s:+.3f}m/s wz={state.angular_rad_s:+.3f}rad/s "
                        f"joint=[{joint[0]:+.2f},{joint[1]:+.2f},{joint[2]:+.2f},{joint[3]:+.2f}]rad"
                    )
                    next_report += 0.25

    if samples < 2:
        print("FAIL: fewer than two WHEEL_STATE samples received", file=sys.stderr)
        return 4

    intervals = [
        (current - previous) & 0xFFFFFFFF
        for previous, current in zip(timestamps, timestamps[1:])
    ]
    invalid_intervals = [value for value in intervals if value == 0 or value > 200]
    interval_sum = sum(intervals)
    measured_hz = 1000.0 * len(intervals) / interval_sum if interval_sum else 0.0
    relative_counts = odometry.relative_counts()
    joint = odometry.relative_joint_radians()
    state = odometry.state

    print("--- summary ---")
    print(
        f"samples={samples} rate={measured_hz:.1f}Hz "
        f"interval={min(intervals)}..{max(intervals)}ms parser_crc_errors={stream_parser.crc_errors}"
    )
    print(f"delta_count LF/LR/RF/RR = {relative_counts}")
    print(f"joint_rad   LF/LR/RF/RR = ({joint[0]:+.3f}, {joint[1]:+.3f}, {joint[2]:+.3f}, {joint[3]:+.3f})")
    print(
        f"side_distance left={state.left_distance_m:+.3f}m right={state.right_distance_m:+.3f}m"
    )
    print(
        f"pose x={state.x_m:+.3f}m y={state.y_m:+.3f}m "
        f"yaw={state.yaw_rad:+.3f}rad ({math.degrees(state.yaw_rad):+.1f}deg)"
    )

    if first_status is None or last_status is None:
        print("FAIL: no STATUS frame received", file=sys.stderr)
        return 5
    crc_delta = last_status[17] - first_status[17]
    overflow_delta = last_status[18] - first_status[18]
    tx_drop_delta = last_status[19] - first_status[19]
    print(
        f"communication delta: protocol_crc={crc_delta} rx_overflow={overflow_delta} "
        f"tx_drop={tx_drop_delta}"
    )

    if invalid_intervals:
        print(f"FAIL: invalid telemetry intervals {invalid_intervals[:8]}", file=sys.stderr)
        return 6
    if stream_parser.crc_errors or crc_delta or overflow_delta or tx_drop_delta:
        print("FAIL: communication errors increased during this run", file=sys.stderr)
        return 7

    passed, reason = validate_expected_motion(args.expect, odometry)
    if not passed:
        print(f"FAIL: {reason}", file=sys.stderr)
        return 8
    print(f"PASS: Stage 9B odometry stream valid; {reason}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
