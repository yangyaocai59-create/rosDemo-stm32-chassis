# Host wheel odometry validation

This Stage 9B utility mirrors the calculations that the future ROS 2 chassis
driver will perform from USB `WHEEL_STATE` frames. It does not send velocity
commands and does not control the motors.

## Geometry and conventions

- Encoder resolution: 1560 counts per wheel revolution
- Nominal wheel diameter: 0.065 m
- Nominal left/right track width: 0.210 m
- Wheel order: LF, LR, RF, RR
- Positive wheel count: physical wheel-forward
- Positive X: chassis forward
- Positive yaw: counter-clockwise/left, following ROS REP-103

Four wheel positions are converted with:

```text
joint_angle_rad = encoder_count * 2*pi / 1560
```

The two wheels on each side are averaged before differential-drive odometry:

```text
left_distance  = mean(LF, LR) * metres_per_count
right_distance = mean(RF, RR) * metres_per_count
distance       = (left_distance + right_distance) / 2
yaw            = (right_distance - left_distance) / track_width
```

The first received count sample is the local odometry origin. This prevents a
host restart from creating a false position jump while leaving the STM32
`int64` counters continuous for diagnostics.

At startup the utility sends a request with a process-unique sequence and waits
for its matching ACK. Frames received before that ACK are discarded. This
prevents data buffered by Windows or a preceding CDC session from being joined
to the new odometry timeline.

## Commands

Local math test without hardware:

```text
python -B Tools/wheel_odometry_test.py --self-test
```

Read-only monitor without a direction assertion:

```text
python Tools/wheel_odometry_test.py --port COM4 --duration 10
```

Manual motion validation, with motor drive remaining disabled:

```text
python Tools/wheel_odometry_test.py --port COM4 --duration 10 --expect straight
python Tools/wheel_odometry_test.py --port COM4 --duration 10 --expect left
python Tools/wheel_odometry_test.py --port COM4 --duration 10 --expect right
```

For `straight`, push the chassis forward at least 3 cm. For `left` or `right`,
rotate the chassis by at least 15 degrees in the named direction during the
monitor window. The utility rejects new CRC errors, RX overflow, TX drops,
invalid telemetry intervals, or a motion sign inconsistent with the requested
test. Straight motion requires all four wheel counts to be positive. A left
pivot requires negative left-wheel counts and positive right-wheel counts; a
right pivot requires the opposite signs.
