# STM32 IMU stationary bias tracking 0.17.0

## Purpose

Firmware `0.17.0-imu-stationary-bias` adds a conservative, RAM-only zero-rate
tracker on top of the persisted milli-RAW bias from 0.16.0. It is intended to
follow a slow residual caused by warm-up or bias movement while the chassis is
known to be stationary.

This is not a replacement for temperature calibration. No temperature slope or
curve is enabled until it has been measured from the actual installed sensor.

## Interlocks

The tracker updates only when all of these are true:

- chassis runtime is IDLE with no valid command;
- motor outputs are zero and the motor standby state is disabled;
- all four encoder samples are valid, stopped, fresh, and have zero delta;
- no emergency stop, health fault, or motor diagnostic owns the path;
- the IMU is valid and has a restored/manual bias;
- acceleration magnitude is within 500 RAW of 8192 RAW;
- each gyro residual is within 24 RAW;
- the conditions remain true for at least 3 seconds;
- no explicit IMU calibration is active.

When any condition fails, the dwell timer is cleared and the tracker freezes.

## Update and persistence policy

Every 100 ms after the dwell period, the runtime bias moves only 1/512 of the
current residual. The update is applied to RAM through
`ImuService_SetGyroBiasMilliRaw()`. It never calls the Flash storage service and
there is no automatic Flash write. A future explicit calibration/save operation
still owns Flash persistence.

RAW v2 reports the actual runtime bias from the same coherent IMU snapshot, so
ROS can observe the slow change without applying a second correction.

The calibration temperature is now retained in the IMU service as diagnostic
state and is printed in the tracking log. It is not yet a temperature model.

## Verification

This change was verified with a real Debug clean build:

```text
Error: 0
Warning: 0
FLASH: 171936 B / 768 KiB
RAM:   80480 B / 128 KiB
```

The next hardware test is a read-only 10-minute static RAW v2 capture after
flashing and one approved calibration. Compare the initial and final
`bias_milli_raw`, current temperature, and integrated Z residual. Do not expect
this change alone to guarantee long-term yaw; temperature compensation and an
external ROS heading/pose constraint remain separate work.
