# 0.17.0 stationary bias hardware verification

Date: 2026-10-17. Firmware: `0.17.0-imu-stationary-bias`, local commit
`91a0dbb`. Motor VM was off and the chassis was stationary. The test sent only
`SET_IMU_STREAM(mode=2)` and periodic `GET_STATUS`; it did not calibrate, drive,
stop, reset, or write Flash.

Evidence directory:

```text
build/hardware_tests/imu_stationary_bias_20261017_175750_186848/
```

## Result

- `PASS_STATIONARY_TRACKING_CAPTURE`
- duration: 600.03 s
- unique RAW v2 samples: 60,003
- source rate: 100.0037 Hz
- temperature: 28.955–29.117 °C
- source timestamp/count continuity: passed
- chassis status: IDLE, zero wheel speed, no fault
- integrated Z residual: **+0.0234°/minute**

The runtime bias changed gradually in RAM from:

```text
[-55099, 21755, 151645] milli-RAW
```

to:

```text
[-55421, 21706, 152355] milli-RAW
```

No Flash write occurred during the read-only capture. This confirms that the
guarded stationary tracker reduces the observed static residual from the prior
approximately 5.9°/minute result at this temperature. This is a static result;
ROS-side 10-minute verification and motion/temperature testing are still
required before calling the release a long-term solution.
