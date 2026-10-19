# STM32 wheel-speed control update 0.18.x

Date: 2026-10-17

## Why this update exists

Direct STM32 USB-CDC tests reproduced the low-speed chassis behavior without
ROS in the loop. USB acceptance was clean (`rejected=0`, `crc=0`). The loaded
pivot trace showed that one common `+/-80 permille` pivot boost made some
corners overspeed while another corner accumulated PI error. At 10 ms, the
encoder delta also moved in 100-count/s steps, which is visible as low-speed
PWM ripple.

## Firmware changes

- Wheel-speed acquisition/control period: 10 ms -> 20 ms. The 50 ms IIR time
  constant and 200 ms stop timeout are unchanged.
- PI integral gain: 0.500 -> 0.250 permille/(count/s)/s.
- Integral limit: +/-80 -> +/-50 permille.
- Output slew limit: 1500 -> 1000 permille/s for a softer start.
- Replaced the single pivot feedforward boost with eight explicit
  wheel/turn-direction slots. The 0.18.2 first trim is deliberately small:
  a dynamic +60/-60 breakaway assist is applied only while a pivoting wheel is
  stationary. Version 0.18.5 adds 300 ms minimum hold time and a 50 cps
  release threshold, with re-arm below 20 cps. Once released, PI and normal
  feedforward take over; there is no sustained per-wheel pivot boost.
  calibration.
- USB protocol, encoder timers/polarity, IMU, ESTOP, standby and PWM safety
  behavior are unchanged.

## Required hardware verification after flashing

With the wheels clear or with a safe open floor, repeat the direct STM32
trace test:

```powershell
python Tools\stm32_pivot_trace_test.py --usb-port COM4 --debug-port COM8 --seconds 5 --mode straight --direction both --linear-mm-s 40
python Tools\stm32_pivot_trace_test.py --usb-port COM4 --debug-port COM8 --seconds 10 --mode pivot --direction left
python Tools\stm32_pivot_trace_test.py --usb-port COM4 --debug-port COM8 --seconds 10 --mode pivot --direction right
```

The trace helper is diagnostic-only: it always sends an explicit stop during
normal completion and again during cleanup, and it does not flash firmware.

Check that target and measured CPS are close, the four PWM outputs no longer
show sustained integral saturation, and the chassis remains responsive to an
explicit stop. Do not use this version as final calibration until those
loaded-ground traces are reviewed.
