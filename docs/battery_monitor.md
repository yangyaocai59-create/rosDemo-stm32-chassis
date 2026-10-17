# Stage 11A battery monitor

## Hardware and acquisition

- CubeMX source of truth: PC1 = ADC1_IN11.
- ADC mode: 12-bit, software-triggered, 144-cycle sample, polling, no DMA.
- D24A nominal divider: 10 kOhm high side / 1 kOhm low side, or 1/11.
- Application sampling: 16 conversions every 100 ms.
- Filtering: first-order IIR with coefficient 1/8.
- Nominal ADC reference: 3300 mV.
- Calibration multiplier: 1000/1000 until checked against a trusted voltage.

`batteryTask` owns ADC1 polling. It remains responsible for its Fault Manager
heartbeat and never initializes USB or controls motors.

## Safety boundary

Voltage acquisition and telemetry are enabled, but undervoltage motion inhibit
is deliberately disabled. A nominal 12 V motor supply does not identify the
battery chemistry, cell count, BMS cutoff, or safe loaded voltage. Enabling a
generic threshold before those facts are known could either stop a healthy
robot or over-discharge a real battery.

The monitor therefore reports `MONITOR_ONLY`. `battery_config.h` contains an
explicit compile-time protection switch and rejects inconsistent thresholds if
that switch is later enabled.

## Telemetry

USB CDC protocol v1 adds message `0x83 BATTERY_STATE` without changing the
existing 52-byte STATUS or WHEEL_STATE layouts. It is published at 2 Hz during
an active host session. `Tools/usb_cdc_test.py` decodes it as, for example:

```text
BATTERY t=5100ms supply=11968mV adc=1088mV raw=1354/1350 state=MONITOR_ONLY flags=0x01 samples=52 adc_errors=0 consecutive_errors=0 thresholds=0/0mV
```

## Hardware validation

1. Flash the Debug firmware with motors stopped.
2. On CH340/USART1, verify `Battery monitor ready` and no `Battery ADC fault`.
3. After five seconds, record the `BAT:` line.
4. Run `python Tools/usb_cdc_test.py --port <CDC_COM>` and verify matching
   `BATTERY` frames, increasing sample count, stable voltage, and zero errors.
5. Do not enable undervoltage protection until battery chemistry and a trusted
   real-voltage reference are supplied.
