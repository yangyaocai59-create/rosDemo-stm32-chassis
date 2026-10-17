# USB CDC soak and recovery validation

Stage 9C verifies that host application sessions can repeatedly open and close
without stale telemetry, new TX drops, protocol errors, or an MCU reset. The
test never sends a velocity command.

The firmware monitors the CDC DTR control line. Deasserting DTR ends the host
telemetry session and discards queued, unsent frames. The existing USB SOF
watchdog remains responsible for detecting physical cable or host loss.

## Quick open/close regression

```text
python Tools/usb_cdc_soak_test.py --port COM4 --cycles 5 --open-seconds 2 --closed-seconds 1
```

Each cycle checks:

- a request/ACK boundary discards data from an earlier host process;
- `WHEEL_STATE` remains near 20 Hz with valid MCU timestamps;
- STATUS reports host-port-open/DTR;
- CRC, RX overflow, and TX drop counters do not increase;
- TX drops do not increase while the COM port is closed;
- MCU uptime remains continuous across reopen cycles.

## Longer test

```text
python Tools/usb_cdc_soak_test.py --port COM4 --cycles 20 --open-seconds 30 --closed-seconds 2
```

Keep the CH340 debug terminal open separately. There must be one boot banner at
the beginning and no additional boot banner or reset-cause report during the
test. Every cycle must pass before physical cable reconnection is tested.

## Active DTR-close safety test

After the soak test passes, place the chassis on clear floor and run:

```text
python Tools/usb_cdc_test.py --port COM4 --dtr-stop-test
```

The chassis is commanded forward at 40 mm/s for about 0.8 seconds. The utility
then deasserts DTR, reconnects the session, and requires the firmware to report
`IDLE / TRANSPORT_LOST`, no runtime or controller fault, and no additional TX
drops. Keep the CH340 debug terminal connected; it should report that the host
port closed and that the active runtime stopped because the transport was lost.
