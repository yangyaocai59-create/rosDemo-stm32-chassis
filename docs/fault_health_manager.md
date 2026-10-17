# Fault and health manager

Firmware `0.10.2-stage10c` contains the production health-supervision layer,
its Debug-only software fault-injection regression path, and the independently
gated IWDG refresh arbiter.

## Task heartbeats

The manager records one heartbeat from each persistent application task:

| Bit | Task | Timeout | Motion-critical now |
|---:|---|---:|---|
| 0 | motorTask | 250 ms | yes |
| 1 | safetyTask | 100 ms | self-observation only |
| 2 | usbRxTask | 250 ms | yes |
| 3 | telemetryTask | 1000 ms | no |
| 4 | imuTask | 2500 ms | no, IMU business logic is not implemented |
| 5 | batteryTask | 2500 ms | no, battery business logic is not implemented |

There is a 3000 ms boot grace period. `systemInitTask` is handled separately as
a required one-shot completion event because it exits after USB initialization.
After the grace period a healthy boot prints:

```text
Task health armed: seen_mask=0x3F stale_mask=0x00 system_init=1
```

Only motorTask and usbRxTask are motion-critical for immediate chassis FAULT.
The watchdog arbiter is stricter: all six persistent tasks must be seen and
current. Because only safetyTask executes the refresh call, loss of safetyTask
execution inherently stops IWDG refresh even though it cannot independently
diagnose its own heartbeat as stale.

## Production wheel-stall detection

Stall detection runs only while ChassisRuntime is ACTIVE. Debug UART motor
diagnostics and an IDLE chassis cannot generate a wheel-stall fault.

For each wheel, monitoring is enabled when:

- the requested magnitude is at least 200 encoder counts/s;
- the production runtime has been ACTIVE for at least 1000 ms;
- controller output magnitude is at least 100/1000;
- speed in the commanded direction remains below 15% of the target.

The condition must remain continuous for 750 ms. A direction change resets the
per-wheel timer. Once detected, the manager latches the LF/LR/RF/RR wheel mask,
ChassisRuntime enters FAULT, and the existing controller shutdown disables the
D24A motor outputs.

The boot-tested 40 mm/s straight command is approximately 306 counts/s, so it
is supervised. Very small wheel commands below 200 counts/s are intentionally
excluded until loaded low-speed calibration is available.

## Fault clearing

Health faults are software-latched. `CMD_CLEAR_FAULT` succeeds only after the
live cause has disappeared, system initialization is complete, critical task
heartbeats are current, and the chassis is stopped. E-stop remains reset-only.

The USB STATUS payload remains 52 bytes. Flags bit 7 is now
`HEALTH_FAULT_LATCHED`; runtime fault value 5 means `HEALTH_MANAGER`.

## Debug-only fault injection

`FAULT_INJECTION_TEST_ENABLED=1` is defined only for the CMake Debug
configuration. Release builds retain inert stubs: they cannot suppress a task
heartbeat or alter health-manager wheel feedback.

Commands are entered on the CH340/USART1 port as two characters:

| Command | Injection |
|---|---|
| `zm` | suppress `motorTask` heartbeat |
| `zu` | suppress `usbRxTask` heartbeat |
| `z1` | present LF speed as zero to Fault Manager |
| `z2` | present LR speed as zero to Fault Manager |
| `z3` | present RF speed as zero to Fault Manager |
| `z4` | present RR speed as zero to Fault Manager |
| `z0` | disable all injection sources |
| `z?` | report current injection selection |

Only one injection source can be selected at a time. Enabling a source requires
the production runtime to be IDLE and diagnostics to be inactive. Disabling an
injection does not clear an already latched fault; the host must subsequently
send `CLEAR_FAULT`, which exercises the normal recovery path.

The automated regression opens both ports, checks both critical task timeouts,
then performs an LF no-feedback run:

```powershell
python Tools\usb_cdc_test.py --port <USB_CDC_COM> --debug-port <CH340_COM> --health-test
```

Close the USART1 serial terminal before running it. The final case drives at
40 mm/s, so the chassis requires at least 0.5 m of clear floor.

## IWDG arbitration

CubeMX enables IWDG from the 32 kHz nominal LSI with prescaler 64 and reload
1999, giving a nominal 4 second timeout. LSI tolerance means the physical
timeout is not exact.

`WatchdogService_Process()` is called only by safetyTask. It refreshes at most
once per 250 ms and only when system initialization is complete and every
persistent task heartbeat is both seen and current. It never refreshes during
the startup eligibility gap or after a task becomes stale.

Operational faults such as wheel stall, invalid commands, and E-stop keep the
motors safely disabled but do not deliberately force a processor reset while
all tasks remain alive. This preserves latched diagnostics for the host.

In Debug builds, `__HAL_DBGMCU_FREEZE_IWDG()` freezes the counter only while a
connected debugger halts the CPU. It has no effect while firmware is running.
