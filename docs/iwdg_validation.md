# Stage 10C IWDG validation

## Configuration

- LSI nominal frequency: 32 kHz
- Prescaler: 64
- Reload: 1999
- Nominal timeout: 4 seconds
- Refresh owner: safetyTask only
- Refresh period while healthy: 250 ms
- Eligibility: system init complete and all six persistent task heartbeats
  seen and current

The firmware does not refresh IWDG during startup until eligibility becomes
true. A healthy boot therefore contains both:

```text
IWDG arbiter ready: CubeMX timeout nominal=4s, refresh=250ms, owner=safetyTask
IWDG refresh armed: system init complete, task_mask=0x3F
```

USB STATUS flags bit 8 (`0x0100`) is set while refresh is eligible. With USB
configured and DTR open, a healthy idle status is normally `flags=0x0150`.
During a task-heartbeat fault bit 8 clears; during an operational wheel-stall
fault it remains set because the scheduler and supervised tasks are alive.

## Normal regression

After flashing, leave the board running for at least 15 seconds. There must be
only one boot banner and no periodic reset. Run the existing read-only USB
status check and Stage 10B health test; their short heartbeat injections must
recover before the IWDG timeout.

## Deliberate reset test

Keep the chassis stopped and use CH340/USART1. Send `zm` once and do not send
`z0`. The manager first latches the motorTask heartbeat fault and the watchdog
arbiter withholds refresh. After the hardware timeout the MCU must reboot and
the next boot banner must contain:

```text
Reset cause = ... (IWDG RESET)
```

After reboot, fault-injection RAM state is cleared. The arbiter must arm again
with `task_mask=0x3F`, and there must not be a second spontaneous reset.

Do not perform this deliberate reset test while the chassis is moving.
