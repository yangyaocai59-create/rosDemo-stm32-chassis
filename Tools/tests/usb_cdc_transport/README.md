# USB/SOF snapshot and STOP regression (host-side)

These tests compile the production transport. The second test also includes
the production protocol parser, dispatch, STATUS and session code using real
application headers. Only hardware/RTOS and other application services are
mocked. They do not access a COM port or move a motor.

Run from the project root with native GCC/Clang (not ARM GCC):

```text
gcc -std=c11 -Wall -Wextra -Werror -ITools/tests/usb_cdc_transport/stubs -IApplication/Communication Tools/tests/usb_cdc_transport/test_usb_cdc_transport.c Application/Communication/usb_cdc_transport.c -o build/tests/usb_cdc_transport_test.exe
gcc -std=c11 -Wall -Wextra -Werror -DDEBUG -ITools/tests/usb_cdc_transport/stubs -IApplication/Communication -IApplication/Chassis -IApplication/Diagnostics -IApplication/Power -IApplication/Safety -IApplication/Sensors -IApplication/Services Tools/tests/usb_cdc_transport/test_usb_protocol_stop.c Application/Communication/usb_cdc_transport.c -o build/tests/usb_protocol_stop_test.exe
```

Create `build/tests` first. Execute both resulting programs; each must print
PASS and return 0. Linux can use the same commands and execute the `.exe`-named
native outputs with `./build/tests/...`; the suffix is not significant there.
Keep assertions enabled (do not add `-DNDEBUG`). Test stubs must never be added
to firmware include paths.

To reproduce the defect against the immutable incoming `rosDemo2` source
snapshot, add `-DTEST_REFERENCE_BASELINE` to the second command and replace its
`Application/Communication/usb_cdc_transport.c` input with
`usb_status_flags_review_20261011/sources/rosDemo2/Application/Communication/usb_cdc_transport.c`.
Use a different output filename. This negative-control build is expected to
exit nonzero on the first STOP with `actual STATUS flags=0x0140`; do not
mistake that intentional baseline failure for a failure of the fixed firmware.

Coverage:

- Link snapshot acquires HAL tick inside the critical section.
- Old caller tick 3000 versus SOF tick 3001: old expression underflows, fresh
  production snapshot stays alive; SOF immediately before entry and deferred
  until exit cannot tear the snapshot.
- Real 50/51 ms cutoff and uint32 HAL tick wrap.
- Configured/unconfigured, SOF unseen, monitor-disabled compatibility.
- DTR fallback, DTR close/reopen, queue flush and deconfiguration.
- 50 parsed velocity -> STOP -> GET_STATUS -> new velocity cycles. A mocked
  STOP wait advances the HAL tick and delivers SOF by 20 ms; STATUS still
  contains 0x0150 and the protocol session is not incorrectly torn down.
- Real SOF timeout/DTR close/deconfiguration invoke the runtime's transport
  stop hook; reconnection does not replay discarded commands.

Limitations: motor stop implementation, RTOS IRQ priorities/masking on the
actual MCU, physical USB behavior and ROS motion gating still require the
raised-wheel hardware comparison described in
`docs/STM32_USB_STATUS_FLAGS_FIX_DELIVERY.md`.

## USB TX burst fix (0.13.3)

Windows native GCC suite (no serial port, no ARM flashing):

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File Tools/tests/run_usb_tx_tests.ps1
```

This also builds/runs `test_usb_tx_queue.c`, `test_usb_tx_burst.c` and the real
logger test in `Tools/tests/debug_log/test_debug_log.c`.

New coverage: full battery/IMU replies, 2-frame RX budget, old pending telemetry
plus VELOCITY/STOP/GET_STATUS, 4 reserved FIFO slots, 3 staged retry records,
FIFO/device sequence order, USBD_BUSY retry, finite overflow accounting,
telemetry deferral without sequence allocation, and in-flight buffer ownership
across DTR/SOF timeout/deconfiguration. Logger tests check startup before the
scheduler, scoped task ownership/nesting, no UART/mutex calls while deferred,
origin timestamps, finite record queue, truncation and failure counters.

An intentionally excessive stalled-host burst is expected to create real
control drops; the test requires those drops to be counted and STOP executed.
The finite queue is not an unbounded delivery guarantee. Native tests use
mocked hardware/other services, not physical USB or motor output timing.
