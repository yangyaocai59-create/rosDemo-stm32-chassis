"""Focused powered motion capture; firmware is never changed."""
import argparse
import datetime
import json
import pathlib
import struct
import sys
import time

import serial

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "Tools"))
import usb_cdc_test as wire


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--mode', choices=('pivot', 'straight'), default='pivot')
    cli.add_argument('--angular-mrad-s', type=int, choices=(200, 350), default=200)
    cli.add_argument('--physical-setup', choices=('ground', 'lifted'), default='ground',
                     help='Record user-confirmed setup only; does not change motor control')
    cli.add_argument('--left-repeat', action='store_true',
                     help='Focused pivot test: two LEFT runs, 6 s each, STOP and trace between runs')
    cli.add_argument('--left-only', action='store_true',
                     help='One LEFT run only, 6 s, followed by STOP and trace')
    cli.add_argument('--right-only', action='store_true',
                     help='One RIGHT run only, 6 s, followed by STOP and trace')
    args = cli.parse_args()
    if (args.left_repeat or args.left_only or args.right_only) and args.mode != 'pivot':
        cli.error('Focused left/right selection requires --mode pivot')
    if sum((args.left_repeat, args.left_only, args.right_only)) > 1:
        cli.error('--left-repeat, --left-only and --right-only are mutually exclusive')
    runs = [('LEFT', 0, args.angular_mrad_s), ('RIGHT', 0, -args.angular_mrad_s)] if args.mode == 'pivot' else [('FORWARD', 37, 0), ('REVERSE', -37, 0)]
    if args.left_repeat:
        runs = [('LEFT1', 0, args.angular_mrad_s),
                ('LEFT2', 0, args.angular_mrad_s)]
    elif args.left_only:
        runs = [('LEFT', 0, args.angular_mrad_s)]
    elif args.right_only:
        runs = [('RIGHT', 0, -args.angular_mrad_s)]
    stamp = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=9))).strftime("%Y%m%d_%H%M%S")
    root = pathlib.Path(__file__).parent / ('loaded_' + args.mode + '_compare_' + stamp)
    root.mkdir()
    report = {"capture_result": "INCOMPLETE", "motion_kind": args.mode, "duration_s_each": 6, "angular_command_mrad_s": args.angular_mrad_s if args.mode == 'pivot' else 0, "linear_command_mm_s": 37 if args.mode == 'straight' else 0, "directions": [], "errors": []}
    report['focused_left_repeat'] = args.left_repeat
    report['focused_left_only'] = args.left_only
    report['focused_right_only'] = args.right_only
    report['physical_setup'] = args.physical_setup
    parser = wire.StreamParser()
    usb = serial.Serial(port=None, baudrate=115200, timeout=0.005, write_timeout=0.2)
    usb.dtr, usb.rts, usb.port = True, False, "COM4"
    uart = serial.Serial(port=None, baudrate=115200, timeout=0.005, write_timeout=0.2)
    uart.dtr, uart.rts, uart.port = False, False, "COM8"
    sequence = 0
    phase = "initial"
    phase_started = time.monotonic()
    statuses, wheels, batteries, imu = [], [], [], []
    latest_status, latest_status_at = None, 0
    text_buffer = ""
    trace_ends = 0
    uart_lines = []
    next_poll = time.monotonic()

    with (root / "ch340.log").open("w", encoding="utf-8") as uart_log, (root / "usb_frames.jsonl").open("w", encoding="utf-8") as usb_log:
        def send(kind, payload=b""):
            nonlocal sequence
            sequence = (sequence + 1) & 65535
            usb.write(wire.encode_frame(kind, sequence, payload))

        def set_phase(label):
            nonlocal phase, phase_started
            phase = label
            phase_started = time.monotonic()
            uart_log.write("HOST_PHASE " + label + "\n")
            uart_log.flush()
            print("PHASE " + label, flush=True)

        def pump(seconds):
            nonlocal latest_status, latest_status_at, text_buffer, trace_ends, next_poll
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                if not phase.endswith("_motion") and time.monotonic() >= next_poll:
                    send(wire.CMD_GET_STATUS)
                    next_poll = time.monotonic() + 0.5
                data = usb.read(min(8192, max(1, usb.in_waiting)))
                for frame in parser.feed(data):
                    elapsed = time.monotonic() - phase_started
                    record = {"phase": phase, "elapsed_s": elapsed, "type": frame.message_type, "seq": frame.sequence, "payload_hex": frame.payload.hex()}
                    usb_log.write(json.dumps(record) + "\n")
                    state = wire.decode_status(frame)
                    if state is not None:
                        latest_status, latest_status_at = state, time.monotonic()
                        statuses.append({**record, "values": list(state)})
                    wheel = wire.decode_wheel_state(frame)
                    if wheel is not None:
                        wheels.append({**record, "values": list(wheel)})
                    if frame.message_type == wire.MSG_BATTERY_STATE and len(frame.payload) == 28:
                        values = struct.unpack("<IIIHHHHBBHHH", frame.payload)
                        batteries.append({**record, "values": list(values)})
                    values = wire.decode_imu_state(frame)
                    if values is not None:
                        imu.append({**record, "values": list(values)})
                raw = uart.read(min(8192, max(1, uart.in_waiting)))
                if raw:
                    text = raw.decode("utf-8", "replace")
                    uart_log.write(text)
                    uart_log.flush()
                    text_buffer += text
                    while "\n" in text_buffer:
                        line, text_buffer = text_buffer.split("\n", 1)
                        uart_lines.append((phase, line.rstrip()))
                        if "TRACE END;" in line:
                            trace_ends += 1
                        if "Firmware version" in line or "Reset cause" in line:
                            print(line.rstrip(), flush=True)
                usb_log.flush()

        def require_stopped():
            if latest_status is None or time.monotonic() - latest_status_at > 0.5:
                raise RuntimeError("Fresh STATUS missing")
            if latest_status[5] != wire.STATE_IDLE or latest_status[6] != 0 or latest_status[8] != 0 or any(latest_status[10:16]):
                raise RuntimeError("Chassis not safely stopped: " + str(latest_status))

        try:
            usb.open()
            uart.open()
            send(wire.CMD_STOP)
            send(wire.CMD_GET_STATUS)
            pump(4.0)
            require_stopped()
            if not batteries or not (8000 <= batteries[-1]["values"][6] <= 13000):
                raise RuntimeError("Motor supply telemetry outside expected test range")
            print("PRECHECK supply_mV=" + str(batteries[-1]["values"][6]), flush=True)
            for label, linear, angular in runs:
                require_stopped()
                set_phase(label + "_motion")
                started = time.monotonic()
                next_command = started
                sent = 0
                while time.monotonic() - started < 6.0:
                    if time.monotonic() >= next_command:
                        send(wire.CMD_VELOCITY, struct.pack("<ii", linear, angular))
                        sent += 1
                        next_command += 0.05
                    pump(0.01)
                    if time.monotonic() - latest_status_at > 0.5:
                        raise RuntimeError("STATUS stale during motion")
                    if latest_status[6] or latest_status[8] or latest_status[5] in (3, 4):
                        raise RuntimeError("Motion fault: " + str(latest_status))
                    if wheels and any(abs(cps) > 1000 for cps in wheels[-1]['values'][5:]):
                        raise RuntimeError('Host diagnostic wheel-speed guard exceeded 1000 cps')
                    if time.monotonic() - started > 0.6 and latest_status[5] != wire.STATE_ACTIVE:
                        raise RuntimeError("Unexpected exit from ACTIVE: " + str(latest_status))
                send(wire.CMD_STOP)
                set_phase(label + "_stopping")
                pump(1.3)
                require_stopped()
                run_status = [r["values"] for r in statuses if r["phase"] == label + "_motion"]
                run_battery = [r["values"][6] for r in batteries if r["phase"] == label + "_motion"]
                info = {"direction": label, "requested_v_mm_s": linear, "requested_w_mrad_s": angular, "commands_sent": sent, "status_samples": len(run_status), "supply_mV_min_max": [min(run_battery), max(run_battery)] if run_battery else [], "stopped_status": list(latest_status)}
                report["directions"].append(info)
                print(json.dumps(info), flush=True)
                set_phase(label + "_trace")
                previous_end = trace_ends
                uart.write(b"t")
                dump_started = time.monotonic()
                while trace_ends == previous_end and time.monotonic() - dump_started < 45:
                    pump(0.05)
                if trace_ends == previous_end:
                    raise RuntimeError(label + " TRACE END missing; do not overwrite ring")
                print(label + " TRACE_COMPLETE", flush=True)
                pump(0.3)
            report["capture_result"] = "COMPLETE_NOT_A_TRACKING_PASS"
        except Exception as error:
            report["errors"].append(str(error))
            print("ERROR " + str(error), flush=True)
        finally:
            try:
                send(wire.CMD_STOP)
                set_phase("final_stop")
                pump(0.7)
                require_stopped()
                report["final_status"] = list(latest_status)
            except Exception as error:
                report["errors"].append("cleanup: " + str(error))
            usb.close()
            uart.close()
            report["host_crc_errors"] = parser.crc_errors
            (root / "decoded.json").write_text(json.dumps({"status": statuses, "wheel": wheels, "battery": batteries, "imu": imu}, indent=2), encoding="utf-8")
            (root / "summary.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
            print("EVIDENCE " + str(root), flush=True)
            print(json.dumps(report), flush=True)
    return int(bool(report["errors"]))


if __name__ == "__main__":
    raise SystemExit(main())
