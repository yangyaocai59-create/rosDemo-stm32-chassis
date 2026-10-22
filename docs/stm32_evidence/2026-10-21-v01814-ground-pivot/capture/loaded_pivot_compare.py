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


def is_stopped_health_fault(state):
    return bool(state is not None and len(state) == 20 and
                state[5] == wire.STATE_FAULT and
                state[6] == wire.RUNTIME_FAULT_HEALTH_MANAGER and
                state[7] == 6 and state[8] == 0 and
                state[9] & wire.STATUS_FLAG_HEALTH_FAULT and
                not (state[9] & 0x0021) and not any(state[10:16]))


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
    cli.add_argument('--observe-recovery-fault', action='store_true',
                     help='Ground RIGHT 200 only: record a natural firmware health-fault stop before any host STOP; never clear/retry after motion')
    cli.add_argument('--clear-prior-health-fault-once', action='store_true',
                     help='Explicitly authorized single repeat only: clear a stopped prior health fault once, require ACK OK and IDLE; no retries')
    args = cli.parse_args()
    if (args.left_repeat or args.left_only or args.right_only) and args.mode != 'pivot':
        cli.error('Focused left/right selection requires --mode pivot')
    if sum((args.left_repeat, args.left_only, args.right_only)) > 1:
        cli.error('--left-repeat, --left-only and --right-only are mutually exclusive')
    if args.observe_recovery_fault and not (args.mode == 'pivot' and
            args.right_only and args.angular_mrad_s == 200 and
            args.physical_setup == 'ground'):
        cli.error('--observe-recovery-fault requires --mode pivot --right-only --angular-mrad-s 200 --physical-setup ground')
    if args.clear_prior_health_fault_once and not args.observe_recovery_fault:
        cli.error('--clear-prior-health-fault-once requires --observe-recovery-fault')
    runs = [('LEFT', 0, args.angular_mrad_s), ('RIGHT', 0, -args.angular_mrad_s)] if args.mode == 'pivot' else [('FORWARD', 37, 0), ('REVERSE', -37, 0)]
    if args.left_repeat:
        runs = [('LEFT1', 0, args.angular_mrad_s),
                ('LEFT2', 0, args.angular_mrad_s)]
    elif args.left_only:
        runs = [('LEFT', 0, args.angular_mrad_s)]
    elif args.right_only:
        runs = [('RIGHT', 0, -args.angular_mrad_s)]
    stamp = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime("%Y%m%d_%H%M%S")
    root = pathlib.Path(__file__).parent / ('loaded_' + args.mode + '_compare_' + stamp)
    root.mkdir()
    report = {"capture_result": "INCOMPLETE", "motion_kind": args.mode, "duration_s_each": 6, "angular_command_mrad_s": args.angular_mrad_s if args.mode == 'pivot' else 0, "linear_command_mm_s": 37 if args.mode == 'straight' else 0, "directions": [], "errors": []}
    report['focused_left_repeat'] = args.left_repeat
    report['focused_left_only'] = args.left_only
    report['focused_right_only'] = args.right_only
    report['physical_setup'] = args.physical_setup
    report['timezone'] = 'Asia/Singapore'
    report['observe_recovery_fault'] = args.observe_recovery_fault
    report['clear_prior_health_fault_once_authorized'] = args.clear_prior_health_fault_once
    report['tx_commands'] = []
    fault_observed = False
    parser = wire.StreamParser()
    usb = serial.Serial(port=None, baudrate=115200, timeout=0.005, write_timeout=0.2)
    usb.dtr, usb.rts, usb.port = True, False, "COM4"
    uart = serial.Serial(port=None, baudrate=115200, timeout=0.005, write_timeout=0.2)
    uart.dtr, uart.rts, uart.port = False, False, "COM8"
    sequence = 0
    phase = "initial"
    phase_started = time.monotonic()
    statuses, wheels, batteries, imu = [], [], [], []
    acknowledgements = []
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
            report['tx_commands'].append({'phase': phase,
                'elapsed_s': time.monotonic() - phase_started,
                'type': kind, 'seq': sequence, 'payload_hex': payload.hex()})
            return sequence

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
                    if frame.message_type == wire.MSG_ACK and len(frame.payload) == 4:
                        acknowledgements.append({**record, 'values': list(struct.unpack('<BBBB', frame.payload))})
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

        def require_fault_stopped():
            if latest_status is None or time.monotonic() - latest_status_at > 0.5:
                raise RuntimeError('Fresh STATUS missing after firmware fault')
            if not is_stopped_health_fault(latest_status):
                raise RuntimeError('Firmware health-fault stop not confirmed: ' + str(latest_status))
            if not wheels or any(wheels[-1]['values'][5:]):
                raise RuntimeError('Encoder wheel speeds not zero after firmware fault')

        try:
            usb.open()
            uart.open()
            send(wire.CMD_STOP)
            send(wire.CMD_GET_STATUS)
            pump(4.0)
            if not batteries or not (8000 <= batteries[-1]["values"][6] <= 13000):
                raise RuntimeError("Motor supply telemetry outside expected test range")
            if args.clear_prior_health_fault_once:
                require_fault_stopped()
                timestamp_gap = (latest_status[0] - wheels[-1]['values'][0]) & 0xFFFFFFFF
                if min(timestamp_gap, (-timestamp_gap) & 0xFFFFFFFF) > 200:
                    raise RuntimeError('Wheel telemetry stale before explicit fault clear')
                report['before_authorized_clear_status'] = list(latest_status)
                set_phase('authorized_clear_once')
                clear_sequence = send(wire.CMD_CLEAR_FAULT)
                pump(0.7)
                matching = [a for a in acknowledgements if a['seq'] == clear_sequence and a['values'][0] == wire.CMD_CLEAR_FAULT]
                if len(matching) != 1 or matching[0]['values'] != [wire.CMD_CLEAR_FAULT, wire.RESULT_OK, wire.STATE_IDLE, 0]:
                    raise RuntimeError('Explicit clear did not receive ACK OK/IDLE; no retry')
                require_stopped()
                if latest_status[9] & wire.STATUS_FLAG_HEALTH_FAULT or not wheels or any(wheels[-1]['values'][5:]):
                    raise RuntimeError('Health fault or wheel motion remains after explicit clear')
                report['authorized_clear_ack'] = matching[0]
                report['after_authorized_clear_status'] = list(latest_status)
                print('EXPLICIT_CLEAR_OK; chassis IDLE, wheels stopped; motion has not started', flush=True)
            require_stopped()
            print("PRECHECK supply_mV=" + str(batteries[-1]["values"][6]), flush=True)
            if args.clear_prior_health_fault_once:
                set_phase('repeat_countdown')
                for remaining in (3, 2, 1):
                    print('Watch RF wheel: starting in ' + str(remaining), flush=True)
                    pump(1.0)
                    require_stopped()
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
                        if (args.observe_recovery_fault and
                                latest_status[5] == wire.STATE_FAULT and
                                latest_status[6] == wire.RUNTIME_FAULT_HEALTH_MANAGER and
                                latest_status[8] == 0 and
                                latest_status[9] & wire.STATUS_FLAG_HEALTH_FAULT and
                                not (latest_status[9] & 0x0021) and
                                not any(latest_status[10:12])):
                            fault_observed = True
                            report['first_firmware_health_fault_status'] = list(latest_status)
                            report['first_fault_host_elapsed_s'] = time.monotonic() - started
                            print('FIRMWARE_HEALTH_FAULT_OBSERVED ' + str(latest_status), flush=True)
                            break
                        raise RuntimeError("Motion fault: " + str(latest_status))
                    if wheels and any(abs(cps) > 1000 for cps in wheels[-1]['values'][5:]):
                        raise RuntimeError('Host diagnostic wheel-speed guard exceeded 1000 cps')
                    if time.monotonic() - started > 0.6 and latest_status[5] != wire.STATE_ACTIVE:
                        raise RuntimeError("Unexpected exit from ACTIVE: " + str(latest_status))
                if fault_observed:
                    set_phase(label + '_firmware_fault_observe')
                    pump(1.3)  # GET_STATUS only: no STOP or velocity after fault.
                    require_fault_stopped()
                    report['firmware_stop_verified_before_host_stop'] = True
                else:
                    send(wire.CMD_STOP)
                    set_phase(label + "_stopping")
                    pump(1.3)
                    require_stopped()
                run_status = [r["values"] for r in statuses if r["phase"] == label + "_motion"]
                run_battery = [r["values"][6] for r in batteries if r["phase"] == label + "_motion"]
                info = {"direction": label, "requested_v_mm_s": linear, "requested_w_mrad_s": angular, "commands_sent": sent, "status_samples": len(run_status), "supply_mV_min_max": [min(run_battery), max(run_battery)] if run_battery else [], "stopped_status": list(latest_status), 'firmware_health_fault_observed': fault_observed}
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
            report["capture_result"] = ('COMPLETE_FIRMWARE_HEALTH_FAULT_OBSERVED_NOT_YET_ATTRIBUTED'
                if fault_observed else 'COMPLETE_NOT_A_TRACKING_PASS')
        except Exception as error:
            report["errors"].append(str(error))
            print("ERROR " + str(error), flush=True)
        finally:
            try:
                if fault_observed and not report['errors']:
                    set_phase('final_fault_hold')
                    pump(0.7)
                    require_fault_stopped()
                    report['final_host_stop_sent'] = False
                else:
                    send(wire.CMD_STOP)
                    report['final_host_stop_sent'] = True
                    set_phase("final_stop")
                    pump(0.7)
                    require_stopped()
                report["final_status"] = list(latest_status)
            except Exception as error:
                report["errors"].append("cleanup: " + str(error))
            usb.close()
            uart.close()
            report["host_crc_errors"] = parser.crc_errors
            (root / "decoded.json").write_text(json.dumps({"status": statuses, "wheel": wheels, "battery": batteries, "imu": imu, "ack": acknowledgements}, indent=2), encoding="utf-8")
            (root / "summary.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
            print("EVIDENCE " + str(root), flush=True)
            print(json.dumps({k: v for k, v in report.items() if k != 'tx_commands'}), flush=True)
    return int(bool(report["errors"]))


if __name__ == "__main__":
    raise SystemExit(main())
