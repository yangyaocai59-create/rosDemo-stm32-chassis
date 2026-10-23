"""One authorized ground left OR right practical check, <=180 deg/30 s.
No fault clear, firmware writes, calibration, automatic second run or resume.
"""
import argparse
import datetime
import json
import math
from pathlib import Path
import statistics
import struct
import sys
import time

import serial

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'Tools'))
import usb_cdc_test as wire


class YawEstimate:
    def __init__(self, baseline):
        self.baseline = baseline
        self.previous = None
        self.radians = 0.0
        self.samples = 0

    def feed(self, stamp, wz_mrad):
        rate = (wz_mrad - self.baseline) / 1000.0
        if self.previous is not None:
            dt_ms = (stamp - self.previous[0]) & 0xffffffff
            if dt_ms == 0:
                return
            if dt_ms > 250:
                raise RuntimeError('IMU source gap/backwards timestamp')
            self.radians += (rate + self.previous[1]) * .5 * dt_ms / 1000
        self.previous = (stamp, rate)
        self.samples += 1


def self_test():
    yaw = YawEstimate(18)
    for ms in range(0, 10001, 100):
        yaw.feed(ms, 318)
    assert abs(yaw.radians - 3) < 1e-10
    yaw.feed(10000, 318)
    assert yaw.samples == 101
    try:
        yaw.feed(11000, 318)
        raise AssertionError('Missing timestamp guard')
    except RuntimeError:
        pass
    yaw = YawEstimate(0)
    yaw.feed(0xfffffff0, 300)
    yaw.feed(84, 300)
    assert abs(yaw.radians - .03) < 1e-10
    for direction_sign in (-1, 1):
        yaw = YawEstimate(39)
        for ms in range(0, 10001, 100):
            yaw.feed(ms, 39 + direction_sign * 300)
        assert abs(direction_sign*yaw.radians-3) < 1e-10
        assert struct.unpack('<ii',struct.pack('<ii',0,direction_sign*300)) == (0,direction_sign*300)
    print('PASS offline positive/negative yaw, signed command, duplicate, gap and wrap guards; no hardware opened')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--self-test', action='store_true')
    ap.add_argument('--confirmed-ground-safe', action='store_true')
    ap.add_argument('--confirmed-external-markers', action='store_true')
    ap.add_argument('--direction', choices=('left','right'))
    args = ap.parse_args()
    if args.self_test:
        self_test()
        return 0
    if not args.confirmed_ground_safe or not args.confirmed_external_markers or args.direction is None:
        ap.error('Operator ground safety, external markers and single direction required')
    label = args.direction.upper()
    direction_sign = 1 if label == 'LEFT' else -1
    angular = direction_sign * 300
    stamp = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    folder = Path(__file__).parent / ('pivot_practical_' + args.direction + '_' + stamp)
    folder.mkdir(exist_ok=False)
    report = dict(result='INCOMPLETE', max_seconds=30, requested_mrad_s=angular,
                  direction=label,
                  target_approx_deg=180, errors=[], tx_commands=[],
                  angle_basis='IMU_STATE SI wz, host-only fresh stationary mean subtraction; approximate, not surveyed heading',
                  firmware_identity='User reports current 0.18.21; no Flash readback/version query',
                  diagnostic_100ms_plateau_stop=False,
                  fault_clear_authorized=False,
                  external_markers_operator_confirmed=True,
                  external_angle_result='PENDING_OPERATOR_OBSERVATION',
                  firmware_safety_unchanged=True)
    usb = serial.Serial(port=None, baudrate=115200, timeout=.003, write_timeout=.15)
    usb.dtr, usb.rts, usb.port = True, False, 'COM4'
    uart = serial.Serial(port=None, baudrate=115200, timeout=.003, write_timeout=.15)
    uart.dtr, uart.rts, uart.port = False, False, 'COM8'
    parser = wire.StreamParser()
    sequence, phase = 0, 'initial'
    phase_started = time.monotonic()
    status = wheel = battery = imu = None
    status_at = wheel_at = battery_at = imu_at = 0.0
    previous_uptime = previous_wheel = previous_imu = None
    records = dict(status=[], wheel=[], battery=[], imu=[], ack=[])
    yaw = None
    trace_ends, text_buffer = 0, ''
    next_poll = 0
    command_count = 0
    trace_captured = False

    with (folder / 'ch340.log').open('w', encoding='utf-8') as ulog, \
         (folder / 'usb_frames.jsonl').open('w', encoding='utf-8') as flog:
        def set_phase(value):
            nonlocal phase, phase_started
            phase, phase_started = value, time.monotonic()
            ulog.write('HOST_PHASE ' + value + '\n')
            ulog.flush()
            print('PHASE ' + value, flush=True)

        def send(kind, payload=b''):
            nonlocal sequence
            if kind not in (wire.CMD_GET_STATUS, wire.CMD_STOP, wire.CMD_VELOCITY):
                raise RuntimeError('Unauthorized command')
            if kind == wire.CMD_VELOCITY and (phase != label + '_motion' or
                    payload != struct.pack('<ii', 0, angular)):
                raise RuntimeError('Unauthorized velocity')
            sequence = (sequence + 1) & 65535
            usb.write(wire.encode_frame(kind, sequence, payload))
            report['tx_commands'].append(dict(phase=phase, elapsed_s=time.monotonic()-phase_started,
                type=kind, seq=sequence, payload_hex=payload.hex()))
            return sequence

        def pump(seconds):
            nonlocal status, wheel, battery, imu, status_at, wheel_at, battery_at, imu_at
            nonlocal previous_uptime, previous_wheel, previous_imu, trace_ends, text_buffer, next_poll
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                if phase != label + '_motion' and time.monotonic() >= next_poll:
                    send(wire.CMD_GET_STATUS)
                    next_poll = time.monotonic() + .5
                for frame in parser.feed(usb.read(min(8192, max(1, usb.in_waiting)))):
                    row = dict(phase=phase, elapsed_s=time.monotonic()-phase_started,
                               type=frame.message_type, seq=frame.sequence, payload_hex=frame.payload.hex())
                    flog.write(json.dumps(row) + '\n')
                    s = wire.decode_status(frame)
                    w = wire.decode_wheel_state(frame)
                    i = wire.decode_imu_state(frame)
                    b = struct.unpack('<IIIHHHHBBHHH', frame.payload) if frame.message_type == 0x83 and len(frame.payload) == 28 else None
                    if s is not None:
                        if previous_uptime is not None and ((s[0]-previous_uptime)&0xffffffff) > 0x7fffffff:
                            raise RuntimeError('MCU reset/backwards uptime, no resume')
                        previous_uptime, status, status_at = s[0], s, time.monotonic()
                        records['status'].append({**row, 'values': list(s)})
                    if w is not None:
                        if previous_wheel is not None and ((w[0]-previous_wheel)&0xffffffff) > 0x7fffffff:
                            raise RuntimeError('WHEEL source backwards')
                        if previous_wheel != w[0]:
                            previous_wheel, wheel_at = w[0], time.monotonic()
                        wheel = w
                        records['wheel'].append({**row, 'values': list(w)})
                    if i is not None:
                        if previous_imu is not None and ((i[0]-previous_imu)&0xffffffff) > 0x7fffffff:
                            raise RuntimeError('IMU source backwards')
                        if previous_imu != i[0]:
                            previous_imu, imu_at = i[0], time.monotonic()
                            if yaw is not None and phase in (label + '_motion', label + '_stopping'):
                                yaw.feed(i[0], i[7])
                        imu = i
                        records['imu'].append({**row, 'values': list(i)})
                    if b is not None:
                        battery, battery_at = b, time.monotonic()
                        records['battery'].append({**row, 'values': list(b)})
                    if frame.message_type == wire.MSG_ACK:
                        records['ack'].append(row)
                        if (len(frame.payload) == 4 and frame.payload[1] != wire.RESULT_OK
                                and phase == label + '_motion'):
                            raise RuntimeError('Rejected command ACK: ' + frame.payload.hex())
                raw = uart.read(min(8192, max(1, uart.in_waiting)))
                if raw:
                    text = raw.decode('utf-8', 'replace')
                    ulog.write(text)
                    ulog.flush()
                    text_buffer += text
                    while '\n' in text_buffer:
                        line, text_buffer = text_buffer.split('\n', 1)
                        if 'TRACE END;' in line:
                            trace_ends += 1
                        if phase == label + '_motion' and ('Firmware version' in line or 'Reset cause' in line):
                            raise RuntimeError('UART boot banner during motion')
                flog.flush()

        def guard(stopped=False):
            now = time.monotonic()
            if status is None or now-status_at > .5 or wheel is None or now-wheel_at > .3:
                raise RuntimeError('STATUS/WHEEL source stale')
            if imu is None or now-imu_at > .35 or imu[11] != 2 or not imu[12]&1 or imu[10]:
                raise RuntimeError('IMU not fresh/valid/READY')
            if battery is None or now-battery_at > 1.5 or not 8000 <= battery[6] <= 13000:
                raise RuntimeError('Supply not fresh/within test bounds')
            if status[6] or status[8] or status[5] in (3, 4):
                raise RuntimeError('Firmware fault/ESTOP: ' + str(status))
            if parser.crc_errors or any(status[17:20]):
                raise RuntimeError('Transport CRC/overflow/drop guard')
            if status[9]&0x0150 != 0x0150:
                raise RuntimeError('Link/init/IWDG flag missing')
            if any(abs(c) > 1000 for c in wheel[5:]):
                raise RuntimeError('Wheel feedback >1000cps diagnostic bound')
            if stopped and (status[5] != 1 or any(status[10:16]) or any(wheel[5:]) or status[9]&0x21):
                raise RuntimeError('Not safely stopped before/after motion')

        try:
            usb.open()
            uart.open()
            send(wire.CMD_STOP)
            pump(5)
            guard(stopped=True)
            baseline_rows = [r['values'][7] for r in records['imu'] if r['elapsed_s'] >= 2]
            if len(baseline_rows) < 15 or statistics.pstdev(baseline_rows) > 3 or abs(statistics.mean(baseline_rows)) > 100:
                raise RuntimeError('Stationary IMU baseline insufficient or moving')
            baseline = statistics.mean(baseline_rows)
            report.update(baseline_wz_mrad_s=baseline, baseline_samples=len(baseline_rows),
                          baseline_std_mrad_s=statistics.pstdev(baseline_rows),
                          initial_status=list(status), initial_counts=list(wheel[1:5]))
            print(f'PRECHECK idle, supply={battery[6]}mV, IMU baseline={baseline:.3f}mrad/s', flush=True)
            set_phase('countdown')
            for remaining in (3, 2, 1):
                print(f'Starting single {label} in {remaining}', flush=True)
                pump(1)
                guard(stopped=True)
            yaw = YawEstimate(baseline)
            yaw.feed(imu[0], imu[7])
            set_phase(label + '_motion')
            started = time.monotonic()
            next_command = started
            next_progress = started + 2
            reason = 'MAX_30_SECONDS'
            while time.monotonic()-started < 30:
                guard()
                if time.monotonic()-started > .6 and status[5] != wire.STATE_ACTIVE:
                    raise RuntimeError('Unexpected departure from ACTIVE')
                if direction_sign * yaw.radians >= math.pi:
                    reason = 'APPROX_IMU_180_DEGREES'
                    break
                if direction_sign * yaw.radians < -.3 or abs(imu[7]-baseline) > 1000:
                    raise RuntimeError('Wrong turn direction/excess angular rate')
                if time.monotonic() >= next_command:
                    send(wire.CMD_VELOCITY, struct.pack('<ii', 0, angular))
                    command_count += 1
                    next_command = time.monotonic() + .05
                pump(.006)
                if time.monotonic() >= next_progress:
                    print(f'MOTION elapsed={time.monotonic()-started:.2f}s approx_yaw={math.degrees(yaw.radians):.1f}deg wheel_cps={wheel[5:]}', flush=True)
                    next_progress = time.monotonic() + 2
            report.update(motion_seconds=time.monotonic()-started, stop_reason=reason,
                          approx_yaw_at_stop_request_deg=math.degrees(yaw.radians),
                          velocity_commands=command_count)
            send(wire.CMD_STOP)
            set_phase(label + '_stopping')
            pump(1.5)
            guard(stopped=True)
            report.update(stopped_status=list(status), stopped_counts=list(wheel[1:5]),
                          approx_yaw_after_coast_deg=math.degrees(yaw.radians))
            print('STOP_CONFIRMED ' + json.dumps({k: report[k] for k in ('motion_seconds', 'stop_reason', 'approx_yaw_after_coast_deg')}), flush=True)
            set_phase(label + '_trace')
            before = trace_ends
            uart.write(b't')
            deadline = time.monotonic() + 45
            while before == trace_ends and time.monotonic() < deadline:
                pump(.04)
            if before == trace_ends:
                raise RuntimeError('TRACE END missing; no motion repeat')
            trace_captured = True
            report['result'] = 'COMPLETE_SINGLE_TURN_NOT_ODOMETRY_ACCEPTANCE'
        except (Exception, KeyboardInterrupt) as error:
            report['errors'].append(type(error).__name__ + ': ' + str(error))
            report['velocity_commands'] = command_count
            if yaw is not None:
                report['approx_yaw_on_error_deg'] = math.degrees(yaw.radians)
            if command_count:
                report['motion_seconds_until_abort'] = time.monotonic()-started
            print('ERROR ' + str(error), flush=True)
        finally:
            try:
                if usb.is_open:
                    send(wire.CMD_STOP)
                    set_phase('final_stop')
                    pump(1.5)
                    if (status is None or time.monotonic()-status_at > .5
                            or status[5] not in (1, 3, 4) or any(status[10:16])
                            or status[9]&0x21 or wheel is None or any(wheel[5:])
                            or time.monotonic()-wheel_at > .3):
                        raise RuntimeError('Fresh zero-speed, output-disabled state not confirmed')
                    report['final_status'] = list(status)
                    report['final_zero_speed_outputs_disabled'] = True
                    print('FINAL_STOP_CONFIRMED', flush=True)
                    if command_count and not trace_captured and uart.is_open:
                        # Read-only dump while same ports remain open. Fault stays
                        # latched; no clear/resume follows. Never dump moving.
                        set_phase(label + '_trace')
                        before = trace_ends
                        uart.write(b't')
                        deadline = time.monotonic() + 45
                        while before == trace_ends and time.monotonic() < deadline:
                            pump(.04)
                        if before == trace_ends:
                            raise RuntimeError('Stopped/fault TRACE END missing; no reopen/retry')
                        trace_captured = True
                        print('POST_STOP_TRACE_COMPLETE; no further movement', flush=True)
                        pump(.3)
                        if (status is None or time.monotonic()-status_at > .5
                                or status[5] not in (1,3,4) or any(status[10:16])
                                or status[9]&0x21 or wheel is None or any(wheel[5:])
                                or time.monotonic()-wheel_at > .3):
                            raise RuntimeError('Stopped state not maintained after dump')
                        report['final_status'] = list(status)
            except Exception as error:
                report['errors'].append('cleanup: ' + str(error))
                print('STOP_NOT_CONFIRMED: switch VM off if vehicle not stopped', flush=True)
            usb.close()
            uart.close()
            report.update(host_crc_errors=parser.crc_errors, ports_closed=True,
                          pre_motion_clear_count=0, detailed_trace_captured=trace_captured)
            (folder / 'decoded.json').write_text(json.dumps(records, indent=2), encoding='utf-8')
            (folder / 'summary.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
            print('EVIDENCE ' + str(folder), flush=True)
            print(json.dumps({k:v for k,v in report.items() if k != 'tx_commands'}), flush=True)
    return int(bool(report['errors']))


if __name__ == '__main__':
    raise SystemExit(main())
