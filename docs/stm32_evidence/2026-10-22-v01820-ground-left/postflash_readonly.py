"""GET_STATUS-only CDC probe plus passive CH340 capture; no motor commands."""
import argparse
import datetime
import json
import pathlib
import re
import struct
import sys
import time

import serial
from serial.tools import list_ports

project_root = next((p for p in pathlib.Path(__file__).resolve().parents
                     if (p / 'Tools' / 'usb_cdc_test.py').is_file()), None)
if project_root is None:
    raise RuntimeError('Cannot locate repository Tools/usb_cdc_test.py')
sys.path.insert(0, str(project_root / 'Tools'))
import usb_cdc_test as wire


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--label', default='v01813')
    parser.add_argument('--duration', type=float, default=4.0)
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_-]+', args.label):
        parser.error('label must be a simple filename label')
    if not 1.0 <= args.duration <= 20.0:
        parser.error('duration must be 1..20 seconds')
    stamp = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    target = pathlib.Path(__file__).parent / (args.label + '_postflash_readonly_' + stamp)
    result = {'note': 'GET_STATUS only; no STOP, velocity, CLEAR_FAULT, debug writes, reset or calibration.',
              'timezone': 'Asia/Singapore', 'duration_s': args.duration,
              'ports': [{'port': p.device, 'vid': p.vid, 'pid': p.pid, 'serial': p.serial_number}
                        for p in list_ports.comports()], 'errors': [],
              'status': [], 'wheel': [], 'battery': [], 'imu': []}
    usb = serial.Serial(port=None, baudrate=115200, timeout=0.005, write_timeout=0.2)
    usb.dtr, usb.rts, usb.port = True, False, 'COM4'
    uart = serial.Serial(port=None, baudrate=115200, timeout=0.005, write_timeout=0.2)
    uart.dtr, uart.rts, uart.port = False, False, 'COM8'
    decoder = wire.StreamParser()
    chunks = []
    try:
        uart.open()
        usb.open()
        end, next_poll, seq = time.monotonic() + args.duration, 0.0, 0
        while time.monotonic() < end:
            if time.monotonic() >= next_poll:
                seq += 1
                usb.write(wire.encode_frame(wire.CMD_GET_STATUS, seq))
                next_poll = time.monotonic() + 0.4
            raw = usb.read(min(8192, max(1, usb.in_waiting)))
            for frame in decoder.feed(raw):
                for key, fn in [('status', wire.decode_status), ('wheel', wire.decode_wheel_state),
                                ('imu', wire.decode_imu_state)]:
                    value = fn(frame)
                    if value is not None:
                        result[key].append(list(value))
                if frame.message_type == wire.MSG_BATTERY_STATE and len(frame.payload) == 28:
                    result['battery'].append(list(struct.unpack('<IIIHHHHBBHHH', frame.payload)))
            raw = uart.read(min(8192, max(1, uart.in_waiting)))
            if raw:
                chunks.append(raw)
    except Exception as exc:
        result['errors'].append(str(exc))
    finally:
        usb.close()
        uart.close()
    text = b''.join(chunks).decode('utf-8', 'replace')
    result['host_crc_errors'] = decoder.crc_errors
    result['version_lines'] = [line for line in text.splitlines() if 'Firmware version' in line]
    result['timing_lines'] = [line for line in text.splitlines() if 'CTRL TIMING' in line]
    result['reset_lines'] = [line for line in text.splitlines() if 'Reset cause' in line]
    if not result['status']:
        result['errors'].append('No STATUS received')
    target.with_suffix('.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    target.with_suffix('.log').write_text(json.dumps({k: v for k, v in result.items()
         if k not in ['status', 'wheel', 'battery', 'imu']}, indent=2) + '\n\n' + text,
         encoding='utf-8')
    print('EVIDENCE', target)
    print(json.dumps({**{k: result[k] for k in ['errors', 'host_crc_errors', 'version_lines', 'reset_lines']},
                      **{'last_' + k: result[k][-1] if result[k] else None
                         for k in ['status', 'wheel', 'battery', 'imu', 'timing_lines']}}, indent=2))
    return int(bool(result['errors']))


if __name__ == '__main__':
    raise SystemExit(main())
