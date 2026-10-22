"""GET_STATUS + one stopped-only UART 't' dump; no STOP/CLEAR/velocity/calibration."""
import datetime
import json
from pathlib import Path
import struct
import sys
import time
import serial

root = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(root/'Tools'))
import usb_cdc_test as wire

stamp = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
folder = root/'build/hardware_tests'/('v01817_latched_fault_trace_'+stamp)
folder.mkdir()
usb = serial.Serial(port=None, baudrate=115200, timeout=.005, write_timeout=.2)
usb.dtr, usb.rts, usb.port = True, False, 'COM4'
uart = serial.Serial(port=None, baudrate=115200, timeout=.005, write_timeout=.2)
uart.dtr, uart.rts, uart.port = False, False, 'COM8'
parser = wire.StreamParser()
decoded = {k: [] for k in ('status', 'wheel', 'battery', 'imu', 'ack')}
report = {'scope': 'GET_STATUS and one stopped-only t; no motion, STOP, clear, reset or calibration',
          'errors': [], 'tx_commands': [], 'uart_tx_hex': [], 'trace_end_seen': False}
sequence, next_poll, latest_at, latest, last_wheel = 0, 0, 0, None, None
phase = 'fault_precheck'
started = time.monotonic()
text_buffer = ''

def require_stopped_fault():
    assert latest is not None and time.monotonic()-latest_at < .5, 'Fresh STATUS required'
    assert latest[5:9] == (3, 5, 6, 0), ('Not expected latched health FAULT', latest)
    assert latest[9] & 0x80 and not latest[9] & 0x21 and not any(latest[10:16]), latest
    assert last_wheel is not None and not any(last_wheel[5:]), 'Wheel speed nonzero'

with (folder/'ch340.log').open('w', encoding='utf-8') as uart_log, \
     (folder/'usb_frames.jsonl').open('w', encoding='utf-8') as usb_log:
    def pump(seconds):
        global sequence, next_poll, latest_at, latest, last_wheel, text_buffer
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            if time.monotonic() >= next_poll:
                sequence += 1
                usb.write(wire.encode_frame(wire.CMD_GET_STATUS, sequence))
                report['tx_commands'].append({'type': wire.CMD_GET_STATUS, 'seq': sequence})
                next_poll = time.monotonic()+.4
            raw = usb.read(min(8192, max(1, usb.in_waiting)))
            for frame in parser.feed(raw):
                rec = {'phase': phase, 'elapsed_s': time.monotonic()-started,
                       'type': frame.message_type, 'seq': frame.sequence, 'payload_hex': frame.payload.hex()}
                usb_log.write(json.dumps(rec)+'\n')
                for key, decoder in [('status', wire.decode_status), ('wheel', wire.decode_wheel_state),
                                     ('imu', wire.decode_imu_state)]:
                    values = decoder(frame)
                    if values is not None:
                        decoded[key].append({**rec, 'values': list(values)})
                        if key == 'status':
                            latest, latest_at = values, time.monotonic()
                        if key == 'wheel':
                            last_wheel = values
                if frame.message_type == wire.MSG_BATTERY_STATE and len(frame.payload) == 28:
                    decoded['battery'].append({**rec, 'values': list(struct.unpack('<IIIHHHHBBHHH', frame.payload))})
            raw = uart.read(min(8192, max(1, uart.in_waiting)))
            if raw:
                text = raw.decode('utf-8', 'replace')
                uart_log.write(text)
                uart_log.flush()
                text_buffer += text
                if 'TRACE END;' in text_buffer:
                    report['trace_end_seen'] = True
            usb_log.flush()
    try:
        uart.open()
        usb.open()
        uart_log.write('HOST_PHASE fault_precheck\n')
        pump(2)
        require_stopped_fault()
        report['precheck_status'] = list(latest)
        phase = 'LEFT_trace'
        uart_log.write('HOST_PHASE LEFT_trace\n')
        uart_log.flush()
        uart.write(b't')
        report['uart_tx_hex'].append('74')
        print('Stopped latched FAULT confirmed; reading frozen trace only', flush=True)
        deadline = time.monotonic()+45
        while not report['trace_end_seen'] and time.monotonic() < deadline:
            pump(.05)
            require_stopped_fault()
        assert report['trace_end_seen'], 'TRACE END absent; do not retry automatically'
        pump(.5)
        require_stopped_fault()
        report['final_status'] = list(latest)
        report['final_wheel'] = list(last_wheel)
    except Exception as exc:
        report['errors'].append(str(exc))
    finally:
        usb.close()
        uart.close()
        report['host_crc_errors'] = parser.crc_errors
        (folder/'decoded.json').write_text(json.dumps(decoded, indent=2), encoding='utf-8')
        (folder/'summary.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        print('EVIDENCE '+str(folder), flush=True)
        print(json.dumps(report, indent=2), flush=True)
if report['errors']:
    raise SystemExit(1)
