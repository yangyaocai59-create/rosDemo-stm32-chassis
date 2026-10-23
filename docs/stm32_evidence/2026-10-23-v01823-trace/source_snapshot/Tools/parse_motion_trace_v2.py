"""Offline UART TRACE2 decoder. Does not import serial or send motor commands.

Require a complete BEGIN/END export; legacy traces remain the old tools' domain.
Counter words are joined as signed int64; no filtering or synthetic counts.
"""
import argparse
import json
from pathlib import Path
import re

WHEELS = ('LF', 'LR', 'RF', 'RR')

def fields(text):
    result = {}
    for key, value in re.findall(r'(\w+)=(-?\d+(?:/-?\d+)?)', text):
        if key in result:
            raise ValueError('duplicate field: ' + key)
        result[key] = [int(x) for x in value.split('/')] if '/' in value else int(value)
    return result

def parse(text):
    captures = []
    capture = frame = None
    def finish_frame():
        if frame is None:
            return
        if 'snapshot' not in frame or set(frame['wheels']) != set(WHEELS):
            raise ValueError('incomplete snapshot/wheels')
        for wheel in frame['wheels'].values():
            if 'total' not in wheel or 'pwm' not in wheel:
                raise ValueError('missing wheel detail or legacy values')
        previous = capture['frames'][-1] if capture['frames'] else None
        if previous and ((frame['snapshot']['seq']-previous['snapshot']['seq']) & 0xffffffff) != 1:
            raise ValueError('missing/duplicate frame sequence')
        capture['frames'].append(frame)
    for line in text.splitlines():
        if 'TRACE2 BEGIN ' in line:
            if capture is not None:
                raise ValueError('new BEGIN before END (incomplete earlier export)')
            header = fields(line.split('TRACE2 BEGIN ', 1)[1])
            if header.get('schema') != 2 or not 1 <= header.get('count', 0) <= 128:
                raise ValueError('unsupported schema or count')
            capture = {'header': header, 'frames': []}
            frame = None
        elif 'TRACE t=' in line and capture is not None:
            finish_frame()
            frame = {**fields(line.split('TRACE ', 1)[1]), 'wheels': {}}
            if any(k not in frame for k in ('t','dt','req','applied','state','stop','fault','stby')):
                raise ValueError('missing frame header field')
        elif 'TRACE2 SNAP ' in line and capture is not None:
            if frame is None or 'snapshot' in frame:
                raise ValueError('orphan/duplicate SNAP')
            frame['snapshot'] = fields(line.split('TRACE2 SNAP ', 1)[1])
            for required in ('seq', 'speed_t', 'control_t', 'flags', 'arr', 'bdtr', 'ccer', 'dirs', 'pin'):
                if required not in frame['snapshot']:
                    raise ValueError('missing SNAP field ' + required)
        elif (match := re.search(r'TRACE(2)? (LF|LR|RF|RR) (.*)', line)) and capture is not None:
            if frame is None:
                raise ValueError('orphan wheel')
            version, name, tail = match.groups()
            data = fields(tail)
            wheel = frame['wheels'].setdefault(name, {})
            required = ('total_hi','total_lo','request','ctrl','ccr','limited','obs') if version else (
                'dc','raw','target','cps','ff_x10','p_x10','i_x10','pwm')
            if any(k not in data for k in required) or any(k in wheel for k in data):
                raise ValueError('missing/duplicate wheel fields')
            if version:
                if not (0 <= data['total_hi'] <= 0xffffffff and 0 <= data['total_lo'] <= 0xffffffff):
                    raise ValueError('invalid counter words')
                total = (data['total_hi'] << 32) | data['total_lo']
                data['total'] = total - (1 << 64) if total & (1 << 63) else total
            wheel.update(data)
        elif 'TRACE END;' in line and capture is not None:
            finish_frame()
            if len(capture['frames']) != capture['header']['count']:
                raise ValueError('frame count does not match BEGIN')
            if capture['header'].get('reason', 0):
                hits = [f for f in capture['frames'] if f['snapshot']['seq'] == capture['header'].get('trigger_seq')]
                if len(hits) != 1 or hits[0]['t'] != capture['header'].get('trigger'):
                    raise ValueError('trigger frame missing or timestamp mismatch')
            captures.append(capture)
            capture = frame = None
    if capture is not None:
        raise ValueError('truncated export: END missing')
    if not captures:
        raise ValueError('no complete TRACE2 capture')
    return captures

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('log', type=Path)
    ap.add_argument('--output', type=Path)
    args = ap.parse_args()
    try:
        result = parse(args.log.read_text(encoding='utf-8-sig', errors='strict'))
    except (ValueError, OSError) as error:
        ap.exit(1, f'FAIL: {error}\n')
    encoded = json.dumps(result, indent=2) + '\n'
    if args.output:
        args.output.write_text(encoded, encoding='utf-8')
        print(f'PASS: {len(result)} complete capture(s); {args.output}')
    else:
        print(encoded, end='')

if __name__ == '__main__':
    main()
