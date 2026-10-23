"""Offline pivot source-time and wheel/IMU comparison; no device access.

Angles include settling after STOP. IMU is a reference, not external truth.
"""
import argparse
import json
import math
import re
import struct
from pathlib import Path


def assess(folder, track, diameter, counts_per_rev):
    summary = json.loads((folder / 'summary.json').read_text(encoding='utf-8-sig'))
    frames = json.loads((folder / 'usb_frames.json').read_text(encoding='utf-8-sig'))
    text = (folder / 'ch340.log').read_text(encoding='utf-8-sig')
    events = []
    pattern = r'RUNTIME_EVENT seq=(\d+) t=(\d+) kind=(\d+) state=(\d+) generation=(\d+) command=(\d+)'
    for match in re.finditer(pattern, text):
        events.append(dict(zip(('sequence', 'time_ms', 'kind', 'state', 'generation', 'command'), map(int, match.groups()))))
    # The debug ring can contain earlier runs. Restrict to this run's initial
    # sample and source uptime; these captures do not cross uptime wrap.
    initial_time = summary['initial_wheel'][0]
    starts = [e for e in events if e['kind'] == 1 and e['time_ms'] >= initial_time]
    if not starts:
        raise ValueError('No START event for this capture')
    start = starts[0]
    stops = [e for e in events if e['kind'] == 2 and e['time_ms'] > start['time_ms']]
    if not stops:
        raise ValueError('No STOP event after START')
    stop = stops[0]
    duration = (stop['time_ms'] - start['time_ms']) / 1000
    counts = [b-a for a, b in zip(summary['initial_wheel'][1:5], summary['final_wheel'][1:5])]
    encoder = ((counts[2]+counts[3])-(counts[0]+counts[1])) / 2 * (math.pi*diameter/counts_per_rev) / track * 180/math.pi
    imu = summary['imu_reference_final_deg']
    command = math.degrees(summary['planned_rad_s'] * duration)
    statuses = [struct.unpack('<IIIIHBBBBHhhhhhhIIII', bytes.fromhex(f['payload_hex'])) for f in frames if f['type'] == 129]
    battery = [struct.unpack('<IIIHHHHBBHHH', bytes.fromhex(f['payload_hex'])) for f in frames if f['type'] == 131]
    result = {
        'host_motion_seconds': summary['motion_seconds'],
        'mcu_active_span_s': duration, 'source_time_at_least_15s': duration >= 15,
        'mcu_start_ms': start['time_ms'], 'mcu_stop_request_ms': stop['time_ms'],
        'requested_rad_s': summary['planned_rad_s'], 'net_counts': counts,
        'geometry': {'effective_track_m': track, 'wheel_diameter_m': diameter, 'counts_per_rev': counts_per_rev},
        'encoder_angle_deg': encoder, 'imu_reference_deg': imu,
        'encoder_minus_imu_deg': encoder-imu,
        'encoder_minus_imu_percent': 100*(encoder-imu)/abs(imu),
        'command_integral_source_deg': command,
        'encoder_minus_command_deg': encoder-command,
        'imu_minus_command_deg': imu-command,
        'encoder_mean_rad_s_including_settling': math.radians(encoder)/duration,
        'imu_mean_rad_s_including_settling': math.radians(imu)/duration,
        'instantaneous_minimum_speed_verified': False,
        'reference_baseline_mrad_s': summary['reference_baseline_mrad_s'],
        'reference_baseline_std_mrad_s': summary['reference_baseline_std_mrad_s'],
        'imu_reference_count_gaps': summary['imu_reference_count_gaps'],
        'events_this_run': [e for e in events if e['time_ms'] >= initial_time],
        'accepted_delta': statuses[-1][2]-statuses[0][2],
        'rejected_delta': statuses[-1][3]-statuses[0][3],
        'protocol_counter_deltas': [statuses[-1][i]-statuses[0][i] for i in (17, 18, 19)],
        'host_crc_errors': summary['host_crc_errors'],
        'vm_mV_range': [min(b[6] for b in battery), max(b[6] for b in battery)],
        'ports_closed': summary['ports_closed'],
        'final_stopped_verified': summary['final_stopped_verified'],
        'capture_error': summary['error'], 'trace_error': summary['trace_error'],
        'external_angle_ground_truth_available': False,
        'acceptance_threshold_selected': False,
    }
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    parser.add_argument('--effective-track-m', type=float, required=True)
    parser.add_argument('--wheel-diameter-m', type=float, default=.065)
    parser.add_argument('--counts-per-rev', type=int, default=1560)
    args = parser.parse_args()
    if min(args.effective_track_m, args.wheel_diameter_m, args.counts_per_rev) <= 0:
        parser.error('geometry must be positive')
    result = assess(args.folder, args.effective_track_m, args.wheel_diameter_m, args.counts_per_rev)
    (args.folder/'angle_comparison.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(result, indent=2))
