"""Offline static Z-gyro residual estimate from RAW v1/v2 JSONL; never opens ports.

Input rows are decoded 20-field arrays, optionally inside {"raw": [...]}.
Operator must establish stationarity separately. This is not fused ROS yaw.
"""
import argparse
import json
from pathlib import Path
import statistics


def analyze(rows):
    samples = []
    duplicates = 0
    baseline = None
    scale = None
    version = None
    gaps = []
    elapsed = [0]
    for record in rows:
        r = record['raw'] if isinstance(record, dict) else record
        if len(r) != 20 or any(type(v) is not int for v in r):
            raise ValueError('expected decoded 20-integer RAW v1 rows')
        if (r[18] not in (1, 2) or r[19] != 0 or r[14] <= 0 or
                r[16] != 2 or (r[17] & 7) != 7 or r[17] & 8):
            raise ValueError('RAW v1/v2, READY, valid bias and not calibrating required')
        if r[13] != 0:
            raise ValueError('consecutive sensor errors present')
        if baseline is None:
            baseline, scale, errors, version = r[8:11], r[14], r[12], r[18]
        if r[8:11] != baseline or r[14] != scale or r[12] != errors or r[18] != version:
            raise ValueError('bias/scale/error changed: split the capture first')
        if samples:
            prev = samples[-1]
            step = (r[1] - prev[1]) & 0xFFFFFFFF
            dt = (r[0] - prev[0]) & 0xFFFFFFFF
            if step == 0:
                if dt != 0 or r[2:14] != prev[2:14]:
                    raise ValueError('same sample count has inconsistent source data')
                duplicates += 1
                continue
            if not 0 < step < 0x80000000 or not 0 < dt <= 1000:
                raise ValueError('source reset/out-of-order or gap >1 second')
            if step != 1:
                gaps.append(dict(before=prev[1], after=r[1], step=step, dt_ms=dt))
            elapsed.append(elapsed[-1] + dt)
        samples.append(r)
    if len(samples) < 2:
        raise ValueError('at least two unique samples required')
    divisor = 1000 if version == 2 else 1
    wz = [(r[7] - r[10] / divisor) / (r[14] / 10.0) for r in samples]
    duration = elapsed[-1] / 1000.0
    angle = sum((a + b) * 0.5 * (tb - ta) / 1000.0
                for a, b, ta, tb in zip(wz, wz[1:], elapsed, elapsed[1:]))
    windows = []
    for start in range(0, elapsed[-1], 10000):
        indices = [i for i, t in enumerate(elapsed) if start <= t < start + 10000]
        if indices:
            values = [wz[i] for i in indices]
            windows.append(dict(start_s=start / 1000, samples=len(values),
                                mean_deg_s=statistics.mean(values),
                                equivalent_deg_min=statistics.mean(values) * 60))
    return dict(result='STATIC_GYRO_RESIDUAL_ESTIMATE', unique_samples=len(samples),
                duplicate_frames_excluded=duplicates, duration_s=duration,
                raw_format_version=version, baseline_wire_values=baseline,
                baseline_raw=[b / divisor for b in baseline],
                gyro_scale_lsb_per_deg_s=scale / 10.0,
                raw_gyro_mean=[statistics.mean(r[i] for r in samples) for i in (5, 6, 7)],
                mean_wz_deg_s=statistics.mean(wz),
                time_weighted_wz_deg_s=angle / duration,
                equivalent_yaw_deg_min=angle / duration * 60,
                observed_integral_deg=angle, source_gaps=gaps,
                strict_source_continuity=not gaps,
                temperature_c_range=[min(r[11] for r in samples) / 1000,
                                     max(r[11] for r in samples) / 1000],
                windows=windows,
                note=('RAW v1 is a coarse bias view on firmware >=0.16. ' if version == 1 else
                      'RAW v2 includes the exact applied milli-RAW baseline. ') +
                     'Stationarity is operator-established. RAW residual, not fused ROS yaw. '
                     'Trapezoidal integration across reported gaps is only an estimate; '
                     'short capture cannot establish long-term/temperature performance.')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('input', type=Path)
    ap.add_argument('--output', type=Path,
                    help='optional NEW derived JSON file; refuses overwrite')
    args = ap.parse_args()
    rows = [json.loads(line) for line in args.input.read_text(encoding='utf8').splitlines()
            if line.strip()]
    report = analyze(rows)
    text = json.dumps(report, indent=2) + '\n'
    if args.output:
        with args.output.open('x', encoding='utf8') as out:
            out.write(text)
    print(text, end='')


if __name__ == '__main__':
    main()
