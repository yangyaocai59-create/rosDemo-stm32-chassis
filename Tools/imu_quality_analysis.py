"""Offline parser for 0.14.4 CH340 quality records; no serial/device access."""
import re

QUALITY = re.compile(
    r"\[(\d+)\].*IMU QUALITY: epoch=(\d+) drdy=(\d+) missed=(\d+) "
    r"race=(\d+) timeout=(\d+) cal=(\d+) persist=(\d+)")


def analyze_quality(text, first_ms, last_ms):
    span = (last_ms - first_ms) & 0xFFFFFFFF
    rows = []
    names = ('time_ms', 'epoch', 'drdy', 'missed', 'race', 'timeout', 'cal', 'persist')
    for match in QUALITY.finditer(text):
        row = dict(zip(names, map(int, match.groups())))
        if ((row['time_ms'] - first_ms) & 0xFFFFFFFF) <= span:
            rows.append(row)
    if len(rows) < 2:
        return dict(result='INSUFFICIENT_QUALITY_RECORDS', records=len(rows))
    deltas = {key: (rows[-1][key] - rows[0][key]) & 0xFFFFFFFF
              for key in ('drdy', 'missed', 'race', 'timeout')}
    epochs = sorted({r['epoch'] for r in rows})
    ok = len(epochs) == 1 and deltas['drdy'] > 0 and all(
        deltas[k] == 0 for k in ('missed', 'race', 'timeout'))
    return dict(result='PASS_OBSERVED_QUALITY' if ok else 'FAIL_QUALITY_CHANGED',
                records=len(rows), epochs=epochs, deltas=deltas,
                first=rows[0], last=rows[-1],
                note='Sparse UART diagnostics, not a lossless per-interrupt trace; '
                     'cal/persist are separate; persist 0/1/2/3 = not-requested/pending/saved/failed.')
