"""Offline audit of full USB bytes, logged frames and RAW capture window.

Preserves original logs/summary. Writes explicitly named derived reports.
No serial imports, device access or changes to firmware/calibration.
"""
import argparse
import json
from pathlib import Path

import usb_cdc_test as wire
from imu_stationary_diagnostic import analyze_uart, analyze_usb


def verify(folder):
    original=[json.loads(l)['raw'] for l in (folder/'raw_samples.jsonl').read_text().splitlines()]
    assert original
    events=list(map(json.loads,(folder/'frames.jsonl').read_text().splitlines()))
    logged=[e for e in events if e['kind']=='rx']
    parser=wire.StreamParser()
    frames=parser.feed((folder/'usb_rx.bin').read_bytes())
    assert parser.crc_errors==0 and len(frames)==len(logged)
    assert all((f.message_type,f.sequence,f.payload.hex())==
               (e['type'],e['sequence'],e['payload']) for f,e in zip(frames,logged))
    first,last=original[0][1],original[-1][1]
    raw=[]
    for frame in frames:
        sample=wire.decode_imu_raw_v1(frame)
        if sample is None or not first<=sample[1]<=last:continue
        if raw and sample[1]==raw[-1][1]:
            assert sample==raw[-1]
            continue
        raw.append(sample)
    assert all(b[1]-a[1]==1 for a,b in zip(raw,raw[1:])), 'source gap exists in full receive log'
    by_count={r[1]:r for r in raw}
    assert all(tuple(r)==by_count[r[1]] for r in original)
    si=[]
    for r in raw:
        converted=wire.imu_raw_v1_to_si(r)
        si.append([r[0],r[1],*[v*1000 for v in converted['accel_m_s2']],
                   *[v*1000 for v in converted['angular_velocity_rad_s']],
                   r[11],r[12],r[13],r[16],r[17]])
    statuses=[s for f in frames if (s:=wire.decode_status(f)) is not None]
    wheels=[s for f in frames if (s:=wire.decode_wheel_state(f)) is not None]
    assert statuses and wheels
    assert all(s[5]==1 and s[6]==s[8]==0 and s[10:12]==(0,0) for s in statuses)
    assert all(w[1:5]==wheels[0][1:5] and not any(w[5:9]) for w in wheels)
    assert all(s[17:20]==(0,0,0) for s in statuses)
    uart,pairs=analyze_uart((folder/'ch340.log').read_text(encoding='utf8'),first,last)
    matched=0
    for pair in pairs:
        r=by_count.get(pair['sample_count'])
        if r is None:continue
        assert pair['gyro']==list(r[5:8]) and pair['accel']==list(r[2:5])
        matched+=1
    assert matched
    report=dict(result='PASS_FULL_RECEIVE_LOG_AUDIT',
        source='usb_rx.bin cross-checked against complete frames.jsonl; original capture window',
        original_files_preserved=True,original_window_samples=len(original),
        collector_suffix_frames_recovered=len(raw)-len(original),
        source_sample_count_gaps=0,crc_errors=0,raw=analyze_usb(si),
        baseline_values=[list(v) for v in sorted({r[8:11] for r in raw})],
        raw_gyro_means=[sum(r[i] for r in raw)/len(raw) for i in (5,6,7)],
        first_time_ms=raw[0][0],last_time_ms=raw[-1][0],
        first_sample_count=first,last_sample_count=last,
        status_frames=len(statuses),status_flags=sorted({s[9] for s in statuses}),
        status_crc_rxoverflow_txdrop=[0,0,0],wheel_count_change=[0,0,0,0],
        uart_raw_exact_matches=matched,uart_raw_mismatches=0,
        calibration_start_lines=uart['calibration_start_lines'],flash_save_lines=uart['flash_save_lines'])
    path=folder/'full_receive_analysis.json'
    assert not path.exists(), 'do not overwrite prior derived evidence'
    path.write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
    (folder/'full_receive_raw_samples.jsonl').write_text(''.join(
        json.dumps(dict(raw=list(r)))+'\n' for r in raw),encoding='utf8')
    print(json.dumps(dict(folder=folder.name,samples=len(raw),
        recovered=report['collector_suffix_frames_recovered'],rate=report['raw']['source_rate_hz'],
        intervals=[report['raw']['interval_min_ms'],report['raw']['interval_max_ms']],
        raw_drift_deg_min=report['raw']['yaw_rate_deg_min'],baseline=report['baseline_values'],
        flags=report['raw']['flags'],UART_matches=matched),indent=2))


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('folders',nargs='+',type=Path)
    for folder in ap.parse_args().folders:verify(folder)
