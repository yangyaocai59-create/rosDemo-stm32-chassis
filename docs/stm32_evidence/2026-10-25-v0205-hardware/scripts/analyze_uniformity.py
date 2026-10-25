"""Offline, source-time one-second yaw-rate windows; no device access."""
import argparse
import json
from pathlib import Path
import statistics
import struct
import sys

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'Tools'))
from compare_encoder_imu_yaw import gyro_radians

def analyze(folder):
    report=json.loads((folder/'yaw_comparison.json').read_text(encoding='utf-8'))
    unique={}
    for frame in json.loads((folder/'usb_frames.json').read_text(encoding='utf-8')):
        if frame['type']==132:
            row=struct.unpack('<IIiiiiiiiIIBB',bytes.fromhex(frame['payload_hex']))
            if row[0] in unique and unique[row[0]]!=row:
                raise ValueError('conflicting source sample')
            unique[row[0]]=row
    rows=sorted(unique.values())
    start,stop=report['mcu_start_ms'],report['mcu_stop_ms']
    bias=report['baselines']['baseline']['mean_z_mrad_s']
    target=report['requested_rad_s']
    rates=[gyro_radians(rows,start+k*1000,start+(k+1)*1000,bias)
           for k in range(1,int((stop-start)/1000))]
    errors=[(rate-target)/abs(target)*100 for rate in rates]
    result=dict(capture=folder.name,
        method='Nonoverlapping 1-second source-time gyro integrals; first 1s and final incomplete second excluded; pre-motion stationary bias subtracted',
        window_seconds=[1,len(rates)+1],requested_rad_s=target,rates_rad_s=rates,
        mean_rate_rad_s=statistics.mean(rates),std_rate_rad_s=statistics.pstdev(rates),
        relative_error_percent=errors,
        rms_relative_error_percent=(sum(x*x for x in errors)/len(errors))**.5,
        max_abs_relative_error_percent=max(map(abs,errors)),
        all_windows_within_10_percent=all(abs(x)<=10 for x in errors),
        is_repeated_same_condition_comparison=False)
    (folder/'yaw_uniformity.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folders',nargs='+',type=Path)
    for folder in parser.parse_args().folders:
        print(json.dumps(analyze(folder),indent=2))
