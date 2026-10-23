"""Offline analysis of an existing trace23_pivot_once folder. No device IO."""
import argparse
import json
from pathlib import Path
import statistics
import struct

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('folder',type=Path)
    args=ap.parse_args(); folder=args.folder
    summary=json.loads((folder/'summary.json').read_text())
    captures=json.loads((folder/'trace_v2.json').read_text())
    trace=captures[-1]; active=[f for f in trace['frames'] if f['state']==2]
    packets=json.loads((folder/'usb_frames.json').read_text())
    battery=[struct.unpack('<IIIHHHHBBHHH',bytes.fromhex(f['payload_hex'])) for f in packets if f['type']==131]
    statuses=[struct.unpack('<IIIIHBBBBHhhhhhhIIII',bytes.fromhex(f['payload_hex'])) for f in packets if f['type']==129]
    wheels=[struct.unpack('<Iqqqqiiii',bytes.fromhex(f['payload_hex'])) for f in packets if f['type']==130]
    assert active and statuses
    first,last=statuses[0],statuses[-1]
    out={'folder':str(folder),'motion_seconds':summary['motion_seconds'],
        'complete_trace_frames':len(trace['frames']),'trace_header':trace['header'],
        'active_trace_span_ms':[active[0]['t'],active[-1]['t']],
        'snapshot_flags':sorted({f['snapshot']['flags'] for f in trace['frames']}),
        'vm_mV_range':[min(b[6] for b in battery),max(b[6] for b in battery)],
        'accepted_delta':last[2]-first[2],'rejected_delta':last[3]-first[3],
        'final_status':last,'final_stopped_verified':summary['final_stopped_verified'],
        'fault_cleared':summary['fault_cleared'],
        'protocol_counter_deltas':{'crc':last[17]-first[17], 'overflow':last[18]-first[18], 'drop':last[19]-first[19]},
        'trace_final_stby':trace['frames'][-1]['stby'],
        'trace_final_pin':trace['frames'][-1]['snapshot']['pin'],
        'trace_final_ccr':[trace['frames'][-1]['wheels'][w]['ccr'] for w in ('LF','LR','RF','RR')],
        'end_of_capture_stopped':bool(last[5] in (1,3,4) and not(last[9]&0x20)
            and not any(last[10:16]) and len(wheels)>=3
            and all(not any(w[5:]) for w in wheels[-3:])),
        'wheels':{}}
    for index,name in enumerate(('LF','LR','RF','RR')):
        vals=[f['wheels'][name] for f in active]
        sign=-1 if statistics.mean(v['target'] for v in vals)<0 else 1
        out['wheels'][name]={
            'trace_mean_target_cps':statistics.mean(sign*v['target'] for v in vals),
            'trace_mean_filtered_cps':statistics.mean(sign*v['cps'] for v in vals),
            'trace_mean_raw_cps':statistics.mean(sign*v['raw'] for v in vals),
            'trace_pwm_magnitude_range':[min(abs(v['pwm']) for v in vals),max(abs(v['pwm']) for v in vals)],
            'trace_mean_pwm_magnitude':statistics.mean(abs(v['pwm']) for v in vals),
            'trace_mean_directional_i':statistics.mean(sign*v['i_x10']/10 for v in vals),
            'trace_i_at_cap_samples':sum(bool(v['obs']&2) for v in vals),
            'trace_zero_dc_samples':sum(v['dc']==0 for v in vals),
            'trace_request_applied_mismatch_samples':sum(v['request']!=v['pwm'] for v in vals),
            'whole_capture_net_counts':wheels[-1][1+index]-wheels[0][1+index]}
    (folder/'analysis.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(out,indent=2))

if __name__=='__main__': main()
