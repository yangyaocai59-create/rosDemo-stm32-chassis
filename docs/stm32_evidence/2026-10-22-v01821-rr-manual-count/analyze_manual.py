"""Offline independent raw-stream/manual endpoints review. No serial I/O."""
import hashlib
import json
from pathlib import Path
import sys

here=Path(__file__).resolve().parent
root=here.parents[2]
sys.path.insert(0,str(root/'Tools'))
import usb_cdc_test as wire

capture=here/'capture'
summary=json.loads((capture/'summary.json').read_text(encoding='utf-8'))
assert not summary['errors'] and summary['ports_closed']
assert summary['motion_commands']==summary['debug_writes']==0
assert len(summary['phases'])==2
fwd,rev=summary['phases']
assert fwd['turns_user_reported']==rev['turns_user_reported']==3
assert fwd['delta']==[0,0,0,4689] and rev['delta']==[0,0,0,-4681]
assert summary['round_trip_delta']==[0,0,0,8]
assert summary['last_status'][5]==1 and not any(summary['last_status'][10:16])
assert summary['last_status'][9]==0x0150 and not any(summary['last_status'][17:20])
points=[summary['baseline'],fwd,rev]
uart_points=[p['uart_rr'] for p in points]
usb_counts=[summary['baseline']['counts'][3],fwd['end_counts'][3],rev['end_counts'][3]]
assert [p['total'] for p in uart_points]==usb_counts
raw_delta=[(b['raw']-a['raw'])&0xffff for a,b in zip(uart_points,uart_points[1:])]
assert raw_delta==[(-4689)&0xffff,4681]
events=[json.loads(line) for line in (capture/'events.jsonl').read_text().splitlines()]
tx=[r for r in events if r['kind']=='host_tx']
assert tx and all(r['type']==wire.CMD_GET_STATUS for r in tx)
baseline=summary['baseline']['counts']
logged_wheels=[r['values'] for r in events if r['kind']=='wheel']
assert logged_wheels and all(list(w[1:4])==baseline[:3] for w in logged_wheels)
parser=wire.StreamParser();raw_wheels=[];raw_status=[]
with (capture/'usb_rx.bin').open('rb') as raw:
    while chunk:=raw.read(65536):
        for frame in parser.feed(chunk):
            if (v:=wire.decode_wheel_state(frame)) is not None:raw_wheels.append(list(v))
            if (v:=wire.decode_status(frame)) is not None:raw_status.append(list(v))
assert parser.crc_errors==0 and raw_wheels==logged_wheels
assert all(s[5]==wire.STATE_IDLE and s[6]==s[8]==0 and not any(s[10:12]) and not s[9]&1 for s in raw_status)
assert all(s[1:4]==summary['baseline']['status'][1:4] for s in raw_status)
result=dict(result='RR_MANUAL_BASIC_MAPPING_DIRECTION_SCALE_CONSISTENT_NOT_POWERED_VALIDATION',
    nominal_counts_per_turn=1560,forward_3_turns=4689,reverse_3_turns=-4681,
    forward_per_turn=4689/3,reverse_magnitude_per_turn=4681/3,
    forward_nominal_error_percent=(4689/4680-1)*100,
    reverse_nominal_error_percent=(4681/4680-1)*100,round_trip_residual_counts=8,
    round_trip_equivalent_nominal_wheel_degrees=8/1560*360,
    UART_normalized_total_equals_USB_at_all_three_stopped_endpoints=True,
    RR_raw_16bit_delta_and_normalized_sign_consistent=True,
    other_three_counts_constant_across_all_USB_wheel_frames=True,
    wheel_frames=len(raw_wheels),status_frames=len(raw_status),GET_STATUS_commands=len(tx),
    raw_stream_redecoded_matches_logged_wheel_frames=True,host_crc_errors=0,
    command_sequence_accepted_rejected_unchanged=True,all_observed_status_IDLE_fault_free_output_disabled=True,
    final_counts=list(summary['last_wheel'][1:5]),final_speeds=list(summary['last_wheel'][5:9]),
    ports_closed=True,powered_motion_sent=False,firmware_modified=False,
    limitations=['Turn count and marker alignment are operator-reported, not an independent precision angle measurement.',
      'Eight counts are about1.85deg of nominal wheel angle; hand positioning/gear lash and actual measurement error are not separated.',
      'This does not prove exact1560counts/rev or absence of every missing pulse.',
      'No powered/loaded test, motor-drive electrical interference diagnosis, all-four-wheel scale calibration, ROS odometry or traction acceptance.',
      'Do not adjust the encoder divisor or declare the previous140ms powered feedback gap fixed from this result.'])
(here/'analysis.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
