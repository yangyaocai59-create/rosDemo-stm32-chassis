"""One-session MANUAL RR measurement. GET_STATUS only; no motor/debug writes.
Operator moves the unpowered lifted wheel. A local JSON file marks phases.
"""
import argparse
import datetime
import json
from pathlib import Path
import re
import struct
import sys
import time

import serial
from serial.tools import list_ports

ROOT=next(p for p in Path(__file__).resolve().parents if (p/'Tools/usb_cdc_test.py').exists())
sys.path.insert(0,str(ROOT/'Tools'))
import usb_cdc_test as wire


def main():
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--confirmed-vm-off-lifted',action='store_true')
    cli.add_argument('--max-seconds',type=int,default=600)
    args=cli.parse_args()
    if not args.confirmed_vm_off_lifted or not 30<=args.max_seconds<=900:
        cli.error('Explicit VM-off/lifted confirmation and max-seconds30..900 required')
    ports={p.device:p for p in list_ports.comports()}
    assert 'COM4' in ports and (ports['COM4'].vid,ports['COM4'].pid)==(0x0483,0x5740)
    assert 'COM8' in ports and (ports['COM8'].vid,ports['COM8'].pid)==(0x1A86,0x7523)
    stamp=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')
    out=ROOT/'build/hardware_tests'/('rr_manual_count_'+stamp)
    out.mkdir(exist_ok=False)
    control=out/'operator_phase.json'
    summary=dict(test='MANUAL RR wheel count, NOT powered motion',timezone='Asia/Singapore',
        firmware_identity='User-reported .21, no independent readback',
        setup='User confirms VM off, encoders powered, all wheels lifted/fixed',
        serial_tx_allowed=['GET_STATUS 0x04'],debug_writes=0,motion_commands=0,
        reset_requested=False,fault_clear_sent=False,calibration_sent=False,
        nominal_counts_per_rev=1560,phases=[],errors=[],result='INCOMPLETE',
        control_path=str(control),out=str(out))
    usb=serial.Serial(port=None,baudrate=115200,timeout=.005,write_timeout=.2)
    usb.dtr,usb.rts,usb.port=True,False,'COM4'
    uart=serial.Serial(port=None,baudrate=115200,timeout=.005,write_timeout=.2)
    uart.dtr,uart.rts,uart.port=False,False,'COM8'
    parser=wire.StreamParser()
    status=wheel=None
    status_at=wheel_at=0
    previous_tick={}
    previous_wheel=None
    uart_buffer=''
    last_uart_rr=None
    changed_at=time.monotonic()
    started=time.monotonic()
    next_poll=next_progress=started
    sequence=0
    stage='baseline'
    baseline=None
    phase_id=0
    settling=None
    after_forward=None
    final=None
    fwd_turns=rev_turns=None
    print('EVIDENCE '+str(out),flush=True)
    print('CONTROL '+str(control),flush=True)
    print('Only GET_STATUS; VM off; keep RR still until READY_FORWARD.',flush=True)
    def stopped_counts():
        return wheel is not None and not any(wheel[5:9]) and time.monotonic()-changed_at>=1.0
    def save():
        summary.update(last_status=list(status) if status else None,last_wheel=list(wheel) if wheel else None,
                       last_uart_rr=last_uart_rr,host_crc_errors=parser.crc_errors,stage=stage)
        (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    def mark(label,turns,start_counts,end_counts):
        delta=[b-a for a,b in zip(start_counts,end_counts)]
        value=dict(label=label,turns_user_reported=turns,start_counts=start_counts,
                   end_counts=end_counts,delta=delta,RR_counts_per_turn=delta[3]/turns if turns else None,
                   wheel_sample_ms=wheel[0],uart_rr=last_uart_rr)
        summary['phases'].append(value)
        print('MEASUREMENT '+json.dumps(value,ensure_ascii=False),flush=True)
        return value
    try:
        with (out/'events.jsonl').open('w',encoding='utf-8') as events, \
             (out/'usb_rx.bin').open('wb') as usb_raw, \
             (out/'ch340.log').open('w',encoding='utf-8') as uart_log:
            def event(kind,**values):
                events.write(json.dumps(dict(host_monotonic=time.monotonic(),stage=stage,kind=kind,**values))+'\n')
            uart.open();usb.open()
            while time.monotonic()-started<args.max_seconds:
                now=time.monotonic()
                if now>=next_poll:
                    sequence=(sequence+1)&0xffff
                    frame=wire.encode_frame(wire.CMD_GET_STATUS,sequence)
                    if usb.write(frame)!=len(frame):raise RuntimeError('Partial GET_STATUS write')
                    event('host_tx',type=wire.CMD_GET_STATUS,seq=sequence,frame_hex=frame.hex())
                    next_poll=now+.5
                raw=usb.read(min(8192,max(1,usb.in_waiting)))
                if raw:usb_raw.write(raw)
                for f in parser.feed(raw):
                    if f.message_type not in (wire.MSG_STATUS,wire.MSG_WHEEL_STATE,wire.MSG_ACK):continue
                    event('frame',type=f.message_type,seq=f.sequence,payload_hex=f.payload.hex())
                    if f.message_type==wire.MSG_ACK:
                        if len(f.payload)!=4 or f.payload[0]!=wire.CMD_GET_STATUS or f.payload[1]!=wire.RESULT_OK:
                            raise RuntimeError('Unexpected/rejected ACK')
                    elif f.message_type==wire.MSG_STATUS:
                        status=wire.decode_status(f);status_at=time.monotonic()
                        if status is None:raise RuntimeError('Bad STATUS')
                        # Manual encoder movement is expected. Requests/output
                        # ownership must remain IDLE/zero/disabled throughout.
                        if status[5]!=wire.STATE_IDLE or status[6] or status[8] or any(status[10:12]) or status[9]&1:
                            raise RuntimeError('Not IDLE/no-fault/zero-request/output-disabled: '+str(status))
                        if any(status[17:20]):raise RuntimeError('Communication counters nonzero')
                        event('status',values=list(status))
                    else:
                        wheel=wire.decode_wheel_state(f);wheel_at=time.monotonic()
                        if wheel is None:raise RuntimeError('Bad WHEEL')
                        if previous_wheel is not None and wheel[1:5]!=previous_wheel[1:5]:changed_at=time.monotonic()
                        previous_wheel=wheel
                        event('wheel',values=list(wheel))
                    if f.message_type in (wire.MSG_STATUS,wire.MSG_WHEEL_STATE):
                        tick=struct.unpack_from('<I',f.payload)[0]
                        old=previous_tick.get(f.message_type)
                        if old is not None and ((tick-old)&0xffffffff)>5000:
                            raise RuntimeError('MCU timestamp discontinuity/reset')
                        previous_tick[f.message_type]=tick
                if parser.crc_errors:raise RuntimeError('CRC error')
                raw=uart.read(min(8192,max(1,uart.in_waiting)))
                if raw:
                    text=raw.decode('utf-8','replace');uart_log.write(text);uart_buffer+=text
                    while '\n' in uart_buffer:
                        line,uart_buffer=uart_buffer.split('\n',1)
                        if 'Firmware version' in line or 'Reset cause' in line:
                            raise RuntimeError('Unexpected startup/reset banner; no measurement resume')
                        m=re.search(r'\[(\d+)\].*ENC R:.*RR raw=(\d+) d=(-?\d+) total=(-?\d+)',line)
                        if m:last_uart_rr=dict(zip(('t_ms','raw','delta','total'),map(int,m.groups())))
                now=time.monotonic()
                if now-started>3 and (status is None or wheel is None or now-status_at>.5 or now-wheel_at>.5):
                    raise RuntimeError('Missing/stale STATUS/WHEEL')
                if status is not None and now-started>3 and status[9]&0x0150!=0x0150:
                    raise RuntimeError('Link/init/IWDG not ready')
                if stage=='baseline' and now-started>=3 and stopped_counts():
                    baseline=list(wheel[1:5]);stage='forward'
                    summary['baseline']=dict(counts=baseline,sample_ms=wheel[0],status=list(status),uart_rr=last_uart_rr)
                    save()
                    print('READY_FORWARD counts='+str(baseline)+'; RR baseline='+str(baseline[3]),flush=True)
                if control.exists():
                    try:cmd=json.loads(control.read_text(encoding='utf-8'))
                    except json.JSONDecodeError:cmd=None
                    if cmd and int(cmd['id'])>phase_id:
                        phase_id=int(cmd['id']);action=cmd['action'];event('operator',**cmd)
                        if action=='abort':raise RuntimeError('Operator abort')
                        if action=='forward_done' and stage=='forward':
                            fwd_turns=int(cmd['turns']);assert 1<=fwd_turns<=10
                            stage='settle_forward';settling=now
                        elif action=='reverse_done' and stage=='reverse':
                            rev_turns=int(cmd['turns']);assert 1<=rev_turns<=10
                            stage='settle_reverse';settling=now
                        else:raise RuntimeError('Unexpected phase command '+str(cmd))
                if stage=='settle_forward' and now-settling>=1.5 and stopped_counts():
                    after_forward=list(wheel[1:5]);measured=mark('manual_forward',fwd_turns,baseline,after_forward)
                    if measured['delta'][3]<=0 or any(measured['delta'][:3]):
                        stage='finished';summary['result']='FORWARD_DIRECTION_MAPPING_OR_NO_COUNTS_NEEDS_REVIEW';break
                    stage='reverse';save()
                    print('READY_REVERSE baseline='+str(after_forward)+'; keep wheel still until operator instructed.',flush=True)
                if stage=='settle_reverse' and now-settling>=1.5 and stopped_counts():
                    final=list(wheel[1:5]);mark('manual_reverse',rev_turns,after_forward,final)
                    summary['round_trip_delta']=[b-a for a,b in zip(baseline,final)]
                    stage='finished';summary['result']='MANUAL_COUNTS_CAPTURED_REVIEW_REQUIRED';break
                if settling and stage.startswith('settle_') and now-settling>20:
                    raise RuntimeError('Wheel not settled after operator completion')
                if now>=next_progress:
                    if wheel:print('LIVE '+stage+' t='+str(wheel[0])+' counts='+str(list(wheel[1:5]))+' cps='+str(list(wheel[5:9])),flush=True)
                    events.flush();uart_log.flush();usb_raw.flush();save();next_progress=now+20
            else:raise RuntimeError('Bounded manual measurement timeout')
    except (Exception,KeyboardInterrupt) as error:
        summary['errors'].append(str(error) or 'Interrupted');summary['result']='INCOMPLETE_OR_ANOMALY'
        print('STOPPED '+str(error),flush=True)
    finally:
        usb.close();uart.close();summary['ports_closed']=True;save()
        print('FINAL '+json.dumps(summary,ensure_ascii=False),flush=True)
    return int(bool(summary['errors']))


if __name__=='__main__':raise SystemExit(main())
