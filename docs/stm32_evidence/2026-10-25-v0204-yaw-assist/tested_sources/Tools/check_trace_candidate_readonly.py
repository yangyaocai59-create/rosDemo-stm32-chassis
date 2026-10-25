"""Read-only board preflight. GET_STATUS only; no reset, motion or fault clear."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import time
import serial
from debug_uart_capture import DebugUartCapture
import usb_cdc_test as wire

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', default='COM4')
    ap.add_argument('--debug-port', default='COM8')
    ap.add_argument('--seconds', type=float, default=10)
    args=ap.parse_args()
    if not 3 <= args.seconds <= 30: ap.error('seconds must be 3..30')
    folder=Path(__file__).resolve().parents[1]/'build/hardware_tests'/('trace23_readonly_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    folder.mkdir(parents=True,exist_ok=False)
    capture=DebugUartCapture(args.debug_port,folder)
    usb=None
    result={'motion_sent':False,'reset_requested':False,'fault_cleared':False,
            'folder':str(folder),'frames':[],'error':None}
    try:
        capture.start()
        usb=serial.Serial(port=None,baudrate=115200,timeout=.02,write_timeout=.2,
                          dsrdtr=False,rtscts=False)
        usb.dtr=True; usb.rts=False; usb.port=args.port; usb.open()
        parser=wire.StreamParser(); sequence=0; next_query=0
        end=time.monotonic()+args.seconds
        while time.monotonic()<end:
            capture.check()
            if time.monotonic()>=next_query:
                sequence+=1; usb.write(wire.encode_frame(wire.CMD_GET_STATUS,sequence))
                next_query=time.monotonic()+.8
            data=usb.read(8192)
            if data:
                for frame in parser.feed(data):
                    decoded=wire.decode_status(frame)
                    kind='STATUS'
                    if decoded is None: decoded=wire.decode_wheel_state(frame); kind='WHEEL'
                    if decoded is not None: result['frames'].append({'kind':kind,'value':decoded})
    except Exception as error:
        result['error']=str(error)
    finally:
        if usb is not None: usb.close()
        capture.close()
        log=(folder/'ch340.log').read_text(encoding='utf-8') if (folder/'ch340.log').exists() else ''
        result['versions']=re.findall(r'Firmware version = ([^\r\n]+)',log)
        result['timing_lines']=[line for line in log.splitlines() if 'CTRL TIMING:' in line]
        result['reset_lines']=[line for line in log.splitlines() if 'Reset cause' in line]
        result['capture_error']=capture.error
        (folder/'result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    status=[f['value'] for f in result['frames'] if f['kind']=='STATUS']
    wheels=[f['value'] for f in result['frames'] if f['kind']=='WHEEL']
    print(json.dumps({k:v for k,v in result.items() if k!='frames'},indent=2))
    print('STATUS first/last:',status[:1],status[-1:])
    print('WHEEL first/last:',wheels[:1],wheels[-1:])
    return 1 if result['error'] or result['capture_error'] or not status else 0

if __name__=='__main__': raise SystemExit(main())
