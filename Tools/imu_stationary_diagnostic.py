#!/usr/bin/env python3
"""Stationary IMU diagnostic; optional explicitly authorized one-shot calibration."""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timedelta, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import statistics
import time

import usb_cdc_test as wire
from debug_uart_capture import DebugUartCapture


RAW_TO_MRAD = math.pi / 180.0 / 65.5 * 1000.0


def describe(values):
    if not values:
        return None
    return dict(n=len(values), mean=statistics.mean(values),
                std=statistics.pstdev(values), minimum=min(values), maximum=max(values))


def analyze_uart(text, sample_min=None, sample_max=None):
    pairs = []
    pending = None
    restores = re.findall(r"IMU gyro bias restored from Flash: raw=\[([^]]+)\] calibration_temp=([^\r\n]+)", text)
    precise_restores = re.findall(r"IMU gyro bias restored from Flash: milli_raw=\[([^]]+)\] calibration_temp=([^\r\n]+)", text)
    precise_saves = re.findall(r"IMU STATIC precise bias milli_raw=\[([^]]+)\] scale=1000", text)
    for line in text.splitlines():
        raw = re.search(r"\[(\d+)\].*IMU RAW: a=\[([^]]+)\] temp=(-?\d+) g=\[([^]]+)\] samples=(\d+) errors=(\d+)", line)
        if raw:
            pending = dict(log_ms=int(raw[1]), accel=[int(x) for x in raw[2].split(',')],
                           temperature_raw=int(raw[3]), gyro=[int(x) for x in raw[4].split(',')],
                           sample_count=int(raw[5]), errors=int(raw[6]))
            continue
        si = re.search(r"\[(\d+)\].*IMU SI:.*w_mrads=\[([^]]+)\].*temp_mC=(-?\d+) bias=(\d+)", line)
        if si and pending is not None:
            if 0 <= int(si[1])-pending['log_ms'] < 500:
                pairs.append(dict(**pending, si_log_ms=int(si[1]),
                                  si_w_mrad=[int(x) for x in si[2].split(',')],
                                  temperature_c=int(si[3])/1000.0, bias_valid=bool(int(si[4]))))
            pending = None
    # Never mix the pre-calibration RAW/SI bias with the post-calibration phase.
    pairs = [p for p in pairs if (sample_min is None or p['sample_count'] >= sample_min)
             and (sample_max is None or p['sample_count'] <= sample_max)]
    candidates = [set(range(-1000,1001)) for _ in range(3)]
    for pair in pairs:
        if not pair['bias_valid']:
            continue
        for axis in range(3):
            candidates[axis] = {bias for bias in candidates[axis]
                                if math.trunc((pair['gyro'][axis]-bias)*RAW_TO_MRAD)
                                == pair['si_w_mrad'][axis]}
    calibrated_pairs = [p for p in pairs if p['bias_valid']]
    bias = None
    bias_milli_raw = None
    method = None
    if precise_restores and not re.search(r'IMU STATIC result=PASS_BIAS_APPLIED', text):
        bias_milli_raw = [int(x) for x in precise_restores[-1][0].split(',')]
        bias = [x/1000 for x in bias_milli_raw]
        method = 'explicit startup milli-RAW restore log'
    elif precise_saves and re.search(r'IMU STATIC result=PASS_BIAS_APPLIED', text):
        bias_milli_raw = [int(x) for x in precise_saves[-1].split(',')]
        bias = [x/1000 for x in bias_milli_raw]
        method = 'explicit milli-RAW calibration result log; persistence checked separately'
    elif restores and not re.search(r'IMU STATIC gyro bias applied and persisted', text):
        bias = [int(x) for x in restores[-1][0].split(',')]
        method = 'explicit startup restore log'
    elif calibrated_pairs and all(len(c)==1 for c in candidates):
        bias = [next(iter(c)) for c in candidates]
        method = 'inferred intersection of RAW/SI truncation constraints, not a Flash read'
    result = dict(pairs=len(pairs), calibrated_pairs=len(calibrated_pairs),
                  observed_restore_logs=restores, bias_raw=bias, bias_evidence=method,
                  observed_precise_restore_logs=precise_restores, bias_milli_raw=bias_milli_raw,
                  bias_candidates=[sorted(c) if len(c)<30 else 'underdetermined' for c in candidates],
                  gyro_raw=[describe([p['gyro'][axis] for p in pairs]) for axis in range(3)],
                  temperature_c=describe([p['temperature_c'] for p in pairs]),
                  flash_save_lines=[l for l in text.splitlines() if 'IMU bias Flash saved:' in l],
                  calibration_start_lines=[l for l in text.splitlines() if 'IMU STATIC started' in l])
    if bias is not None and calibrated_pairs:
        # Existing UART is a sparse 1Hz snapshot, NOT per-frame USB raw data.
        residual = [p['gyro'][2]-bias[2] for p in calibrated_pairs]
        result['sparse_z_residual_raw'] = describe(residual)
        result['sparse_z_unquantized_equivalent_deg_min'] = statistics.mean(residual)*60/65.5
        result['raw_si_arithmetic_mismatches'] = sum(
            math.trunc((p['gyro'][axis]-bias[axis])*RAW_TO_MRAD)!=p['si_w_mrad'][axis]
            for p in calibrated_pairs for axis in range(3))
    return result, pairs


def analyze_usb(samples):
    if len(samples)<2:
        return dict(samples=len(samples), result='insufficient data')
    intervals=[(b[0]-a[0]) & 0xFFFFFFFF for a,b in zip(samples,samples[1:])]
    span=sum(intervals)/1000.0
    integral=sum(dt/1000.0*(a[7]+b[7])/2000.0
                 for dt,a,b in zip(intervals,samples,samples[1:]))
    windows=[]
    elapsed=0
    bins={}
    for i,s in enumerate(samples):
        if i: elapsed+=intervals[i-1]
        bins.setdefault(elapsed//10000,[]).append(s)
    for index,rows in sorted(bins.items()):
        windows.append(dict(start_s=index*10, n=len(rows),
                            mean_wz_mrad_s=statistics.mean(s[7] for s in rows),
                            equivalent_deg_min=statistics.mean(s[7] for s in rows)*.001*180/math.pi*60,
                            temperature_c=describe([s[8]/1000 for s in rows])))
    return dict(samples=len(samples), source_span_s=span,
                source_rate_hz=(len(samples)-1)/span if span else None,
                interval_min_ms=min(intervals), interval_max_ms=max(intervals),
                angular_velocity_mrad_s=[describe([s[axis] for s in samples]) for axis in (5,6,7)],
                temperature_c=describe([s[8]/1000 for s in samples]),
                yaw_integral_deg=integral*180/math.pi,
                yaw_rate_deg_min=integral*180/math.pi*60/span if span else None,
                wz_histogram_mrad_s=dict(sorted(Counter(s[7] for s in samples).items())),
                flags=sorted(set(s[12] for s in samples)),
                read_error_counts=sorted(set(s[9] for s in samples)),
                consecutive_error_counts=sorted(set(s[10] for s in samples)), windows=windows)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port',required=True)
    ap.add_argument('--debug-port',required=True)
    ap.add_argument('--duration',type=float,default=120)
    ap.add_argument('--stationary-confirmed',action='store_true')
    ap.add_argument('--vm-off-confirmed',action='store_true')
    ap.add_argument('--calibrate-once',action='store_true',
                    help='explicit authorization: send one calibration; append Flash record; then capture')
    args=ap.parse_args()
    if not args.stationary_confirmed or not args.vm_off_confirmed or not 120<=args.duration<=600:
        ap.error('require stationary/VM-off confirmation and duration 120..600 seconds')
    import serial
    from serial.tools import list_ports
    devices={p.device.casefold():p for p in list_ports.comports()}
    cdc=devices.get(args.port.casefold()); debug=devices.get(args.debug_port.casefold())
    if cdc is None or (cdc.vid,cdc.pid)!=(0x0483,0x5740):
        ap.error('CDC port is not an enumerated STM32 0483:5740 device')
    if debug is None or (debug.vid,debug.pid)!=(0x1A86,0x7523) or debug.device==cdc.device:
        ap.error('debug port is not a separate CH340 1A86:7523 device')
    stamp=datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')
    out=Path(__file__).resolve().parents[1]/'build'/'hardware_tests'/('imu_stationary_'+stamp)
    out.mkdir(parents=True,exist_ok=False)
    summary=dict(port=cdc.device,debug_port=debug.device,usb_serial=cdc.serial_number,
                 user_reports_stationary=True,user_reports_vm_off=True,
                 host_tx_allowed=['CMD_GET_STATUS 0x04'],motion_commands_sent=0,
                 calibration_commands_sent=0,debug_tx_bytes=0,firmware_modified=False,
                 duration_requested_s=args.duration, errors=[],get_status_queries=0,
                 wheel_nonzero_cps_frames=0,imu_invalid_frames=0,imu_duplicates=0,
                 script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    if args.calibrate_once:
        summary['host_tx_allowed'].append('CMD_CALIBRATE_IMU 0x05 exactly once')
    summary['calibration'] = dict(requested=args.calibrate_once, ack_ok=False, active_seen=False)
    uart=DebugUartCapture(debug.device,out)
    parser=wire.StreamParser();samples=[];statuses=[];start=None;last_imu=None
    port=None;sequence=1;pending={};first_wheel=None;last_wheel=None
    begun=time.monotonic();last_status=None;next_query=begun;progress=30
    calibration_sequence=None;calibration_sent_at=None;calibration_completed_at=None
    result=1
    print('Evidence directory: '+str(out),flush=True)
    print('GET_STATUS plus one authorized calibration.' if args.calibrate_once else 'GET_STATUS only.',flush=True)
    print('No CH340 writes, STOP or velocity commands; no flashing.',flush=True)
    try:
        uart.start()
        port=serial.Serial(port=None,baudrate=115200,timeout=.01,write_timeout=.2,
                           rtscts=False,dsrdtr=False)
        port.dtr=True;port.rts=False;port.port=cdc.device;port.open()
        with (out/'usb_rx.bin').open('wb') as binary,(out/'usb_frames.jsonl').open('w',encoding='utf8') as frames_log:
            while True:
                now=time.monotonic()
                if start is not None and now-start>=args.duration:
                    summary['duration_observed_s']=now-start
                    break
                uart.check()
                if now>=next_query:
                    query=wire.encode_frame(wire.CMD_GET_STATUS,sequence)
                    if port.write(query)!=len(query):raise RuntimeError('partial status query write')
                    frames_log.write(json.dumps(dict(host_monotonic=now,kind='tx',message_type=4,sequence=sequence,hex=query.hex()))+'\n')
                    pending[sequence]=now;sequence=(sequence+1)&0xFFFF
                    summary['get_status_queries']+=1;next_query=now+.8
                data=port.read(8192)
                if data:
                    binary.write(data)
                    for frame in parser.feed(data):
                        frames_log.write(json.dumps(dict(host_monotonic=time.monotonic(),kind='rx',message_type=frame.message_type,sequence=frame.sequence,payload_hex=frame.payload.hex()))+'\n')
                        if frame.message_type==wire.MSG_ACK and frame.sequence in pending:
                            if len(frame.payload)!=4 or frame.payload[0]!=4 or frame.payload[1]!=0:
                                raise RuntimeError('GET_STATUS rejected or malformed')
                            del pending[frame.sequence]
                        if frame.message_type==wire.MSG_ACK and frame.sequence==calibration_sequence:
                            if len(frame.payload)!=4 or frame.payload[0]!=wire.CMD_CALIBRATE_IMU or frame.payload[1]!=0:
                                raise RuntimeError('calibration rejected; no automatic retry')
                            summary['calibration']['ack_ok']=True
                        state=wire.decode_status(frame)
                        if state is not None:
                            last_status=time.monotonic();statuses.append(state)
                            if state[5]!=wire.STATE_IDLE or state[6] or state[8] or state[10:12]!=(0,0):
                                raise RuntimeError('chassis is not fault-free IDLE with zero request')
                        wheel=wire.decode_wheel_state(frame)
                        if wheel is not None:
                            if first_wheel is None:first_wheel=wheel
                            last_wheel=wheel
                            if any(wheel[5:9]):summary['wheel_nonzero_cps_frames']+=1
                        imu=wire.decode_imu_state(frame)
                        if imu is None:continue
                        last_imu=time.monotonic()
                        if imu[11]!=2 or not imu[12]&1:
                            summary['imu_invalid_frames']+=1
                            if start is not None:raise RuntimeError('IMU lost READY/valid state')
                            continue
                        if args.calibrate_once and start is None:
                            if calibration_sequence is None:
                                if not statuses or first_wheel is None:continue
                                if summary['wheel_nonzero_cps_frames']:
                                    raise RuntimeError('nonzero wheel speed before calibration')
                                if imu[12]&8:raise RuntimeError('calibration already active; will not send another')
                                calibration_sequence=sequence;sequence=(sequence+1)&0xFFFF
                                query=wire.encode_frame(wire.CMD_CALIBRATE_IMU,calibration_sequence)
                                # Count an attempted send before write: never retry after uncertain delivery.
                                summary['calibration_commands_sent']=1
                                calibration_sent_at=time.monotonic()
                                summary['calibration']['before_imu']=list(imu)
                                frames_log.write(json.dumps(dict(host_monotonic=calibration_sent_at,kind='tx',message_type=5,sequence=calibration_sequence,hex=query.hex()))+'\n')
                                frames_log.flush()
                                if port.write(query)!=len(query):raise RuntimeError('partial calibration write; will not retry')
                                print('One calibration requested; keep completely still.',flush=True)
                                continue
                            if imu[12]&8:
                                summary['calibration']['active_seen']=True
                                continue
                            if not summary['calibration']['ack_ok'] or not summary['calibration']['active_seen']:continue
                            if not imu[12]&0x10:continue
                            if imu[12]&0xC0 or (imu[12]&0x37)!=0x37:
                                raise RuntimeError('calibration failed; keep evidence, no automatic retry')
                            if calibration_completed_at is None:
                                calibration_completed_at=time.monotonic()
                                summary['calibration']['completion_imu']=list(imu)
                                print('Calibration PASS; waiting for Flash save log.',flush=True)
                            if time.monotonic()-calibration_completed_at<.5:continue
                            log=(out/'ch340_raw.bin').read_bytes().decode('utf8',errors='replace')
                            if 'IMU STATIC gyro bias applied and persisted for future boots' not in log:
                                if time.monotonic()-calibration_completed_at>2:
                                    raise RuntimeError('calibration passed but Flash persistence not confirmed')
                                continue
                            summary['calibration']['flash_save_confirmed']=True
                        if imu[12]&8:raise RuntimeError('unexpected calibration is active')
                        if not imu[12]&2:raise RuntimeError('IMU bias is not restored; no automatic calibration permitted')
                        if start is None:
                            if not statuses:continue
                            start=time.monotonic()
                            print('CAPTURE STARTED: keep chassis untouched for 120 seconds.',flush=True)
                        if samples and imu[1]==samples[-1][1]:
                            summary['imu_duplicates']+=1;continue
                        if samples and ((imu[0]-samples[-1][0])&0xFFFFFFFF)>1000:
                            raise RuntimeError('IMU time discontinuity; do not combine separate observations')
                        samples.append(imu)
                if parser.crc_errors:raise RuntimeError('USB CRC/version error')
                now=time.monotonic()
                if pending and now-min(pending.values())>2:raise RuntimeError('status query ACK timeout')
                if calibration_sent_at is not None and not summary['calibration']['ack_ok'] and now-calibration_sent_at>2:
                    raise RuntimeError('calibration ACK timeout; will not retry')
                if start is None and now-begun>(15 if args.calibrate_once else 8):
                    raise RuntimeError('no initial READY telemetry or calibration completion timeout')
                if start is not None:
                    if now-last_imu>1 or last_status is None or now-last_status>1:
                        raise RuntimeError('telemetry stale')
                    if now-start>=progress:
                        tail=samples[-1000:]
                        print(f"{now-start:.1f}s samples={len(samples)} wz_mean_last10s={statistics.mean(s[7] for s in tail):+.4f}mrad/s temp={tail[-1][8]/1000:.3f}C errors={tail[-1][9:11]}",flush=True)
                        frames_log.flush();binary.flush();progress+=30
        if len(samples)<args.duration*80:raise RuntimeError('insufficient unique IMU samples')
        summary['capture_result']='complete'
        result=0
    except (Exception,KeyboardInterrupt) as error:
        summary['errors'].append(str(error) or 'operator interrupted')
        summary['capture_result']='incomplete'
        print('Capture stopped: '+summary['errors'][-1],flush=True)
    finally:
        if port is not None:
            try:port.close()
            except Exception as error:summary['errors'].append('CDC close: '+str(error))
        uart.close()
        if uart.error:summary['errors'].append('UART: '+uart.error)
        summary['uart_rx_bytes']=uart.bytes_received
        summary['parser_crc_errors']=parser.crc_errors
        summary['usb']=analyze_usb(samples)
        if statuses:
            summary['first_status']=list(statuses[0]);summary['last_status']=list(statuses[-1])
            summary['status_flags']=sorted(set(s[9] for s in statuses))
        if first_wheel is not None and last_wheel is not None:
            summary['wheel_count_change']=[b-a for a,b in zip(first_wheel[1:5],last_wheel[1:5])]
        uart_file=out/'ch340.log'
        if uart_file.exists():
            summary['uart'],pairs=analyze_uart(uart_file.read_text(encoding='utf8'),
                samples[0][1] if samples else None, samples[-1][1] if samples else None)
            (out/'uart_pairs.json').write_text(json.dumps(pairs,indent=2)+'\n',encoding='utf8')
        (out/'imu_samples.jsonl').write_text(''.join(json.dumps(dict(sample=list(s)))+'\n' for s in samples),encoding='utf8')
        (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf8')
        print(json.dumps(summary,indent=2),flush=True)
    return result


if __name__=='__main__':
    raise SystemExit(main())
