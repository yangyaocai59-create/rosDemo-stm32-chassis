"""Regress/clean-build/freeze .22. No ports, programming, motion or git writes."""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess

HERE=Path(__file__).resolve().parent
ROOT=next(p for p in HERE.parents if (p/'Application').is_dir())
ARCHIVE=ROOT/'build/firmware/0.18.22'
assert not ARCHIVE.exists(), 'Never overwrite a frozen release directory'
VERSION='0.18.22-unified-pivot-candidate'
BUNDLE=Path('C:/Users/yang/AppData/Local/stm32cube/bundles')
BINS=[BUNDLE/'gnu-tools-for-stm32/14.3.1+st.2/bin',
      BUNDLE/'cmake/4.3.1+st.1/bin', BUNDLE/'ninja/1.13.2+st.1/bin',
      Path('D:/x86_64-15.2.0-release-posix-seh-ucrt-rt_v13-rev1/mingw64/bin')]
ENV={**os.environ,'PATH':os.pathsep.join(map(str,BINS))+os.pathsep+os.environ['PATH']}
LOGS=HERE/'logs'
LOGS.mkdir(exist_ok=True)

def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def run(name,command):
    r=subprocess.run(list(map(str,command)),cwd=ROOT,env=ENV,capture_output=True,
                     text=True,encoding='utf-8',errors='replace')
    output=r.stdout+r.stderr
    (LOGS/(name+'.log')).write_text(output,encoding='utf-8')
    assert r.returncode==0,(name,output[-6000:])
    assert not re.search(r'\b(?:warning|error):',output,re.I),(name,output[-6000:])
    print(name+': PASS',flush=True)
    return output

baseline=json.loads((ROOT/'build/firmware/0.18.21/source_manifest.json').read_text())
expected={
 'Application/Chassis/chassis_controller.c',
 'Application/Chassis/wheel_speed_controller.c',
 'Application/Chassis/wheel_speed_controller.h',
 'Application/Chassis/wheel_speed_control_config.h',
 'Application/Safety/fault_manager.c','Application/System/firmware_info.h'}
changed={p for p,h in baseline['files'].items() if sha(ROOT/p)!=h}
assert changed==expected,changed
frozen=ROOT/'docs/stm32_evidence/2026-10-22-unified-pivot-v1/baseline_0.18.21'
for p in frozen.iterdir():
    matches=[h for n,h in baseline['files'].items() if Path(n).name==p.name]
    assert len(matches)==1 and sha(p)==matches[0],p.name
old_hashes=json.loads((ROOT/'build/firmware/0.18.21/verification.json').read_text())['sha256']
def check_old():
    for ext,h in old_hashes.items():
        assert sha(ROOT/f'build/firmware/0.18.21/rosDemo-0.18.21.{ext}')==h
check_old()
added=['Application/Chassis/unified_pivot_control.h']+[
 'Tools/tests/motion/'+p for p in ('test_unified_pivot.c','test_unified_runtime.c',
                                 'unified_normal_compare.c','run_unified_tests.py')]
sources=sorted(set(baseline['files'])|set(added))
hashes={p:sha(ROOT/p) for p in sources}
run('unified_regression',['python','Tools/tests/motion/run_unified_tests.py'])
usb=run('usb_regression',['powershell','-NoProfile','-ExecutionPolicy','Bypass',
                         '-File','Tools/tests/run_usb_tx_tests.ps1'])
assert len(re.findall(r'^PASS:',usb,re.M))==20
imu=run('imu_python_regression',['python','-m','unittest','discover',
                                '-s','Tools/tests','-p','test_imu*.py','-v'])
assert re.search(r'^OK$',imu,re.M)
run('protocol_selftest',['python','Tools/usb_cdc_test.py','--self-test'])
build=run('clean_debug_build',[BINS[1]/'cmake.exe','--build','--preset','Debug','--clean-first'])
assert '[95/95]' in build
memory={n:int(v) for n,v in re.findall(r'^\s*(RAM|CCMRAM|FLASH):\s+(\d+)\s+B\b',build,re.M)}
assert memory['FLASH']<768*1024 and memory['RAM']<128*1024
image=ROOT/'build/Debug/rosDemo'
run('objcopy_bin',[BINS[0]/'arm-none-eabi-objcopy.exe','-O','binary',
                   image.with_suffix('.elf'),image.with_suffix('.bin')])
run('elf_sections',[BINS[0]/'arm-none-eabi-objdump.exe','-h',image.with_suffix('.elf')])
data={}; upper=0; eof=False; records=0
for line in image.with_suffix('.hex').read_text().splitlines():
    assert not eof and line.startswith(':')
    record=bytes.fromhex(line[1:]); records+=1
    assert len(record)==record[0]+5 and sum(record)&255==0
    length,offset,kind=record[0],int.from_bytes(record[1:3],'big'),record[3]
    payload=record[4:4+length]
    if kind==4:
        assert length==2 and offset==0
        upper=int.from_bytes(payload,'big')<<16
    elif kind==0:
        for i,value in enumerate(payload):
            address=upper+offset+i
            assert address not in data
            data[address]=value
    elif kind==1:
        assert length==0 and offset==0
        eof=True
    elif kind==5: assert length==4
    else: raise AssertionError(kind)
binary=image.with_suffix('.bin').read_bytes()
assert eof and min(data)==0x08000000 and max(data)<0x080C0000
assert len(binary)==max(data)-min(data)+1==memory['FLASH']
assert all(binary[a-0x08000000]==v for a,v in data.items())
gaps=[a for a in range(min(data),max(data)+1) if a not in data]
assert all(binary[a-0x08000000]==0 for a in gaps)
sp,reset=struct.unpack_from('<II',binary)
assert sp==0x20020000 and reset&1 and min(data)<=reset&~1<=max(data)
assert VERSION.encode() in binary and b'0.18.21-rr-recovery-exit' not in binary

# Compiler-frame evidence only, not a libc/exception worst-case stack proof.
frames={}
for p in (ROOT/'build/Debug').rglob('*.su'):
    for line in p.read_text(errors='replace').splitlines():
        fields=line.split('\t')
        if len(fields)>=3: frames[fields[0].split(':')[-1]]=int(fields[1])
paths={
 'motor_pi':(['StartMotorTask','ChassisRuntime_Process','ChassisController_Process',
               'WheelSpeedController_UpdateWithMode','UnifiedPivot_Update'],2048),
 'safety_guard':(['StartSafetyTask','FaultManager_ObserveMotion',
                 'WheelSpeedController_GetFrontPivotState',
                 'WheelSpeedController_GetPivotLoadFeedforward'],1024)}
stack={}
for name,(chain,allocated) in paths.items():
    assert all(f in frames for f in chain),chain
    subtotal=sum(frames[f] for f in chain)
    assert subtotal<allocated,(name,subtotal)
    stack[name]={'functions':chain,'compiler_frame_bytes':[frames[f] for f in chain],
                 'application_subtotal_bytes':subtotal,'allocated_bytes':allocated,
                 'complete_worst_case_bound':False}
assert hashes=={p:sha(ROOT/p) for p in sources}
check_old()
results=json.loads((ROOT/'build/tests/unified_pivot_logs/results.json').read_text())
assert results['pi_strict']['pass_groups']==5 and results['runtime_strict']['pass_groups']==5
head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
manifest={'firmware':VERSION,'git_base':head,'uncommitted_candidate':True,
 'files':hashes,'changed_relative_to_0.18.21':sorted(changed),'added':added,
 'baseline_copy_sha256':{p.name:sha(p) for p in frozen.iterdir()},
 'note':'Base commit is not release identity. Source hashes and frozen artifacts identify this uncommitted candidate.'}
verification={'result':'SCOPED_SOFTWARE_PASS_NOT_HARDWARE_ACCEPTED','firmware':VERSION,
 'candidate_pi_groups':5,'candidate_runtime_groups':5,
 'candidate_modes':['strict','O2','fanalyzer'],'driver_groups':results['driver']['pass_groups'],
 'legacy_reference_only_groups':results['legacy_core']['pass_groups'],
 'normal_byte_identical_updates':40000,'usb_groups':20,
 'imu_python_tests':int(re.search(r'Ran (\d+) tests',imu)[1]),
 'strict_extended_native_imu_suite_rerun':False,
 'native_imu_limitation':'Unchanged baseline disabled-calibration/probe unused ImuService_AbsI32 diagnostic; no claim that full strict extended native suite passes.',
 'build':'Debug clean GNU STM32 14.3.1+st.2, 95 steps',
 'build_warnings':0,'build_errors':0,'memory_bytes':memory,
 'hex_records':records,'hex_bin_explicit_data_equal':True,'binary_gap_bytes':len(gaps),
 'gap_fill':'BIN 0x00; HEX leaves alignment gaps unwritten',
 'bytes':len(binary),'highest':hex(max(data)),'stack_pointer':hex(sp),'reset_vector':hex(reset),
 'application_stack_subtotals':stack,'stack_hardware_watermark_measured':False,
 'source_items':len(sources),'old_0.18.21_artifacts_unchanged':True,
 'loaded_ff_calibrated':False,'flashed':False,'hardware_tested':False,
 'serial_opened':False,'motion_sent':False,'fault_cleared':False,
 'new_commit_created':False,'pushed':False}
ARCHIVE.mkdir()
verification['sha256']={}
for ext in ('hex','bin','elf','map'):
    dest=ARCHIVE/f'rosDemo-0.18.22.{ext}'
    shutil.copy2(image.with_suffix('.'+ext),dest)
    verification['sha256'][ext]=sha(dest)
for folder in (HERE,ARCHIVE):
    (folder/'source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    (folder/'verification.json').write_text(json.dumps(verification,indent=2)+'\n',encoding='utf-8')
shutil.copy2(ROOT/'build/tests/unified_pivot_logs/results.json',HERE/'test_results.json')
for label in ('strict','optimized','analyzer'):
    for name in ('pi','runtime'):
        shutil.copy2(ROOT/f'build/tests/unified_pivot_logs/{name}_{label}.log',LOGS)
snapshot=HERE/'source_snapshot'
for name in sorted(changed)+added:
    dest=snapshot/name
    dest.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(ROOT/name,dest)
print(json.dumps(verification,indent=2),flush=True)
print('ARTIFACT '+str(ARCHIVE),flush=True)
