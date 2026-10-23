"""Build/test/freeze diagnostic candidate only. No device or Git mutation."""
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
ARCHIVE=ROOT/'build/firmware/0.18.23'
assert not ARCHIVE.exists(), 'Do not overwrite frozen artifacts'
VERSION='0.18.23-trace-candidate'
BUNDLE=Path('C:/Users/yang/AppData/Local/stm32cube/bundles')
ARM=BUNDLE/'gnu-tools-for-stm32/14.3.1+st.2/bin'
CMAKE=BUNDLE/'cmake/4.3.1+st.1/bin/cmake.exe'
ENV={**os.environ,'PATH':str(ARM)+os.pathsep+os.environ['PATH']}
LOGS=HERE/'logs'; LOGS.mkdir(exist_ok=True)
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def run(name,cmd):
    r=subprocess.run(list(map(str,cmd)),cwd=ROOT,env=ENV,capture_output=True,
                     text=True,encoding='utf-8',errors='replace')
    out=r.stdout+r.stderr
    (LOGS/(name+'.log')).write_text(out,encoding='utf-8')
    assert r.returncode==0,(name,out[-5000:])
    assert not re.search(r'\b(?:warning|error):',out,re.I),(name,out[-5000:])
    print(name+': PASS',flush=True)
    return out

baseline=json.loads((ROOT/'build/firmware/0.18.22/source_manifest.json').read_text())
changed={p for p,h in baseline['files'].items() if sha(ROOT/p)!=h}
expected={'Application/Chassis/motor_driver.c','Application/Chassis/motor_driver.h',
          'Application/Diagnostics/motion_trace.c','Application/Diagnostics/motion_trace.h',
          'Application/System/firmware_info.h',
          'Tools/tests/motion/stubs/main.h','Tools/tests/motion/test_motor_driver.c'}
# Some test files may not have been in the old production manifest.
assert changed==expected.intersection(baseline['files']),changed
added=['Tools/parse_motion_trace_v2.py','Tools/tests/motion/test_trace_capture.c',
       'Tools/tests/motion/test_trace_runtime.c','Tools/tests/motion/run_trace_tests.py']
sources=sorted(set(baseline['files']) | expected | set(added))
hashes={p:sha(ROOT/p) for p in sources}
old_artifacts={}
for version in ('0.18.21','0.18.22'):
    folder=ROOT/'build/firmware'/version
    old=json.loads((folder/'verification.json').read_text())['sha256']
    for ext,h in old.items():
        p=folder/f'rosDemo-{version}.{ext}'; assert sha(p)==h
        old_artifacts[str(p)]=h

run('trace_tests',['python','Tools/tests/motion/run_trace_tests.py'])
run('unified_tests',['python','Tools/tests/motion/run_unified_tests.py'])
usb=run('usb_tests',['powershell','-NoProfile','-ExecutionPolicy','Bypass',
                    '-File','Tools/tests/run_usb_tx_tests.ps1'])
assert len(re.findall(r'^PASS:',usb,re.M))==20
run('odometry_selftest',['python','Tools/wheel_odometry_test.py','--self-test'])
run('protocol_selftest',['python','Tools/usb_cdc_test.py','--self-test'])
build=run('clean_debug_build',[CMAKE,'--build','--preset','Debug','--clean-first'])
memory={n:int(v) for n,v in re.findall(r'^\s*(RAM|CCMRAM|FLASH):\s+(\d+)\s+B\b',build,re.M)}
assert memory['FLASH']<768*1024 and memory['RAM']<128*1024
image=ROOT/'build/Debug/rosDemo'
run('objcopy_bin',[ARM/'arm-none-eabi-objcopy.exe','-O','binary',image.with_suffix('.elf'),image.with_suffix('.bin')])
nm=run('symbols',[ARM/'arm-none-eabi-nm.exe','-S',image.with_suffix('.elf')])
ring=int(re.search(r'^\w+ (\w+) [bB] s_ring$',nm,re.M)[1],16)
assert ring==31744

data={}; upper=0; eof=False
for line in image.with_suffix('.hex').read_text().splitlines():
    assert not eof and line.startswith(':')
    r=bytes.fromhex(line[1:]); assert len(r)==r[0]+5 and sum(r)&255==0
    offset=int.from_bytes(r[1:3],'big'); kind=r[3]; payload=r[4:4+r[0]]
    if kind==4: assert len(payload)==2; upper=int.from_bytes(payload,'big')<<16
    elif kind==0:
        for i,v in enumerate(payload):
            address=upper+offset+i; assert address not in data; data[address]=v
    elif kind==1: assert r[0]==0; eof=True
    elif kind==5: assert r[0]==4
    else: raise AssertionError(kind)
binary=image.with_suffix('.bin').read_bytes()
assert eof and min(data)==0x08000000 and max(data)<0x080c0000
assert len(binary)==max(data)-min(data)+1==memory['FLASH']
assert all(binary[a-0x08000000]==v for a,v in data.items())
assert all(binary[a-0x08000000]==0 for a in range(min(data),max(data)+1) if a not in data)
sp,reset=struct.unpack_from('<II',binary)
assert sp==0x20020000 and reset&1 and min(data)<=reset&~1<=max(data)
assert VERSION.encode() in binary
frames={}
for p in (ROOT/'build/Debug').rglob('*.su'):
    for line in p.read_text(errors='replace').splitlines():
        f=line.split('\t')
        if len(f)>=3: frames[f[0].split(':')[-1]]=int(f[1])
chain=['StartMotorTask','MotionTrace_Record','MotionTrace_Snapshot','MotorDriver_GetDiagnostics']
subtotal=sum(frames[f] for f in chain)
assert subtotal<1200
assert hashes=={p:sha(ROOT/p) for p in sources}
assert all(sha(Path(p))==h for p,h in old_artifacts.items())
tests=json.loads((ROOT/'build/tests/trace_capture/results.json').read_text())
verification={'firmware':VERSION,'result':'DIAGNOSTIC_SOFTWARE_PASS_NOT_LOADED_ACCEPTANCE',
    'trace_tests':tests,'usb_groups':20,'normal_identity_updates':40000,
    'memory_bytes':memory,'trace_ring_bytes':ring,'build_warnings':0,'build_errors':0,
    'hex_bin_explicit_bytes_equal':True,'binary_gap_fill':0,
    'motor_snapshot_stack_chain':chain,'compiler_frames':{f:frames[f] for f in chain},
    'motor_application_stack_subtotal':subtotal,'motor_allocated_stack':2048,
    'complete_worst_case_stack_bound':False,'hardware_stack_watermark_measured':False,
    'prior_artifacts_unchanged':True,'control_parameters_unchanged_from_0_18_22':True,
    'safety_predicates_unchanged_from_0_18_22':True,
    'flashed':False,'serial_opened':False,'motion_sent':False,'hardware_tested':False,
    'new_commit_created':False,'pushed':False,'sha256':{}}
manifest={'firmware':VERSION,'git_base':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
    'uncommitted_candidate':True,'files':hashes,'changed_relative_to_0_18_22':sorted(changed),
    'added':added,'note':'Git base is not this uncommitted release identity; use source/artifact hashes.'}
ARCHIVE.mkdir()
for ext in ('hex','bin','elf','map'):
    dest=ARCHIVE/f'rosDemo-0.18.23.{ext}'; shutil.copy2(image.with_suffix('.'+ext),dest)
    verification['sha256'][ext]=sha(dest)
for dest in (HERE,ARCHIVE):
    (dest/'verification.json').write_text(json.dumps(verification,indent=2)+'\n',encoding='utf-8')
    (dest/'source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
for folder in ('trace_capture','unified_pivot_logs'):
    target=LOGS/folder; target.mkdir(exist_ok=True)
    for p in (ROOT/'build/tests'/folder).glob('*'):
        if p.suffix in ('.json','.log'): shutil.copy2(p,target/p.name)
for name in sources:
    dest=HERE/'source_snapshot'/name; dest.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(ROOT/name,dest)
print(json.dumps(verification,indent=2),flush=True)
