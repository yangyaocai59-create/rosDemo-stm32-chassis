"""Offline build/test/archive only. Never connects to hardware or modifies Git."""
import hashlib,json,os,re,shutil,struct,subprocess
from pathlib import Path
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'Application').is_dir())
HERE=Path(__file__).resolve().parent
ARCHIVE=ROOT/'build/firmware/0.18.24'
assert not ARCHIVE.exists(),'Frozen directory already exists'
VERSION='0.18.24-encoder-diag-candidate'
BUNDLE=Path('C:/Users/yang/AppData/Local/stm32cube/bundles')
ARM=BUNDLE/'gnu-tools-for-stm32/14.3.1+st.2/bin'
CMAKE=BUNDLE/'cmake/4.3.1+st.1/bin/cmake.exe'
ENV={**os.environ,'PATH':str(ARM)+os.pathsep+os.environ['PATH']}
LOGS=HERE/'logs';LOGS.mkdir(exist_ok=True)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def run(name,cmd):
 r=subprocess.run(list(map(str,cmd)),cwd=ROOT,env=ENV,capture_output=True,text=True,encoding='utf-8',errors='replace')
 out=r.stdout+r.stderr;(LOGS/(name+'.log')).write_text(out,encoding='utf-8')
 assert r.returncode==0,(name,out[-5000:])
 assert not re.search(r'\b(?:warning|error):',out,re.I),(name,out[-5000:])
 print(name+': PASS',flush=True);return out
baseline=json.loads((ROOT/'build/firmware/0.18.23/source_manifest.json').read_text())
changed={p for p,h in baseline['files'].items() if sha(ROOT/p)!=h}
expected={'CMakeLists.txt','Application/Diagnostics/motion_trace.c','Application/System/firmware_info.h',
 'Tools/parse_motion_trace_v2.py','Tools/tests/motion/test_trace_capture.c',
 'Tools/tests/motion/test_trace_runtime.c','Tools/tests/motion/run_trace_tests.py'}
assert changed==expected,changed
added=['Application/Diagnostics/encoder_diagnostics.c','Application/Diagnostics/encoder_diagnostics.h',
 'Tools/tests/encoder_diag/stubs/board_config.h','Tools/tests/encoder_diag/test_encoder_diag.c',
 'Tools/tests/encoder_diag/test_encoder_acquisition.c']
sources=sorted(set(baseline['files'])|set(added)); hashes={p:sha(ROOT/p) for p in sources}
old_artifacts={}
for version in ('0.18.21','0.18.22','0.18.23'):
 old=json.loads((ROOT/f'build/firmware/{version}/verification.json').read_text())['sha256']
 for ext,h in old.items():
  p=ROOT/f'build/firmware/{version}/rosDemo-{version}.{ext}';assert sha(p)==h;old_artifacts[str(p)]=h
cc=shutil.which('gcc');assert cc
flags=['-std=c11','-Wall','-Wextra','-Werror','-ITools/tests/encoder_diag/stubs',
 '-ITools/tests/imu_spi_probe/stubs','-IApplication/Chassis','-IApplication/Diagnostics']
for mode,extra in [('strict',[]),('O2',['-O2']),('analyzer',['-fanalyzer'])]:
 for test in ('encoder_diag','encoder_acquisition'):
  exe=ROOT/f'build/tests/{test}_{mode}.exe'
  run(test+'_'+mode+'_build',[cc]+flags+extra+[f'Tools/tests/encoder_diag/test_{test}.c','-o',exe])
  run(test+'_'+mode,[exe])
run('trace_tests',['python','Tools/tests/motion/run_trace_tests.py'])
run('control_regression',['python','Tools/tests/motion/run_unified_tests.py'])
usb=run('usb_tests',['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File','Tools/tests/run_usb_tx_tests.ps1'])
assert len(re.findall(r'^PASS:',usb,re.M))==20
run('protocol_selftest',['python','Tools/usb_cdc_test.py','--self-test'])
run('odometry_selftest',['python','Tools/wheel_odometry_test.py','--self-test'])
build=run('clean_build',[CMAKE,'--build','--preset','Debug','--clean-first'])
memory={k:int(v) for k,v in re.findall(r'^\s*(RAM|CCMRAM|FLASH):\s+(\d+)\s+B\b',build,re.M)}
assert memory['RAM']<128*1024 and memory['FLASH']<768*1024
image=ROOT/'build/Debug/rosDemo'
run('bin',[ARM/'arm-none-eabi-objcopy.exe','-O','binary',image.with_suffix('.elf'),image.with_suffix('.bin')])
nm=run('symbols',[ARM/'arm-none-eabi-nm.exe','-S',image.with_suffix('.elf')])
ring=int(re.search(r'^\w+ (\w+) [bB] s_ring$',nm,re.M)[1],16)
data={};upper=0;eof=False
for line in image.with_suffix('.hex').read_text().splitlines():
 assert not eof and line.startswith(':');r=bytes.fromhex(line[1:]);assert sum(r)&255==0 and len(r)==r[0]+5
 offset=int.from_bytes(r[1:3],'big');kind=r[3];payload=r[4:4+r[0]]
 if kind==4:upper=int.from_bytes(payload,'big')<<16
 elif kind==0:
  for i,v in enumerate(payload):a=upper+offset+i;assert a not in data;data[a]=v
 elif kind==1:eof=True
 elif kind==5:assert len(payload)==4
 else:raise AssertionError(kind)
binary=image.with_suffix('.bin').read_bytes();assert eof
assert min(data)==0x08000000 and max(data)<0x080c0000
assert len(binary)==memory['FLASH']==max(data)-min(data)+1
assert all(binary[a-0x08000000]==v for a,v in data.items())
assert all(binary[a-0x08000000]==0 for a in range(min(data),max(data)+1) if a not in data)
sp,reset=struct.unpack_from('<II',binary)
assert sp==0x20020000 and reset&1 and min(data)<=reset&~1<=max(data) and VERSION.encode() in binary
frames={}
for p in (ROOT/'build/Debug').rglob('*.su'):
 for line in p.read_text(errors='replace').splitlines():
  f=line.split('\t')
  if len(f)>=3:frames[f[0].split(':')[-1]]=int(f[1])
chain=['StartMotorTask','MotionTrace_Record','MotionTrace_Snapshot','EncoderDiagnostics_Read','EncoderService_GetSnapshot']
subtotal=sum(frames[f] for f in chain);assert subtotal<1400
assert hashes=={p:sha(ROOT/p) for p in sources}
assert all(sha(Path(p))==h for p,h in old_artifacts.items())
verification={'version':VERSION,'result':'SOFTWARE_PASS_NOT_HARDWARE_VALIDATED',
 'memory_bytes':memory,'ring_bytes':ring,'trace_schema':3,'compiler_frames':{f:frames[f] for f in chain},
 'motor_application_stack_subtotal':subtotal,'motor_stack_bytes':2048,'complete_worst_case_stack_bound':False,
 'real_encoder_update_checks':40000,'encoder_test_modes':['strict','O2','analyzer'],
 'trace_tests':json.loads((ROOT/'build/tests/trace_capture/results.json').read_text()),
 'usb_groups':20,'normal_identity_updates':40000,'build_warnings':0,'build_errors':0,
 'hex_bin_checked':True,'prior_artifacts_unchanged':True,'flashed':False,'motion_sent':False,
 'fault_cleared':False,'serial_opened':False,'hardware_tested':False,'git_committed':False,'pushed':False,
 'sha256':{}}
manifest={'version':VERSION,'files':hashes,'changed_from_0_18_23':sorted(changed),'added':added,
 'git_base':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'uncommitted_candidate':True}
ARCHIVE.mkdir()
for ext in ('hex','bin','elf','map'):
 dest=ARCHIVE/f'rosDemo-0.18.24.{ext}';shutil.copy2(image.with_suffix('.'+ext),dest);verification['sha256'][ext]=sha(dest)
for folder in (HERE,ARCHIVE):
 (folder/'verification.json').write_text(json.dumps(verification,indent=2)+'\n',encoding='utf-8')
 (folder/'source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
for name in sources:
 dest=HERE/'source_snapshot'/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/name,dest)
for name in ('trace_capture','unified_pivot_logs'):
 dest=LOGS/name;dest.mkdir(exist_ok=True)
 for p in (ROOT/'build/tests'/name).glob('*'):
  if p.suffix in ('.json','.log'):shutil.copy2(p,dest/p.name)
print(json.dumps(verification,indent=2),flush=True)
