"""Regress, clean-build, verify and freeze .20. No ports, motion or flash."""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess

here=Path(__file__).resolve().parent
root=next(p for p in here.parents if (p/'Application').is_dir())
version='0.18.20-rr-running-recovery'
archive=root/'build/firmware/0.18.20'
assert not archive.exists(),'Refuse to overwrite an existing candidate'
bundle=Path('C:/Users/yang/AppData/Local/stm32cube/bundles')
bins=[bundle/'gnu-tools-for-stm32/14.3.1+st.2/bin',
      bundle/'cmake/4.3.1+st.1/bin',bundle/'ninja/1.13.2+st.1/bin']
env={**os.environ,'PATH':os.pathsep.join(map(str,bins))+os.pathsep+os.environ['PATH']}
baseline=json.loads((root/'build/firmware/0.18.19/source_manifest.json').read_text())
old_artifacts=json.loads((root/'build/firmware/0.18.19/verification.json').read_text())['sha256']
prototype=root/'docs/stm32_evidence/2026-10-22-rr-running-prototype-v1/prototype'
new_files=['Tools/tests/motion/'+p for p in (
    'rr_running_fixture.h','run_rr_running_tests.py','test_rr_running.c',
    'test_rr_running_safety.c','test_rr_running_fault_link.c','test_rr_running_handoff.c',
    'test_rr_running_fault_audit.c','test_rr_running_isolation.c','test_rr_running_identity.c')]
files=sorted(list(baseline['files'])+new_files)
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def check_old_artifacts():
    assert all(sha(root/f'build/firmware/0.18.19/rosDemo-0.18.19.{ext}')==h
               for ext,h in old_artifacts.items())
check_old_artifacts()
for p in (here/'baseline_0.18.19').iterdir():
    matches=[h for n,h in baseline['files'].items() if Path(n).name==p.name]
    assert len(matches)==1 and sha(p)==matches[0],p.name
prototype_proof=json.loads((prototype.parent/'verification.json').read_text())
for n,h in prototype_proof['files'].items():
    assert sha(prototype.parent/n)==h,n
hashes={p:sha(root/p) for p in files}
changed=sorted(p for p,h in baseline['files'].items() if hashes[p]!=h)
assert set(changed)=={
 'Application/Chassis/chassis_controller.c','Application/Chassis/wheel_speed_controller.c',
 'Application/Chassis/wheel_speed_controller.h','Application/Chassis/wheel_speed_control_config.h',
 'Application/Safety/fault_manager.c','Application/System/firmware_info.h',
 'Tools/tests/motion/test_runtime_motion.c','Tools/tests/run_motion_tests.ps1',
 'Tools/tests/motion/run_lr_running_tests.py','Tools/tests/motion/test_lr_running_isolation.c',
 'Tools/tests/motion/test_lr_running_identity.c'},changed
def tokens(p): return re.sub(r'\s+','',re.sub(r'/\*.*?\*/|//[^\n]*','',p.read_text(),flags=re.S))
for p in changed:
    if p.startswith('Application/') and not p.endswith('firmware_info.h'):
        assert tokens(root/p)==tokens(prototype/Path(p).name),p
def run(label,command,expected_unused_helper_warning=False):
    r=subprocess.run(list(map(str,command)),cwd=root,env=env,capture_output=True,
                     text=True,encoding='utf-8',errors='replace')
    output=r.stdout+r.stderr
    (here/f'v01820_{label}.log').write_text(output,encoding='utf-8')
    print(f'{label}: exit={r.returncode}, PASS groups={len(re.findall(r"^PASS:",output,re.M))}',flush=True)
    if r.returncode: raise RuntimeError(label+' failed: '+output[-6000:])
    if expected_unused_helper_warning:
        warnings=re.findall(r'^.*warning:.*$',output,re.M)
        assert len(warnings)==1 and 'ImuService_AbsI32' in warnings[0] and 'defined but not used' in warnings[0],output
        assert not re.search(r'\berror:',output,re.I),output
    else:
        assert not re.search(r'\b(?:warning|error):',output,re.I),(label,output[-6000:])
    return output

motion=run('motion_regression',['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File','Tools/tests/run_motion_tests.ps1'])
usb=run('usb_regression',['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File','Tools/tests/run_usb_tx_tests.ps1'])
assert len(re.findall(r'^PASS:',motion,re.M))==79
assert len(re.findall(r'^PASS:',usb,re.M))==20
try:
    imu=run('imu_quality_regression',['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File','Tools/tests/run_imu_quality_tests.ps1'])
    raise AssertionError('Expected documented baseline configuration diagnostic changed; review before packaging')
except RuntimeError:
    # Do not change the IMU source or pretend its strict disabled-calibration
    # configuration passes. Preserve the failure and allow ONLY the known,
    # unchanged unused helper warning in separate additional host builds.
    imu=(here/'v01820_imu_quality_regression.log').read_text()
    errors=re.findall(r'^.*\berror:.*$',imu,re.M)
    assert len(errors)==1 and 'ImuService_AbsI32' in errors[0] and 'defined but not used' in errors[0],imu
    assert len(re.findall(r'^PASS:',imu,re.M))==11,imu
    for name in ('imu_service.c','mpu6500.c','mpu_spi_probe.c','imu_config.h'):
        relative='Application/Sensors/'+name
        assert sha(root/relative)==baseline['files'][relative]
host_flags=['gcc','-std=c11','-Wall','-Wextra','-Werror','-Wno-error=unused-function',
 '-ITools/tests/imu_spi_probe/stubs','-IApplication/Diagnostics','-IApplication/Sensors',
 '-IApplication/Services','-IApplication/Storage']
common=['Application/Sensors/mpu_spi_probe.c','Application/Sensors/imu_service.c',
        'Application/Diagnostics/imu_static_test.c']
polling_build=run('imu_polling_known_baseline_warning_build',host_flags+['-DIMU_CALIBRATION_ENABLED=0',
 'Tools/tests/imu_spi_probe/test_mpu6500_polling.c','Application/Sensors/mpu6500.c']+common+
 ['-lm','-o','build/tests/v01820_polling_known_warning.exe'],True)
polling=run('imu_polling_known_warning_execution',[root/'build/tests/v01820_polling_known_warning.exe'])
probe_build=run('imu_probe_known_baseline_warning_build',host_flags+['-DIMU_SPI_ID_PROBE_ONLY=1',
 '-DIMU_USE_MPU6500_SPI=0','Tools/tests/imu_spi_probe/test_imu_spi_probe.c',
 'Application/Diagnostics/imu_spi_identity_test.c']+common+
 ['-lm','-o','build/tests/v01820_probe_known_warning.exe'],True)
probe=run('imu_probe_known_warning_execution',[root/'build/tests/v01820_probe_known_warning.exe'])
python=run('imu_python_regression',['python','-m','unittest','discover','-s','Tools/tests','-p','test_imu*.py','-v'])
assert re.search(r'Ran (\d+) tests',python) and re.search(r'^OK$',python,re.M)
run('protocol_self_test',['python','Tools/usb_cdc_test.py','--self-test'])
build=run('debug_build',[bins[1]/'cmake.exe','--build','--preset','Debug','--clean-first'])
assert '[95/95]' in build and 'Linking C executable' in build
memory={n:int(v) for n,v in re.findall(r'^\s*(RAM|CCMRAM|FLASH):\s+(\d+)\s+B\b',build,re.M)}
assert set(memory)=={'RAM','CCMRAM','FLASH'}
assert memory['RAM']<128*1024 and memory['FLASH']<768*1024
assert {p:sha(root/p) for p in files}==hashes

# Review concrete linked-call edges plus compiler application-frame sizes.
# This is NOT a complete interrupt/libc worst-case stack bound.
frames={}
for p in (root/'build/Debug').rglob('*.su'):
    for line in p.read_text(errors='replace').splitlines():
        fields=line.split('\t')
        if len(fields)>=3:
            frames[fields[0].split(':')[-1]]=int(fields[1])
disassembly=run('linked_disassembly',[bins[0]/'arm-none-eabi-objdump.exe','-d','--no-show-raw-insn','build/Debug/rosDemo.elf'])
bodies={}
current=None
for line in disassembly.splitlines():
    match=re.match(r'^[0-9a-f]+ <([^>]+)>:',line)
    if match: current=match[1]; bodies[current]=[]
    elif current: bodies[current].append(line)
paths={
 'motor_RR_running':(['StartMotorTask','ChassisRuntime_Process','ChassisController_Process',
    'WheelSpeedController_UpdateWithMode','WheelSpeedController_RrRunningSupport'],2048),
 'motor_RR_selection':(['StartMotorTask','ChassisRuntime_Process','ChassisController_Process',
    'ChassisController_GetFeedforwardBias','WheelSpeedController_SelectRrRunningSupport'],2048),
 'safety_RR_query':(['StartSafetyTask','FaultManager_ObserveMotion',
    'WheelSpeedController_IsRrRunningSupportOwned'],1024),
 'motor_LR_running':(['StartMotorTask','ChassisRuntime_Process','ChassisController_Process',
    'WheelSpeedController_UpdateWithMode','WheelSpeedController_LrRunningSupport'],2048),
 'motor_front_existing':(['StartMotorTask','ChassisRuntime_Process','ChassisController_Process',
    'WheelSpeedController_UpdateWithMode','WheelSpeedController_FrontSupport'],2048),
 'motor_LR_selection':(['StartMotorTask','ChassisRuntime_Process','ChassisController_Process',
    'ChassisController_GetFeedforwardBias','WheelSpeedController_SelectLrRunningSupport'],2048),
 'safety_LR_query':(['StartSafetyTask','FaultManager_ObserveMotion',
    'WheelSpeedController_IsLrRunningSupportOwned'],1024),
 'safety_fault_log':(['StartSafetyTask','FaultManager_ObserveMotion','DebugLog_Printf',
    'DebugLog_DeferToBuffer'],1024)}
review={}
for name,(chain,allocated) in paths.items():
    for caller,callee in zip(chain,chain[1:]):
        assert re.search(r'\bbl(?:\.w)?\s+[^\n]*<'+re.escape(callee)+r'>','\n'.join(bodies[caller])),(caller,callee)
    subtotal=sum(frames[f] for f in chain)
    assert subtotal<allocated,(name,subtotal,allocated)
    review[name]={'application_chain':chain,'function_frames_bytes':[frames[f] for f in chain],
      'application_frame_subtotal_bytes':subtotal,'task_allocated_bytes':allocated,
      'complete_worst_case_bound':False}
stack={'paths':review,'function_frames_bytes':frames,
 'baseline_hardware_observation_only':{'motor_free_bytes':1224,'safety_free_bytes':352,
   'source':'docs/stm32_evidence/2026-10-22-v01819-ground-left/capture/ch340.log',
   'not_current_candidate_measurement':True},
 'limits':['Application-frame subtotals omit deeper library/RTOS/exception frames and alternate paths.',
           'No new hardware stack high-watermark measurement; verify after flash.',
           'Task allocation, IRQ configuration and overflow hook remain unchanged.']}
(here/'stack_review.json').write_text(json.dumps(stack,indent=2)+'\n')
selected=set(f for chain,_ in paths.values() for f in chain)
(here/'stack_disassembly_excerpt.txt').write_text('\n'.join(
   f'<{f}>\n'+'\n'.join(bodies[f]) for f in sorted(selected)))

image=root/'build/Debug/rosDemo'
run('objcopy_bin',[bins[0]/'arm-none-eabi-objcopy.exe','-O','binary',image.with_suffix('.elf'),image.with_suffix('.bin')])
data={}; upper=0; records=0; eof=False
for line in image.with_suffix('.hex').read_text().splitlines():
    assert not eof and line.startswith(':')
    record=bytes.fromhex(line[1:]); records+=1
    assert len(record)==record[0]+5 and sum(record)&255==0
    length,offset,kind=record[0],int.from_bytes(record[1:3],'big'),record[3]
    payload=record[4:4+length]
    if kind==4:
        assert length==2 and offset==0; upper=int.from_bytes(payload,'big')<<16
    elif kind==0:
        for i,value in enumerate(payload):
            address=upper+offset+i; assert address not in data; data[address]=value
    elif kind==1:
        assert length==0 and offset==0; eof=True
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
assert version.encode() in binary and b'0.18.19-lr-running-recovery' not in binary
check_old_artifacts()
assert {p:sha(root/p) for p in files}==hashes
archive.mkdir()
artifact_hashes={}
for ext in ('hex','bin','elf','map'):
    dest=archive/f'rosDemo-0.18.20.{ext}'
    shutil.copy2(image.with_suffix('.'+ext),dest); artifact_hashes[ext]=sha(dest)
manifest={'firmware':version,'git_base':baseline['git_base'],'dirty_worktree':True,
 'files':hashes,'changed_relative_to_0.18.19':changed,'new_relative_to_0.18.19':new_files,
 'frozen_0.18.19_baseline':{str(p.relative_to(root)):sha(p) for p in (here/'baseline_0.18.19').iterdir()},
 'validated_offline_prototype':{str(p.relative_to(root)):sha(p) for p in prototype.iterdir()},
 'production_matches_prototype_except_comments_whitespace':True,
 'note':'Uncommitted candidate; base commit is not release identity. Not a standalone vendor/toolchain snapshot.'}
verification={'result':'PASS_SCOPED_SOFTWARE_CANDIDATE_WITH_KNOWN_EXTENDED_HOST_TEST_LIMITATION','firmware':version,
 'motion_pass_lines':79,'usb_pass_lines':20,'imu_native_pass_lines':len(re.findall(r'^PASS:',imu,re.M)),
 'imu_python_test_count':int(re.search(r'Ran (\d+) tests',python)[1]),
 'firmware_build_warnings':0,'firmware_build_errors':0,
 'strict_extended_imu_quality_suite_pass':False,
 'extended_host_test_limitation':'Unchanged ImuService_AbsI32 unused under CALIBRATION=0/probe; original strict failure preserved. Two additional host builds demote only unused-function, assert the sole diagnostic is this helper, then execute tests. No IMU source changes.',
 'additional_host_known_warning_count':2,
 'additional_polling_pass_lines':len(re.findall(r'^PASS:',polling,re.M)),
 'additional_probe_pass_lines':len(re.findall(r'^PASS:',probe,re.M)),
 'build':'Debug clean, GNU STM32 14.3.1+st.2, 95 steps',
 'memory_bytes':memory,'sha256':artifact_hashes,'hex_records':records,
 'hex_data_bytes':len(data),'hex_bin_explicit_data_equal':True,
 'binary_gap_bytes':len(gaps),'binary_gap_fill':'0x00; HEX leaves alignment gaps unwritten',
 'bytes':len(binary),'highest':hex(max(data)),'stack':hex(sp),'reset':hex(reset),
 'stack_review':'stack_review.json; partial application-frame subtotals, not a complete bound',
 'source_items':len(files),'old_0.18.19_artifacts_unchanged':True,
 'flashed':False,'hardware_tested':False,'serial_opened':False,'motion_sent':False,
 'fault_cleared_on_hardware':False,'new_commit_created':False,'pushed':False,
 'lr_regression':json.loads((root/'build/tests/lr_running/verification.json').read_text()),
 'rr_regression':json.loads((root/'build/tests/rr_running/verification.json').read_text())}
for folder in (here,archive):
    (folder/'source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (folder/'verification.json').write_text(json.dumps(verification,indent=2)+'\n')
snapshot=here/'source_snapshot'
snapshot.mkdir()
for name in changed+new_files:
    shutil.copy2(root/name,snapshot/Path(name).name)
for policy in ('lr','rr'):
    for name in ('test_run.log','fault_grid.json'):
        shutil.copy2(root/('build/tests/'+policy+'_running')/name,here/(policy+'_'+name))
shutil.copy2(here/'stack_review.json',archive/'stack_review.json')
print(json.dumps(verification,indent=2),flush=True)
print('ARTIFACT '+str(archive),flush=True)
