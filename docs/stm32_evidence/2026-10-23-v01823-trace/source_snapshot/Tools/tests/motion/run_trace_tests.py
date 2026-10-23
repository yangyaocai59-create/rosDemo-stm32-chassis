"""Native diagnostic tests only. Never opens a serial port or flashes."""
import importlib.util
import json
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / 'build/tests/trace_capture'
OUT.mkdir(parents=True, exist_ok=True)
CC = shutil.which('gcc')
if not CC:
    raise RuntimeError('native gcc missing')

def run(name, command):
    p = subprocess.run(list(map(str, command)), cwd=ROOT, capture_output=True,
                       text=True, encoding='utf-8', errors='replace')
    out = p.stdout + p.stderr
    (OUT / (name + '.log')).write_text(out, encoding='utf-8')
    if p.returncode or re.search(r'\b(?:warning|error):', out, re.I):
        raise RuntimeError(name + ': ' + out)
    print(name + ': PASS', flush=True)
    return out

flags = ['-std=c11','-Wall','-Wextra','-Werror',
         '-ITools/tests/imu_spi_probe/stubs','-IApplication/Chassis',
         '-IApplication/Safety','-IApplication/Services','-IApplication/Diagnostics']
for label, more in [('strict',[]),('optimized',['-O2']),('analyzer',['-fanalyzer'])]:
    binary = OUT / (label + '.exe')
    run(label + '_build', [CC] + flags + more + ['Tools/tests/motion/test_trace_capture.c','-lm','-o',binary])
    output = run(label, [binary, '--emit'])

runtime_sources = ['Application/Chassis/'+x+'.c' for x in (
    'chassis_runtime','chassis_controller','chassis_kinematics',
    'chassis_motion_profile','wheel_speed_controller')]
runtime_binary = OUT/'runtime.exe'
run('runtime_build',[CC]+flags+['Tools/tests/motion/test_trace_runtime.c']+
    runtime_sources+['Application/Safety/fault_manager.c','-lm','-o',runtime_binary])
without = run('runtime_without_trace',[runtime_binary])
with_trace = run('runtime_with_trace',[runtime_binary,'--trace'])
assert without==with_trace, 'recorder changed commands/fault timing'

spec = importlib.util.spec_from_file_location('trace_parser', ROOT/'Tools/parse_motion_trace_v2.py')
parser = importlib.util.module_from_spec(spec)
spec.loader.exec_module(parser)
captures = parser.parse(output)
assert len(captures)==1 and len(captures[0]['frames'])==2
assert captures[0]['frames'][1]['wheels']['RR']['total']==-(1<<40)-3+10
assert captures[0]['frames'][1]['wheels']['LF']['obs'] & 2
# Simulated missing lines, fields, truncated tail, duplicate frame cannot PASS.
bad = [output.replace('TRACE END;', 'MISSING END;'),
       re.sub(r'^TRACE2 SNAP.*\n','',output,count=1,flags=re.M),
       re.sub(r'^TRACE2 LF.*\n','',output,count=1,flags=re.M),
       output.replace('count=2','count=3'),
       output.replace('seq=2 ', 'seq=1 '),
       output.replace('total_hi=', 'unknown='),
       output.replace('trigger_seq=2','trigger_seq=99')]
for damaged in bad:
    try:
        parser.parse(damaged)
    except ValueError:
        pass
    else:
        raise AssertionError('damaged export accepted')
(OUT/'parsed.json').write_text(json.dumps(captures,indent=2)+'\n',encoding='utf-8')
(OUT/'results.json').write_text(json.dumps({'native_modes':3,'groups_each':9,
    'runtime_differential_rows':len(without.splitlines()),'runtime_byte_identical':True,
    'signed_counter_roundtrip':True,'malformed_exports_rejected':len(bad),
    'hardware_validated':False},indent=2)+'\n',encoding='utf-8')
print('PASS: TRACE2 signed counters and complete-export validation; 7 corruptions rejected')
