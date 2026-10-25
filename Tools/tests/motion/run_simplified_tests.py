"""Offline regression for the simplified production controller. No device access."""
from pathlib import Path
import subprocess,sys,json,hashlib,re,shutil
ROOT=Path(__file__).resolve().parents[3]
OUT=ROOT/'build/tests/control_simplification';OUT.mkdir(parents=True,exist_ok=True)
BASE='f62c66d'
CC=shutil.which('gcc')
results={}
def run(name,args):
 p=subprocess.run(list(map(str,args)),cwd=ROOT,capture_output=True,text=True,encoding='utf-8',errors='replace')
 output=p.stdout+p.stderr;(OUT/(name+'.log')).write_text(output,encoding='utf-8')
 if p.returncode or re.search(r'\b(?:warning|error):',output,re.I):raise RuntimeError(name+': '+output[-5000:])
 results[name]={'exit':0,'sha256':hashlib.sha256(output.encode()).hexdigest()}
 print(name+': PASS',flush=True);return output
flags=['-std=c11','-Wall','-Wextra','-Werror']
inc=['-ITools/tests/imu_spi_probe/stubs','-IApplication/Chassis','-IApplication/Safety','-IApplication/Services','-IApplication/Diagnostics']
run('unified',[sys.executable,'-B','Tools/tests/motion/run_unified_tests.py'])
run('trace',[sys.executable,'-B','Tools/tests/motion/run_trace_tests.py'])
base=OUT/'baseline';base.mkdir(exist_ok=True)
for name in ['wheel_speed_controller.c','wheel_speed_controller.h','wheel_speed_control_config.h','unified_pivot_control.h']:
 cache=ROOT/'Tools/tests/baselines/v01825'
 if cache.exists():
  data=(cache/name).read_bytes()
  assert hashlib.sha256(data).hexdigest()==json.loads((cache/'manifest.json').read_text())['files'][name]
 else:data=subprocess.check_output(['git','show',BASE+':Application/Chassis/'+name],cwd=ROOT)
 (base/name).write_bytes(data)
outputs=[]
for label,path,extra in [('baseline',base/'wheel_speed_controller.c',['-I'+str(base)]),('candidate','Application/Chassis/wheel_speed_controller.c',[])]:
 exe=OUT/(label+'.exe')
 run(label+'_build',[CC]+flags+extra+inc+['Tools/tests/motion/control_refactor_compare.c',path,'-lm','-o',exe])
 outputs.append(run(label,[exe]))
assert outputs[0]==outputs[1] and len(outputs[0].splitlines())==80000
results['mixed_mode_identity']={'steps':80000,'identical':True}
print('80000 NORMAL/PIVOT transitions, reset/prime/dt/sign differential: PASS',flush=True)
runtime=['Application/Chassis/'+n+'.c' for n in ['chassis_runtime','chassis_controller','chassis_kinematics','chassis_motion_profile','wheel_speed_controller']]+['Application/Safety/fault_manager.c']
for test,deps in [('pivot_guard',[]),('pivot_guard_runtime',runtime)]:
 for label,extra in [('strict',[]),('optimized',['-O2']),('analyzer',['-fanalyzer'])]:
  exe=OUT/(test+label+'.exe');name=test+'_'+label
  run(name+'_build',[CC]+flags+extra+inc+['Tools/tests/motion/test_'+test+'.c']+deps+['-lm','-o',exe]);run(name,[exe])
for name in ['encoder_acquisition','encoder_diag']:
 exe=OUT/(name+'.exe')
 run(name+'_build',[CC]+flags+['-ITools/tests/encoder_diag/stubs']+inc+['Tools/tests/encoder_diag/test_'+name+'.c','-lm','-o',exe]);run(name,[exe])
run('odometry',[sys.executable,'-B','Tools/wheel_odometry_test.py','--self-test'])
(OUT/'results.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
