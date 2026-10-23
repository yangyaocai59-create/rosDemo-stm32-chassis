"""Offline effective-track model tests. Never opens hardware."""
from pathlib import Path
import subprocess,shutil,re,json
root=Path(__file__).resolve().parents[3];out=root/'build/tests/effective_track';out.mkdir(parents=True,exist_ok=True)
cc=shutil.which('gcc');results={}
for label,extra in [('strict',[]),('optimized',['-O2']),('analyzer',['-fanalyzer'])]:
 exe=out/(label+'.exe')
 commands=[('build',[cc,'-std=c11','-Wall','-Wextra','-Werror']+extra+['-ITools/tests/imu_spi_probe/stubs','-IApplication/Chassis','Tools/tests/motion/test_effective_track.c','Application/Chassis/chassis_kinematics.c','-lm','-o',str(exe)]),('run',[str(exe)])]
 for kind,cmd in commands:
  p=subprocess.run(cmd,cwd=root,capture_output=True,text=True);text=p.stdout+p.stderr;(out/(label+'_'+kind+'.log')).write_text(text)
  assert p.returncode==0 and not re.search(r'\b(?:warning|error):',text,re.I),text
  print(label+' '+kind+': PASS',flush=True)
  if kind=='run':print(text,end='');results[label]=text
(out/'results.json').write_text(json.dumps(results,indent=2))

runtime=['Application/Chassis/'+n+'.c' for n in ['chassis_runtime','chassis_controller','chassis_kinematics','chassis_motion_profile','wheel_speed_controller']]+['Application/Safety/fault_manager.c']
for label,extra in [('strict',[]),('optimized',['-O2']),('analyzer',['-fanalyzer'])]:
 exe=out/('runtime_'+label+'.exe')
 args=[cc,'-std=c11','-Wall','-Wextra','-Werror']+extra+['-ITools/tests/imu_spi_probe/stubs','-IApplication/Chassis','-IApplication/Safety','-IApplication/Services','Tools/tests/motion/test_effective_track_runtime.c']+runtime+['-lm','-o',str(exe)]
 for kind,cmd in [('build',args),('run',[str(exe)])]:
  p=subprocess.run(cmd,cwd=root,capture_output=True,text=True);text=p.stdout+p.stderr;(out/('runtime_'+label+'_'+kind+'.log')).write_text(text)
  assert p.returncode==0 and not re.search(r'\b(?:warning|error):',text,re.I),text
  print('runtime '+label+' '+kind+': PASS',flush=True)
  if kind=='run':print(text,end='');results['runtime_'+label]=text
(out/'results.json').write_text(json.dumps(results,indent=2))
