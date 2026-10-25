"""Revision 200 host regressions. No serial or firmware flashing."""
from pathlib import Path
import json
import re
import shutil
import subprocess
import importlib.util
import hashlib
import argparse

ROOT=Path(__file__).resolve().parents[3]
arguments=argparse.ArgumentParser(description=__doc__)
arguments.add_argument('--revision',choices=('200','201','202','203','204','205'),default='200')
revision=arguments.parse_args().revision
OUT=ROOT/('build/tests/wheel_v020'+revision[-1])
OUT.mkdir(parents=True,exist_ok=True)
CC=shutil.which('gcc')
FLAGS=['-std=c11','-Wall','-Wextra','-Werror','-DCHASSIS_CONTROL_REVISION='+revision]
INC=['-ITools/tests/imu_spi_probe/stubs','-IApplication/Chassis',
     '-IApplication/Safety','-IApplication/Services','-IApplication/Diagnostics']
RUNTIME=['Application/Chassis/'+name+'.c' for name in
         ('chassis_runtime','chassis_controller','chassis_kinematics',
          'chassis_motion_profile','wheel_speed_controller')]+['Application/Safety/fault_manager.c']
RESULTS={}
if int(revision)>=203:
    RUNTIME+=['Application/Chassis/chassis_yaw_feedback.c']

def run(name,command):
    p=subprocess.run(list(map(str,command)),cwd=ROOT,capture_output=True,text=True,
                     encoding='utf-8',errors='replace')
    output=p.stdout+p.stderr
    (OUT/(name+'.log')).write_text(output,encoding='utf-8')
    if p.returncode or re.search(r'\b(?:warning|error):',output,re.I):
        raise RuntimeError(name+': '+output[-7000:])
    RESULTS[name]={'exit_code':0,'groups':len(re.findall(r'^PASS:',output,re.M))}
    print(name+': PASS',flush=True)

if __name__=='__main__':
    for label,extra in [('strict',[]),('optimized',['-O2']),('analyzer',['-fanalyzer'])]:
        if int(revision)>=205:
            for name,control in [('mpu6500_readback',revision),('mpu6500_legacy','191')]:
                binary=OUT/(name+'_'+label+'.exe')
                sources=['Tools/tests/imu_spi_probe/test_mpu6500_calibration.c',
                    'Application/Sensors/mpu6500.c','Application/Sensors/mpu_spi_probe.c',
                    'Application/Sensors/imu_service.c','Application/Diagnostics/imu_static_test.c']
                options=[flag for flag in FLAGS if not flag.startswith('-DCHASSIS_CONTROL_REVISION=')]
                options+=['-DCHASSIS_CONTROL_REVISION='+control]
                run(name+'_'+label+'_build',[CC]+options+extra+INC+
                    ['-IApplication/Sensors','-IApplication/Storage']+sources+['-lm','-o',binary])
                run(name+'_'+label,[binary])
        if int(revision)>=203:
            for name,sources in [
                ('yaw_assist',['Tools/tests/motion/test_yaw_assist.c','Application/Chassis/chassis_yaw_feedback.c']),
                ('yaw_runtime',['Tools/tests/motion/test_yaw_runtime.c']+RUNTIME)]:
                binary=OUT/(name+'_'+label+'.exe')
                run(name+'_'+label+'_build',[CC]+FLAGS+extra+INC+sources+['-lm','-o',binary])
                run(name+'_'+label,[binary])
        for name,sources in [
            ('core',['Tools/tests/motion/test_wheel_v2.c','Application/Chassis/wheel_speed_controller.c']),
            ('runtime',['Tools/tests/motion/test_runtime_v2.c']+RUNTIME),
            ('geometry_runtime',['Tools/tests/motion/test_effective_track_runtime.c']+RUNTIME),
            ('guard_runtime',['Tools/tests/motion/test_pivot_guard_runtime.c']+RUNTIME)]:
            binary=OUT/(name+'_'+label+'.exe')
            run(name+'_'+label+'_build',[CC]+FLAGS+extra+INC+sources+['-lm','-o',binary])
            run(name+'_'+label,[binary])
        binary=OUT/('trace_export_'+label+'.exe')
        run('trace_export_'+label+'_build',[CC]+FLAGS+extra+INC+
            ['Tools/tests/motion/test_trace_capture.c','-lm','-o',binary])
        run('trace_export_'+label,[binary,'--emit'])
        spec=importlib.util.spec_from_file_location('trace_parser',ROOT/'Tools/parse_motion_trace_v2.py')
        parser=importlib.util.module_from_spec(spec);spec.loader.exec_module(parser)
        export=(OUT/('trace_export_'+label+'.log')).read_text(encoding='utf-8')
        captures=parser.parse(export)
        assert captures[0]['header']['control']==1
        assert all('control' in frame for frame in captures[0]['frames'])
        damaged=[re.sub(r'^CONTROL2.*\n','',export,count=1,flags=re.M),
                 export.replace('limits=0/0/0/0','limits=0/0/0'),
                 export.replace('pivot=0','pivot=16')]
        for text in damaged:
            try:parser.parse(text)
            except ValueError:pass
            else:raise AssertionError('Damaged CONTROL2 accepted')
        RESULTS['trace_extension_'+label]={'complete':True,'corruptions_rejected':len(damaged)}
        binary=OUT/('trace_runtime_'+label+'.exe')
        run('trace_runtime_'+label+'_build',[CC]+FLAGS+extra+INC+
            ['Tools/tests/motion/test_trace_runtime.c']+RUNTIME+['-lm','-o',binary])
        run('trace_runtime_'+label,[binary])
        run('trace_enabled_'+label,[binary,'--trace'])
        assert (OUT/('trace_runtime_'+label+'.log')).read_bytes()==(OUT/('trace_enabled_'+label+'.log')).read_bytes(), 'Trace altered outputs'
        for name,sources in [
            ('estimator',['Tools/tests/motion/test_estimator_v2.c',
                          'Application/Chassis/wheel_speed_estimator.c','Application/Services/control_timing.c']),
            ('motion_profile',['Tools/tests/motion/test_motion_profile_v2.c','Application/Chassis/chassis_motion_profile.c']),
            ('feedback_guard',['Tools/tests/motion/test_feedback_guard_v2.c']),
            ('driver',['Tools/tests/motion/test_motor_driver.c','Application/Chassis/motor_driver.c'])]:
            binary=OUT/(name+'_'+label+'.exe')
            includes=(['-ITools/tests/motion/stubs'] if name=='driver' else [])+INC
            run(name+'_'+label+'_build',[CC]+FLAGS+extra+includes+sources+['-lm','-o',binary])
            run(name+'_'+label,[binary])
    (OUT/'results.json').write_text(json.dumps(RESULTS,indent=2),encoding='utf-8')
    sources={}
    for folder in ('Application','Core','USB_DEVICE','cmake','Tools'):
        for path in sorted((ROOT/folder).rglob('*')):
            if path.is_file() and path.suffix in ('.c','.h','.inc','.py','.ps1','.cmake','.txt','.json','.js'):
                sources[path.relative_to(ROOT).as_posix()]=hashlib.sha256(path.read_bytes()).hexdigest()
    for name in ('CMakeLists.txt','CMakePresets.json','rosDemo.ioc','STM32F407xx_FLASH.ld','startup_stm32f407xx.s'):
        sources[name]=hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
    (OUT/'tested_source_manifest.json').write_text(json.dumps(sources,indent=2),encoding='utf-8')
