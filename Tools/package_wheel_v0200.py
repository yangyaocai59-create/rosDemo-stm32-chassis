"""Test, build and archive revision 200 without replacing any old firmware.

No serial/device access. Refuses an existing version directory or release ZIP.
The complete working-tree source and exact build logs accompany both images.
"""
from pathlib import Path
import hashlib
import json
import os
import re
import shutil
import struct
import subprocess
import sys
import zipfile
import argparse

ROOT=Path(__file__).resolve().parents[1]
VERSION='0.20.0'
OUT=ROOT/'build/firmware/0.20.0'
RELEASE=ROOT/'releases/0.20.0'
LOGS=ROOT/'build/tests/wheel_v0200_delivery'
BUNDLES=Path('C:/Users/yang/AppData/Local/stm32cube/bundles')


def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def tool(name,relative):
    candidate=shutil.which(name)
    if candidate: return Path(candidate)
    candidate=BUNDLES/relative/name
    if not candidate.is_file(): raise RuntimeError('Missing tool: '+str(candidate))
    return candidate


def run(name,args,environment):
    print(name+': running',flush=True)
    process=subprocess.run(list(map(str,args)),cwd=ROOT,env=environment,
                           capture_output=True,text=True,encoding='utf-8',errors='replace')
    output=process.stdout+process.stderr
    (LOGS/(name+'.log')).write_text(output,encoding='utf-8')
    if process.returncode or re.search(r'\b(?:warning|error):',output,re.I):
        raise RuntimeError(name+': '+output[-7000:])
    print(name+': PASS',flush=True)
    return output


def verify_image(hex_path,binary_path):
    image={};upper=0;eof=False
    for line in hex_path.read_text(encoding='ascii').splitlines():
        assert line.startswith(':') and not eof
        row=bytes.fromhex(line[1:])
        assert len(row)==row[0]+5 and sum(row)%256==0
        offset=int.from_bytes(row[1:3],'big');kind=row[3];payload=row[4:-1]
        if kind==4:
            assert len(payload)==2;upper=int.from_bytes(payload,'big')<<16
        elif kind==0:
            for i,value in enumerate(payload):
                address=upper+offset+i
                assert address not in image
                image[address]=value
        elif kind==1:
            assert not payload;eof=True
        elif kind==5: assert len(payload)==4
        else: raise ValueError('Unsupported HEX record '+str(kind))
    binary=binary_path.read_bytes()
    assert eof and min(image)==0x08000000 and max(image)<0x080C0000
    assert max(image)-min(image)+1==len(binary)
    assert all(binary[address-0x08000000]==value for address,value in image.items())
    sp,reset=struct.unpack_from('<II',binary)
    assert sp==0x20020000 and reset&1 and 0x08000000<(reset&~1)<0x080C0000
    assert (VERSION+'-wheel-control-candidate').encode() in binary
    assert b'0.19.1-effective-track-candidate' not in binary
    return {'bytes':len(binary),'last_flash_address':hex(max(image)),
            'initial_sp':hex(sp),'reset_vector':hex(reset)}


def main():
    global VERSION,OUT,RELEASE,LOGS
    arguments=argparse.ArgumentParser(description=__doc__)
    arguments.add_argument('--version',choices=('0.20.0','0.20.1','0.20.2','0.20.3','0.20.4','0.20.5'),default='0.20.0')
    VERSION=arguments.parse_args().version
    revision='20'+VERSION.rsplit('.',1)[1]
    suffix='v020'+revision[-1]
    OUT=ROOT/'build/firmware'/VERSION
    RELEASE=ROOT/'releases'/VERSION
    LOGS=ROOT/('build/tests/wheel_'+suffix+'_delivery')
    host_out=ROOT/('build/tests/wheel_'+suffix)
    assert not OUT.exists(),'Refusing to replace archived '+VERSION
    assert not RELEASE.exists(),'Refusing to replace release '+VERSION
    LOGS.mkdir(parents=True,exist_ok=True)
    cmake=tool('cmake.exe','cmake/4.3.1+st.1/bin')
    ninja=tool('ninja.exe','ninja/1.13.2+st.1/bin')
    compiler=tool('arm-none-eabi-gcc.exe','gnu-tools-for-stm32/14.3.1+st.2/bin')
    objcopy=compiler.parent/'arm-none-eabi-objcopy.exe'
    nm=compiler.parent/'arm-none-eabi-nm.exe'
    environment={**os.environ,'PATH':os.pathsep.join((str(compiler.parent),str(ninja.parent),os.environ['PATH']))}
    run('preserve_before',[sys.executable,'-B','Tools/preserve_control_baseline.py'],environment)
    previous={}
    for earlier in ('0.20.0','0.20.1','0.20.2','0.20.3','0.20.4'):
        if tuple(map(int,earlier.split('.'))) < tuple(map(int,VERSION.split('.'))):
            for folder in (ROOT/'build/firmware'/earlier,ROOT/'releases'/earlier):
                for path in folder.rglob('*'):
                    if path.is_file(): previous[path.relative_to(ROOT).as_posix()]=digest(path)
    run('host_regressions',['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File',
                            'Tools/tests/run_wheel_v0200_tests.ps1','-ControlRevision',revision],environment)
    tested=json.loads((host_out/'tested_source_manifest.json').read_text())
    assert all(digest(ROOT/name)==expected for name,expected in tested.items()),'Sources changed after tests'
    builds={}
    for configuration in ('Debug','Release'):
        preset='WheelV'+VERSION.replace('.','')+configuration
        run('configure_'+configuration,[cmake,'--preset',preset],environment)
        output=run('build_'+configuration,[cmake,'--build','--preset',preset,'--clean-first'],environment)
        memory={name:int(value) for name,value in re.findall(r'^\s*(RAM|CCMRAM|FLASH):\s+(\d+)\s+B\b',output,re.M)}
        assert memory['RAM']<128*1024 and memory['FLASH']<768*1024
        folder=ROOT/'build'/preset
        elf=folder/('rosDemo-'+VERSION+'.elf')
        binary=elf.with_suffix('.bin')
        run('objcopy_'+configuration,[objcopy,'-O','binary',elf,binary],environment)
        image=verify_image(elf.with_suffix('.hex'),binary)
        symbols=run('symbols_'+configuration,[nm,'--defined-only',elf],environment)
        assert 'WheelSpeedController_Commit' in symbols and 's_unified_pivot' not in symbols
        stack={}
        for path in folder.rglob('*.su'):
            for row in path.read_text(encoding='utf-8').splitlines():
                parts=row.split('\t')
                if len(parts)>=3: stack[parts[0].rsplit(':',1)[-1]]={'bytes':int(parts[1]),'kind':parts[2]}
        builds[configuration]={'memory_bytes':memory,'image':image,'stack_frames':stack}
    baseline=json.loads((ROOT/'build/backups/v0191-before-v0200/manifest.json').read_text())
    protected={name:expected for name,expected in baseline['source'].items() if
        name.startswith(('Application/Sensors/','Application/Storage/','Application/Power/',
                         'Application/Communication/','Application/Board/','USB_DEVICE/','Drivers/','Middlewares/')) or
        name in ('Application/Chassis/encoder_service.c','Application/Chassis/encoder_service.h',
                 'Application/Chassis/chassis_config.h','Application/Chassis/wheel_speed_control_config.h',
                 'Core/Src/tim.c','Core/Src/stm32f4xx_it.c','rosDemo.ioc','STM32F407xx_FLASH.ld')}
    allowed_protected_changes={}
    if int(revision)>=205:
        name='Application/Sensors/mpu6500.c'
        allowed_protected_changes[name]={
            'original_sha256':protected.pop(name),'updated_sha256':digest(ROOT/name),
            'reason':'Revision >=205 excludes self-clearing USER_CTRL bit from readback comparison and logs initialization failures; settings writes, signature and earlier revision behavior retained'}
    assert all(digest(ROOT/name)==expected for name,expected in protected.items()),'Protected source changed'
    run('preserve_after',[sys.executable,'-B','Tools/preserve_control_baseline.py'],environment)
    assert all(digest(ROOT/name)==expected for name,expected in previous.items()),'Previous candidate changed'
    assert all(digest(ROOT/name)==expected for name,expected in tested.items()),'Sources changed during build'
    OUT.mkdir(parents=True,exist_ok=False)
    artifacts={}
    for configuration in builds:
        source=ROOT/'build'/('WheelV'+VERSION.replace('.','')+configuration)
        destination=OUT/configuration
        destination.mkdir()
        for extension in ('elf','hex','bin'):
            src=source/('rosDemo-'+VERSION+'.'+extension)
            dest=destination/src.name;shutil.copy2(src,dest)
            artifacts[dest.relative_to(OUT).as_posix()]=digest(dest)
        dest=destination/('rosDemo-'+VERSION+'.map');shutil.copy2(source/'rosDemo.map',dest)
        artifacts[dest.relative_to(OUT).as_posix()]=digest(dest)
    source_files={}
    for folder in ('Application','Core','USB_DEVICE','cmake','Tools','Drivers','Middlewares'):
        for path in sorted((ROOT/folder).rglob('*')):
            if path.is_file() and path.suffix in ('.c','.h','.inc','.py','.ps1','.cmake','.txt','.json','.js'):
                source_files[path.relative_to(ROOT).as_posix()]=digest(path)
    for name in ('CMakeLists.txt','CMakePresets.json','rosDemo.ioc','STM32F407xx_FLASH.ld',
                 'startup_stm32f407xx.s','docs/STM32_WHEEL_CONTROL_0.20.0_20261024.md'):
        source_files[name]=digest(ROOT/name)
    if int(revision)>=201:
        name='docs/STM32_WHEEL_CONTROL_0.20.1_20261024.md'
        source_files[name]=digest(ROOT/name)
        source_files['.gitattributes']=digest(ROOT/'.gitattributes')
    if int(revision)>=202:
        name='docs/STM32_WHEEL_CONTROL_0.20.2_20261025.md'
        source_files[name]=digest(ROOT/name)
    if int(revision)>=203:
        name='docs/STM32_WHEEL_CONTROL_0.20.3_20261025.md'
        source_files[name]=digest(ROOT/name)
    if int(revision)>=204:
        name='docs/STM32_WHEEL_CONTROL_0.20.4_20261025.md'
        source_files[name]=digest(ROOT/name)
    if int(revision)>=205:
        name='docs/STM32_WHEEL_CONTROL_0.20.5_20261025.md'
        source_files[name]=digest(ROOT/name)
        for name in ('docs/VSCODE_FIRMWARE_STATUS_20261025.md',
                     'docs/STM32_FULL_ERASE_FLASH_WORKFLOW_20261025.md',
                     '.vscode/settings.json','.vscode/tasks.json','.vscode/c_cpp_properties.json'):
            if (ROOT/name).is_file():source_files[name]=digest(ROOT/name)
    with zipfile.ZipFile(OUT/'source.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for name in source_files: archive.write(ROOT/name,name)
    (OUT/'source_manifest.json').write_text(json.dumps({'base_commit':baseline['head'],
        'working_tree_changes_included':True,'files':source_files},indent=2),encoding='utf-8')
    shutil.copytree(LOGS,OUT/'validation/build')
    shutil.copytree(host_out,OUT/'validation/host',
                    ignore=shutil.ignore_patterns('*.exe'))
    verification={'version':VERSION+'-wheel-control-candidate',
        'status':'OFFLINE_VALIDATED_NOT_FLASHED_NOT_HARDWARE_ACCEPTED',
        'hardware_operations':False,'builds':builds,'sha256':artifacts,
        'source_zip_sha256':digest(OUT/'source.zip'),
        'protected_source_count':len(protected),'old_artifacts_preserved':baseline['artifacts'],
        'intentional_protected_source_changes':allowed_protected_changes,
        'previous_candidate_artifacts_preserved':previous,
        'calibration_parameters_changed':False,'control_behavior_changed':True,
        'yaw_rate_assist_enabled':int(revision)>=203,
        'imu_readback_fix_enabled':int(revision)>=205,
        'pwm_limit_permille':300,'sample_period_ms':20,'feedback_age_limit_ms':40}
    (OUT/'verification.json').write_text(json.dumps(verification,indent=2),encoding='utf-8')
    RELEASE.mkdir(parents=True,exist_ok=False)
    package=RELEASE/('rosDemo-'+VERSION+'-offline-candidate.zip')
    with zipfile.ZipFile(package,'x',zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(OUT.rglob('*')):
            if path.is_file(): archive.write(path,path.relative_to(OUT).as_posix())
    (RELEASE/'SHA256SUMS.txt').write_text(digest(package)+'  '+package.name+'\n',encoding='ascii')
    (RELEASE/'README.md').write_text(
        '**'+VERSION+' 四轮控制候选，尚未烧录或实车验收。**\n\n'
        'ZIP 内含 Debug/Release 的 HEX、BIN、ELF、MAP，完整源码和验证记录。'
        'Release 用于后续常规控制验证；Debug 保留现有调试入口。两者均为新版控制。'
        '\n\n旧版 0.19.1 和所有既有发布产物保留，哈希复核通过。'
        '\n\n详细行为变化和剩余标定工作见包内 '
        '`source.zip` 中的 `docs/STM32_WHEEL_CONTROL_'+VERSION+('_20261025.md' if int(revision)>=202 else '_20261024.md')+'`。'
        '本包没有执行烧录、运动或校准；不要以离线测试通过替代实车性能验收。\n',encoding='utf-8')
    run('update_vscode',[sys.executable,'-B','Tools/sync_vscode_firmware.py',
        '--elf',ROOT/('build/WheelV'+VERSION.replace('.','')+'Release')/('rosDemo-'+VERSION+'.elf'),
        '--version',VERSION,'--configuration','Release'],environment)
    print(json.dumps({'firmware':str(OUT),'release':str(package),'sha256':digest(package),
                      'memory':{k:v['memory_bytes'] for k,v in builds.items()},
                      'preserved_artifacts':len(baseline['artifacts'])},indent=2),flush=True)


if __name__=='__main__': main()
