"""Build/archive the .19.0 source-refactor candidate; no hardware operations."""
from pathlib import Path
import subprocess,os,re,json,hashlib,zipfile,shutil,struct
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'build/firmware/0.19.0'
assert not OUT.exists(),'Refuse to overwrite an archived candidate'
ARM=Path('C:/Users/yang/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin')
CMAKE=Path('C:/Users/yang/AppData/Local/stm32cube/bundles/cmake/4.3.1+st.1/bin/cmake.exe')
env={**os.environ,'PATH':str(ARM)+os.pathsep+os.environ['PATH']}
sha=lambda b:hashlib.sha256(b).hexdigest()
logs=ROOT/'build/tests/control_simplification'
build=subprocess.run([str(CMAKE),'--build','--preset','Debug','--clean-first'],cwd=ROOT,env=env,capture_output=True,text=True)
output=build.stdout+build.stderr
(logs/'final_arm_build.log').write_text(output)
assert build.returncode==0 and not re.search(r'\b(?:warning|error):',output,re.I),output[-5000:]
memory={k:int(v) for k,v in re.findall(r'^\s*(RAM|CCMRAM|FLASH):\s+(\d+)\s+B\b',output,re.M)}
assert memory['RAM']<128*1024 and memory['FLASH']<768*1024
elf=ROOT/'build/Debug/rosDemo.elf';binary=elf.with_suffix('.bin')
subprocess.run([str(ARM/'arm-none-eabi-objcopy.exe'),'-O','binary',str(elf),str(binary)],check=True)
b=binary.read_bytes();image={};upper=0;eof=False
for line in elf.with_suffix('.hex').read_text().splitlines():
 r=bytes.fromhex(line[1:]);assert line.startswith(':') and sum(r)%256==0 and len(r)==r[0]+5 and not eof
 offset=int.from_bytes(r[1:3],'big');kind=r[3];payload=r[4:4+r[0]]
 if kind==4:upper=int.from_bytes(payload,'big')<<16
 elif kind==0:
  for i,v in enumerate(payload):
   a=upper+offset+i;assert a not in image;image[a]=v
 elif kind==1:eof=True
 elif kind==5:assert len(payload)==4
 else:raise AssertionError(kind)
assert eof and min(image)==0x08000000 and max(image)<0x080c0000
assert all(b[a-0x08000000]==v for a,v in image.items())
assert len(b)==memory['FLASH']==max(image)-min(image)+1
sp,reset=struct.unpack_from('<II',b);assert sp==0x20020000 and reset&1 and 0x08000000<(reset&~1)<0x080c0000
assert b'0.19.0-control-simplification-candidate' in b
protected=[]
with zipfile.ZipFile(ROOT/'build/backups/v0.18.25-before-simplification.zip') as z:
 manifest=json.loads(z.read('BACKUP_SHA256.json'))
 for name,h in manifest.items():
  if name.startswith(('Application/Sensors/','Application/Storage/','Core/','USB_DEVICE/','Application/Board/')) or name in ['Application/Chassis/encoder_service.c','Application/Chassis/encoder_service.h','Application/Chassis/wheel_speed_estimator.c','Application/Chassis/wheel_speed_estimator.h','Application/Chassis/motor_driver.c','Application/Chassis/motor_driver.h','Application/Chassis/chassis_kinematics.c','Application/Chassis/chassis_config.h']:
   assert sha((ROOT/name).read_bytes())==h,name;protected.append(name)
old_artifacts={}
for version in ['0.18.21','0.18.22','0.18.23','0.18.24','0.18.25']:
 folder=ROOT/'build/firmware'/version
 vf=folder/'verification.json'
 if vf.exists():
  old=json.loads(vf.read_text(encoding='utf-8-sig'))
  for ext,h in old.get('sha256',{}).items():
   p=folder/('rosDemo-'+version+'.'+ext);assert sha(p.read_bytes())==h,str(p)
   old_artifacts[str(p.relative_to(ROOT))]=h
OUT.mkdir(parents=True)
artifacts={}
for ext in ['elf','hex','bin','map']:
 src=elf.with_suffix('.'+ext);dest=OUT/('rosDemo-0.19.0.'+ext);shutil.copy2(src,dest);artifacts[ext]=sha(dest.read_bytes())
paths=[]
for folder in ['Application','Core','USB_DEVICE','cmake','Tools']:
 paths.extend(p for p in (ROOT/folder).rglob('*') if p.is_file() and p.suffix in ['.c','.h','.py','.ps1','.cmake','.txt','.json'])
paths.extend(ROOT/n for n in ['CMakeLists.txt','CMakePresets.json','rosDemo.ioc','STM32F407xx_FLASH.ld','startup_stm32f407xx.s'])
source={str(p.relative_to(ROOT)).replace('\\','/'):sha(p.read_bytes()) for p in paths}
(OUT/'source_manifest.json').write_text(json.dumps({'baseline':'f62c66d','files':source},indent=2))
shutil.copytree(logs,OUT/'validation',ignore=shutil.ignore_patterns('*.exe','baseline'))
verification={'version':'0.19.0-control-simplification-candidate','status':'OFFLINE_VALIDATED_NOT_HARDWARE_ACCEPTED','memory_bytes':memory,'sha256':artifacts,'mixed_mode_differential_updates':80000,'normal_differential_updates':40000,'protected_files_unchanged':protected,'old_artifacts_unchanged':old_artifacts,'hex_data_bytes':len(image),'vector_sp':hex(sp),'vector_reset':hex(reset),'hardware_operations':False,'parameter_changes':False}
(OUT/'verification.json').write_text(json.dumps(verification,indent=2))
print(json.dumps({'artifacts':str(OUT),'memory':memory,'protected_files':len(protected),'old_artifacts':len(old_artifacts),'hex_sha256':artifacts['hex']},indent=2))
