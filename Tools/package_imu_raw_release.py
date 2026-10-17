"""Freeze 0.14.3 software candidate without touching hardware or old deliveries.

Refuses overwrite. This does NOT produce a hardware-VERIFIED release.
"""
import hashlib
import json
from pathlib import Path
import re
import shutil
import zipfile


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    root=Path(__file__).resolve().parents[1]
    verification=root/'build/verification/imu_raw_0.14.3_20261016'
    artifacts=json.loads((verification/'artifact_verification.json').read_text())
    for name,expected in artifacts['sha256'].items():
        assert digest(root/'build/Debug'/name)==expected, 'build image changed after verification'
    text=(verification/'debug_clean_build.log').read_text(encoding='utf-16')
    assert not re.search(r'\b(?:warning|error):',text,re.I)
    assert '[91/91]' in text
    baseline=root/'build/deliveries/mpu6500_ros_handoff_0.14.2_verified_20261014/source_snapshot.zip'
    firmware_changes=[]
    with zipfile.ZipFile(baseline) as old:
        for name in old.namelist():
            if name.startswith(('Application/','Core/','Drivers/','Middlewares/','USB_DEVICE/','cmake/')) or name.endswith(('.ioc','.ld')) or name in ('CMakeLists.txt','CMakePresets.json','startup_stm32f407xx.s'):
                assert (root/name).is_file(), 'missing baseline file '+name
                if (root/name).read_bytes()!=old.read(name):firmware_changes.append(name)
    assert sorted(firmware_changes)==sorted([
        'Application/Communication/chassis_usb_protocol.c',
        'Application/Communication/chassis_usb_protocol.h',
        'Application/System/firmware_info.h'])
    destination=root/'build/deliveries/imu_raw_0.14.3_20261016'
    archive=destination.with_name('STM32_IMU_RAW_0.14.3_SOFTWARE_CANDIDATE_20261016.zip')
    assert not archive.exists(), 'refuse to replace an existing archive'
    destination.mkdir(parents=True,exist_ok=False)
    for folder in ('firmware','docs','verification'):(destination/folder).mkdir()
    for name in artifacts['sha256']:
        shutil.copy2(root/'build/Debug'/name,destination/'firmware'/name)
    docs=['STM32_IMU_RAW_V1_HANDOFF_20261016.md','IMU_DRIFT_CORRECTION_PLAN_20261016.md',
          'ROS_IMU_DRIFT_ENGINEER_NOTICE_20261016.md','IMU_STATIC_DRIFT_DIAGNOSIS_20261016.md',
          'ROS2_UBUNTU_HANDOFF.md']
    for name in docs:shutil.copy2(root/'docs'/name,destination/'docs'/name)
    for name in ('configure.log','debug_clean_build.log','native_tests.log','python_tests.log',
                 'legacy_python_selftest.log','artifact_verification.json'):
        shutil.copy2(verification/name,destination/'verification'/name)
    status=dict(version='0.14.3-imu-raw-v1',build='Oct 16 2026 09:56:40',
        status='SOFTWARE_CANDIDATE_NOT_HARDWARE_VERIFIED',flash_bytes=159508,ram_bytes=54936,
        ccm_bytes=0,compiler_errors=0,compiler_warnings=0,native_programs_passed=9,
        python_raw_tests_passed=4,legacy_python_selftest_passed=True,
        firmware_files_changed=firmware_changes,assistant_flashed=False,
        new_firmware_hardware_tests_run=False,ros_estimator_implemented=False,
        final_yaw_drift_target_verified=False,git_commit=None,artifacts=artifacts)
    (destination/'release_status.json').write_text(json.dumps(status,indent=2)+'\n',encoding='utf8')
    (destination/'README_FIRST.md').write_text(
        '# 0.14.3 软件候选：等待用户烧录与新版本实板验证\n\n'
        '只烧录 firmware/rosDemo.hex；配套 BIN 地址为 0x08000000。\n'
        '版本 0.14.3-imu-raw-v1，Build Oct 16 2026 09:56:40。\n'
        'Error 0、Warning 0；FLASH 159508 B、RAM 54936 B、CCM 0 B。\n\n'
        '用户自行全片擦除烧录，电机 VM 关闭；全片擦除会删除原 seq=4 校准，刷后需显式校准一次。\n'
        '助手未烧录或复位设备。新版本实板/ROS 验收尚未进行；不要当成最终 VERIFIED 包交接。\n'
        '本固件仅增加高精度 RAW 接口，不自动校准、不在线修正残余偏置；ROS 独立估计器待实现。\n\n'
        '详情见 docs/STM32_IMU_RAW_V1_HANDOFF_20261016.md。\n'
        'source_snapshot.zip 包含本版完整工程源码/Tools/文档；校验见 SHA256SUMS.txt。\n'
        '旧 0.14.2 冻结交付未改；没有 Git 元数据，提交号为无。\n',encoding='utf8')
    source=[]
    for name in ('Application','Core','Drivers','Middlewares','USB_DEVICE','cmake','Tools','docs'):
        source.extend(p for p in (root/name).rglob('*') if p.is_file() and
                      '__pycache__' not in p.parts and p.suffix not in ('.pyc','.exe','.o','.obj'))
    for name in ('CMakeLists.txt','CMakePresets.json','rosDemo.ioc','STM32F407xx_FLASH.ld',
                 'startup_stm32f407xx.s','.gitignore'):
        source.append(root/name)
    with zipfile.ZipFile(destination/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as snapshot:
        for path in sorted(source):snapshot.write(path,path.relative_to(root).as_posix())
    (destination/'SOURCE_SHA256SUMS.txt').write_text(''.join(
        digest(p)+'  '+p.relative_to(root).as_posix()+'\n' for p in sorted(source)),encoding='utf8')
    files=sorted(p for p in destination.rglob('*') if p.is_file())
    (destination/'SHA256SUMS.txt').write_text(''.join(
        digest(p)+'  '+p.relative_to(destination).as_posix()+'\n' for p in files),encoding='utf8')
    with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as package:
        for path in sorted(destination.rglob('*')):
            if path.is_file():package.write(path,path.relative_to(destination).as_posix())
    # Verify extracted content logically without overwriting any existing files.
    with zipfile.ZipFile(archive) as package:
        assert package.testzip() is None
        for line in package.read('SHA256SUMS.txt').decode().splitlines():
            expected,name=line.split('  ',1)
            assert hashlib.sha256(package.read(name)).hexdigest()==expected
    print('CANDIDATE ONLY: '+str(destination))
    print('HEX: '+str(destination/'firmware/rosDemo.hex'))
    print('ZIP: '+str(archive))
    print('ZIP SHA256: '+digest(archive))
    print('Sources: '+str(len(source)))


if __name__=='__main__':main()
