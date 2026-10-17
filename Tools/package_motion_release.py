"""Clean build, regress and freeze a motion candidate. No hardware access.

Creates a new immutable delivery; refuses to overwrite earlier deliveries.
Reuses the existing release's Intel HEX and USER-region verification helpers.
"""
from pathlib import Path
import argparse
import json
import re
import shutil
import struct
import subprocess
import tempfile
import zipfile
from package_imu_quality_release import sha, decode_hex, outside_user_regions

ROOT = Path(__file__).resolve().parents[1]
VERSION = '0.15.0-motion-smoothness'
STEM = 'motion_smoothness_0.15.0_20261017'
VERIFY = ROOT / 'build/verification' / STEM / 'final'
DEST = ROOT / 'build/deliveries' / STEM
ARCHIVE = DEST.parent / 'STM32_MOTION_SMOOTHNESS_0.15.0_SOFTWARE_CANDIDATE_20261017.zip'
BUNDLE = Path.home() / 'AppData/Local/stm32cube/bundles'
BIN = BUNDLE / 'gnu-tools-for-stm32/14.3.1+st.2/bin'
CMAKE = BUNDLE / 'cmake/4.3.1+st.1/bin/cmake.exe'


def run(name, command):
    result = subprocess.run([str(x) for x in command], cwd=ROOT,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    output = result.stdout.decode('utf8', errors='replace')
    (VERIFY / name).write_text(output, encoding='utf8')
    if result.returncode:
        raise RuntimeError(f'{name} failed: {output[-5000:]}')
    return output


def main():
    global VERSION, STEM, VERIFY, DEST, ARCHIVE
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--safety-stack', action='store_true',
                        help='freeze the narrow 0.15.1 safety-stack update')
    parser.add_argument('--imu-telemetry', action='store_true',
                        help='freeze the narrow 0.15.2 sample-driven IMU telemetry update')
    parser.add_argument('--fractional-bias', action='store_true',
                        help='freeze 0.16.0 milli-RAW calibration/storage/RAW v2')
    args = parser.parse_args()
    assert sum((args.safety_stack, args.imu_telemetry, args.fractional_bias)) <= 1, 'select only one release'
    baseline_version = '0.14.4-imu-drdy-quality'
    baseline = ROOT/'build/deliveries/imu_quality_0.14.4_20261016/source_snapshot.zip'
    handoff = ROOT/'docs/STM32_MOTION_SMOOTHNESS_0.15.0_HANDOFF_20261017.md'
    if args.safety_stack:
        VERSION = '0.15.1-safety-stack'
        STEM = 'safety_stack_0.15.1_20261017'
        VERIFY = ROOT/'build/verification'/STEM/'final'
        DEST = ROOT/'build/deliveries'/STEM
        ARCHIVE = DEST.parent/'STM32_SAFETY_STACK_0.15.1_SOFTWARE_CANDIDATE_20261017.zip'
        baseline_version = '0.15.0-motion-smoothness'
        baseline = ROOT/'build/deliveries/motion_smoothness_0.15.0_20261017/source_snapshot.zip'
        handoff = ROOT/'docs/STM32_SAFETY_STACK_0.15.1_HANDOFF_20261017.md'
    if args.imu_telemetry:
        VERSION = '0.15.2-imu-sample-telemetry'
        STEM = 'imu_sample_telemetry_0.15.2_20261017'
        VERIFY = ROOT/'build/verification'/STEM/'final'
        DEST = ROOT/'build/deliveries'/STEM
        ARCHIVE = DEST.parent/'STM32_IMU_SAMPLE_TELEMETRY_0.15.2_SOFTWARE_CANDIDATE_20261017.zip'
        baseline_version = '0.15.1-safety-stack'
        baseline = ROOT/'build/deliveries/safety_stack_0.15.1_20261017/source_snapshot.zip'
        handoff = ROOT/'docs/STM32_IMU_SAMPLE_TELEMETRY_0.15.2_HANDOFF_20261017.md'
    if args.fractional_bias:
        VERSION = '0.16.0-imu-fractional-bias'
        STEM = 'imu_fractional_bias_0.16.0_20261017'
        VERIFY = ROOT/'build/verification'/STEM/'final'
        DEST = ROOT/'build/deliveries'/STEM
        ARCHIVE = DEST.parent/'STM32_IMU_FRACTIONAL_BIAS_0.16.0_SOFTWARE_CANDIDATE_20261017.zip'
        baseline_version = '0.15.2-imu-sample-telemetry'
        baseline = ROOT/'build/deliveries/imu_sample_telemetry_0.15.2_20261017/source_snapshot.zip'
        handoff = ROOT/'docs/STM32_IMU_FRACTIONAL_BIAS_0.16.0_HANDOFF_20261017.md'
    assert not DEST.exists() and not ARCHIVE.exists() and not VERIFY.exists(), 'refusing overwrite'
    assert not (ROOT / '.git').exists(), 'review Git state before packaging'
    VERIFY.mkdir(parents=True)
    build = run('debug_clean_build.log', [CMAKE, '--build', '--preset', 'Debug', '--clean-first'])
    assert '[95/95]' in build and 'Linking C executable' in build
    assert not re.search(r'\b(?:error|warning):', build, re.I)
    stack_report = None
    if args.safety_stack:
        def frame_bytes(path, function):
            return int(re.search(rf':{function}\t(\d+)\t',
                                 path.read_text(encoding='utf8'))[1])
        old_stack = ROOT/'build/deliveries/motion_smoothness_0.15.0_20261017/verification/stack_usage'
        new_stack = ROOT/'build/Debug/CMakeFiles/rosDemo.dir'
        old_task = frame_bytes(old_stack/'freertos.c.su', 'StartSafetyTask')
        old_nested = frame_bytes(old_stack/'chassis_runtime.c.su', 'ChassisRuntime_SafetyProcess')
        new_task = frame_bytes(new_stack/'Core/Src/freertos.c.su', 'StartSafetyTask')
        new_nested = frame_bytes(new_stack/'Application/Chassis/chassis_runtime.c.su', 'ChassisRuntime_SafetyProcess')
        assert old_task == 168 and old_nested == 144
        assert new_task <= 32 and new_nested <= 32, 'safety stack regression'
        stack_report = dict(allocated_safety_stack_bytes=1024,
            start_safety_frame_before=old_task, start_safety_frame_after=new_task,
            runtime_safety_frame_before=old_nested, runtime_safety_frame_after=new_nested,
            measured_previous_static_min_free_bytes=216,
            new_hardware_high_watermark_verified=False,
            note='Function frames are not complete call-chain bounds or measured free stack.')
    usage = {name: int(re.search(rf'\b{name}:\s+(\d+) B', build)[1])
             for name in ('FLASH', 'RAM', 'CCMRAM')}
    motion = run('motion_native_tests.log', ['powershell', '-NoProfile', '-ExecutionPolicy',
                                          'Bypass', '-File', 'Tools/tests/run_motion_tests.ps1'])
    assert 'PASS: real >100ms' in motion and 'PASS: STOP preempts stale' in motion
    native = run('imu_usb_log_native_tests.log', ['powershell', '-NoProfile', '-ExecutionPolicy',
                                  'Bypass', '-File', 'Tools/tests/run_imu_quality_tests.ps1'])
    assert 'PASS: permanent motor/safety/IMU' in native and 'PASS: Cancel/new Start' in native
    py = run('python_tests.log', ['python', '-m', 'unittest', 'discover', '-s', 'Tools/tests',
                                  '-p', 'test_imu*.py', '-v'])
    python_count = 19 if args.fractional_bias else (13 if args.imu_telemetry else 8)
    assert f'Ran {python_count} tests' in py and '\nOK' in py
    run('legacy_python_selftest.log', ['python', 'Tools/usb_cdc_test.py', '--self-test'])
    run('size.log', [BIN/'arm-none-eabi-size.exe', 'build/Debug/rosDemo.elf'])
    symbols = run('symbols.log', [BIN/'arm-none-eabi-nm.exe', '-n', 'build/Debug/rosDemo.elf'])
    assert 'HAL_FLASHEx_Erase' not in symbols, 'unexpected runtime erase'

    debug = ROOT / 'build/Debug'
    with tempfile.TemporaryDirectory(prefix='motion_artifacts_') as temp:
        check_hex = Path(temp) / 'from_elf.hex'
        run('objcopy_hex.log', [BIN/'arm-none-eabi-objcopy.exe', '-O', 'ihex', debug/'rosDemo.elf', check_hex])
        assert check_hex.read_bytes() == (debug/'rosDemo.hex').read_bytes()
    run('objcopy_bin.log', [BIN/'arm-none-eabi-objcopy.exe', '-O', 'binary', debug/'rosDemo.elf', debug/'rosDemo.bin'])
    image = (debug/'rosDemo.bin').read_bytes()
    memory = decode_hex((debug/'rosDemo.hex').read_bytes())
    first, end = min(memory), max(memory) + 1
    assert first == 0x08000000 and end < 0x080C0000
    assert len(image) == end-first and all(image[a-first] == v for a,v in memory.items())
    sp, reset = struct.unpack_from('<II', image)
    assert sp == 0x20020000 and reset & 1 and first <= reset-1 < end
    assert VERSION.encode() in image
    dates = sorted(set(x.decode() for x in re.findall(rb'(?:Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|Sep|Oct|Nov|Dec) [ \d]\d 20\d\d', image)))
    times = sorted(set(x.decode() for x in re.findall(rb'[012]\d:[0-5]\d:[0-5]\d', image)))
    artifacts = ('rosDemo.elf', 'rosDemo.hex', 'rosDemo.bin', 'rosDemo.map')
    hashes = {name: sha((debug/name).read_bytes()) for name in artifacts}

    source = []
    for folder in ('Application','Core','Drivers','Middlewares','USB_DEVICE','cmake','Tools','docs'):
        source.extend(p for p in (ROOT/folder).rglob('*') if p.is_file() and
                      '__pycache__' not in p.parts and p.suffix not in ('.pyc','.exe','.o','.obj'))
    source += [ROOT/name for name in ('CMakeLists.txt','CMakePresets.json','rosDemo.ioc',
                 'STM32F407xx_FLASH.ld','startup_stm32f407xx.s','.gitignore')]
    source = sorted(source)
    protected = ('Drivers/','Middlewares/','USB_DEVICE/','cmake/', 'Application/Board/',
                 'Application/Sensors/','Application/Storage/','Application/Communication/',
                 'Application/Power/')
    exact = ('rosDemo.ioc','CMakePresets.json','STM32F407xx_FLASH.ld','startup_stm32f407xx.s',
             'Application/Chassis/encoder_service.c','Application/Chassis/encoder_service.h',
             'Application/Chassis/wheel_speed_estimator.c','Application/Chassis/wheel_speed_estimator.h',
             'Application/Chassis/wheel_speed_control_config.h','Application/Chassis/chassis_config.h')
    changed = []
    with zipfile.ZipFile(baseline) as old:
        old_names = set(old.namelist())
        current_names = {p.relative_to(ROOT).as_posix() for p in source}
        assert not (old_names-current_names), 'baseline source removed'
        for p in source:
            name = p.relative_to(ROOT).as_posix()
            before = old.read(name) if name in old_names else None
            after = p.read_bytes()
            if before == after: continue
            if args.safety_stack and name.startswith(('Application/', 'Core/')):
                assert name in {
                    'Core/Src/freertos.c', 'Application/Chassis/chassis_runtime.c',
                    'Application/Chassis/chassis_controller.c',
                    'Application/Chassis/chassis_controller.h',
                    'Application/System/firmware_info.h'
                }, f'outside narrow stack update: {name}'
            telemetry_allowed = {
                'Application/Communication/chassis_usb_protocol.c',
                'Application/Communication/chassis_usb_protocol.h',
                'Application/System/firmware_info.h'
            }
            if args.imu_telemetry and name.startswith(('Application/', 'Core/')):
                assert name in telemetry_allowed, f'outside narrow telemetry update: {name}'
            fractional_allowed = telemetry_allowed | {
                'Application/Sensors/imu_bias_units.h',
                'Application/Sensors/imu_service.c', 'Application/Sensors/imu_service.h',
                'Application/Storage/imu_bias_storage.c', 'Application/Storage/imu_bias_storage.h',
                'Application/Diagnostics/imu_static_test.c', 'Core/Src/freertos.c'
            }
            if args.fractional_bias and name.startswith(('Application/', 'Core/')):
                assert name in fractional_allowed, f'outside fractional bias update: {name}'
            assert ((args.imu_telemetry and name in telemetry_allowed) or
                    (args.fractional_bias and name in fractional_allowed) or
                    (not name.startswith(protected) and name not in exact)), f'protected changed: {name}'
            if name.startswith('Core/'):
                assert name == 'Core/Src/freertos.c' and before is not None
                assert outside_user_regions(before) == outside_user_regions(after)
            changed.append(dict(path=name, kind='added' if before is None else 'modified', sha256=sha(after)))
    status = dict(version=VERSION, baseline=baseline_version,
        status='SOFTWARE_CANDIDATE_NOT_HARDWARE_VERIFIED', git_commit=None,
        build_date=dates, build_time=times, compiler_errors=0, compiler_warnings=0,
        memory_bytes=usage, native_programs_passed=14 if (args.imu_telemetry or args.fractional_bias) else 13,
        python_tests_passed=python_count,
        legacy_python_selftest_passed=True, image_base=hex(first), image_end_exclusive=hex(end),
        initial_sp=hex(sp), reset_vector=hex(reset), sha256=hashes,
        source_files=len(source), changes_vs_baseline=changed,
        protected_sources_unchanged=True, core_changes_user_regions_only=True,
        assistant_opened_serial=False, assistant_flashed=False, assistant_moved_motors=False,
        hardware_verified=False, smoothness_improvement_verified=False,
        usb_frame_layout_changed=False, bias_profile_changed=False,
        explicit_calibration_required_after_full_erase=True)
    if args.imu_telemetry:
        status['protected_sources_note'] = 'Communication protocol .c/.h intentionally changed; all other protected sources, Core, pins, acquisition and bias storage unchanged.'
        status['sample_driven_telemetry_native_verified'] = True
        status['telemetry_continuity_hardware_verified'] = False
        status['drift_improvement_verified'] = False
        run('previous_static_raw_drift.json', ['python', 'Tools/imu_raw_drift_report.py',
            'build/hardware_tests/cold_restore_0.15.1_20261017_060512_265678/raw_samples.jsonl'])
    if args.fractional_bias:
        assert 'fingerprint=0x53974331; format=3' in native
        assert 'PASS: actual 500-sample fractional calibration' in native
        assert 'PASS: RAW v2 exact signed milli-baseline' in native
        status.update(bias_profile_changed=True, flash_record_format=3,
            bias_units='signed milli-RAW, 1000 units per sensor LSB',
            imu_profile_fingerprint='0x53974331', raw_v2_message='0x86', raw_v2_selector=2,
            new_raw_v2_payload_bytes=52, existing_legacy_layouts_unchanged=True,
            raw_v1_precision='coarse integer bias view; use RAW v2 for exact correction',
            previous_integer_flash_records_rejected=True, drift_improvement_verified=False,
            boot_recalibration_enabled=False, runtime_bias_learning_enabled=False,
            protected_sources_note='Only ten allowlisted production files changed; Core changes confined to USER regions. Pins, CubeMX, sensor registers, motor, encoder, timing and safety policy unchanged.')
    if stack_report is not None:
        status['safety_stack_analysis'] = stack_report
        (VERIFY/'safety_stack_analysis.json').write_text(
            json.dumps(stack_report,indent=2)+'\n',encoding='utf8')
    (VERIFY/'artifact_verification.json').write_text(json.dumps(status,indent=2)+'\n',encoding='utf8')
    DEST.mkdir(parents=True)
    (DEST/'firmware').mkdir()
    for name in artifacts: shutil.copy2(debug/name, DEST/'firmware'/name)
    shutil.copytree(VERIFY, DEST/'verification')
    stack = DEST/'verification/stack_usage'
    stack.mkdir()
    for name in ('Core/Src/freertos.c.su','Application/Services/debug_log.c.su',
                 'Application/Chassis/chassis_runtime.c.su', 'Application/Chassis/chassis_controller.c.su',
                 'Application/Diagnostics/motion_trace.c.su',
                 'Application/Sensors/imu_service.c.su',
                 'Application/Diagnostics/imu_static_test.c.su',
                 'Application/Communication/chassis_usb_protocol.c.su'):
        shutil.copy2(debug/'CMakeFiles/rosDemo.dir'/name, stack/Path(name).name)
    shutil.copy2(handoff, DEST/'README_FIRST.md')
    (DEST/'release_status.json').write_text(json.dumps(status,indent=2)+'\n',encoding='utf8')
    with zipfile.ZipFile(DEST/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as snapshot:
        for p in source: snapshot.write(p,p.relative_to(ROOT).as_posix())
    (DEST/'SOURCE_SHA256SUMS.txt').write_text(''.join(sha(p.read_bytes())+'  '+p.relative_to(ROOT).as_posix()+'\n'
        for p in source),encoding='utf8')
    (DEST/'SHA256SUMS.txt').write_text(''.join(sha(p.read_bytes())+'  '+p.relative_to(DEST).as_posix()+'\n'
        for p in sorted(DEST.rglob('*')) if p.is_file()),encoding='utf8')
    with zipfile.ZipFile(ARCHIVE,'x',zipfile.ZIP_DEFLATED) as package:
        for p in sorted(DEST.rglob('*')):
            if p.is_file(): package.write(p,p.relative_to(DEST).as_posix())
    with zipfile.ZipFile(ARCHIVE) as package:
        assert package.testzip() is None
        for line in package.read('SHA256SUMS.txt').decode().splitlines():
            expected,name=line.split('  ',1)
            assert sha(package.read(name)) == expected
    with zipfile.ZipFile(DEST/'source_snapshot.zip') as snapshot:
        assert snapshot.testzip() is None
        for line in (DEST/'SOURCE_SHA256SUMS.txt').read_text().splitlines():
            expected,name=line.split('  ',1)
            assert sha(snapshot.read(name)) == expected
    print(json.dumps(dict(result='SOFTWARE_CANDIDATE_PACKAGED', folder=str(DEST), archive=str(ARCHIVE),
        archive_sha256=sha(ARCHIVE.read_bytes()), firmware_sha256=hashes, memory_bytes=usage,
        build_date=dates, build_time=times, changed_files=len(changed), source_files=len(source)),indent=2))


if __name__ == '__main__':
    main()
