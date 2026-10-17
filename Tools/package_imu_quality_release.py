"""Build/test/freeze 0.14.4 SOFTWARE CANDIDATE. No serial or programmer calls.

Refuses to overwrite a delivery or its final verification evidence. Historical
0.14.3 delivery and hardware evidence are only read, never changed.
"""
import hashlib
import json
from pathlib import Path
import re
import shutil
import struct
import subprocess
import tempfile
import zipfile


ROOT = Path(__file__).resolve().parents[1]
VERSION = '0.14.4-imu-drdy-quality'
STEM = 'imu_quality_0.14.4_20261016'
VERIFY = ROOT / 'build/verification' / STEM / 'final'
DEST = ROOT / 'build/deliveries' / STEM
ARCHIVE = DEST.parent / 'STM32_IMU_QUALITY_0.14.4_SOFTWARE_CANDIDATE_20261016.zip'
BUNDLE = Path.home() / 'AppData/Local/stm32cube/bundles'
BIN = BUNDLE / 'gnu-tools-for-stm32/14.3.1+st.2/bin'
CMAKE = BUNDLE / 'cmake/4.3.1+st.1/bin/cmake.exe'


def sha(data):
    return hashlib.sha256(data).hexdigest()


def run(name, command):
    result = subprocess.run([str(x) for x in command], cwd=ROOT,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    text = result.stdout.decode('utf8', errors='replace')
    (VERIFY / name).write_text(text, encoding='utf8')
    if result.returncode:
        raise RuntimeError(f'{name} failed ({result.returncode}):\n{text[-5000:]}')
    return text


def decode_hex(data):
    memory = {}
    base = 0
    eof = False
    for line in data.decode('ascii').splitlines():
        assert not eof, 'data after HEX EOF'
        assert line.startswith(':')
        row = bytes.fromhex(line[1:])
        assert len(row) == row[0] + 5 and sum(row) & 255 == 0, 'HEX checksum'
        address = int.from_bytes(row[1:3], 'big')
        kind, payload = row[3], row[4:-1]
        if kind == 0:
            for offset, value in enumerate(payload):
                absolute = base + address + offset
                assert absolute not in memory, 'overlapping HEX'
                memory[absolute] = value
        elif kind == 4:
            assert len(payload) == 2
            base = int.from_bytes(payload, 'big') << 16
        elif kind == 1:
            assert len(payload) == 0
            eof = True
        else:
            assert kind == 5 and len(payload) == 4, 'unexpected HEX record'
    assert eof and memory
    return memory


def outside_user_regions(data):
    text = data.decode('utf8').replace('\r\n', '\n')
    return re.sub(r'/\* USER CODE BEGIN ([^*]+)\*/.*?/\* USER CODE END \1\*/',
                  r'/* USER REGION \1*/', text, flags=re.S)


def main():
    assert not DEST.exists() and not ARCHIVE.exists() and not VERIFY.exists(), 'refusing overwrite'
    assert not (ROOT / '.git').exists(), 'review Git status before changing release workflow'
    VERIFY.mkdir(parents=True)
    build = run('debug_clean_build.log', [CMAKE, '--build', '--preset', 'Debug', '--clean-first'])
    assert '[92/92]' in build
    assert not re.search(r'\b(?:error|warning):', build, re.I), 'build diagnostics'
    usage = {key: int(re.search(rf'\b{key}:\s+(\d+) B', build)[1])
             for key in ('FLASH', 'RAM', 'CCMRAM')}
    assert usage == dict(FLASH=163516, RAM=55304, CCMRAM=0), 'update documented footprint after review'
    native = run('native_tests.log', ['powershell', '-NoProfile', '-ExecutionPolicy', 'Bypass',
                                     '-File', 'Tools/tests/run_imu_quality_tests.ps1'])
    assert 'PASS: Cancel/new Start' in native and 'PASS: production calibration guard' in native
    python = run('python_tests.log', ['python', '-m', 'unittest', 'discover', '-s', 'Tools/tests',
                                     '-p', 'test_imu*.py', '-v'])
    assert 'Ran 8 tests' in python and '\nOK' in python
    run('legacy_python_selftest.log', ['python', 'Tools/usb_cdc_test.py', '--self-test'])
    run('size.log', [BIN / 'arm-none-eabi-size.exe', 'build/Debug/rosDemo.elf'])
    symbols = run('symbols.log', [BIN / 'arm-none-eabi-nm.exe', '-n', 'build/Debug/rosDemo.elf'])
    assert 'HAL_FLASHEx_Erase' not in symbols, 'runtime erase unexpectedly linked'

    debug = ROOT / 'build/Debug'
    with tempfile.TemporaryDirectory(prefix='imu_quality_artifacts_') as temp:
        check_hex = Path(temp) / 'from_elf.hex'
        run('objcopy_hex.log', [BIN / 'arm-none-eabi-objcopy.exe', '-O', 'ihex', debug/'rosDemo.elf', check_hex])
        assert check_hex.read_bytes() == (debug/'rosDemo.hex').read_bytes(), 'HEX not from this ELF'
    run('objcopy_bin.log', [BIN / 'arm-none-eabi-objcopy.exe', '-O', 'binary', debug/'rosDemo.elf', debug/'rosDemo.bin'])
    image = (debug / 'rosDemo.bin').read_bytes()
    memory = decode_hex((debug/'rosDemo.hex').read_bytes())
    first, end = min(memory), max(memory) + 1
    assert first == 0x08000000 and end < 0x080C0000, 'image enters calibration Flash'
    assert len(image) == end - first
    assert all(image[address-first] == value for address, value in memory.items()), 'BIN/HEX mismatch'
    sp, reset = struct.unpack_from('<II', image)
    assert sp == 0x20020000 and reset & 1 and first <= reset-1 < end, 'vector table'
    assert VERSION.encode() in image, 'wrong embedded firmware identity'
    dates = sorted(set(x.decode() for x in re.findall(rb'(?:Jan|Feb|Mar|Apr|May|Jun|Jul|Aug|Sep|Oct|Nov|Dec) [ \d]\d 20\d\d', image)))
    times = sorted(set(x.decode() for x in re.findall(rb'[012]\d:[0-5]\d:[0-5]\d', image)))
    hashes = {name: sha((debug/name).read_bytes()) for name in ('rosDemo.elf','rosDemo.hex','rosDemo.bin')}

    source = []
    for folder in ('Application','Core','Drivers','Middlewares','USB_DEVICE','cmake','Tools','docs'):
        source.extend(p for p in (ROOT/folder).rglob('*') if p.is_file() and
                      '__pycache__' not in p.parts and p.suffix not in ('.pyc','.exe','.o','.obj'))
    source += [ROOT/name for name in ('CMakeLists.txt','CMakePresets.json','rosDemo.ioc',
                                      'STM32F407xx_FLASH.ld','startup_stm32f407xx.s','.gitignore')]
    source = sorted(source)
    baseline = ROOT/'build/deliveries/imu_raw_0.14.3_interface_verified_20261016/source_snapshot.zip'
    protected_prefixes = ('Drivers/','Middlewares/','USB_DEVICE/','cmake/',
                          'Application/Board/','Application/Chassis/','Application/Safety/',
                          'Application/Power/','Application/Communication/','Application/Storage/')
    protected_files = ('rosDemo.ioc','STM32F407xx_FLASH.ld','CMakePresets.json','startup_stm32f407xx.s')
    changed = []
    with zipfile.ZipFile(baseline) as old:
        names = set(old.namelist())
        for path in source:
            name = path.relative_to(ROOT).as_posix()
            prior = old.read(name) if name in names else None
            current = path.read_bytes()
            if prior == current:
                continue
            assert not name.startswith(protected_prefixes) and name not in protected_files, f'protected changed: {name}'
            if name.startswith('Core/'):
                assert name in ('Core/Src/freertos.c','Core/Src/stm32f4xx_it.c')
                assert prior and outside_user_regions(prior) == outside_user_regions(current), f'generated region changed: {name}'
            changed.append(dict(path=name,kind='added' if prior is None else 'modified',sha256=sha(current)))

    status = dict(version=VERSION, status='SOFTWARE_CANDIDATE_NOT_HARDWARE_VERIFIED', git_commit=None,
                  build_date=dates, build_time=times, compiler_errors=0, compiler_warnings=0,
                  memory_bytes=usage, native_programs_passed=10, python_tests_passed=8,
                  legacy_python_selftest_passed=True, image_base=hex(first), image_end_exclusive=hex(end),
                  initial_sp=hex(sp), reset_vector=hex(reset), sha256=hashes,
                  source_files=len(source), changes_vs_frozen_0_14_3=changed,
                  protected_sources_unchanged=True, core_changes_user_regions_only=True,
                  assistant_opened_serial=False, assistant_flashed=False, assistant_calibrated=False,
                  hardware_verified=False, drift_improvement_verified=False,
                  usb_frame_layout_changed=False, flash_record_layout_changed=False,
                  bias_profile_changed=True, explicit_calibration_required_after_full_erase=True,
                  ros_runtime_estimator_implemented=False)
    (VERIFY/'artifact_verification.json').write_text(json.dumps(status,indent=2)+'\n',encoding='utf8')
    DEST.mkdir(parents=True)
    (DEST/'firmware').mkdir()
    for name in hashes:
        shutil.copy2(debug/name, DEST/'firmware'/name)
    shutil.copytree(VERIFY, DEST/'verification')
    stack = DEST/'verification/stack_usage'
    stack.mkdir()
    for name in ('Application/Diagnostics/imu_static_test.c.su',
                 'Application/Diagnostics/imu_calibration_guard.c.su',
                 'Application/Sensors/imu_service.c.su','Core/Src/freertos.c.su'):
        shutil.copy2(debug/'CMakeFiles/rosDemo.dir'/name,stack/Path(name).name)
    shutil.copy2(ROOT/'docs/STM32_IMU_QUALITY_0.14.4_HANDOFF_20261016.md',DEST/'README_FIRST.md')
    (DEST/'release_status.json').write_text(json.dumps(status,indent=2)+'\n',encoding='utf8')
    with zipfile.ZipFile(DEST/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as snapshot:
        for path in source:
            snapshot.write(path,path.relative_to(ROOT).as_posix())
    manifest = ''.join(sha(path.read_bytes())+'  '+path.relative_to(ROOT).as_posix()+'\n' for path in source)
    (DEST/'SOURCE_SHA256SUMS.txt').write_text(manifest,encoding='utf8')
    manifest = ''.join(sha(path.read_bytes())+'  '+path.relative_to(DEST).as_posix()+'\n'
                       for path in sorted(DEST.rglob('*')) if path.is_file())
    (DEST/'SHA256SUMS.txt').write_text(manifest,encoding='utf8')
    with zipfile.ZipFile(ARCHIVE,'x',zipfile.ZIP_DEFLATED) as package:
        for path in sorted(DEST.rglob('*')):
            if path.is_file(): package.write(path,path.relative_to(DEST).as_posix())
    with zipfile.ZipFile(ARCHIVE) as package:
        assert package.testzip() is None
        for line in package.read('SHA256SUMS.txt').decode().splitlines():
            expected,name=line.split('  ',1)
            assert sha(package.read(name)) == expected
        with zipfile.ZipFile(DEST/'source_snapshot.zip') as snapshot:
            assert snapshot.testzip() is None
            for line in package.read('SOURCE_SHA256SUMS.txt').decode().splitlines():
                expected,name=line.split('  ',1)
                assert sha(snapshot.read(name)) == expected
    print(json.dumps(dict(result='SOFTWARE_CANDIDATE_PACKAGED',folder=str(DEST),archive=str(ARCHIVE),
                          archive_sha256=sha(ARCHIVE.read_bytes()),firmware_sha256=hashes,
                          memory_bytes=usage,build_date=dates,build_time=times,
                          changed_files=len(changed),source_files=len(source)),indent=2))


if __name__ == '__main__':
    main()
