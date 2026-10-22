"""Seal completed evidence and reviewed sources; no hardware access."""
import hashlib
import json
from pathlib import Path

here = Path(__file__).resolve().parent
root = next(p for p in here.parents if (p / 'Application').is_dir())
target = here / 'capture_manifest.json'
assert not target.exists(), 'Refuse to overwrite sealed evidence'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sources = [
    'Application/Board/board_config.c', 'Application/Board/board_config.h',
    'Application/Chassis/encoder_service.c',
    'Application/Chassis/wheel_speed_estimator.c',
    'Application/Chassis/motor_driver.c',
    'Application/Diagnostics/motion_trace.c',
    'Application/Communication/chassis_usb_protocol.c',
    'Core/Inc/main.h', 'Core/Src/tim.c', 'Core/Src/gpio.c', 'Core/Src/freertos.c',
    'Core/Src/system_stm32f4xx.c', 'build/Debug/compile_commands.json',
]
(here / 'source_audit_manifest.json').write_text(
    json.dumps({n: sha(root / n) for n in sources}, indent=2) + '\n', encoding='utf-8')
files = {p.relative_to(here).as_posix(): sha(p) for p in sorted(here.rglob('*'))
         if p.is_file() and p != target and '__pycache__' not in p.parts}
target.write_text(json.dumps(files, indent=2) + '\n', encoding='utf-8')
assert all(sha(here / n) == h for n, h in files.items())
print(f'Sealed {len(files)} evidence files; audited {len(sources)} source/build files; no hardware access.')
