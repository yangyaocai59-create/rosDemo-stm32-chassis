"""Host-only comparison with frozen .17. Never opens ports or flashes."""
import hashlib
from pathlib import Path
import re
import subprocess

root = Path(__file__).resolve().parents[3]
baseline = root/'docs/stm32_evidence/2026-10-21-v01817-lf-start-review/source_snapshot'
frozen = {
    'wheel_speed_controller.c': 'da64e35ae9f1daa5eff077a2b2b4c42fbb61c19e4021fd3767edeaaa590c08d0',
    'wheel_speed_controller.h': '836110055ab2e64a459942dc2dc433c46b8b916879b47bb6c127530286bb9171',
    'wheel_speed_control_config.h': 'd08f7cf26ebe8ce07adef2b3c79837401393361e1790ce546e7602e5bcfa2512',
    'motor_driver.h': '906df9369cabea75a3913541fd52271ef7bb877a4e7513f0b184e0418681429a',
    'chassis_config.h': 'd135bb847057595bf6965408e8ab3a7e47ecfb0b2e1c85bba4a2cf3e8b383047',
}
for name, expected in frozen.items():
    assert hashlib.sha256((baseline/name).read_bytes()).hexdigest() == expected, name
out = root/'build/tests/first_start_isolation'
out.mkdir(parents=True, exist_ok=True)
flags = ['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-IApplication/Chassis']
public = re.findall(r'^(?:bool|void|float)\s+(WheelSpeedController_\w+)\(',
                    (baseline/'wheel_speed_controller.h').read_text(), re.M)
assert len(public) >= 10

def run(command):
    subprocess.run(list(map(str, command)), cwd=root, check=True)

run(flags+['-D'+n+'=Baseline_'+n for n in public]+
    ['-c', baseline/'wheel_speed_controller.c', '-o', out/'baseline.o'])
run(flags+['-c', 'Application/Chassis/wheel_speed_controller.c', '-o', out/'candidate.o'])
run(flags+['Tools/tests/motion/test_first_start_isolation.c', out/'baseline.o', out/'candidate.o',
           '-lm', '-o', out/'isolation.exe'])
run([out/'isolation.exe'])
