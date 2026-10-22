# Host-only regression. Does not open ports, move motors or flash firmware.
$ErrorActionPreference = 'Stop'
$motionRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Push-Location -LiteralPath $motionRoot
try {
    New-Item -ItemType Directory -Path 'build\tests' -Force | Out-Null
    & gcc -std=c11 -Wall -Wextra -Werror -ITools/tests/imu_spi_probe/stubs -IApplication/Chassis -IApplication/Safety -IApplication/Services Tools/tests/motion/test_motion_core.c Application/Chassis/chassis_motion_profile.c Application/Chassis/wheel_speed_controller.c Application/Safety/fault_manager.c Application/Services/control_timing.c -lm -o build/tests/motion_core_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'motion core native build failed' }
    & .\build\tests\motion_core_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'motion core regression failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -ITools/tests/motion/stubs -IApplication/Chassis -IApplication/Safety Tools/tests/motion/test_motor_driver.c Application/Chassis/motor_driver.c -o build/tests/motor_driver_motion_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'motor driver native build failed' }
    & .\build\tests\motor_driver_motion_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'motor driver regression failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -ITools/tests/imu_spi_probe/stubs -IApplication/Chassis -IApplication/Safety -IApplication/Services -IApplication/Diagnostics Tools/tests/motion/test_runtime_motion.c Application/Chassis/chassis_runtime.c Application/Chassis/chassis_controller.c Application/Chassis/chassis_kinematics.c Application/Chassis/chassis_motion_profile.c Application/Chassis/wheel_speed_controller.c Application/Diagnostics/motion_trace.c -lm -o build/tests/runtime_motion_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'runtime motion native build failed' }
    & .\build\tests\runtime_motion_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'runtime motion regression failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -ITools/tests/imu_spi_probe/stubs -IApplication/Chassis Tools/tests/motion/test_front_pivot.c Application/Chassis/wheel_speed_controller.c -lm -o build/tests/front_pivot_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'front pivot native build failed' }
    & .\build\tests\front_pivot_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'front pivot regression failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -IApplication/Chassis Tools/tests/motion/test_front_support_handoff.c Application/Chassis/wheel_speed_controller.c -lm -o build/tests/front_support_handoff_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'front support handoff native build failed' }
    & .\build\tests\front_support_handoff_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'front support handoff regression failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -IApplication/Chassis Tools/tests/motion/test_front_load_confirmation.c Application/Chassis/wheel_speed_controller.c -lm -o build/tests/front_load_confirmation_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'front load confirmation native build failed' }
    & .\build\tests\front_load_confirmation_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'front load confirmation regression failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -IApplication/Chassis Tools/tests/motion/test_lf_coordination.c -lm -o build/tests/lf_coordination_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'LF coordination native build failed' }
    & .\build\tests\lf_coordination_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'LF coordination regression failed' }
    & python Tools/tests/motion/run_first_start_isolation.py
    if ($LASTEXITCODE -ne 0) { throw 'first-start isolation regression failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -IApplication/Chassis Tools/tests/motion/test_lf_first_start.c -lm -o build/tests/lf_first_start_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'LF first-start native build failed' }
    & .\build\tests\lf_first_start_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'LF first-start regression failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -ITools/tests/imu_spi_probe/stubs -IApplication/Chassis -IApplication/Safety -IApplication/Services -IApplication/Diagnostics Tools/tests/motion/test_lf_first_start_safety.c Application/Chassis/chassis_runtime.c Application/Chassis/chassis_controller.c Application/Chassis/chassis_kinematics.c Application/Chassis/chassis_motion_profile.c Application/Diagnostics/motion_trace.c -lm -o build/tests/lf_first_start_safety_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'LF first-start safety native build failed' }
    & .\build\tests\lf_first_start_safety_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'LF first-start safety regression failed' }
    & gcc -std=c11 -Wall -Wextra -Werror -ITools/tests/imu_spi_probe/stubs -IApplication/Chassis -IApplication/Safety -IApplication/Services Tools/tests/motion/test_front_recovery_safety.c Application/Chassis/chassis_runtime.c Application/Chassis/chassis_controller.c Application/Chassis/chassis_kinematics.c Application/Chassis/chassis_motion_profile.c Application/Chassis/wheel_speed_controller.c Application/Safety/fault_manager.c -lm -o build/tests/front_recovery_safety_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'front recovery safety native build failed' }
    & .\build\tests\front_recovery_safety_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'front recovery safety regression failed' }
    & python Tools/tests/motion/run_lr_running_tests.py
    if ($LASTEXITCODE -ne 0) { throw 'LR running recovery/safety regression failed' }
} finally { Pop-Location }
