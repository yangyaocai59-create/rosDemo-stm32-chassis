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
} finally { Pop-Location }
