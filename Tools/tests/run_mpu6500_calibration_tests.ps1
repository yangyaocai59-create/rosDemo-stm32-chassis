# Windows native virtual-memory Flash/register mocks only; no serial/hardware.
$ErrorActionPreference = 'Stop'
$mpuCalibrationRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Push-Location -LiteralPath $mpuCalibrationRoot
try {
    New-Item -ItemType Directory -Path 'build\tests' -Force | Out-Null
    & gcc -std=c11 -Wall -Wextra -Werror `
        -ITools/tests/imu_spi_probe/stubs -IApplication/Diagnostics `
        -IApplication/Sensors -IApplication/Services -IApplication/Storage `
        Tools/tests/imu_spi_probe/test_mpu6500_calibration.c `
        Application/Sensors/mpu6500.c Application/Sensors/mpu_spi_probe.c `
        Application/Sensors/imu_service.c Application/Diagnostics/imu_static_test.c `
        -lm -o build/tests/mpu6500_calibration_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'MPU6500 calibration native build failed' }
    & (Join-Path $mpuCalibrationRoot 'build/tests/mpu6500_calibration_test.exe')
    if ($LASTEXITCODE -ne 0) { throw 'MPU6500 calibration regression failed' }
    & (Join-Path $PSScriptRoot 'run_mpu6500_polling_tests.ps1')
} finally { Pop-Location }
