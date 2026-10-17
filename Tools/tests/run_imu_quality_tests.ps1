# Native-only regression. Never opens COM ports or programs hardware.
$ErrorActionPreference = 'Stop'
$imuQualityRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Push-Location -LiteralPath $imuQualityRoot
try {
    New-Item -ItemType Directory -Path 'build\tests' -Force | Out-Null
    & gcc -std=c11 -Wall -Wextra -Werror `
        -ITools/tests/imu_spi_probe/stubs -IApplication/Diagnostics `
        -IApplication/Chassis -IApplication/Safety `
        Tools/tests/imu_spi_probe/test_imu_calibration_guard.c `
        Application/Diagnostics/imu_calibration_guard.c `
        -o build/tests/imu_calibration_guard_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'IMU calibration guard build failed' }
    & (Join-Path $imuQualityRoot 'build/tests/imu_calibration_guard_test.exe')
    if ($LASTEXITCODE -ne 0) { throw 'IMU calibration guard regression failed' }
    & (Join-Path $PSScriptRoot 'run_mpu6500_calibration_tests.ps1')
} finally { Pop-Location }
