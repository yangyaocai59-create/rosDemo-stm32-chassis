# Native mocks only: no COM port, real sensor write or Flash programming.
$ErrorActionPreference = 'Stop'
$mpuPollingRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Push-Location -LiteralPath $mpuPollingRoot
try {
    New-Item -ItemType Directory -Path 'build\tests' -Force | Out-Null
    & gcc -std=c11 -Wall -Wextra -Werror -DIMU_CALIBRATION_ENABLED=0 `
        -ITools/tests/imu_spi_probe/stubs -IApplication/Diagnostics `
        -IApplication/Sensors -IApplication/Services -IApplication/Storage `
        Tools/tests/imu_spi_probe/test_mpu6500_polling.c `
        Application/Sensors/mpu6500.c Application/Sensors/mpu_spi_probe.c `
        Application/Sensors/imu_service.c Application/Diagnostics/imu_static_test.c `
        -lm -o build/tests/mpu6500_polling_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'MPU6500 native build failed' }
    & (Join-Path $mpuPollingRoot 'build/tests/mpu6500_polling_test.exe')
    if ($LASTEXITCODE -ne 0) { throw 'MPU6500 native regression failed' }
    & (Join-Path $PSScriptRoot 'run_imu_spi_probe_tests.ps1')
} finally { Pop-Location }
