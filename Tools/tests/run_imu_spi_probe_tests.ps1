# Native tests only: no serial port, reset, sensor command or firmware download.
$ErrorActionPreference = 'Stop'
$spiProbeTestRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Push-Location -LiteralPath $spiProbeTestRoot
try {
    New-Item -ItemType Directory -Path 'build\tests' -Force | Out-Null
    & gcc -std=c11 -Wall -Wextra -Werror -DIMU_SPI_ID_PROBE_ONLY=1 -DIMU_USE_MPU6500_SPI=0 `
        -ITools/tests/imu_spi_probe/stubs -IApplication/Diagnostics `
        -IApplication/Sensors -IApplication/Services -IApplication/Storage `
        Tools/tests/imu_spi_probe/test_imu_spi_probe.c `
        Application/Sensors/mpu_spi_probe.c `
        Application/Diagnostics/imu_spi_identity_test.c `
        Application/Sensors/imu_service.c `
        Application/Diagnostics/imu_static_test.c `
        -lm -o build/tests/imu_spi_probe_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'SPI identity native build failed' }
    & (Join-Path $spiProbeTestRoot 'build/tests/imu_spi_probe_test.exe')
    if ($LASTEXITCODE -ne 0) { throw 'SPI identity native regression failed' }
    & (Join-Path $PSScriptRoot 'run_usb_tx_tests.ps1')
} finally { Pop-Location }
