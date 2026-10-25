# Native host tests only. Does not open COM ports or build/flash ARM firmware.
param([ValidateSet('191','200','201','202','203','204','205')][string]$ControlRevision = '191')
$ErrorActionPreference = 'Stop'
$testProjectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Push-Location -LiteralPath $testProjectRoot
try {
    New-Item -ItemType Directory -Path 'build\tests' -Force | Out-Null
    function Invoke-HostTest([string]$name, [string[]]$compilerArgs) {
        & gcc -std=c11 -Wall -Wextra -Werror "-DCHASSIS_CONTROL_REVISION=$ControlRevision" @compilerArgs -o "build/tests/$name.exe"
        if ($LASTEXITCODE -ne 0) { throw "$name native build failed" }
        & (Join-Path $testProjectRoot "build/tests/$name.exe")
        if ($LASTEXITCODE -ne 0) { throw "$name failed" }
    }
    $transportIncludes = @('-ITools/tests/usb_cdc_transport/stubs', '-IApplication/Communication')
    $protocolIncludes = $transportIncludes + @('-DDEBUG','-IApplication/Chassis','-IApplication/Diagnostics','-IApplication/Power','-IApplication/Safety','-IApplication/Sensors','-IApplication/Services')
    Invoke-HostTest 'usb_cdc_transport_test' ($transportIncludes + @('Tools/tests/usb_cdc_transport/test_usb_cdc_transport.c','Application/Communication/usb_cdc_transport.c'))
    Invoke-HostTest 'usb_protocol_stop_test' ($protocolIncludes + @('Tools/tests/usb_cdc_transport/test_usb_protocol_stop.c','Application/Communication/usb_cdc_transport.c'))
    Invoke-HostTest 'usb_tx_queue_test' ($transportIncludes + @('Tools/tests/usb_cdc_transport/test_usb_tx_queue.c'))
    Invoke-HostTest 'usb_tx_burst_test' ($protocolIncludes + @('Tools/tests/usb_cdc_transport/test_usb_tx_burst.c','Application/Communication/usb_cdc_transport.c'))
    Invoke-HostTest 'usb_imu_raw_test' ($protocolIncludes + @('Tools/tests/usb_cdc_transport/test_usb_imu_raw.c','Application/Communication/usb_cdc_transport.c'))
    Invoke-HostTest 'usb_imu_schedule_test' ($protocolIncludes + @('Tools/tests/usb_cdc_transport/test_usb_imu_schedule.c','Application/Communication/usb_cdc_transport.c'))
    Invoke-HostTest 'debug_log_deferred_test' @('-ITools/tests/debug_log/stubs','-IApplication/Services','-IApplication/Chassis','-IApplication/Sensors','-IApplication/System','Tools/tests/debug_log/test_debug_log.c')
} finally { Pop-Location }
