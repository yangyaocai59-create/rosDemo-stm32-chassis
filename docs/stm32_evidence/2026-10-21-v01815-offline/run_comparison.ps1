# Offline only. Re-run from any directory; no serial, GPIO or flashing.
$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..\..')).Path
Push-Location -LiteralPath $repoRoot
try {
  $oldRoot = 'docs/stm32_evidence/2026-10-21-v01814-handoff-review/source_snapshot'
  foreach ($policy in @(@{Name='v01814';Root=$oldRoot},@{Name='v01815';Root='Application/Chassis'})) {
    $name = $policy.Name
    $sourceRoot = $policy.Root
    & gcc -std=c11 -Wall -Wextra -Werror "-I$sourceRoot" "$PSScriptRoot/compare_support.c" "$sourceRoot/wheel_speed_controller.c" -lm -o "build/tests/$($name)_support_compare.exe"
    if ($LASTEXITCODE -ne 0) { throw 'support comparison build failed' }
    & "./build/tests/$($name)_support_compare.exe" | Tee-Object -FilePath "$PSScriptRoot/$($name)_support_compare.log"
    if ($LASTEXITCODE -ne 0) { throw 'support comparison failed' }
  }
} finally { Pop-Location }
