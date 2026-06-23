$ErrorActionPreference = 'Stop'
& (Join-Path $PSScriptRoot 'build_ecu_test.ps1') -ConfigureOnly
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

$repoRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$project = Join-Path $repoRoot 'tmp\ecu_board_test_build\segger_embedded_studio\ecu_board_test.emProject'
if (-not (Test-Path -LiteralPath $project -PathType Leaf)) {
    throw "SEGGER Embedded Studio project was not generated: $project"
}
Write-Output $project
