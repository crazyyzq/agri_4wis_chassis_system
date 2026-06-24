# Generate a native SES 8.28 project and repair the known SDK 1.11 XML defect.
[CmdletBinding()]
param(
    [switch]$PeriodicCanTx,
    [switch]$PeriodicRs485Tx,
    [switch]$PeriodicRs232Tx
)

$ErrorActionPreference = 'Stop'
$buildParams = @{
    ConfigureOnly = $true
    PeriodicCanTx = $PeriodicCanTx.IsPresent
    PeriodicRs485Tx = $PeriodicRs485Tx.IsPresent
    PeriodicRs232Tx = $PeriodicRs232Tx.IsPresent
}
& (Join-Path $PSScriptRoot 'build_ecu_test.ps1') @buildParams
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

$repoRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$project = Join-Path $repoRoot 'tmp\ecu_board_test_build\segger_embedded_studio\ecu_board_test.emProject'
if (-not (Test-Path -LiteralPath $project -PathType Leaf)) {
    throw "SEGGER Embedded Studio project was not generated: $project"
}

# HPM SDK 1.11.0's SES XML template emits linker_output_format twice (hex and
# bin) on the same configuration element. SES then opens without loading the
# solution, and emBuild silently completes in zero seconds. Remove only the
# known obsolete hex entry and validate the generated XML before handing it off.
$projectText = [System.IO.File]::ReadAllText($project)
$formatMatches = [regex]::Matches($projectText, 'linker_output_format="(?:hex|bin)"')
if ($formatMatches.Count -eq 2 -and
    $projectText.Contains('linker_output_format="hex"') -and
    $projectText.Contains('linker_output_format="bin"')) {
    $projectText = [regex]::Replace(
        $projectText,
        '\r?\n\s*linker_output_format="hex"',
        '',
        1)
    [System.IO.File]::WriteAllText(
        $project,
        $projectText,
        [System.Text.UTF8Encoding]::new($false))
}
try {
    [xml](Get-Content -LiteralPath $project -Raw) | Out-Null
} catch {
    throw "Generated SES project is not valid XML: $($_.Exception.Message)"
}
Write-Output $project
