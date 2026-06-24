# Download the GNU ELF through JTAG and reject ambiguous J-Link transcripts.
[CmdletBinding()]
param(
    [string]$JLinkExe = 'C:\Program Files\SEGGER\JLink_V916\JLink.exe'
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$elf = Join-Path $repoRoot 'tmp\ecu_board_test_build\output\demo.elf'
if (-not (Test-Path -LiteralPath $elf -PathType Leaf)) {
    throw "Firmware image not found. Run build_ecu_test.ps1 first: $elf"
}
if (-not (Test-Path -LiteralPath $JLinkExe -PathType Leaf)) {
    throw "J-Link Commander not found: $JLinkExe"
}

# J-Link Commander does not expand environment variables in loadfile commands.
# Generate a local command file so worktrees and cloned repositories remain portable.
$template = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'load_ecu_test.jlink') -Raw
$commandFile = Join-Path $repoRoot 'tmp\load_ecu_test.generated.jlink'
$template.Replace('__ECU_TEST_ELF__', $elf.Replace('\', '/')) |
    Set-Content -LiteralPath $commandFile -Encoding ascii

$output = & $JLinkExe -device HPM6750xVMx -if JTAG -speed 4000 `
    -AutoConnect 1 -ExitOnError 1 -JTAGConf -1,-1 `
    -CommanderScript $commandFile 2>&1
$exitCode = $LASTEXITCODE
$output | Write-Output
$transcript = $output -join "`n"

# Commander may return zero after an interactive connection prompt without
# executing loadfile. Treat missing target power or missing download evidence
# as failure so a production report can never record a false flash success.
if ($exitCode -ne 0 -or $transcript -match 'VTref=0\.000V' -or
    $transcript -notmatch 'Downloading file' -or $transcript -notmatch 'O\.K\.') {
    Write-Error 'J-Link did not confirm a powered target and completed ELF download.'
    exit 1
}
exit 0
