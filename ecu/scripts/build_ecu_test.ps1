# Configure and build the flash_sdram_xip image with the pinned SDK 1.11 tools.
[CmdletBinding()]
param(
    [switch]$ConfigureOnly,
    [switch]$PeriodicCanTx,
    [switch]$PeriodicRs485Tx,
    [switch]$PeriodicRs232Tx
)

$ErrorActionPreference = 'Stop'
$ecuRoot = Split-Path -Parent $PSScriptRoot
$repoRoot = Split-Path -Parent $ecuRoot
$sdkEnv = Join-Path $ecuRoot 'sdk_env_v1.11.0'
$env:HPM_SDK_BASE = Join-Path $sdkEnv 'hpm_sdk'
$env:GNURISCV_TOOLCHAIN_PATH = Join-Path $sdkEnv 'toolchains\rv32imac_zicsr_zifencei_multilib_b_ext-win'
$env:PATH = @(
    (Join-Path $sdkEnv 'tools\cmake\bin')
    (Join-Path $env:GNURISCV_TOOLCHAIN_PATH 'bin')
    (Join-Path $sdkEnv 'tools\python3')
    (Join-Path $sdkEnv 'tools\ninja')
    $env:PATH
) -join ';'

$app = Join-Path $ecuRoot 'apps\ecu_board_test'
$boards = Join-Path $ecuRoot 'boards'
$build = Join-Path $repoRoot 'tmp\ecu_board_test_build'
$canTx = if ($PeriodicCanTx) { 1 } else { 0 }
$rs485Tx = if ($PeriodicRs485Tx) { 1 } else { 0 }
$rs232Tx = if ($PeriodicRs232Tx) { 1 } else { 0 }

cmake --fresh -GNinja -S $app -B $build `
    '-DBOARD=ecu_isolation' "-DBOARD_SEARCH_PATH=$boards" `
    '-DCMAKE_BUILD_TYPE=flash_sdram_xip' `
    "-DECU_PERIODIC_CAN_TX=$canTx" `
    "-DECU_PERIODIC_RS485_TX=$rs485Tx" `
    "-DECU_PERIODIC_RS232_TX=$rs232Tx"
if ($LASTEXITCODE -ne 0 -or $ConfigureOnly) { exit $LASTEXITCODE }

cmake --build $build -j 4
exit $LASTEXITCODE
