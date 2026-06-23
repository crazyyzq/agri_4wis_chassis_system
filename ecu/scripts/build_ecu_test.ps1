[CmdletBinding()]
param(
    [switch]$ConfigureOnly
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

cmake --fresh -GNinja -S $app -B $build `
    '-DBOARD=ecu_isolation' "-DBOARD_SEARCH_PATH=$boards" `
    '-DCMAKE_BUILD_TYPE=flash_sdram_xip'
if ($LASTEXITCODE -ne 0 -or $ConfigureOnly) { exit $LASTEXITCODE }

cmake --build $build -j 4
exit $LASTEXITCODE
