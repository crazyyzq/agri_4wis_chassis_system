[CmdletBinding()]
param(
    [Parameter(Mandatory)] [ValidatePattern('^COM\d+$')] [string]$Port,
    [Parameter(Mandatory)] [string]$LogPath
)

$ErrorActionPreference = 'Stop'
$resolvedLog = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($LogPath)
$logDir = Split-Path -Parent $resolvedLog
if ($logDir) { New-Item -ItemType Directory -Force -Path $logDir | Out-Null }

$serial = [System.IO.Ports.SerialPort]::new($Port, 115200,
    [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.ReadTimeout = 100
$serial.WriteTimeout = 500
$writer = [System.IO.StreamWriter]::new($resolvedLog, $true, [System.Text.UTF8Encoding]::new($false))
$writer.AutoFlush = $true

try {
    $serial.Open()
    Write-Host "Connected to $Port at 115200 8N1. Press Ctrl-C to stop."
    while ($true) {
        try {
            $text = $serial.ReadExisting()
            if ($text.Length -gt 0) {
                [Console]::Write($text)
                $writer.Write($text)
            }
        } catch [System.TimeoutException] { }

        while ([Console]::KeyAvailable) {
            $key = [Console]::ReadKey($true)
            if ($key.Key -eq [ConsoleKey]::Enter) {
                $serial.Write("`r`n")
            } elseif ($key.KeyChar -ne [char]0) {
                $serial.Write($key.KeyChar.ToString())
            }
        }
        Start-Sleep -Milliseconds 10
    }
} finally {
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
    $writer.Dispose()
}
