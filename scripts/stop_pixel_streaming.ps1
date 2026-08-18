param([switch]$Quiet)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$statePath = Join-Path $projectRoot 'run\pixel_streaming.json'
if (!(Test-Path -LiteralPath $statePath)) { exit 0 }
$state = Get-Content -Raw -LiteralPath $statePath | ConvertFrom-Json
$process = Get-Process -Id ([int]$state.pid) -ErrorAction SilentlyContinue
if ($process -and $process.StartTime.ToUniversalTime().Ticks -eq [long]$state.start_ticks) {
    Stop-Process -Id $process.Id -Force
    if (!$Quiet) { Write-Output "Stopped Pixel Streaming signalling server PID $($process.Id)." }
}
Remove-Item -LiteralPath $statePath -Force
