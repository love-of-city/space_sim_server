param(
    [string]$ApiHost = '127.0.0.1',
    [int]$ApiPort = 8000,
    [int]$ControlPort = 8766,
    [int]$CapturePort = 8767,
    [int]$PixelStreamingPlayerPort = 8080,
    [string]$PixelStreamingId = 'BskRenderer',
    [string]$PixelStreamingSignallingUrl = '',
    [string]$PixelStreamingCameraStreamers = '',
    [string]$StreamAccessJwtSecret = '',
    [string]$StreamAccessKey = '',
    [string]$DataRoot = ''
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
if (!$DataRoot) { $DataRoot = Join-Path $projectRoot 'data\episodes' }
$frontendRoot = Join-Path $projectRoot 'frontend'
Push-Location $frontendRoot
try {
    if (!(Test-Path -LiteralPath (Join-Path $frontendRoot 'node_modules'))) {
        npm.cmd install --no-audit --no-fund
        if ($LASTEXITCODE -ne 0) { throw 'Operator console dependency installation failed.' }
    }
    npm.cmd run build
    if ($LASTEXITCODE -ne 0) { throw 'Operator console build failed.' }
} finally {
    Pop-Location
}
$env:PYTHONPATH = Join-Path $projectRoot 'backend'
Set-Location $projectRoot
$arguments = @('-m', 'space_arm_platform.main', '--host', $ApiHost, '--port', $ApiPort, '--simulation-port', $ControlPort,
    '--capture-port', $CapturePort, '--pixel-streaming-player-port', $PixelStreamingPlayerPort,
    '--pixel-streaming-streamer-id', $PixelStreamingId, '--data-root', ([IO.Path]::GetFullPath($DataRoot)))
if ($PixelStreamingSignallingUrl) { $arguments += @('--pixel-streaming-signalling-url', $PixelStreamingSignallingUrl) }
if ($StreamAccessJwtSecret) { $arguments += @('--stream-access-jwt-secret', $StreamAccessJwtSecret) }
if ($StreamAccessKey) { $arguments += @('--stream-access-key', $StreamAccessKey) }
foreach ($streamer in ($PixelStreamingCameraStreamers -split ';' | Where-Object { $_ })) {
    $arguments += @('--pixel-streaming-camera-streamer', $streamer)
}
python @arguments
