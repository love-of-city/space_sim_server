param(
    [string]$Python = '',
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
    [string]$DataRoot = '',
    [string]$AdapterRoot = '',
    [string]$ModelRoot = '',
    [string]$UnrealRoot = '',
    [string]$PowerShellExe = '',
    [int]$RenderPort = 5558,
    [int]$PixelStreamerPort = 8888,
    [string]$PixelStreamingCameraIds = '',
    [int]$PixelStreamingCameraWidth = 640,
    [int]$PixelStreamingCameraHeight = 360,
    [ValidateRange(1, 120)]
    [double]$PreviewRate = 90.0,
    [ValidateRange(0, 100)]
    [int]$EncoderMinQuality = 60,
    [int]$RendererReadyTimeout = 240,
    [double]$IkRate = 120.0,
    [double]$SimulationRate = 1.0,
    [double]$CaptureRate = 30.0,
    [switch]$DefaultDatasetCapture = $true
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
. (Join-Path $PSScriptRoot 'python_runtime.ps1')
$pythonExe = Resolve-SpaceSimPython -RepositoryRoot $projectRoot -RequestedPython $Python -RequiredModules @('fastapi', 'pydantic', 'uvicorn', 'lerobot.datasets.lerobot_dataset')
$env:SPACE_SIM_PYTHON = $pythonExe
if (!$AdapterRoot) { $AdapterRoot = $env:SPACE_SIM_RUNTIME_ADAPTER_ROOT }
if (!$ModelRoot) { $ModelRoot = $env:SPACE_SIM_RUNTIME_MODEL_ROOT }
if (!$UnrealRoot) { $UnrealRoot = $env:SPACE_SIM_RUNTIME_UNREAL_ROOT }
if (!$PowerShellExe) { $PowerShellExe = $env:SPACE_SIM_RUNTIME_POWERSHELL_EXE }
if (!$DataRoot) { $DataRoot = Join-Path $projectRoot 'data\episodes' }
$frontendRoot = Join-Path $projectRoot 'frontend'
Push-Location $frontendRoot
try {
    if (!(Test-Path -LiteralPath (Join-Path $frontendRoot 'node_modules'))) {
        npm.cmd ci --no-audit --no-fund
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
    '--pixel-streaming-streamer-id', $PixelStreamingId, '--data-root', ([IO.Path]::GetFullPath($DataRoot)),
    '--runtime-encoder-min-quality', $EncoderMinQuality)
if ($PixelStreamingSignallingUrl) { $arguments += @('--pixel-streaming-signalling-url', $PixelStreamingSignallingUrl) }
# Secrets reach Python through its environment, not process command lines.
if ($StreamAccessJwtSecret) { $env:SPACE_SIM_STREAM_JWT_SECRET = $StreamAccessJwtSecret }
if ($StreamAccessKey) { $env:SPACE_SIM_STREAM_ACCESS_KEY = $StreamAccessKey }
if ($AdapterRoot -and $ModelRoot -and $UnrealRoot -and $PowerShellExe) {
    $arguments += @(
        '--runtime-adapter-root', ([IO.Path]::GetFullPath($AdapterRoot)),
        '--runtime-model-root', ([IO.Path]::GetFullPath($ModelRoot)),
        '--runtime-unreal-root', ([IO.Path]::GetFullPath($UnrealRoot)),
        '--runtime-powershell-exe', ([IO.Path]::GetFullPath($PowerShellExe)),
        '--runtime-render-port', $RenderPort, '--runtime-pixel-streamer-port', $PixelStreamerPort,
        '--runtime-pixel-streaming-camera-width', $PixelStreamingCameraWidth,
        '--runtime-pixel-streaming-camera-height', $PixelStreamingCameraHeight,
        '--runtime-preview-rate', $PreviewRate, '--runtime-renderer-ready-timeout', $RendererReadyTimeout,
        '--runtime-ik-rate', $IkRate, '--runtime-simulation-rate', $SimulationRate,
        '--runtime-capture-rate', $CaptureRate
    )
    if ($DefaultDatasetCapture) { $arguments += '--runtime-default-dataset-capture' }
    else { $arguments += '--no-runtime-default-dataset-capture' }
    foreach ($cameraId in ($PixelStreamingCameraIds -split ',' | Where-Object { $_ })) {
        $arguments += @('--runtime-pixel-streaming-camera-id', $cameraId.Trim())
    }
}
foreach ($streamer in ($PixelStreamingCameraStreamers -split ';' | Where-Object { $_ })) {
    $arguments += @('--pixel-streaming-camera-streamer', $streamer)
}
& $pythonExe @arguments
