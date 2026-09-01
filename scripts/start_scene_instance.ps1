param(
    [Parameter(Mandatory = $true)]
    [string]$SceneInstancePath,
    [Parameter(Mandatory = $true)]
    [string]$AdapterRoot,
    [Parameter(Mandatory = $true)]
    [string]$ModelRoot,
    [Parameter(Mandatory = $true)]
    [string]$UnrealRoot,
    [int]$ControlPort = 8766,
    [int]$CapturePort = 8767,
    [int]$RenderPort = 5558,
    [int]$PixelStreamerPort = 8888,
    [string]$PixelStreamingId = 'BskRenderer',
    [string[]]$PixelStreamingCameraIds = @(),
    [ValidateRange(160, 1920)]
    [int]$PixelStreamingCameraWidth = 640,
    [ValidateRange(90, 1080)]
    [int]$PixelStreamingCameraHeight = 360,
    [double]$PreviewRate = 60.0,
    [ValidateRange(30, 600)]
    [int]$RendererReadyTimeout = 240
)

$ErrorActionPreference = 'Stop'
if ($PSVersionTable.PSVersion.Major -lt 7) { throw 'PowerShell 7 or later is required.' }
$projectRoot = Split-Path -Parent $PSScriptRoot
$runDirectory = Join-Path $projectRoot 'run'
$logDirectory = Join-Path $projectRoot 'logs'
$statePath = Join-Path $runDirectory 'scene_runtime.json'
$ueProject = Join-Path $AdapterRoot 'Unreal\BskUnrealRenderer'
$ueScripts = Join-Path $ueProject 'scripts'
New-Item -ItemType Directory -Path $runDirectory,$logDirectory -Force | Out-Null

$SceneInstancePath = [IO.Path]::GetFullPath($SceneInstancePath)
$AdapterRoot = [IO.Path]::GetFullPath($AdapterRoot)
$ModelRoot = [IO.Path]::GetFullPath($ModelRoot)
$UnrealRoot = [IO.Path]::GetFullPath($UnrealRoot)
foreach ($path in @($SceneInstancePath, $AdapterRoot, $ModelRoot, $UnrealRoot, $ueScripts)) {
    if (!(Test-Path -LiteralPath $path)) { throw "Required scene runtime path does not exist: $path" }
}

$instance = Get-Content -Raw -LiteralPath $SceneInstancePath | ConvertFrom-Json
if ([string]$instance.schema -ne 'space-arm-scene-instance/1') {
    throw "Unsupported scene instance schema: $($instance.schema)"
}
$duration = [double]$instance.runtime.duration_s
$simulationRate = [double]$instance.runtime.simulation_rate
$captureRate = [double]$instance.runtime.capture_rate_hz
$ikRate = [double]$instance.runtime.ik_rate_hz
$datasetCapture = [bool]$instance.runtime.dataset_capture
if ($duration -le 0 -or $simulationRate -le 0 -or $captureRate -le 0 -or $ikRate -le 0) {
    throw 'Scene runtime rates and duration must be positive.'
}

$normalizedCameraIds = @()
foreach ($item in $PixelStreamingCameraIds) {
    foreach ($cameraId in ($item -split ',')) {
        if ($cameraId.Trim()) { $normalizedCameraIds += $cameraId.Trim() }
    }
}

$launcherProcess = Get-Process -Id $PID
$runtimeState = [ordered]@{
    phase = 'launching'
    updated_at_ns = [string][DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds() + '000000'
    launcher_pid = $PID
    launcher_start = $launcherProcess.StartTime.ToUniversalTime().Ticks
    instance_path = $SceneInstancePath
    instance = $instance
    adapter_root = $AdapterRoot
    renderer_pid = 0
    renderer_start = 0
    simulation_pid = 0
    simulation_start = 0
    error = ''
}

function Write-RuntimeState([string]$Phase, [string]$ErrorMessage = '') {
    $runtimeState.phase = $Phase
    $runtimeState.updated_at_ns = [string][DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds() + '000000'
    $runtimeState.error = $ErrorMessage
    $temporaryPath = "$statePath.tmp.$PID"
    $runtimeState | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath $temporaryPath -Encoding utf8
    Move-Item -LiteralPath $temporaryPath -Destination $statePath -Force
}

function Test-TcpPort([int]$Port) {
    $client = [Net.Sockets.TcpClient]::new()
    try {
        $async = $client.BeginConnect('127.0.0.1', $Port, $null, $null)
        if (!$async.AsyncWaitHandle.WaitOne(300)) { return $false }
        $client.EndConnect($async)
        return $client.Connected
    } catch { return $false } finally { $client.Dispose() }
}

$rendererPid = 0
$simulation = $null
try {
    & (Join-Path $PSScriptRoot 'stop_scene_instance.ps1') -Quiet -PreserveState -AdapterRoot $AdapterRoot
    Write-RuntimeState 'starting_renderer'

    if (Test-TcpPort $RenderPort) { throw "UE render receiver port $RenderPort is already occupied." }
    $rendererArgs = @{
        UnrealRoot = $UnrealRoot
        Port = $RenderPort
        PixelStreamingURL = "ws://127.0.0.1:$PixelStreamerPort"
        PixelStreamingId = $PixelStreamingId
        PixelStreamingFps = [int][Math]::Min(60, [Math]::Max(1, [Math]::Round($PreviewRate)))
        PixelStreamingCameraIds = $normalizedCameraIds
        PixelStreamingCameraWidth = $PixelStreamingCameraWidth
        PixelStreamingCameraHeight = $PixelStreamingCameraHeight
        PixelStreamingCameraFps = [int][Math]::Min(30, [Math]::Max(1, [Math]::Round($PreviewRate)))
    }
    if ($datasetCapture) {
        $rendererArgs.CaptureProducts = @('rgb', 'depth', 'segmentation')
        $rendererArgs.CaptureRate = $captureRate
        $rendererArgs.CaptureNetworkHost = '127.0.0.1'
        $rendererArgs.CaptureNetworkPort = $CapturePort
    }
    & (Join-Path $ueScripts 'start_renderer.ps1') @rendererArgs
    $rendererPid = [int](Get-Content -Raw -LiteralPath (Join-Path $ueProject 'Saved\BskRenderer.pid'))
    $rendererProcess = Get-Process -Id $rendererPid -ErrorAction SilentlyContinue
    if (!$rendererProcess) { throw 'The UE renderer process disappeared immediately after startup.' }
    $runtimeState.renderer_pid = $rendererPid
    $runtimeState.renderer_start = $rendererProcess.StartTime.ToUniversalTime().Ticks
    Write-RuntimeState 'starting_renderer'

    $deadline = [DateTime]::UtcNow.AddSeconds($RendererReadyTimeout)
    while (!(Test-TcpPort $RenderPort) -and [DateTime]::UtcNow -lt $deadline) {
        if (!(Get-Process -Id $rendererPid -ErrorAction SilentlyContinue)) {
            throw 'UE exited before its render receiver became ready.'
        }
        Start-Sleep -Milliseconds 300
    }
    if (!(Test-TcpPort $RenderPort)) {
        $ueLog = Join-Path $ueProject 'Saved\Logs\BskUnrealRenderer.log'
        throw "UE receiver did not become ready on port $RenderPort within $RendererReadyTimeout seconds. Check $ueLog."
    }

    Write-RuntimeState 'starting_simulation'
    $powershellExe = (Get-Process -Id $PID).Path
    $simulationArgs = @(
        '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', (Join-Path $PSScriptRoot 'run_simulation.ps1'),
        '-AdapterRoot', $AdapterRoot, '-ModelRoot', $ModelRoot,
        '-ControlPort', $ControlPort, '-RenderPort', $RenderPort,
        '-Duration', $duration, '-SimulationRate', $simulationRate,
        '-CaptureRate', $captureRate, '-IkRate', $ikRate,
        '-SceneInstancePath', $SceneInstancePath
    )
    $stamp = [string]$instance.instance_id
    $simulation = Start-Process -FilePath $powershellExe -ArgumentList $simulationArgs -PassThru -WindowStyle Hidden `
        -RedirectStandardOutput (Join-Path $logDirectory "$stamp.simulation.out.log") `
        -RedirectStandardError (Join-Path $logDirectory "$stamp.simulation.err.log")
    $runtimeState.simulation_pid = $simulation.Id
    $runtimeState.simulation_start = $simulation.StartTime.ToUniversalTime().Ticks
    Write-RuntimeState 'running'

    $simulation.WaitForExit()
    $exitCode = $simulation.ExitCode
    $persistedPhase = ''
    if (Test-Path -LiteralPath $statePath) {
        try { $persistedPhase = [string](Get-Content -Raw -LiteralPath $statePath | ConvertFrom-Json).phase } catch { }
    }
    if ($persistedPhase -eq 'stopped') { exit 0 }
    if ($exitCode -ne 0) { throw "Basilisk/MJScene exited with code $exitCode. Check logs\$stamp.simulation.err.log." }
    Write-RuntimeState 'completed'
} catch {
    $message = $_.Exception.Message
    Write-RuntimeState 'failed' $message
    throw
} finally {
    if ($simulation -and !$simulation.HasExited) {
        Stop-Process -Id $simulation.Id -Force -ErrorAction SilentlyContinue
    }
    if ($rendererPid -gt 0) {
        $renderer = Get-Process -Id $rendererPid -ErrorAction SilentlyContinue
        if ($renderer) { Stop-Process -Id $rendererPid -Force -ErrorAction SilentlyContinue }
    }
    $pidFile = Join-Path $ueProject 'Saved\BskRenderer.pid'
    if (Test-Path -LiteralPath $pidFile) { Remove-Item -LiteralPath $pidFile -Force -ErrorAction SilentlyContinue }
}
