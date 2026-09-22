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
    [ValidateRange(1, 120)]
    [double]$PreviewRate = 90.0,
    [ValidateRange(0, 100)]
    [int]$EncoderMinQuality = 60,
    [ValidateRange(30, 600)]
    [int]$RendererReadyTimeout = 240
)

$ErrorActionPreference = 'Stop'
if ($PSVersionTable.PSVersion.Major -lt 7) { throw 'PowerShell 7 or later is required.' }
$projectRoot = Split-Path -Parent $PSScriptRoot
$runDirectory = Join-Path $projectRoot 'run'
$logDirectory = Join-Path $projectRoot 'logs'
$statePath = Join-Path $runDirectory 'scene_runtime.json'
. (Join-Path $PSScriptRoot 'scene_process_helpers.ps1')
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
$simulationRate = [double]$instance.runtime.simulation_rate
$captureRate = [double]$instance.runtime.capture_rate_hz
$ikRate = [double]$instance.runtime.ik_rate_hz
$datasetCapture = [bool]$instance.runtime.dataset_capture
if ($datasetCapture -and $captureRate -notin @(1, 2, 5, 10, 30)) {
    throw 'LeRobot capture rate must be 1, 2, 5, 10 or 30 Hz on the 240 Hz dynamics / 30 Hz render clock.'
}
if ($ikRate -le 0 -or $ikRate -gt 240 -or $ikRate -ne [Math]::Truncate($ikRate) -or 240 % $ikRate -ne 0) {
    throw 'IK rate must be an integer divisor of 240 Hz (default 120 Hz); recreate scenes using the old 100 Hz configuration.'
}
if ($simulationRate -le 0 -or $captureRate -le 0 -or $ikRate -le 0) {
    throw 'Scene runtime rates must be positive.'
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
    preview_fps = [int][Math]::Round($PreviewRate)
    encoder_min_quality = $EncoderMinQuality
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
    if ((Split-Path -Leaf $ModelRoot) -eq 'platform') {
        & (Join-Path $PSScriptRoot 'prepare_sarm_scene.ps1') `
            -AdapterRoot $AdapterRoot -ModelRoot $ModelRoot -UnrealRoot $UnrealRoot -TemplateId ([string]$instance.template_id)
        if ($LASTEXITCODE -ne 0) { throw 'Ground-validation target asset preparation failed.' }
    }

    if (Test-TcpPort $RenderPort) { throw "UE render receiver port $RenderPort is already occupied." }
    $rendererArgs = @{
        UnrealRoot = $UnrealRoot
        Port = $RenderPort
        PixelStreamingURL = "ws://127.0.0.1:$PixelStreamerPort"
        PixelStreamingId = $PixelStreamingId
        PixelStreamingFps = [int][Math]::Round($PreviewRate)
        EncoderMinQuality = $EncoderMinQuality
        PixelStreamingCameraIds = $normalizedCameraIds
        PixelStreamingCameraWidth = $PixelStreamingCameraWidth
        PixelStreamingCameraHeight = $PixelStreamingCameraHeight
        PixelStreamingCameraFps = [int][Math]::Round($PreviewRate)
    }
    if ($datasetCapture) {
        $rendererArgs.CaptureProducts = @('rgb')
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
    $rendererReady = $false
    while (!$rendererReady -and [DateTime]::UtcNow -lt $deadline) {
        if (!(Get-Process -Id $rendererPid -ErrorAction SilentlyContinue)) {
            throw 'UE exited before its render receiver became ready.'
        }
        $rendererReady = Test-TcpPort $RenderPort
        if (!$rendererReady) { Start-Sleep -Milliseconds 300 }
    }
    if (!$rendererReady) {
        $ueLog = Join-Path $ueProject 'Saved\Logs\BskUnrealRenderer.log'
        throw "UE receiver did not become ready on port $RenderPort within $RendererReadyTimeout seconds. Check $ueLog."
    }
    # The readiness probe is a real TCP client. UE accepts one sender at a time,
    # so give its receiver thread a brief moment to observe the probe disconnect
    # before the Basilisk render bridge establishes the authoritative connection.
    Start-Sleep -Milliseconds 250

    Write-RuntimeState 'starting_simulation'
    $powershellExe = (Get-Process -Id $PID).Path
    $simulationArgs = @(
        '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', (Join-Path $PSScriptRoot 'run_simulation.ps1'),
        '-AdapterRoot', $AdapterRoot, '-ModelRoot', $ModelRoot,
        '-ControlPort', $ControlPort, '-RenderPort', $RenderPort,
        '-Duration', 0.0, '-SimulationRate', $simulationRate,
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

    $outcome = Wait-SceneRuntimeExit $rendererProcess $simulation {
        if (Test-Path -LiteralPath $statePath) {
            try { return (Get-Content -Raw -LiteralPath $statePath | ConvertFrom-Json).phase -eq 'stopped' } catch { }
        }
        return $false
    }
    if ($outcome.component -eq 'stopped') { exit 0 }
    if ($outcome.component -eq 'renderer') {
        throw "UE renderer exited unexpectedly with code $($outcome.code). See $ueProject\Saved\Logs\BskUnrealRenderer.log and Saved\Crashes; the associated simulation will be stopped."
    }
    if ($outcome.code -ne 0) {
        $errorLog = Join-Path $logDirectory "$stamp.simulation.err.log"
        $detail = ''
        if (Test-Path -LiteralPath $errorLog) {
            $cause = Get-Content -LiteralPath $errorLog -Tail 40 | Select-String 'BasiliskError:|ValueError:|RuntimeError:' | Select-Object -Last 1
            if ($cause) { $detail = ' ' + $cause.Line.Trim() }
        }
        throw "Basilisk/MJScene exited with code $($outcome.code).$detail UE is stopped because the authoritative simulation exited. Check logs\$stamp.simulation.err.log."
    }
    Write-RuntimeState 'completed'
} catch {
    $message = $_.Exception.Message
    Write-RuntimeState 'failed' $message
    throw
} finally {
    if ($simulation -and !$simulation.HasExited) {
        Stop-RecordedProcessTree $simulation.Id $runtimeState.simulation_start
    }
    if ($rendererPid -gt 0) {
        $renderer = Get-Process -Id $rendererPid -ErrorAction SilentlyContinue
        if ($renderer) { Stop-RecordedProcessTree $rendererPid $runtimeState.renderer_start }
    }
    $pidFile = Join-Path $ueProject 'Saved\BskRenderer.pid'
    if (Test-Path -LiteralPath $pidFile) { Remove-Item -LiteralPath $pidFile -Force -ErrorAction SilentlyContinue }
}
