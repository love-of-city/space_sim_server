param(
    [string]$AdapterRoot = '',
    [string]$ModelRoot = '',
    [string]$UnrealRoot = 'E:\UE5.6',
    [int]$ApiPort = 8000,
    [int]$ControlPort = 8766,
    [int]$CapturePort = 8767,
    [int]$RenderPort = 5558,
    [int]$PixelStreamerPort = 8888,
    [int]$PixelPlayerPort = 8080,
    [string]$PixelStreamingId = 'BskRenderer',
    [string[]]$PixelStreamingCameraIds = @(
        'teleop/camera/spacecraft_overview',
        'teleop/camera/so101_wrist_cam'
    ),
    [ValidateRange(160, 1920)]
    [int]$PixelStreamingCameraWidth = 640,
    [ValidateRange(90, 1080)]
    [int]$PixelStreamingCameraHeight = 360,
    [double]$Duration = 300.0,
    [double]$SimulationRate = 1.0,
    [double]$CaptureRate = 10.0,
    [double]$PreviewRate = 60.0,
    [switch]$RemoteAccess,
    [string]$PublicHost = '127.0.0.1',
    [string]$PixelPlayerPublicUrl = '',
    [string]$StreamAccessKey = '',
    [string]$StreamJwtSecret = '',
    [string]$IceServersJson = '[]',
    [string]$TurnUrlsJson = '[]',
    [string]$TurnAuthSecret = '',
    [switch]$Rebuild,
    [switch]$ReimportAssets,
    [switch]$NoBrowser
)

$ErrorActionPreference = 'Stop'
if ($PreviewRate -le 0 -or $PreviewRate -gt 60) { throw 'PreviewRate must be in (0, 60].' }
$projectRoot = Split-Path -Parent $PSScriptRoot
$workspaceRoot = Split-Path -Parent $projectRoot
if (!$AdapterRoot) { $AdapterRoot = Join-Path $workspaceRoot 'space_sim_UE_adapter' }
if (!$ModelRoot) { $ModelRoot = Join-Path $workspaceRoot 'test\model\spacecraft_and_arm' }
$ueProject = Join-Path $AdapterRoot 'Unreal\BskUnrealRenderer'
$ueScripts = Join-Path $ueProject 'scripts'
$runDirectory = Join-Path $projectRoot 'run'
$logDirectory = Join-Path $projectRoot 'logs'
New-Item -ItemType Directory -Path $runDirectory,$logDirectory -Force | Out-Null

foreach ($path in @($AdapterRoot, $ModelRoot, $UnrealRoot, $ueScripts)) {
    if (!(Test-Path -LiteralPath $path)) { throw "Required path does not exist: $path" }
}

# Stop only PIDs previously recorded by this project before binding fixed ports.
& (Join-Path $PSScriptRoot 'stop_platform.ps1') -Quiet

if ($RemoteAccess) {
    if (!$StreamAccessKey) {
        $bytes = New-Object byte[] 24
        $generator = [Security.Cryptography.RandomNumberGenerator]::Create()
        try { $generator.GetBytes($bytes) } finally { $generator.Dispose() }
        $StreamAccessKey = [Convert]::ToBase64String($bytes).TrimEnd('=').Replace('+','-').Replace('/','_')
    }
    if (!$StreamJwtSecret) {
        $bytes = New-Object byte[] 48
        $generator = [Security.Cryptography.RandomNumberGenerator]::Create()
        try { $generator.GetBytes($bytes) } finally { $generator.Dispose() }
        $StreamJwtSecret = [Convert]::ToBase64String($bytes)
    }
    if (!$PixelPlayerPublicUrl) { $PixelPlayerPublicUrl = "ws://$PublicHost`:$PixelPlayerPort" }
    Write-Output 'Starting JWT-protected Pixel Streaming 2 signalling server ...'
    & (Join-Path $PSScriptRoot 'start_secure_pixel_streaming.ps1') `
        -StreamerPort $PixelStreamerPort -PlayerPort $PixelPlayerPort -JwtSecret $StreamJwtSecret `
        -IceServersJson $IceServersJson -TurnUrlsJson $TurnUrlsJson -TurnAuthSecret $TurnAuthSecret
} else {
    Write-Output 'Starting the local Pixel Streaming 2 signalling server ...'
    & (Join-Path $PSScriptRoot 'start_pixel_streaming.ps1') -UnrealRoot $UnrealRoot `
        -StreamerPort $PixelStreamerPort -PlayerPort $PixelPlayerPort
}

Write-Output 'Checking the existing UE assets and runtime plugin ...'
$catalog = Join-Path $ueProject 'Saved\AssetImport\cubesat_so101.catalog.json'
if ($ReimportAssets -or !(Test-Path -LiteralPath $catalog -PathType Leaf)) {
    & (Join-Path $ueScripts 'prepare_spacecraft_arm_assets.ps1') -ModelRoot $ModelRoot `
        -Variant combined -UnrealRoot $UnrealRoot -NormalMode preserve -Force:$ReimportAssets
    if ($LASTEXITCODE -ne 0) { throw 'Spacecraft/arm asset preparation failed.' }
} else {
    Write-Output "Reusing prepared CubeSat + SO-101 asset catalog: $catalog"
}
& (Join-Path $ueScripts 'prepare_runtime_materials.ps1') -UnrealRoot $UnrealRoot
$pluginBinary = Join-Path $ueProject 'Plugins\BskUnrealRuntime\Binaries\Win64\UnrealEditor-BskUnrealRuntime.dll'
if ($Rebuild -or !(Test-Path -LiteralPath $pluginBinary)) {
    & (Join-Path $ueScripts 'build.ps1') -UnrealRoot $UnrealRoot
    if ($LASTEXITCODE -ne 0) { throw 'UE runtime plugin build failed.' }
}

$powershellExe = (Get-Command powershell.exe).Source
$cameraStreamers = @()
foreach ($cameraId in $PixelStreamingCameraIds) {
    $safeId = $cameraId -replace '[^A-Za-z0-9_-]', '_'
    $label = if ($cameraId -match 'wrist') { 'WristCamera' } elseif ($cameraId -match 'overview') { 'SpacecraftOverview' } else { $cameraId }
    $cameraStreamers += "${PixelStreamingId}__${safeId}=$label"
}
$backendArgs = @(
    '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', (Join-Path $PSScriptRoot 'run_backend.ps1'),
    '-ApiHost', $(if ($RemoteAccess) { '0.0.0.0' } else { '127.0.0.1' }),
    '-ApiPort', $ApiPort, '-ControlPort', $ControlPort, '-CapturePort', $CapturePort,
    '-PixelStreamingPlayerPort', $PixelPlayerPort, '-PixelStreamingId', $PixelStreamingId,
    '-PixelStreamingCameraStreamers', ($cameraStreamers -join ';')
)
if ($RemoteAccess) {
    $backendArgs += @(
        '-PixelStreamingSignallingUrl', $PixelPlayerPublicUrl,
        '-StreamAccessJwtSecret', $StreamJwtSecret,
        '-StreamAccessKey', $StreamAccessKey
    )
}
$backend = Start-Process -FilePath $powershellExe -ArgumentList $backendArgs -PassThru -WindowStyle Hidden `
    -RedirectStandardOutput (Join-Path $logDirectory 'backend.out.log') `
    -RedirectStandardError (Join-Path $logDirectory 'backend.err.log')

try {
    $deadline = [DateTime]::UtcNow.AddSeconds(20)
    $backendReady = $false
    while (!$backendReady -and [DateTime]::UtcNow -lt $deadline) {
        if ($backend.HasExited) { throw 'Backend exited during startup. Check logs\backend.err.log.' }
        try {
            $health = Invoke-RestMethod -Uri "http://127.0.0.1:$ApiPort/api/health" -TimeoutSec 1
            $backendReady = [bool]$health.ok
        } catch { Start-Sleep -Milliseconds 250 }
    }
    if (!$backendReady) { throw "Backend did not become ready on port $ApiPort." }

    & (Join-Path $ueScripts 'start_renderer.ps1') -UnrealRoot $UnrealRoot -Port $RenderPort `
        -CaptureProducts @('rgb','depth','segmentation') -CaptureRate $CaptureRate `
        -CaptureNetworkHost '127.0.0.1' -CaptureNetworkPort $CapturePort `
        -PixelStreamingURL "ws://127.0.0.1:$PixelStreamerPort" -PixelStreamingId $PixelStreamingId `
        -PixelStreamingFps ([int][Math]::Round($PreviewRate)) `
        -PixelStreamingCameraIds $PixelStreamingCameraIds `
        -PixelStreamingCameraWidth $PixelStreamingCameraWidth `
        -PixelStreamingCameraHeight $PixelStreamingCameraHeight `
        -PixelStreamingCameraFps ([int][Math]::Min(30, [Math]::Round($PreviewRate)))
    $rendererPid = [int](Get-Content -Raw -LiteralPath (Join-Path $ueProject 'Saved\BskRenderer.pid'))

    $deadline = [DateTime]::UtcNow.AddSeconds(90)
    $rendererReady = $false
    while (!$rendererReady -and [DateTime]::UtcNow -lt $deadline) {
        if (!(Get-Process -Id $rendererPid -ErrorAction SilentlyContinue)) {
            throw 'UE exited before its render receiver became ready.'
        }
        $probe = [Net.Sockets.TcpClient]::new()
        try {
            $connect = $probe.BeginConnect('127.0.0.1', $RenderPort, $null, $null)
            if ($connect.AsyncWaitHandle.WaitOne(300)) {
                $probe.EndConnect($connect)
                $rendererReady = $probe.Connected
            }
        } catch { $rendererReady = $false } finally { $probe.Dispose() }
        if (!$rendererReady) { Start-Sleep -Milliseconds 300 }
    }
    if (!$rendererReady) { throw "UE receiver did not become ready on port $RenderPort." }

    $simulationArgs = @(
        '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', (Join-Path $PSScriptRoot 'run_simulation.ps1'),
        '-AdapterRoot', $AdapterRoot, '-ModelRoot', $ModelRoot,
        '-ControlPort', $ControlPort, '-RenderPort', $RenderPort,
        '-Duration', $Duration, '-SimulationRate', $SimulationRate, '-CaptureRate', $CaptureRate
    )
    $simulation = Start-Process -FilePath $powershellExe -ArgumentList $simulationArgs -PassThru -WindowStyle Hidden `
        -RedirectStandardOutput (Join-Path $logDirectory 'simulation.out.log') `
        -RedirectStandardError (Join-Path $logDirectory 'simulation.err.log')

    @{
        backend_pid = $backend.Id
        backend_start = $backend.StartTime.ToUniversalTime().Ticks
        simulation_pid = $simulation.Id
        simulation_start = $simulation.StartTime.ToUniversalTime().Ticks
        renderer_pid = $rendererPid
        pixel_streamer_port = $PixelStreamerPort
        pixel_player_port = $PixelPlayerPort
        api_url = "http://127.0.0.1:$ApiPort"
    } | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $runDirectory 'platform.json') -Encoding utf8

    $operatorUrl = if ($RemoteAccess) {
        "http://$PublicHost`:$ApiPort/?access_key=$([Uri]::EscapeDataString($StreamAccessKey))"
    } else { "http://127.0.0.1:$ApiPort" }
    if (!$NoBrowser) { Start-Process $operatorUrl }
    Write-Output "Space Arm Data Platform is running: $operatorUrl"
    Write-Output "Preview: $PreviewRate FPS Pixel Streaming 2/WebRTC; dataset: $CaptureRate Hz authoritative RGB/depth/segmentation."
    Write-Output 'Use W/S, A/D, Q/E directly for XYZ; hold Shift for rotation; use F/R for the gripper; Esc latches emergency stop.'
    Write-Output 'Run .\scripts\stop_platform.ps1 when finished.'
} catch {
    if (!$backend.HasExited) { Stop-Process -Id $backend.Id -Force -ErrorAction SilentlyContinue }
    & (Join-Path $ueScripts 'stop_renderer.ps1') -ErrorAction SilentlyContinue
    & (Join-Path $PSScriptRoot 'stop_pixel_streaming.ps1') -Quiet -ErrorAction SilentlyContinue
    throw
}
