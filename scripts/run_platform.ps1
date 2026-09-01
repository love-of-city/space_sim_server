param(
    [string]$AdapterRoot = '',
    [string]$ModelRoot = '',
    [string]$UnrealRoot = '',
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
    # Authoritative RGB/depth/segmentation capture performs synchronous GPU readbacks
    # in the UE game thread. Keep it opt-in for interactive preview; enable it only
    # when dataset/episode recording actually needs those products.
    [switch]$EnableDatasetCapture,
    [ValidateRange(1.0, 500.0)]
    [double]$IkRate = 100.0,
    [double]$PreviewRate = 60.0,
    [ValidateRange(30, 600)]
    [int]$RendererReadyTimeout = 240,
    [switch]$RemoteAccess,
    [string]$PublicHost = '127.0.0.1',
    [string]$PixelPlayerPublicUrl = '',
    [string]$StreamAccessKey = '',
    [string]$StreamJwtSecret = '',
    [string]$IceServersJson = '[]',
    [string]$TurnUrlsJson = '[]',
    [string]$TurnAuthSecret = '',
    [string]$AdminUsername = 'admin',
    [string]$AdminPassword = 'ChangeMe123!',
    [switch]$Rebuild,
    [switch]$ReimportAssets,
    [switch]$NoBrowser
)

$ErrorActionPreference = 'Stop'
if ($PSVersionTable.PSVersion.Major -lt 7) {
    throw 'This project requires PowerShell 7 or later. Start it with: pwsh -NoProfile -File .\scripts\run_platform.ps1'
}

# Reuse the exact PowerShell 7 executable that launched this entry script so
# backend and simulation child processes cannot silently fall back to Windows
# PowerShell 5.1 through powershell.exe.
$powershellExe = (Get-Process -Id $PID).Path
if (!$powershellExe -or !(Test-Path -LiteralPath $powershellExe -PathType Leaf)) {
    throw 'Unable to resolve the current PowerShell 7 executable.'
}

if ($PreviewRate -le 0 -or $PreviewRate -gt 60) { throw 'PreviewRate must be in (0, 60].' }
$projectRoot = Split-Path -Parent $PSScriptRoot
$workspaceRoot = Split-Path -Parent $projectRoot

function Resolve-Unreal56Root([string]$RequestedRoot) {
    $candidates = @()
    if ($RequestedRoot) { $candidates += $RequestedRoot }
    if ($env:UE56_ROOT) { $candidates += $env:UE56_ROOT }
    $candidates += @('E:\UE5.6', 'E:\UE5.6\UE_5.6', 'C:\Program Files\Epic Games\UE_5.6')
    foreach ($candidate in $candidates | Select-Object -Unique) {
        # Use System.IO path composition here: Join-Path throws when a candidate drive (for example E:) is absent.
        foreach ($root in @($candidate, ([IO.Path]::Combine($candidate, 'UE_5.6')))) {
            $editor = [IO.Path]::Combine($root, 'Engine', 'Binaries', 'Win64', 'UnrealEditor.exe')
            if (Test-Path -LiteralPath $editor -PathType Leaf) {
                return (Resolve-Path -LiteralPath $root).Path
            }
        }
    }
    throw 'Unreal Engine 5.6 was not found. Pass -UnrealRoot or set UE56_ROOT.'
}

if (!$AdapterRoot) { $AdapterRoot = Join-Path $workspaceRoot 'space_sim_UE_adapter' }
if (!$ModelRoot) {
    $modelCandidates = @(
        (Join-Path $AdapterRoot 'test\model\spacecraft_and_arm'),
        (Join-Path $workspaceRoot 'test\model\spacecraft_and_arm')
    )
    $ModelRoot = $modelCandidates | Where-Object { Test-Path -LiteralPath $_ -PathType Container } |
        Select-Object -First 1
}
if (!$ModelRoot) {
    throw 'spacecraft_and_arm model was not found in the adapter repository. Pull Git LFS assets or pass -ModelRoot explicitly.'
}
$UnrealRoot = Resolve-Unreal56Root $UnrealRoot
$ueProject = Join-Path $AdapterRoot 'Unreal\BskUnrealRenderer'
$ueScripts = Join-Path $ueProject 'scripts'
$runDirectory = Join-Path $projectRoot 'run'
$logDirectory = Join-Path $projectRoot 'logs'
New-Item -ItemType Directory -Path $runDirectory,$logDirectory -Force | Out-Null

foreach ($path in @($AdapterRoot, $ModelRoot, $UnrealRoot, $ueScripts)) {
    if (!(Test-Path -LiteralPath $path)) { throw "Required path does not exist: $path" }
}

foreach ($command in @('python', 'conda', 'npm.cmd')) {
    if (!(Get-Command $command -ErrorAction SilentlyContinue)) {
        throw "Required command '$command' was not found on PATH. See README.md first-deployment prerequisites."
    }
}
$meshProbe = Get-ChildItem -LiteralPath (Join-Path $ModelRoot 'assets\robotstudio_so101\assets') `
    -File -Filter '*.stl' -ErrorAction SilentlyContinue | Select-Object -First 1
if (!$meshProbe -or $meshProbe.Length -lt 1024) {
    throw 'Spacecraft-arm STL assets are missing or still Git LFS pointers. Run git lfs install and git lfs pull in space_sim_UE_adapter.'
}
$environmentAssetRoot = Join-Path $ueProject 'Content\Planets'
$environmentAssetFiles = @(
    'Earth\8k_earth_clouds.uasset',
    'Earth\8k_earth_daymap.uasset',
    'Earth\8k_earth_nightmap.uasset',
    'Earth\8k_earth_normal_map.uasset',
    'Earth\8k_earth_specular_map.uasset',
    'Earth\M_Atmosphere.uasset',
    'Earth\M_Clouds.uasset',
    'Earth\M_Earth.uasset',
    'Stars\8k_stars_milky_way.uasset',
    'Stars\M_Stars.uasset'
)
foreach ($relativeAsset in $environmentAssetFiles) {
    $assetPath = Join-Path $environmentAssetRoot $relativeAsset
    $asset = Get-Item -LiteralPath $assetPath -ErrorAction SilentlyContinue
    if (!$asset -or $asset.Length -lt 1024) {
        throw "Earth/star environment asset is missing or still a Git LFS pointer: $assetPath. Run git lfs pull in space_sim_UE_adapter."
    }
}
$environmentSpherePath = Join-Path $ueProject 'Content\_GENERATED\Hyperlovimia\Sphere_732702C4.uasset'
$environmentSphere = Get-Item -LiteralPath $environmentSpherePath -ErrorAction SilentlyContinue
if (!$environmentSphere -or $environmentSphere.Length -lt 1024) {
    throw "MyProject2 environment Sphere is missing or still a Git LFS pointer: $environmentSpherePath. Run git lfs pull in space_sim_UE_adapter."
}
& python -c 'import fastapi, pydantic, uvicorn' 2>$null
if ($LASTEXITCODE -ne 0) {
    throw 'Backend Python dependencies are missing. Run: python -m pip install -e ".[test]"'
}

# Stop only PIDs previously recorded by this project before binding fixed ports.
& (Join-Path $PSScriptRoot 'stop_platform.ps1') -Quiet
Remove-Item -LiteralPath (Join-Path $runDirectory 'scene_runtime.json') -Force -ErrorAction SilentlyContinue

function Assert-TcpPortAvailable([int]$Port, [string]$Purpose) {
    $listener = [Net.Sockets.TcpListener]::new([Net.IPAddress]::Loopback, $Port)
    try {
        $listener.Start()
    } catch {
        throw "$Purpose port $Port is already occupied after platform cleanup. Close the process using it or choose another port."
    } finally {
        try { $listener.Stop() } catch { }
    }
}

foreach ($portCheck in @(
    @{ Port = $ApiPort; Purpose = 'Backend API' },
    @{ Port = $ControlPort; Purpose = 'Simulation control' },
    @{ Port = $CapturePort; Purpose = 'Authoritative capture' },
    @{ Port = $RenderPort; Purpose = 'UE render receiver' }
)) {
    Assert-TcpPortAvailable ([int]$portCheck.Port) ([string]$portCheck.Purpose)
}

if ($RemoteAccess) {
    if ($AdminPassword -eq 'ChangeMe123!') {
        throw 'RemoteAccess requires a non-default -AdminPassword.'
    }
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

Write-Output 'Checking the existing UE modules and assets ...'
$projectBinary = Join-Path $ueProject 'Binaries\Win64\UnrealEditor-BskUnrealRenderer.dll'
$pluginBinary = Join-Path $ueProject 'Plugins\BskUnrealRuntime\Binaries\Win64\UnrealEditor-BskUnrealRuntime.dll'
if ($Rebuild -or !(Test-Path -LiteralPath $projectBinary -PathType Leaf) -or !(Test-Path -LiteralPath $pluginBinary -PathType Leaf)) {
    # Unreal commandlets must be able to load both modules before importing or configuring assets.
    & (Join-Path $ueScripts 'build.ps1') -UnrealRoot $UnrealRoot
    if ($LASTEXITCODE -ne 0) { throw 'UE project/runtime plugin build failed.' }
}

$catalog = Join-Path $ueProject 'Saved\AssetImport\cubesat_so101.catalog.json'
# The preparation script validates both the catalog fingerprint and every expected .uasset.
# Do not skip it merely because a catalog survived an earlier failed import.
& (Join-Path $ueScripts 'prepare_spacecraft_arm_assets.ps1') -ModelRoot $ModelRoot `
    -Variant combined -UnrealRoot $UnrealRoot -NormalMode preserve -Force:$ReimportAssets
if ($LASTEXITCODE -ne 0) { throw 'Spacecraft/arm asset preparation failed.' }
& (Join-Path $ueScripts 'prepare_runtime_materials.ps1') -UnrealRoot $UnrealRoot

$cameraStreamers = @()
foreach ($cameraId in $PixelStreamingCameraIds) {
    $safeId = $cameraId -replace '[^A-Za-z0-9_-]', '_'
    $label = if ($cameraId -match 'wrist') { 'WristCamera' } elseif ($cameraId -match 'overview') { 'SpacecraftOverview' } else { $cameraId }
    $cameraStreamers += "${PixelStreamingId}__${safeId}=$label"
}
# Start-Process flattens -ArgumentList into one Windows command line and can split
# values containing spaces (for example C:\Program Files\...).  Pass runtime
# filesystem paths through the inherited environment instead; run_backend.ps1
# then forwards them to Python using PowerShell's native argument binder.
$env:SPACE_SIM_RUNTIME_ADAPTER_ROOT = $AdapterRoot
$env:SPACE_SIM_RUNTIME_MODEL_ROOT = $ModelRoot
$env:SPACE_SIM_RUNTIME_UNREAL_ROOT = $UnrealRoot
$env:SPACE_SIM_RUNTIME_POWERSHELL_EXE = $powershellExe
$env:SPACE_SIM_ADMIN_USERNAME = $AdminUsername
$env:SPACE_SIM_ADMIN_PASSWORD = $AdminPassword

$backendArgs = @(
    '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', (Join-Path $PSScriptRoot 'run_backend.ps1'),
    '-ApiHost', $(if ($RemoteAccess) { '0.0.0.0' } else { '127.0.0.1' }),
    '-ApiPort', $ApiPort, '-ControlPort', $ControlPort, '-CapturePort', $CapturePort,
    '-PixelStreamingPlayerPort', $PixelPlayerPort, '-PixelStreamingId', $PixelStreamingId,
    '-PixelStreamingCameraStreamers', ($cameraStreamers -join ';'),
    '-RenderPort', $RenderPort,
    '-PixelStreamerPort', $PixelStreamerPort,
    '-PixelStreamingCameraIds', ($PixelStreamingCameraIds -join ','),
    '-PixelStreamingCameraWidth', $PixelStreamingCameraWidth,
    '-PixelStreamingCameraHeight', $PixelStreamingCameraHeight,
    '-PreviewRate', $PreviewRate, '-RendererReadyTimeout', $RendererReadyTimeout,
    '-IkRate', $IkRate, '-SimulationRate', $SimulationRate,
    '-CaptureRate', $CaptureRate, '-DefaultDuration', $Duration
)
if ($EnableDatasetCapture) { $backendArgs += '-DefaultDatasetCapture' }
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
    if ($backend.HasExited) {
        throw 'The backend launcher exited even though an API health endpoint responded; refusing to use a stale backend instance.'
    }

    $backendServiceConnection = Get-NetTCPConnection -State Listen -LocalPort $ControlPort -ErrorAction SilentlyContinue |
        Select-Object -First 1
    if (!$backendServiceConnection) {
        throw "The new backend did not own simulation control port $ControlPort."
    }
    $backendServicePid = [int]$backendServiceConnection.OwningProcess
    $backendService = Get-Process -Id $backendServicePid -ErrorAction SilentlyContinue
    if (!$backendService) { throw 'The backend service process disappeared during startup.' }

    @{
        backend_pid = $backend.Id
        backend_start = $backend.StartTime.ToUniversalTime().Ticks
        backend_service_pid = $backendServicePid
        backend_service_start = $backendService.StartTime.ToUniversalTime().Ticks
        adapter_root = $AdapterRoot
        model_root = $ModelRoot
        unreal_root = $UnrealRoot
        pixel_streamer_port = $PixelStreamerPort
        pixel_player_port = $PixelPlayerPort
        api_url = "http://127.0.0.1:$ApiPort"
        mode = 'control-plane'
    } | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $runDirectory 'platform.json') -Encoding utf8

    $operatorUrl = if ($RemoteAccess) {
        "http://$PublicHost`:$ApiPort/?access_key=$([Uri]::EscapeDataString($StreamAccessKey))"
    } else { "http://127.0.0.1:$ApiPort" }
    if (!$NoBrowser) { Start-Process $operatorUrl }
    Write-Output "Space Arm control platform is running: $operatorUrl"
    Write-Output "Login username: $AdminUsername (bootstrap password applies only when the first admin is created)."
    Write-Output 'UE and Basilisk/MJScene are not started yet. Select a scene template and randomization in the frontend, then click Start Scene.'
    Write-Output "Scene defaults: duration $Duration s; preview $PreviewRate FPS; capture $CaptureRate Hz; IK $IkRate Hz."
    Write-Output 'Run .\scripts\stop_platform.ps1 when finished.'
} catch {
    if (!$backend.HasExited) { Stop-Process -Id $backend.Id -Force -ErrorAction SilentlyContinue }
    # Also removes a Python backend child if its PowerShell parent was killed
    # before platform.json could be written.
    & (Join-Path $PSScriptRoot 'stop_platform.ps1') -Quiet -ErrorAction SilentlyContinue
    throw
}
