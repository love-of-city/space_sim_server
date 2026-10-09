param(
    [string]$Python = '',
    [string]$ModelRoot = '',
    [string]$UnrealRoot = '',
    [ValidateRange(34.0, 120.0)]
    [double]$Duration = 34.0,
    [ValidateRange(0.1, 100.0)]
    [double]$SimulationRate = 1.0,
    [int]$Port = 5558,
    [string]$ScreenshotPath = '',
    [string]$MetricsPath = '',
    [string]$RecordingPath = '',
    [ValidateRange(0.001, 1000.0)]
    [double]$PlaybackRate = 1.0,
    [switch]$ReuseRecording,
    [ValidateSet('auto', 'preserve', 'recompute')]
    [string]$NormalMode = 'auto',
    [ValidateRange(0.0, 180.0)]
    [double]$StlSmoothingAngle = 60.0,
    [string]$CaptureDirectory = '',
    [string[]]$CaptureProducts = @(),
    [ValidateRange(0.0, 60.0)]
    [double]$CaptureRate = 0.0,
    [ValidateRange(0, 65535)]
    [int]$CaptureNetworkPort = 0,
    [switch]$KeepRendererOpen,
    [switch]$Rebuild,
    [switch]$ReimportAssets
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
Set-BskPythonPath
$pythonExe = Resolve-BskPython -RequestedPython $Python -RequiredModules @('numpy', 'Basilisk.simulation.mujoco')
$ue = Resolve-UnrealRoot $UnrealRoot

if (!$ModelRoot) {
    $candidates = @(
        (Join-Path $WorkspaceRoot 'test\model\spacecraft_and_arm'),
        (Join-Path (Split-Path -Parent $WorkspaceRoot) 'test\model\spacecraft_and_arm')
    )
    $ModelRoot = $candidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
}
if (!$ModelRoot -or !(Test-Path -LiteralPath $ModelRoot)) {
    throw 'spacecraft_and_arm model was not found. Pass -ModelRoot explicitly.'
}
$resolvedModelRoot = (Resolve-Path -LiteralPath $ModelRoot).Path

& (Join-Path $PSScriptRoot 'prepare_spacecraft_arm_assets.ps1') -ModelRoot $resolvedModelRoot `
    -Variant combined -UnrealRoot $ue -NormalMode $NormalMode `
    -StlSmoothingAngle $StlSmoothingAngle -Force:$ReimportAssets
if ($LASTEXITCODE -ne 0) { throw 'CubeSat + SO-101 asset preparation failed.' }
& (Join-Path $PSScriptRoot 'prepare_runtime_materials.ps1') -UnrealRoot $ue

$pluginBinary = Join-Path $ProjectRoot 'Plugins\BskUnrealRuntime\Binaries\Win64\UnrealEditor-BskUnrealRuntime.dll'
if ($Rebuild -or !(Test-Path -LiteralPath $pluginBinary)) {
    & (Join-Path $PSScriptRoot 'build.ps1') -UnrealRoot $ue
}

if (!$MetricsPath) { $MetricsPath = Join-Path $ProjectRoot 'Saved\orbital_grasp_metrics.json' }
$resolvedMetricsPath = [IO.Path]::GetFullPath($MetricsPath)
$generatedMjcf = Join-Path $ProjectRoot 'Saved\Generated\orbital_grasp.xml'
$scenario = Join-Path $ProjectRoot 'examples\scenario_orbital_grasp_unreal.py'
$catalog = Join-Path $ProjectRoot 'Saved\AssetImport\cubesat_so101.catalog.json'

# Supplying a recording path selects the common record-then-replay workflow.
# Without it, retain the original live TCP behavior for backward compatibility.
$replayMode = [bool]$RecordingPath -or $ReuseRecording
if ($replayMode) {
    if (!$RecordingPath) {
        $RecordingPath = Join-Path $ProjectRoot 'Saved\Recordings\orbital_grasp.bskrec'
    }
    if ([IO.Path]::IsPathRooted($RecordingPath)) {
        $resolvedRecordingPath = [IO.Path]::GetFullPath($RecordingPath)
    } else {
        # Accept paths relative to either the caller, the repository root, or
        # the UE project. This keeps the same command valid from both script
        # directories used in the documentation and during development.
        $recordingCandidates = @(
            [IO.Path]::GetFullPath($RecordingPath),
            [IO.Path]::GetFullPath((Join-Path $WorkspaceRoot $RecordingPath)),
            [IO.Path]::GetFullPath((Join-Path $ProjectRoot $RecordingPath))
        ) | Select-Object -Unique
        $existingRecording = $recordingCandidates | Where-Object {
            Test-Path -LiteralPath $_ -PathType Leaf
        } | Select-Object -First 1
        if ($existingRecording) {
            $resolvedRecordingPath = $existingRecording
        } else {
            $repoRelative = $RecordingPath -replace '^[.][\\/]', ''
            $resolvedRecordingPath = if ($repoRelative -like 'Unreal\*') {
                [IO.Path]::GetFullPath((Join-Path $WorkspaceRoot $repoRelative))
            } else {
                $recordingCandidates[0]
            }
        }
    }
    if ($ReuseRecording -and !(Test-Path -LiteralPath $resolvedRecordingPath -PathType Leaf)) {
        throw "Cannot reuse missing orbital grasp recording: $resolvedRecordingPath"
    }

    if (!$ReuseRecording) {
        New-Item -ItemType Directory -Path (Split-Path -Parent $resolvedRecordingPath) -Force | Out-Null
        Write-Output 'Running authoritative BSK/MJScene orbital grasp and writing a renderer recording ...'
        & $pythonExe $scenario --model-root $resolvedModelRoot `
            --catalog $catalog --host 127.0.0.1 --port $Port --duration $Duration `
            --simulation-rate $SimulationRate --generated-mjcf $generatedMjcf --metrics $resolvedMetricsPath `
            --recording $resolvedRecordingPath --record-only
        if ($LASTEXITCODE -ne 0) { throw "Orbital grasp recording failed with exit code $LASTEXITCODE." }

        $metrics = Get-Content -Raw -LiteralPath $resolvedMetricsPath | ConvertFrom-Json
        if (!$metrics.orbit_safe) { throw 'Orbital mission validation failed: unsafe orbit altitude.' }
        if (!$metrics.rendezvous_handoff_stable) {
            throw 'Orbital mission validation failed: rendezvous did not settle before grasp handoff.'
        }
        if ([double]$metrics.approach_translation_m -lt 0.70) {
            throw 'Orbital mission validation failed: visible approach distance was not completed.'
        }
        if (!$metrics.grasp_motion_detected) {
            throw 'Orbital mission validation failed: target did not remain bilaterally captured through retraction.'
        }
        if (!$metrics.wheel_speed_within_limit) {
            throw 'Orbital mission validation failed: a reaction wheel exceeded its configured speed limit.'
        }
        Write-Output "Orbital mission metrics: $resolvedMetricsPath"
    } else {
        Write-Output "Reusing orbital grasp recording: $resolvedRecordingPath"
    }

    & (Join-Path $PSScriptRoot 'start_renderer.ps1') -UnrealRoot $ue -ReplayPath $resolvedRecordingPath `
        -ReplayRate $PlaybackRate -ScreenshotPath $ScreenshotPath -CaptureDirectory $CaptureDirectory `
        -CaptureProducts $CaptureProducts -CaptureRate $CaptureRate -CaptureNetworkPort $CaptureNetworkPort
    $rendererPid = [int](Get-Content -Raw -LiteralPath (Join-Path $ProjectRoot 'Saved\BskRenderer.pid'))

    if ($KeepRendererOpen) {
        Write-Output "Orbital grasp replay is running at ${PlaybackRate}x. Run .\scripts\stop_renderer.ps1 when finished."
        exit 0
    }

    try {
        # Replay timing starts inside the UE game world, not when the editor
        # process is spawned. Wait for the current launch to apply its first
        # frame so cold editor startup cannot consume the playback wait time.
        $startupDeadline = [DateTime]::UtcNow.AddSeconds(120)
        $replayStarted = $false
        $logPath = Join-Path $ProjectRoot 'Saved\Logs\BskUnrealRenderer.log'
        while (!$replayStarted -and [DateTime]::UtcNow -lt $startupDeadline) {
            if (!(Get-Process -Id $rendererPid -ErrorAction SilentlyContinue)) {
                throw 'Unreal Engine exited before the replay applied its first frame.'
            }
            if (Test-Path -LiteralPath $logPath) {
                $logText = Get-Content -Raw -LiteralPath $logPath
                $commandIndex = $logText.LastIndexOf("-BskReplay=$resolvedRecordingPath")
                if ($commandIndex -ge 0) {
                    $replayStarted = $logText.IndexOf('Applying first BSK frame', $commandIndex) -ge 0
                }
            }
            if (!$replayStarted) { Start-Sleep -Milliseconds 500 }
        }
        if (!$replayStarted) { throw 'UE replay did not apply its first frame within 120 seconds.' }

        $durationScript = Join-Path $ProjectRoot 'examples\recording_duration.py'
        $simulationSpan = & $pythonExe $durationScript $resolvedRecordingPath
        if ($LASTEXITCODE -ne 0) { throw 'Could not determine orbital grasp recording duration.' }
        $wallDuration = [Math]::Ceiling(([double]$simulationSpan / $PlaybackRate) + 3.0)
        Write-Output "Playing orbital grasp recording for approximately $wallDuration wall-clock seconds ..."
        Start-Sleep -Seconds $wallDuration
    } finally {
        & (Join-Path $PSScriptRoot 'stop_renderer.ps1')
    }
    exit 0
}

# Live mode: UE listens first, then BSK/MJScene streams non-blocking latest frames.
& (Join-Path $PSScriptRoot 'start_renderer.ps1') -UnrealRoot $ue -Port $Port -ScreenshotPath $ScreenshotPath `
    -CaptureDirectory $CaptureDirectory -CaptureProducts $CaptureProducts -CaptureRate $CaptureRate `
    -CaptureNetworkPort $CaptureNetworkPort
$rendererPid = [int](Get-Content -Raw -LiteralPath (Join-Path $ProjectRoot 'Saved\BskRenderer.pid'))
try {
    Write-Output "Waiting for UE receiver on 127.0.0.1:$Port ..."
    $deadline = [DateTime]::UtcNow.AddSeconds(90)
    $ready = $false
    while (!$ready -and [DateTime]::UtcNow -lt $deadline) {
        if (!(Get-Process -Id $rendererPid -ErrorAction SilentlyContinue)) {
            throw 'Unreal Engine exited before the receiver became ready.'
        }
        $probe = [Net.Sockets.TcpClient]::new()
        try {
            $connect = $probe.BeginConnect('127.0.0.1', $Port, $null, $null)
            if ($connect.AsyncWaitHandle.WaitOne(500)) {
                $probe.EndConnect($connect)
                $ready = $probe.Connected
            }
        } catch { $ready = $false } finally { $probe.Dispose() }
        if (!$ready) { Start-Sleep -Milliseconds 500 }
    }
    if (!$ready) { throw "UE receiver was not ready on port $Port after 90 seconds." }
    Start-Sleep -Milliseconds 500

    Write-Output "Running the 500 km Earth-orbit closed-loop rendezvous, hold, and grasp at ${SimulationRate}x ..."
    & $pythonExe $scenario --model-root $resolvedModelRoot `
        --catalog $catalog --host 127.0.0.1 --port $Port --duration $Duration `
        --simulation-rate $SimulationRate --generated-mjcf $generatedMjcf --metrics $resolvedMetricsPath
    if ($LASTEXITCODE -ne 0) { throw "Orbital grasp sender failed with exit code $LASTEXITCODE." }
    $metrics = Get-Content -Raw -LiteralPath $resolvedMetricsPath | ConvertFrom-Json
    if (!$metrics.orbit_safe) { throw 'Orbital mission validation failed: unsafe orbit altitude.' }
    if (!$metrics.rendezvous_handoff_stable) {
        throw 'Orbital mission validation failed: rendezvous did not settle before grasp handoff.'
    }
    if ([double]$metrics.approach_translation_m -lt 0.70) {
        throw 'Orbital mission validation failed: visible approach distance was not completed.'
    }
    if (!$metrics.grasp_motion_detected) {
        throw 'Orbital mission validation failed: target did not remain bilaterally captured through retraction.'
    }
    if (!$metrics.wheel_speed_within_limit) {
        throw 'Orbital mission validation failed: a reaction wheel exceeded its configured speed limit.'
    }
} finally {
    if (!$KeepRendererOpen) { & (Join-Path $PSScriptRoot 'stop_renderer.ps1') }
}

Write-Output "Orbital mission metrics: $resolvedMetricsPath"
if ($KeepRendererOpen) {
    Write-Output 'Mission finished; UE retains the last frame. Run .\scripts\stop_renderer.ps1 to close it.'
}
