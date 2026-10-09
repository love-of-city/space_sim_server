param(
    [string]$Python = '',
    [string]$UnrealRoot = '',
    [string]$BasiliskRoot = '',
    [string]$RecordingPath = '',
    [double]$PlaybackRate = 120.0,
    [switch]$Live,
    [double]$LiveRate = 120.0,
    [int]$Port = 5558,
    [switch]$ReuseRecording,
    [switch]$KeepRendererOpen,
    [switch]$Rebuild
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
Set-BskPythonPath
$pythonExe = Resolve-BskPython -RequestedPython $Python -RequiredModules @('numpy', 'Basilisk.simulation.mujoco')
$ue = Resolve-UnrealRoot $UnrealRoot
if (!$RecordingPath) {
    $RecordingPath = Join-Path $ProjectRoot 'Saved\Recordings\demo8.bskrec'
}
$RecordingPath = [IO.Path]::GetFullPath($RecordingPath)
$pluginBinary = Join-Path $ProjectRoot 'Plugins\BskUnrealRuntime\Binaries\Win64\UnrealEditor-BskUnrealRuntime.dll'

if ($Rebuild -or !(Test-Path -LiteralPath $pluginBinary)) {
    & (Join-Path $PSScriptRoot 'build.ps1') -UnrealRoot $ue
}

if ($Live) {
    if ($ReuseRecording) {
        throw '-ReuseRecording cannot be combined with -Live.'
    }
    if ($LiveRate -le 0.0) {
        throw '-LiveRate must be greater than zero.'
    }

    & (Join-Path $PSScriptRoot 'start_renderer.ps1') -UnrealRoot $ue -Port $Port
    $pidFile = Join-Path $ProjectRoot 'Saved\BskRenderer.pid'
    $rendererPid = [int](Get-Content -Raw -LiteralPath $pidFile)

    try {
        Write-Output "Waiting for UE receiver on 127.0.0.1:$Port ..."
        $deadline = [DateTime]::UtcNow.AddSeconds(90)
        $ready = $false
        while (!$ready -and [DateTime]::UtcNow -lt $deadline) {
            $renderer = Get-Process -Id $rendererPid -ErrorAction SilentlyContinue
            if (!$renderer) { throw 'Unreal Engine exited before the receiver became ready.' }
            $probe = [Net.Sockets.TcpClient]::new()
            try {
                $connect = $probe.BeginConnect('127.0.0.1', $Port, $null, $null)
                if ($connect.AsyncWaitHandle.WaitOne(500)) {
                    $probe.EndConnect($connect)
                    $ready = $probe.Connected
                }
            } catch {
                $ready = $false
            } finally {
                $probe.Dispose()
            }
            if (!$ready) { Start-Sleep -Milliseconds 500 }
        }
        if (!$ready) { throw "UE receiver was not ready on port $Port after 90 seconds." }
        Start-Sleep -Milliseconds 500

        $basiliskSource = Resolve-BasiliskRoot $BasiliskRoot
        $scenario = Join-Path $ProjectRoot 'examples\scenario_mjscene_unreal.py'
        Write-Output "Running authoritative Basilisk + MJScene Demo 8 live at ${LiveRate}x simulation rate ..."
        & $pythonExe $scenario --basilisk-root $basiliskSource --live --host 127.0.0.1 --port $Port --simulation-rate $LiveRate
        if ($LASTEXITCODE -ne 0) { throw "Demo 8 live sender failed with exit code $LASTEXITCODE." }
    } finally {
        if (!$KeepRendererOpen) {
            & (Join-Path $PSScriptRoot 'stop_renderer.ps1')
        }
    }

    if ($KeepRendererOpen) {
        Write-Output 'Demo 8 live simulation finished; UE remains open. Run .\scripts\stop_renderer.ps1 to close it.'
    }
    exit 0
}

if (!$ReuseRecording -or !(Test-Path -LiteralPath $RecordingPath)) {
    $recordingDirectory = Split-Path -Parent $RecordingPath
    New-Item -ItemType Directory -Path $recordingDirectory -Force | Out-Null
    $basiliskSource = Resolve-BasiliskRoot $BasiliskRoot
    $scenario = Join-Path $ProjectRoot 'examples\scenario_mjscene_unreal.py'
    Write-Output 'Running authoritative Basilisk + MJScene Demo 8 and recording renderer states ...'
    & $pythonExe $scenario --basilisk-root $basiliskSource --output $RecordingPath
    if ($LASTEXITCODE -ne 0) { throw "Demo 8 recording failed with exit code $LASTEXITCODE." }
}

& (Join-Path $PSScriptRoot 'start_renderer.ps1') -UnrealRoot $ue -ReplayPath $RecordingPath -ReplayRate $PlaybackRate

if ($KeepRendererOpen) {
    Write-Output 'Demo 8 replay is running. Use .\scripts\stop_renderer.ps1 when finished.'
    exit 0
}

try {
    $durationScript = Join-Path $ProjectRoot 'examples\recording_duration.py'
    $simulationSpan = & $pythonExe $durationScript $RecordingPath
    if ($LASTEXITCODE -ne 0) { throw 'Could not determine Demo 8 recording duration.' }
    $wallDuration = [Math]::Ceiling(([double]$simulationSpan / $PlaybackRate) + 5.0)
    Write-Output "Playing Demo 8 for approximately $wallDuration wall-clock seconds ..."
    Start-Sleep -Seconds $wallDuration
} finally {
    & (Join-Path $PSScriptRoot 'stop_renderer.ps1')
}
