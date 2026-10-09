param(
    [string]$Python = '',
    [string]$UnrealRoot = '',
    [double]$Duration = 22.0,
    [double]$SimulationRate = 1.0,
    [ValidateRange(-45.0, 45.0)]
    [double]$JointAngleDegrees = 20.0,
    [double]$MoveSeconds = 2.0,
    [double]$HoldSeconds = 0.75,
    [int]$Port = 5558,
    [string]$ScreenshotPath = '',
    [ValidateSet('auto', 'preserve', 'recompute')]
    [string]$NormalMode = 'auto',
    [switch]$KeepRendererOpen,
    [switch]$Rebuild,
    [switch]$ReimportAssets
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
Set-BskPythonPath
$pythonExe = Resolve-BskPython -RequestedPython $Python -RequiredModules @('numpy', 'Basilisk.simulation.mujoco')
$ue = Resolve-UnrealRoot $UnrealRoot

& (Join-Path $PSScriptRoot 'prepare_ur5e_assets.ps1') -UnrealRoot $ue -NormalMode $NormalMode -Force:$ReimportAssets
if ($LASTEXITCODE -ne 0) { throw 'UR5e asset preparation failed.' }
& (Join-Path $PSScriptRoot 'prepare_runtime_materials.ps1') -UnrealRoot $ue

$pluginBinary = Join-Path $ProjectRoot 'Plugins\BskUnrealRuntime\Binaries\Win64\UnrealEditor-BskUnrealRuntime.dll'
if ($Rebuild -or !(Test-Path -LiteralPath $pluginBinary)) {
    & (Join-Path $PSScriptRoot 'build.ps1') -UnrealRoot $ue
}

& (Join-Path $PSScriptRoot 'start_renderer.ps1') -UnrealRoot $ue -Port $Port -ScreenshotPath $ScreenshotPath
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

    $scenario = Join-Path $ProjectRoot 'examples\scenario_ur5e_unreal.py'
    Write-Output "Running UR5e sequential fixed-angle motion (${JointAngleDegrees} deg/joint) with authoritative MJScene dynamics at ${SimulationRate}x real time ..."
    & $pythonExe $scenario --workspace $WorkspaceRoot --host 127.0.0.1 --port $Port --duration $Duration --simulation-rate $SimulationRate --joint-angle-deg $JointAngleDegrees --move-seconds $MoveSeconds --hold-seconds $HoldSeconds
    if ($LASTEXITCODE -ne 0) { throw "UR5e sender failed with exit code $LASTEXITCODE." }
} finally {
    if (!$KeepRendererOpen) { & (Join-Path $PSScriptRoot 'stop_renderer.ps1') }
}

if ($KeepRendererOpen) {
    Write-Output 'UR5e simulation finished; UE remains open. Run .\scripts\stop_renderer.ps1 to close it.'
}
