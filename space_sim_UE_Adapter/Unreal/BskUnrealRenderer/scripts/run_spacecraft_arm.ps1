param(
    [string]$Python = '',
    [string]$ModelRoot = '',
    [string]$UnrealRoot = '',
    [double]$Duration = 10.0,
    [double]$SimulationRate = 1.0,
    [int]$Port = 5558,
    [string]$ScreenshotPath = '',
    [ValidateSet('auto', 'preserve', 'recompute')]
    [string]$NormalMode = 'auto',
    [ValidateRange(0.0, 180.0)]
    [double]$StlSmoothingAngle = 60.0,
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

    $scenario = Join-Path $ProjectRoot 'examples\scenario_spacecraft_arm_unreal.py'
    $catalog = Join-Path $ProjectRoot 'Saved\AssetImport\cubesat_so101.catalog.json'
    Write-Output "Running authoritative free-floating CubeSat + SO-101 dynamics at ${SimulationRate}x real time ..."
    & $pythonExe $scenario --model-root $resolvedModelRoot `
        --catalog $catalog --host 127.0.0.1 --port $Port --duration $Duration --simulation-rate $SimulationRate
    if ($LASTEXITCODE -ne 0) { throw "CubeSat + SO-101 sender failed with exit code $LASTEXITCODE." }
} finally {
    if (!$KeepRendererOpen) { & (Join-Path $PSScriptRoot 'stop_renderer.ps1') }
}

if ($KeepRendererOpen) {
    Write-Output 'Simulation finished; UE remains open. Run .\scripts\stop_renderer.ps1 to close it.'
}
