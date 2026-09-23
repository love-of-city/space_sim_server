param(
    [string]$Python = '',
    [string]$AdapterRoot = '',
    [string]$ModelRoot = '',
    [int]$ControlPort = 8766,
    [int]$RenderPort = 5558,
    [double]$Duration = 0.0,
    [double]$SimulationRate = 1.0,
    [double]$CaptureRate = 30.0,
    [string]$SceneInstancePath = '',
    [ValidateRange(1.0, 240.0)]
    [double]$IkRate = 120.0,
    [switch]$DisableAttitudeControl
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
. (Join-Path $PSScriptRoot 'python_runtime.ps1')
$pythonExe = Resolve-SpaceSimPython -RepositoryRoot $projectRoot -RequestedPython $Python -RequiredModules @('numpy', 'Basilisk.simulation.mujoco')
$env:SPACE_SIM_PYTHON = $pythonExe
$workspaceRoot = Split-Path -Parent $projectRoot
if (!$AdapterRoot) { $AdapterRoot = Join-Path $workspaceRoot 'space_sim_UE_adapter' }
if (!$ModelRoot) {
    $modelCandidates = @(
        (Join-Path $projectRoot 'model\SARM\platform'),
        (Join-Path $workspaceRoot 'model\SARM\platform'),
        (Join-Path $AdapterRoot 'test\model\spacecraft_and_arm'),
        (Join-Path $workspaceRoot 'test\model\spacecraft_and_arm')
    )
    $ModelRoot = $modelCandidates | Where-Object { Test-Path -LiteralPath $_ -PathType Container } |
        Select-Object -First 1
}
if (!$ModelRoot) {
    throw 'SARM model was not found in this repository or the legacy workspace/adapter locations. Pass -ModelRoot explicitly.'
}
$catalog = if ((Split-Path -Leaf $ModelRoot) -eq 'platform') { Join-Path $AdapterRoot 'Unreal\BskUnrealRenderer\Saved\AssetImport\sarm_platform.catalog.json' } else { Join-Path $AdapterRoot 'Unreal\BskUnrealRenderer\Saved\AssetImport\cubesat_so101.catalog.json' }
if ($SceneInstancePath) {
    $selectedInstance = Get-Content -Raw -LiteralPath $SceneInstancePath | ConvertFrom-Json
    if ($selectedInstance.template_id -eq 'sarm-task-box-contacts') {
        $catalog = Join-Path $AdapterRoot 'Unreal\BskUnrealRenderer\Saved\AssetImport\sarm_task_box.catalog.json'
    }
}
$scenario = Join-Path $projectRoot 'simulation\teleop_grasp_unreal.py'
$env:PYTHONPATH = @(
    (Join-Path $projectRoot 'backend'),
    (Join-Path $AdapterRoot 'Adapters')
) -join [IO.Path]::PathSeparator

$arguments = @(
    $scenario,
    '--adapter-root', ([IO.Path]::GetFullPath($AdapterRoot)),
    '--model-root', ([IO.Path]::GetFullPath($ModelRoot)),
    '--catalog', ([IO.Path]::GetFullPath($catalog)),
    '--control-port', $ControlPort, '--render-port', $RenderPort,
    '--duration', $Duration, '--simulation-rate', $SimulationRate,
    '--capture-rate', $CaptureRate, '--ik-rate', $IkRate
)
if ($SceneInstancePath) {
    $arguments += @('--scene-instance', ([IO.Path]::GetFullPath($SceneInstancePath)))
}
if ($DisableAttitudeControl) { $arguments += '--disable-attitude-control' }
& $pythonExe @arguments
$exitCode = $LASTEXITCODE
exit $exitCode
