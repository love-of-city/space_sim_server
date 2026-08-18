param(
    [string]$AdapterRoot = '',
    [string]$ModelRoot = '',
    [int]$ControlPort = 8766,
    [int]$RenderPort = 5558,
    [double]$Duration = 300.0,
    [double]$SimulationRate = 1.0,
    [double]$CaptureRate = 10.0
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$workspaceRoot = Split-Path -Parent $projectRoot
if (!$AdapterRoot) { $AdapterRoot = Join-Path $workspaceRoot 'space_sim_UE_adapter' }
if (!$ModelRoot) { $ModelRoot = Join-Path $workspaceRoot 'test\model\spacecraft_and_arm' }
$catalog = Join-Path $AdapterRoot 'Unreal\BskUnrealRenderer\Saved\AssetImport\cubesat_so101.catalog.json'
$scenario = Join-Path $projectRoot 'simulation\teleop_grasp_unreal.py'
$env:PYTHONPATH = @(
    (Join-Path $projectRoot 'backend'),
    (Join-Path $AdapterRoot 'Adapters')
) -join [IO.Path]::PathSeparator

conda run --no-capture-output -n mujoco-dev python $scenario `
    --adapter-root ([IO.Path]::GetFullPath($AdapterRoot)) `
    --model-root ([IO.Path]::GetFullPath($ModelRoot)) `
    --catalog ([IO.Path]::GetFullPath($catalog)) `
    --control-port $ControlPort --render-port $RenderPort `
    --duration $Duration --simulation-rate $SimulationRate --capture-rate $CaptureRate

