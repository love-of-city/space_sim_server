param(
    [string]$Python = '',
    [Parameter(Mandatory=$true)][string]$AdapterRoot,
    [Parameter(Mandatory=$true)][string]$ModelRoot,
    [Parameter(Mandatory=$true)][string]$UnrealRoot,
    [switch]$Force,
    [string]$TemplateId = ''
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
. (Join-Path $PSScriptRoot 'python_runtime.ps1')
$pythonExe = Resolve-SpaceSimPython -RepositoryRoot $projectRoot -RequestedPython $Python -RequiredModules @('numpy', 'Basilisk.simulation.mujoco')
$env:SPACE_SIM_PYTHON = $pythonExe
if ($TemplateId -in @('sarm-task-box-20261012', 'sarm-task-box-20261012-sdf', 'sarm-task-box-20261012-demo')) {
    $fastenerModel = if ($TemplateId -eq 'sarm-task-box-20261012-sdf') { 'sdf' } else { 'captive' }
    & $pythonExe (Join-Path $projectRoot 'tools\build_sarm_20261012.py') --fastener-model $fastenerModel
    if ($LASTEXITCODE -ne 0) { throw 'SARM assembly generation failed.' }
    & $pythonExe (Join-Path $projectRoot 'tools\build_sarm_plugin.py')
    if ($LASTEXITCODE -ne 0) { throw 'SARM thread plugin build failed.' }
}
$selectionArgs = @((Join-Path $projectRoot 'tools\select_sarm_scene.py'), '--model-root', $ModelRoot, '--check')
if ($TemplateId) { $selectionArgs += @('--template-id', $TemplateId) }
$selected = @(& $pythonExe @selectionArgs)
if ($LASTEXITCODE -ne 0 -or $selected.Count -ne 1) { throw 'Selected SARM scene/assets failed validation; see the model diagnostic above.' }
$scene = [string]$selected[0]
$ueProject = Join-Path $AdapterRoot 'Unreal\BskUnrealRenderer'
$catalogName = 'sarm_platform.catalog.json'
$destination = '/Game/BSK/Generated/SARM'
if ($TemplateId -in @('sarm-task-box-20261012', 'sarm-task-box-20261012-sdf', 'sarm-task-box-20261012-demo')) {
    $catalogName = 'sarm_20261012.catalog.json'
    $destination = '/Game/BSK/Generated/Sarm20261012'
}
if ($TemplateId -eq 'sarm-task-box-contacts') {
    $catalogName = 'sarm_task_box.catalog.json'
    $destination = '/Game/BSK/Generated/TaskBox'
}
if ($TemplateId -eq 'sarm-task-box-free-plugs') {
    $catalogName = 'sarm_task_box_plugs.catalog.json'
    $destination = '/Game/BSK/Generated/TaskBoxPlugs'
}
if (!(Test-Path -LiteralPath $scene -PathType Leaf)) { throw "Selected scene is missing: $scene" }
& (Join-Path $ueProject 'scripts\prepare_mjcf_assets.ps1') `
    -MjcfPath $scene -Destination $destination `
    -CatalogPath (Join-Path $ueProject "Saved\AssetImport\$catalogName") `
    -UnrealRoot $UnrealRoot -NormalMode preserve -Force:$Force
if ($LASTEXITCODE -ne 0) { throw 'SARM and articulated capture-target asset preparation failed.' }
