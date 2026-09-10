param(
    [Parameter(Mandatory=$true)][string]$AdapterRoot,
    [Parameter(Mandatory=$true)][string]$ModelRoot,
    [Parameter(Mandatory=$true)][string]$UnrealRoot,
    [switch]$Force,
    [string]$TemplateId = ''
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$selectionArgs = @((Join-Path $projectRoot 'tools\select_sarm_scene.py'), '--model-root', $ModelRoot, '--check')
if ($TemplateId) { $selectionArgs += @('--template-id', $TemplateId) }
$selected = @(& python @selectionArgs)
if ($LASTEXITCODE -ne 0 -or $selected.Count -ne 1) { throw 'Selected SARM scene/assets failed validation; see the model diagnostic above.' }
$scene = [string]$selected[0]
$ueProject = Join-Path $AdapterRoot 'Unreal\BskUnrealRenderer'
if (!(Test-Path -LiteralPath $scene -PathType Leaf)) { throw "Selected scene is missing: $scene" }
& (Join-Path $ueProject 'scripts\prepare_mjcf_assets.ps1') `
    -MjcfPath $scene -Destination '/Game/BSK/Generated/SARM' `
    -CatalogPath (Join-Path $ueProject 'Saved\AssetImport\sarm_platform.catalog.json') `
    -UnrealRoot $UnrealRoot -NormalMode preserve -Force:$Force
if ($LASTEXITCODE -ne 0) { throw 'SARM and articulated capture-target asset preparation failed.' }
