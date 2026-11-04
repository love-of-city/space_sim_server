param(
    [string]$UnrealRoot = '',
    [ValidateSet('Development', 'DebugGame', 'Shipping')]
    [string]$Configuration = 'Development',
    [switch]$Game
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
$ue = Resolve-UnrealRoot $UnrealRoot
$target = if ($Game) { 'BskUnrealRenderer' } else { 'BskUnrealRendererEditor' }
$python = Resolve-BskPython -RequiredModules @()
$buildTool = Join-Path $WorkspaceRoot 'scripts/build_ue_project.py'
& $python $buildTool --project $ProjectFile --engine $ue --target $target --configuration $Configuration
if ($LASTEXITCODE -ne 0) { throw 'UE build or provenance validation failed; capture remains disabled.' }
if (!$Game) {
    & (Join-Path $PSScriptRoot 'prepare_environment_level.ps1') -UnrealRoot $ue
}
