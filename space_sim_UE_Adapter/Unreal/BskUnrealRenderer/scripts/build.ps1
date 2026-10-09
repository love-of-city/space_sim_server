param(
    [string]$UnrealRoot = '',
    [ValidateSet('Development', 'DebugGame', 'Shipping')]
    [string]$Configuration = 'Development',
    [switch]$Game
)

. (Join-Path $PSScriptRoot 'common.ps1')
$ue = Resolve-UnrealRoot $UnrealRoot
$target = if ($Game) { 'BskUnrealRenderer' } else { 'BskUnrealRendererEditor' }
$build = Join-Path $ue 'Engine\Build\BatchFiles\Build.bat'
& $build $target Win64 $Configuration $ProjectFile -WaitMutex -NoHotReloadFromIDE
if ($LASTEXITCODE -ne 0) { throw "Unreal build failed with exit code $LASTEXITCODE" }
