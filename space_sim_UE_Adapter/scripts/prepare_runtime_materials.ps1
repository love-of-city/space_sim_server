$ErrorActionPreference = 'Stop'
$target = Join-Path $PSScriptRoot '..\Unreal\BskUnrealRenderer\scripts\prepare_runtime_materials.ps1'
& $target @args
exit $LASTEXITCODE
