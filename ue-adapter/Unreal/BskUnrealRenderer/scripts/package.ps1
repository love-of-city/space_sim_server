param(
    [string]$UnrealRoot = '',
    [string]$OutputDirectory = ''
)

. (Join-Path $PSScriptRoot 'common.ps1')
$ue = Resolve-UnrealRoot $UnrealRoot
if (!$OutputDirectory) { $OutputDirectory = Join-Path $ProjectRoot 'Dist' }
$uat = Join-Path $ue 'Engine\Build\BatchFiles\RunUAT.bat'
& $uat BuildCookRun -project=$ProjectFile -noP4 -platform=Win64 -clientconfig=Development -build -cook -stage -pak -archive -archivedirectory=$OutputDirectory
if ($LASTEXITCODE -ne 0) { throw "Packaging failed with exit code $LASTEXITCODE" }
