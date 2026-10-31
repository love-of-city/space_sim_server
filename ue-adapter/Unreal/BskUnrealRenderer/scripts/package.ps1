param(
    [string]$UnrealRoot = '',
    [Parameter(Mandatory=$true)][string]$StagingDirectory,
    [Parameter(Mandatory=$true)][string]$OutputDirectory,
    [string[]]$Manifest = @(),
    [string]$Python = ''
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
$ue = Resolve-UnrealRoot $UnrealRoot
$pythonExe = Resolve-BskPython -RequestedPython $Python -RequiredModules @()
$tool = Join-Path $WorkspaceRoot 'scripts/package_ue_project.py'
$arguments = @($tool, '--engine', $ue, '--project', $ProjectRoot,
    '--staging', $StagingDirectory, '--output', $OutputDirectory)
foreach ($path in $Manifest) { $arguments += @('--manifest', $path) }
& $pythonExe @arguments
if ($LASTEXITCODE -ne 0) { throw "Isolated packaging failed with exit code $LASTEXITCODE" }
