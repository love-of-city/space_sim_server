param(
    [string]$UnrealRoot = ''
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
$ue = Resolve-UnrealRoot $UnrealRoot
$editorCmd = Join-Path $ue 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$script = Join-Path $PSScriptRoot 'validate_sp_space_sun_assets.py'

& $editorCmd $ProjectFile -run=pythonscript "-script=$script" -unattended -nop4 -nosplash -nullrhi
if ($LASTEXITCODE -ne 0) {
    throw "SP_space Sun asset validation failed with exit code $LASTEXITCODE."
}
