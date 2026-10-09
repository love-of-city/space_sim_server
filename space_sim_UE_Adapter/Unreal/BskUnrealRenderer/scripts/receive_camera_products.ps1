param(
    [string]$Python = '',
    [string]$ListenAddress = '127.0.0.1',
    [ValidateRange(1, 65535)]
    [int]$Port = 5560,
    [string]$OutputDirectory = ''
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
Set-BskPythonPath
$pythonExe = Resolve-BskPython -RequestedPython $Python -RequiredModules @('numpy')
if (!$OutputDirectory) { $OutputDirectory = Join-Path $ProjectRoot 'Saved\BskCaptureNetwork' }
$receiver = Join-Path $ProjectRoot 'examples\receive_camera_products.py'
& $pythonExe $receiver --host $ListenAddress --port $Port --output ([IO.Path]::GetFullPath($OutputDirectory))
exit $LASTEXITCODE
