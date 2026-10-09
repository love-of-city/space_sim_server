param(
    [string]$Python = '',
    [string]$HostName = '127.0.0.1',
    [int]$Port = 5558,
    [double]$Duration = 60.0
)

. (Join-Path $PSScriptRoot 'common.ps1')
Set-BskPythonPath
$pythonExe = Resolve-BskPython -RequestedPython $Python -RequiredModules @('numpy', 'Basilisk')
$example = Join-Path $ProjectRoot 'examples\bsk_two_spacecraft_stream.py'
& $pythonExe $example --host $HostName --port $Port --duration $Duration
exit $LASTEXITCODE
