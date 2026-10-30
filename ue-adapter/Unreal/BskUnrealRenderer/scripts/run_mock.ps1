param(
    [string]$Python = '',
    [string]$HostName = '127.0.0.1',
    [int]$Port = 5558,
    [double]$Rate = 30.0,
    [double]$Duration = 60.0
)

. (Join-Path $PSScriptRoot 'common.ps1')
Set-BskPythonPath
$pythonExe = Resolve-BskPython -RequestedPython $Python -RequiredModules @('numpy')
$example = Join-Path $ProjectRoot 'examples\mock_two_spacecraft_stream.py'
& $pythonExe $example --host $HostName --port $Port --rate $Rate --duration $Duration
exit $LASTEXITCODE
