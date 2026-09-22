param(
    [ValidateSet('Start', 'Stop', 'Show')][string]$Action = 'Start',
    [string]$Python = 'C:\tools\miniconda3\envs\mujoco-dev\python.exe',
    [string]$ConfigPath,
    [switch]$Restart
)

$ErrorActionPreference = 'Stop'
if ($PSVersionTable.PSVersion.Major -lt 7) { throw 'Use PowerShell 7 for remote deployment.' }
$root = Split-Path $PSScriptRoot -Parent
if (!$ConfigPath) {
    $ConfigPath = Join-Path $root 'deploy\ip.local.json'
    if (!(Test-Path -LiteralPath $ConfigPath)) { $ConfigPath = Join-Path $root 'deploy\remote.local.json' }
}
$env:PATH = (Join-Path $env:ProgramFiles 'nodejs') + ';' + $env:PATH
if ($Action -eq 'Show') {
    & (Join-Path $PSScriptRoot 'show_public_access.ps1') -ConfigPath $ConfigPath
    return
}
if ($Action -eq 'Stop') {
    & (Join-Path $PSScriptRoot 'deployment_launcher.ps1') -Action Stop -NonInteractive
    return
}
if (!(Test-Path -LiteralPath $ConfigPath)) { throw ('Remote deployment configuration is missing: ' + $ConfigPath) }
. (Join-Path $PSScriptRoot 'python_runtime.ps1')
$pythonExe = Resolve-SpaceSimPython -RepositoryRoot $root -RequestedPython $Python -RequiredModules @('fastapi', 'pydantic', 'uvicorn', 'numpy', 'Basilisk.simulation.mujoco')
$localStatePath = Join-Path $root 'run\local-visualization\state.json'
if (Test-Path -LiteralPath $localStatePath) {
    & powershell.exe -NoProfile -File (Join-Path $PSScriptRoot 'local_visualization.ps1') -Action Stop
    if ($LASTEXITCODE -ne 0) { throw 'Local platform did not stop cleanly; remote setup was not started.' }
}
& $pythonExe (Join-Path $root 'tools\migrate_local_auth.py') --source (Join-Path $root 'data\visualization-test-auth.sqlite3') --destination (Join-Path $root 'data\auth.sqlite3')
if ($LASTEXITCODE -ne 0) { throw 'Account database migration failed.' }
$settings = Get-Content -LiteralPath $ConfigPath -Raw | ConvertFrom-Json
$mode = switch ($settings.tls_mode) {
    'ip-acme' { 'Ip' }
    'tunnel' { 'Public' }
    default { 'Direct' }
}
& (Join-Path $PSScriptRoot 'deployment_launcher.ps1') -Action Start -Mode $mode -ConfigPath $ConfigPath -Python $pythonExe -NonInteractive -Restart:$Restart
if ($settings.require_access_key) {
    Write-Host 'Use show_remote_access.cmd to copy the protected access link. Open that link on your own computer, not inside Remote Desktop.'
} else {
    Write-Host ('Open ' + $settings.public_url + '/ on your own computer and log in. No access_key parameter is required.')
}
