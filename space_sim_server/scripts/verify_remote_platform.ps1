param([string]$Python)
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
if (!$Python) {
    $Python = Join-Path ([Environment]::GetFolderPath('Desktop')) 'space_sim_workspace\space_sim_server\.venv\Scripts\python.exe'
}
$report = Get-Content -LiteralPath (Join-Path $root 'run/public-access.json') -Raw | ConvertFrom-Json
$hash = [Security.Cryptography.SHA256]::Create()
try { $identifier = [BitConverter]::ToString($hash.ComputeHash([Text.Encoding]::UTF8.GetBytes(([string]$report.config_path).ToLowerInvariant()))).Replace('-', '') }
finally { $hash.Dispose() }
$saved = Import-Clixml -LiteralPath (Join-Path $root "deploy/secrets/$identifier.clixml")
$previous = $env:PIXEL_STREAMING_ACCESS_KEY
try {
    $env:PIXEL_STREAMING_ACCESS_KEY = if ($report.require_access_key -ne $false) {
        [pscredential]::new('deployment', $saved.SPACE_SIM_STREAM_ACCESS_KEY).GetNetworkCredential().Password
    } else { $null }
    Push-Location $root
    try {
        & $Python (Join-Path $root 'tools/verify_remote_platform.py')
        if ($LASTEXITCODE -ne 0) { throw 'Remote platform verification failed.' }
    } finally { Pop-Location }
} finally {
    $env:PIXEL_STREAMING_ACCESS_KEY = $previous
}
