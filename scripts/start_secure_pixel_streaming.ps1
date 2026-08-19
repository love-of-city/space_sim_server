param(
    [ValidateRange(1, 65535)]
    [int]$StreamerPort = 8888,
    [ValidateRange(1, 65535)]
    [int]$PlayerPort = 8080,
    [Parameter(Mandatory = $true)]
    [string]$JwtSecret,
    [string]$IceServersJson = '[]',
    [string]$TurnUrlsJson = '[]',
    [string]$TurnAuthSecret = '',
    [ValidateRange(1, 64)]
    [int]$MaxSubscribers = 4
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$serviceRoot = Join-Path $projectRoot 'signalling'
$frontendRoot = Join-Path $projectRoot 'frontend'
$runDirectory = Join-Path $projectRoot 'run'
$logDirectory = Join-Path $projectRoot 'logs'
$statePath = Join-Path $runDirectory 'pixel_streaming.json'
New-Item -ItemType Directory -Path $runDirectory,$logDirectory -Force | Out-Null

if (Test-Path -LiteralPath $statePath) {
    $old = Get-Content -Raw -LiteralPath $statePath | ConvertFrom-Json
    $oldProcess = Get-Process -Id ([int]$old.pid) -ErrorAction SilentlyContinue
    if ($oldProcess -and $oldProcess.StartTime.ToUniversalTime().Ticks -eq [long]$old.start_ticks) {
        throw "A Pixel Streaming signalling process is already running (PID $($oldProcess.Id))."
    }
    Remove-Item -LiteralPath $statePath -Force
}

Push-Location $serviceRoot
try {
    if (!(Test-Path -LiteralPath (Join-Path $serviceRoot 'node_modules'))) {
        npm.cmd install --no-audit --no-fund
        if ($LASTEXITCODE -ne 0) { throw 'Secure signalling dependency installation failed.' }
    }
    npm.cmd run check
    if ($LASTEXITCODE -ne 0) { throw 'Secure signalling syntax check failed.' }
} finally {
    Pop-Location
}

Push-Location $frontendRoot
try {
    if (!(Test-Path -LiteralPath (Join-Path $frontendRoot 'node_modules'))) {
        npm.cmd install --no-audit --no-fund
        if ($LASTEXITCODE -ne 0) { throw 'Operator console dependency installation failed.' }
    }
    npm.cmd run build
    if ($LASTEXITCODE -ne 0) { throw 'Operator console Pixel Streaming SDK build failed.' }
} finally {
    Pop-Location
}

$previous = @{
    PS_PLAYER_PORT = $env:PS_PLAYER_PORT
    PS_PLAYER_HOST = $env:PS_PLAYER_HOST
    PS_STREAMER_PORT = $env:PS_STREAMER_PORT
    PS_STREAMER_HOST = $env:PS_STREAMER_HOST
    PS_JWT_SECRET = $env:PS_JWT_SECRET
    PS_ICE_SERVERS_JSON = $env:PS_ICE_SERVERS_JSON
    PS_TURN_URLS_JSON = $env:PS_TURN_URLS_JSON
    PS_TURN_AUTH_SECRET = $env:PS_TURN_AUTH_SECRET
    PS_MAX_SUBSCRIBERS = $env:PS_MAX_SUBSCRIBERS
}
try {
    $env:PS_PLAYER_PORT = "$PlayerPort"
    $env:PS_PLAYER_HOST = '0.0.0.0'
    $env:PS_STREAMER_PORT = "$StreamerPort"
    $env:PS_STREAMER_HOST = '127.0.0.1'
    $env:PS_JWT_SECRET = $JwtSecret
    $env:PS_ICE_SERVERS_JSON = $IceServersJson
    $env:PS_TURN_URLS_JSON = $TurnUrlsJson
    $env:PS_TURN_AUTH_SECRET = $TurnAuthSecret
    $env:PS_MAX_SUBSCRIBERS = "$MaxSubscribers"
    $process = Start-Process -FilePath (Get-Command node.exe).Source -ArgumentList 'server.mjs' `
        -WorkingDirectory $serviceRoot -PassThru -WindowStyle Hidden `
        -RedirectStandardOutput (Join-Path $logDirectory 'pixel-streaming-secure.out.log') `
        -RedirectStandardError (Join-Path $logDirectory 'pixel-streaming-secure.err.log')
} finally {
    foreach ($entry in $previous.GetEnumerator()) {
        if ($null -eq $entry.Value) { Remove-Item -Path "Env:$($entry.Key)" -ErrorAction SilentlyContinue }
        else { Set-Item -Path "Env:$($entry.Key)" -Value $entry.Value }
    }
}

@{
    pid = $process.Id
    start_ticks = $process.StartTime.ToUniversalTime().Ticks
    streamer_port = $StreamerPort
    player_port = $PlayerPort
    mode = 'secure'
} | ConvertTo-Json | Set-Content -LiteralPath $statePath -Encoding utf8

$deadline = [DateTime]::UtcNow.AddSeconds(20)
$ready = $false
while (!$ready -and [DateTime]::UtcNow -lt $deadline) {
    if ($process.HasExited) { throw 'Secure Pixel Streaming signalling exited during startup.' }
    $probe = [Net.Sockets.TcpClient]::new()
    try {
        $connect = $probe.BeginConnect('127.0.0.1', $PlayerPort, $null, $null)
        if ($connect.AsyncWaitHandle.WaitOne(300)) {
            $probe.EndConnect($connect)
            $ready = $probe.Connected
        }
    } catch { $ready = $false } finally { $probe.Dispose() }
}
if (!$ready) { throw "Secure Pixel Streaming signalling did not bind player port $PlayerPort." }
Write-Output "Secure Pixel Streaming signalling ready: player=$PlayerPort streamer=$StreamerPort"
