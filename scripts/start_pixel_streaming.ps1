param(
    [string]$UnrealRoot = 'E:\UE5.6',
    [ValidateRange(1, 65535)]
    [int]$StreamerPort = 8888,
    [ValidateRange(1, 65535)]
    [int]$PlayerPort = 8080,
    [string]$PublicIp = '127.0.0.1'
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$runDirectory = Join-Path $projectRoot 'run'
$logDirectory = Join-Path $projectRoot 'logs'
$statePath = Join-Path $runDirectory 'pixel_streaming.json'
New-Item -ItemType Directory -Path $runDirectory,$logDirectory -Force | Out-Null

& (Join-Path $PSScriptRoot 'prepare_pixel_streaming.ps1') -UnrealRoot $UnrealRoot

if (Test-Path -LiteralPath $statePath) {
    $old = Get-Content -Raw -LiteralPath $statePath | ConvertFrom-Json
    $process = Get-Process -Id ([int]$old.pid) -ErrorAction SilentlyContinue
    if ($process -and $process.StartTime.ToUniversalTime().Ticks -eq [long]$old.start_ticks) {
        Write-Output "Pixel Streaming signalling server is already running (PID $($process.Id))."
        exit 0
    }
    Remove-Item -LiteralPath $statePath -Force
}

$ueCandidates = @($UnrealRoot, (Join-Path $UnrealRoot 'UE_5.6'), 'E:\UE5.6\UE_5.6')
$ue = $ueCandidates | Where-Object { Test-Path -LiteralPath (Join-Path $_ 'Engine\Binaries\Win64\UnrealEditor.exe') } | Select-Object -First 1
if (!$ue) { throw 'Unreal Engine 5.6 root could not be resolved.' }
$server = Join-Path $ue 'Engine\Plugins\Media\PixelStreaming2\Resources\WebServers\SignallingWebServer'
$node = Join-Path $server 'platform_scripts\cmd\node\node.exe'
$arguments = @(
    'dist/index.js', '--streamer_port', "$StreamerPort", '--player_port', "$PlayerPort",
    '--serve', '--http_root', 'www', '--homepage', 'player.html',
    '--public_ip', $PublicIp, '--console_messages', 'basic', '--log_config'
)
$process = Start-Process -FilePath $node -ArgumentList $arguments -WorkingDirectory $server -PassThru -WindowStyle Hidden `
    -RedirectStandardOutput (Join-Path $logDirectory 'pixel-streaming.out.log') `
    -RedirectStandardError (Join-Path $logDirectory 'pixel-streaming.err.log')
@{
    pid = $process.Id
    start_ticks = $process.StartTime.ToUniversalTime().Ticks
    streamer_port = $StreamerPort
    player_port = $PlayerPort
} | ConvertTo-Json | Set-Content -LiteralPath $statePath -Encoding utf8

$deadline = [DateTime]::UtcNow.AddSeconds(20)
do {
    if ($process.HasExited) { throw 'Pixel Streaming signalling server exited during startup.' }
    try {
        $response = Invoke-WebRequest -UseBasicParsing -Uri "http://127.0.0.1:$PlayerPort/player.html" -TimeoutSec 1
        if ($response.StatusCode -eq 200) { break }
    } catch { Start-Sleep -Milliseconds 250 }
} while ([DateTime]::UtcNow -lt $deadline)
if ([DateTime]::UtcNow -ge $deadline) { throw "Pixel Streaming player did not become ready on port $PlayerPort." }
Write-Output "Pixel Streaming player ready: http://127.0.0.1:$PlayerPort/player.html"

