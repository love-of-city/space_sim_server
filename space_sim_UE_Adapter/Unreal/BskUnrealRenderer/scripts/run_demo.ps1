param(
    [ValidateSet('Mock', 'Basilisk')]
    [string]$Sender = 'Mock',
    [double]$Duration = 60.0,
    [int]$Port = 5558,
    [string]$UnrealRoot = '',
    [string]$CaptureDirectory = '',
    [string[]]$CaptureProducts = @(),
    [ValidateRange(0.0, 60.0)]
    [double]$CaptureRate = 0.0,
    [ValidateRange(0, 65535)]
    [int]$CaptureNetworkPort = 0,
    [string]$AutoCommand = '',
    [switch]$KeepRendererOpen,
    [switch]$Rebuild
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
$ue = Resolve-UnrealRoot $UnrealRoot
$pluginBinary = Join-Path $ProjectRoot 'Plugins\BskUnrealRuntime\Binaries\Win64\UnrealEditor-BskUnrealRuntime.dll'

if ($Rebuild -or !(Test-Path -LiteralPath $pluginBinary)) {
    & (Join-Path $PSScriptRoot 'build.ps1') -UnrealRoot $ue
}

& (Join-Path $PSScriptRoot 'start_renderer.ps1') -UnrealRoot $ue -Port $Port `
    -CaptureDirectory $CaptureDirectory -CaptureProducts $CaptureProducts -CaptureRate $CaptureRate `
    -CaptureNetworkPort $CaptureNetworkPort -AutoCommand $AutoCommand
$pidFile = Join-Path $ProjectRoot 'Saved\BskRenderer.pid'
$rendererPid = [int](Get-Content -Raw -LiteralPath $pidFile)

try {
    Write-Output "Waiting for UE receiver on 127.0.0.1:$Port ..."
    $deadline = [DateTime]::UtcNow.AddSeconds(90)
    $ready = $false
    while (!$ready -and [DateTime]::UtcNow -lt $deadline) {
        $renderer = Get-Process -Id $rendererPid -ErrorAction SilentlyContinue
        if (!$renderer) { throw 'Unreal Engine exited before the receiver became ready.' }
        $probe = [Net.Sockets.TcpClient]::new()
        try {
            $connect = $probe.BeginConnect('127.0.0.1', $Port, $null, $null)
            if ($connect.AsyncWaitHandle.WaitOne(500)) {
                $probe.EndConnect($connect)
                $ready = $probe.Connected
            }
        } catch {
            $ready = $false
        } finally {
            $probe.Dispose()
        }
        if (!$ready) { Start-Sleep -Milliseconds 500 }
    }
    if (!$ready) { throw "UE receiver was not ready on port $Port after 90 seconds." }
    Start-Sleep -Milliseconds 500

    Write-Output "Starting $Sender sender for $Duration seconds ..."
    if ($Sender -eq 'Basilisk') {
        & (Join-Path $PSScriptRoot 'run_bsk.ps1') -Port $Port -Duration $Duration
    } else {
        & (Join-Path $PSScriptRoot 'run_mock.ps1') -Port $Port -Duration $Duration
    }
    if ($LASTEXITCODE -ne 0) { throw "$Sender sender failed with exit code $LASTEXITCODE." }
} finally {
    if (!$KeepRendererOpen) {
        & (Join-Path $PSScriptRoot 'stop_renderer.ps1')
    }
}

if ($KeepRendererOpen) {
    Write-Output 'Demo sender finished; UE remains open. Run .\scripts\stop_renderer.ps1 to close it.'
}
