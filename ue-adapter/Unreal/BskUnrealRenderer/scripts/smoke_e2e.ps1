param(
    [string]$UnrealRoot = '',
    [int]$Port = 5561,
    [ValidateSet('Mock', 'Basilisk')]
    [string]$Sender = 'Mock',
    [switch]$WithRendering,
    [string]$ScreenshotPath = ''
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
$ue = Resolve-UnrealRoot $UnrealRoot
$editor = Join-Path $ue 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$logName = 'E2E-{0}.log' -f [DateTime]::UtcNow.ToString('yyyyMMdd-HHmmss')
$logPath = Join-Path $ProjectRoot (Join-Path 'Saved\Logs' $logName)
$arguments = @($ProjectFile, '-game', '-unattended', '-nosplash', "-BskPort=$Port", "-abslog=$logPath")
if ($WithRendering) {
    $arguments += @('-RenderOffscreen', '-ResX=640', '-ResY=360')
} else {
    $arguments += '-NullRHI'
}
if ($ScreenshotPath) {
    if (![IO.Path]::IsPathRooted($ScreenshotPath)) {
        $ScreenshotPath = [IO.Path]::GetFullPath((Join-Path $ProjectRoot $ScreenshotPath))
    }
    $screenshotDirectory = Split-Path -Parent $ScreenshotPath
    New-Item -ItemType Directory -Path $screenshotDirectory -Force | Out-Null
    $arguments += "-BskScreenshot=$ScreenshotPath"
}
$process = Start-Process -FilePath $editor -ArgumentList $arguments -WindowStyle Hidden -PassThru
try {
    $deadline = [DateTime]::UtcNow.AddSeconds(45)
    do {
        Start-Sleep -Milliseconds 500
        if ($process.HasExited) { throw "UE exited early with code $($process.ExitCode)" }
        $ready = (Test-Path -LiteralPath $logPath) -and
            (Select-String -LiteralPath $logPath -SimpleMatch "BSK receiver listening on 127.0.0.1:$Port" -Quiet)
    } while (!$ready -and [DateTime]::UtcNow -lt $deadline)
    if (!$ready) { throw 'UE receiver did not become ready.' }

    if ($Sender -eq 'Basilisk') {
        & (Join-Path $PSScriptRoot 'run_bsk.ps1') -Port $Port -Duration 2
    } else {
        & (Join-Path $PSScriptRoot 'run_mock.ps1') -Port $Port -Duration 2 -Rate 30
    }
    if ($LASTEXITCODE -ne 0) { throw 'First mock sender run failed.' }
    Start-Sleep -Seconds 2
    if ($process.HasExited) { throw 'UE exited after sender disconnected.' }

    if ($Sender -eq 'Basilisk') {
        & (Join-Path $PSScriptRoot 'run_bsk.ps1') -Port $Port -Duration 2
    } else {
        & (Join-Path $PSScriptRoot 'run_mock.ps1') -Port $Port -Duration 2 -Rate 30
    }
    if ($LASTEXITCODE -ne 0) { throw 'Second mock sender run failed.' }
    Start-Sleep -Seconds 2
    if ($process.HasExited) { throw 'UE exited after sender reconnected.' }

    $markers = Select-String -LiteralPath $logPath -Pattern (
        'BSK receiver listening|BSK sender connected|Applying first BSK frame|BSK sender disconnected'
    )
    $markers | ForEach-Object { Write-Output $_.Line }
    $connected = @($markers | Where-Object { $_.Line -like '*BSK sender connected*' }).Count
    if ($connected -lt 2) { throw "Expected two sender connections, observed $connected." }
    if (!($markers | Where-Object { $_.Line -like '*Applying first BSK frame*' })) {
        throw 'No BSK frame was applied on the Game Thread.'
    }
    Write-Output "E2E passed; UE stayed alive and accepted two sender sessions. Log: $logPath"
} finally {
    if (!$process.HasExited -and $process.ProcessName -like 'UnrealEditor*') {
        Stop-Process -Id $process.Id
        [void]$process.WaitForExit(10000)
    }
}
