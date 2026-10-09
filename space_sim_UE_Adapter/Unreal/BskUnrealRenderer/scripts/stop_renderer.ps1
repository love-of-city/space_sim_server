param()

. (Join-Path $PSScriptRoot 'common.ps1')
$pidFile = Join-Path $ProjectRoot 'Saved\BskRenderer.pid'
if (!(Test-Path -LiteralPath $pidFile)) {
    Write-Output 'No renderer PID file exists; nothing to stop.'
    exit 0
}
$rendererPid = [int](Get-Content -Raw -LiteralPath $pidFile)
$process = Get-Process -Id $rendererPid -ErrorAction SilentlyContinue
if ($process -and $process.ProcessName -like 'UnrealEditor*') {
    Stop-Process -Id $rendererPid
    [void]$process.WaitForExit(10000)
    Write-Output "Stopped BSK Unreal Renderer PID $rendererPid"
} else {
    Write-Output "PID $rendererPid is no longer an Unreal Editor process."
}
Remove-Item -LiteralPath $pidFile -Force
