param(
    [switch]$Quiet,
    [switch]$PreserveState,
    [string]$AdapterRoot = ''
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$statePath = Join-Path $projectRoot 'run\scene_runtime.json'

function Stop-RecordedProcessTree([int]$ProcessId, [long]$ExpectedStartTicks) {
    if ($ProcessId -le 0 -or $ProcessId -eq $PID) { return }
    $process = Get-Process -Id $ProcessId -ErrorAction SilentlyContinue
    if (!$process) { return }
    if ($ExpectedStartTicks -gt 0 -and $process.StartTime.ToUniversalTime().Ticks -ne $ExpectedStartTicks) {
        if (!$Quiet) { Write-Warning "Skipped reused PID $ProcessId because its start time no longer matches." }
        return
    }
    $children = @(Get-CimInstance Win32_Process -Filter "ParentProcessId=$ProcessId" -ErrorAction SilentlyContinue)
    foreach ($child in $children) { Stop-RecordedProcessTree ([int]$child.ProcessId) 0 }
    Stop-Process -Id $ProcessId -Force -ErrorAction SilentlyContinue
}

$state = $null
if (Test-Path -LiteralPath $statePath) {
    try { $state = Get-Content -Raw -LiteralPath $statePath | ConvertFrom-Json } catch { }
}
if ($state) {
    if ($state.PSObject.Properties.Name -contains 'simulation_pid') {
        $start = if ($state.PSObject.Properties.Name -contains 'simulation_start') { [long]$state.simulation_start } else { 0 }
        Stop-RecordedProcessTree ([int]$state.simulation_pid) $start
    }
    if ($state.PSObject.Properties.Name -contains 'renderer_pid') {
        $start = if ($state.PSObject.Properties.Name -contains 'renderer_start') { [long]$state.renderer_start } else { 0 }
        Stop-RecordedProcessTree ([int]$state.renderer_pid) $start
    }
    if (!$AdapterRoot -and $state.PSObject.Properties.Name -contains 'adapter_root') {
        $AdapterRoot = [string]$state.adapter_root
    }
    if (!$PreserveState) {
        $state.phase = 'stopped'
        $state.updated_at_ns = [string][DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds() + '000000'
        $state.error = ''
        $state | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath $statePath -Encoding utf8
    }
    if ($state.PSObject.Properties.Name -contains 'launcher_pid') {
        $start = if ($state.PSObject.Properties.Name -contains 'launcher_start') { [long]$state.launcher_start } else { 0 }
        Stop-RecordedProcessTree ([int]$state.launcher_pid) $start
    }
}

if ($AdapterRoot) {
    $pidFile = Join-Path $AdapterRoot 'Unreal\BskUnrealRenderer\Saved\BskRenderer.pid'
    if (Test-Path -LiteralPath $pidFile) {
        try {
            $rendererPid = [int](Get-Content -Raw -LiteralPath $pidFile)
            Stop-RecordedProcessTree $rendererPid 0
        } catch { }
        Remove-Item -LiteralPath $pidFile -Force -ErrorAction SilentlyContinue
    }
}
if (!$Quiet) { Write-Output 'The active UE and Basilisk/MJScene scene instance has been stopped.' }
