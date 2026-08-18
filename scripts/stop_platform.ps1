param([switch]$Quiet)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$workspaceRoot = Split-Path -Parent $projectRoot
$statePath = Join-Path $projectRoot 'run\platform.json'
$adapterRoot = Join-Path $workspaceRoot 'space_sim_UE_adapter'

function Stop-RecordedProcessTree([int]$ProcessId, [long]$ExpectedStartTicks) {
    $process = Get-Process -Id $ProcessId -ErrorAction SilentlyContinue
    if (!$process) { return }
    if ($ExpectedStartTicks -gt 0 -and $process.StartTime.ToUniversalTime().Ticks -ne $ExpectedStartTicks) {
        Write-Warning "Skipped reused PID $ProcessId because its start time no longer matches."
        return
    }
    $children = @(Get-CimInstance Win32_Process -Filter "ParentProcessId=$ProcessId" -ErrorAction SilentlyContinue)
    foreach ($child in $children) { Stop-RecordedProcessTree ([int]$child.ProcessId) 0 }
    Stop-Process -Id $ProcessId -Force -ErrorAction SilentlyContinue
}

if (Test-Path -LiteralPath $statePath) {
    $state = Get-Content -Raw -LiteralPath $statePath | ConvertFrom-Json
    Stop-RecordedProcessTree ([int]$state.simulation_pid) ([long]$state.simulation_start)
    Stop-RecordedProcessTree ([int]$state.backend_pid) ([long]$state.backend_start)
    Remove-Item -LiteralPath $statePath -Force
}

$stopRenderer = Join-Path $adapterRoot 'Unreal\BskUnrealRenderer\scripts\stop_renderer.ps1'
if (Test-Path -LiteralPath $stopRenderer) { & $stopRenderer }
if (!$Quiet) { Write-Output 'Space Arm Data Platform processes have been stopped.' }

