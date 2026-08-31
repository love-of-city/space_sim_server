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

function Stop-OrphanedPlatformBackends {
    # A PowerShell launcher can be terminated before its Python child.  Such a
    # child has no platform.json entry but still owns the API/control ports.
    # Match the project's exact module entry point so unrelated Python jobs are
    # never touched.
    $backends = @(Get-CimInstance Win32_Process -Filter "Name = 'python.exe' OR Name = 'pythonw.exe'" -ErrorAction SilentlyContinue |
        Where-Object { $_.CommandLine -match '(?i)(^|\s)-m\s+space_arm_platform\.main(\s|$)' })
    foreach ($backend in $backends) {
        Stop-RecordedProcessTree ([int]$backend.ProcessId) 0
        if (!$Quiet) { Write-Output "Stopped orphaned Space Arm backend (PID $($backend.ProcessId))." }
    }
}

if (Test-Path -LiteralPath $statePath) {
    $state = Get-Content -Raw -LiteralPath $statePath | ConvertFrom-Json
    Stop-RecordedProcessTree ([int]$state.simulation_pid) ([long]$state.simulation_start)
    if ($state.PSObject.Properties.Name -contains 'backend_service_pid') {
        Stop-RecordedProcessTree ([int]$state.backend_service_pid) ([long]$state.backend_service_start)
    }
    Stop-RecordedProcessTree ([int]$state.backend_pid) ([long]$state.backend_start)
    Remove-Item -LiteralPath $statePath -Force
}

Stop-OrphanedPlatformBackends

$stopRenderer = Join-Path $adapterRoot 'Unreal\BskUnrealRenderer\scripts\stop_renderer.ps1'
if (Test-Path -LiteralPath $stopRenderer) { & $stopRenderer }
& (Join-Path $PSScriptRoot 'stop_pixel_streaming.ps1') -Quiet
if (!$Quiet) { Write-Output 'Space Arm Data Platform processes have been stopped.' }
