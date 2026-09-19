# Shared scene-process lifecycle helpers; dot-sourcing does not start/stop anything.
function Get-SceneProcessStartTicks($Process) {
    # A recycled PID can now belong to a protected Windows process. StartTime
    # may be null or throw (including when a process exits during inspection).
    # Unknown identity must fail closed: never stop it and never abort startup.
    if (!$Process) { return $null }
    try {
        $startTime = $Process.StartTime
        if ($null -eq $startTime) { return $null }
        return $startTime.ToUniversalTime().Ticks
    } catch {
        return $null
    }
}

function Stop-RecordedProcessTree([int]$ProcessId, [long]$ExpectedStartTicks) {
    if ($ProcessId -le 0 -or $ProcessId -eq $PID) { return }
    $process = Get-Process -Id $ProcessId -ErrorAction SilentlyContinue
    if (!$process) { return }
    $actualStart = Get-SceneProcessStartTicks $process
    if ($null -eq $actualStart) {
        Write-Warning "Skipped PID $ProcessId because its start time is unavailable; process identity was not verified."
        return
    }
    if ($ExpectedStartTicks -gt 0 -and $actualStart -ne $ExpectedStartTicks) { return }
    $children = @(Get-CimInstance Win32_Process -Filter "ParentProcessId=$ProcessId" -ErrorAction SilentlyContinue)
    foreach ($child in $children) {
        # Parent IDs can be reused too; do not touch older unrelated processes.
        $childProcess = Get-Process -Id ([int]$child.ProcessId) -ErrorAction SilentlyContinue
        $childStart = Get-SceneProcessStartTicks $childProcess
        if ($null -ne $childStart -and $childStart -ge $actualStart) {
            Stop-RecordedProcessTree ([int]$child.ProcessId) $childStart
        }
    }
    $current = Get-Process -Id $ProcessId -ErrorAction SilentlyContinue
    $currentStart = Get-SceneProcessStartTicks $current
    if ($null -ne $currentStart -and $currentStart -eq $actualStart) {
        Stop-Process -Id $ProcessId -Force -ErrorAction SilentlyContinue
    }
}

function Wait-SceneRuntimeExit($Renderer, $Simulation, [scriptblock]$StopRequested) {
    while ($true) {
        if (& $StopRequested) { return @{component='stopped'; code=0} }
        # UE is just as essential as physics. Waiting only for simulation used
        # to leave an orphan simulation and a misleading 'running' status.
        if ($Renderer.HasExited) { return @{component='renderer'; code=$Renderer.ExitCode} }
        if ($Simulation.HasExited) { return @{component='simulation'; code=$Simulation.ExitCode} }
        Start-Sleep -Milliseconds 250
    }
}
