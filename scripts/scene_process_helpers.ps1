# Shared scene-process lifecycle helpers; dot-sourcing does not start/stop anything.
function Stop-RecordedProcessTree([int]$ProcessId, [long]$ExpectedStartTicks) {
    if ($ProcessId -le 0 -or $ProcessId -eq $PID) { return }
    $process = Get-Process -Id $ProcessId -ErrorAction SilentlyContinue
    if (!$process) { return }
    $actualStart = $process.StartTime.ToUniversalTime().Ticks
    if ($ExpectedStartTicks -gt 0 -and $actualStart -ne $ExpectedStartTicks) { return }
    $children = @(Get-CimInstance Win32_Process -Filter "ParentProcessId=$ProcessId" -ErrorAction SilentlyContinue)
    foreach ($child in $children) {
        # Parent IDs can be reused too; do not touch older unrelated processes.
        $childProcess = Get-Process -Id ([int]$child.ProcessId) -ErrorAction SilentlyContinue
        if ($childProcess -and $childProcess.StartTime.ToUniversalTime().Ticks -ge $actualStart) {
            Stop-RecordedProcessTree ([int]$child.ProcessId) $childProcess.StartTime.ToUniversalTime().Ticks
        }
    }
    $current = Get-Process -Id $ProcessId -ErrorAction SilentlyContinue
    if ($current -and $current.StartTime.ToUniversalTime().Ticks -eq $actualStart) {
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
