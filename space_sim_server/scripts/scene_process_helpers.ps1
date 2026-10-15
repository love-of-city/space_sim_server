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
    $null = $process.Handle
    # Stop spawning before enumerating descendants. The old child-first walk
    # let a warming PowerShell launcher create Python after its child snapshot.
    # Retain the process handle while walking, so this parent PID is not reused.
    $current = Get-Process -Id $ProcessId -ErrorAction SilentlyContinue
    $currentStart = Get-SceneProcessStartTicks $current
    if ($null -eq $currentStart -or $currentStart -ne $actualStart) { return }
    Stop-Process -Id $ProcessId -Force -ErrorAction SilentlyContinue
    if (!$process.WaitForExit(15000)) {
        throw "Scene process $ProcessId did not exit within 15 seconds; refusing to start a competing scene."
    }
    $children = @(Get-CimInstance Win32_Process -Filter "ParentProcessId=$ProcessId" -ErrorAction SilentlyContinue)
    foreach ($child in $children) {
        $childProcess = Get-Process -Id ([int]$child.ProcessId) -ErrorAction SilentlyContinue
        $childStart = Get-SceneProcessStartTicks $childProcess
        if ($null -ne $childStart -and $childStart -ge $actualStart) {
            if ($child.CreationDate -and [Math]::Abs($childStart - $child.CreationDate.ToUniversalTime().Ticks) -ge 10000) { continue }
            Stop-RecordedProcessTree ([int]$child.ProcessId) $childStart
        }
    }
}

function Stop-ProjectSimulations([string]$ProjectRoot) {
    # Recover already orphaned native clients, including virtualenv launchers.
    # Both the entry point and generated instance path must belong to this
    # checkout; a module name or a Python process name alone is not ownership.
    $entry = [IO.Path]::GetFullPath((Join-Path $ProjectRoot 'simulation\teleop_grasp_unreal.py')).Replace('\', '/')
    $instances = [IO.Path]::GetFullPath((Join-Path $ProjectRoot 'run\scenes')).Replace('\', '/')
    $entryPattern = '(?i)^\s*(?:"[^"]+"|\S+)\s+(?:-u\s+)?"?' + [regex]::Escape($entry) + '"?(?=\s|$)'
    $instancePattern = '(?i)(?:^|\s)--scene-instance\s+"?' + [regex]::Escape($instances) + '/scene-[0-9]{8}-[0-9]{6}-[a-f0-9]+\.json"?(?=\s|$)'
    $candidates = @(Get-CimInstance Win32_Process -Filter "Name = 'python.exe' OR Name = 'pythonw.exe'" -ErrorAction Stop)
    foreach ($candidate in $candidates) {
        $command = ([string]$candidate.CommandLine).Replace('\', '/')
        if ($command -notmatch $entryPattern -or $command -notmatch $instancePattern) { continue }
        $process = Get-Process -Id ([int]$candidate.ProcessId) -ErrorAction SilentlyContinue
        $startTicks = Get-SceneProcessStartTicks $process
        if ($null -eq $startTicks -or !$candidate.CreationDate) { continue }
        if ([Math]::Abs($startTicks - $candidate.CreationDate.ToUniversalTime().Ticks) -ge 10000) { continue }
        Stop-RecordedProcessTree ([int]$candidate.ProcessId) $startTicks
    }
}

function Stop-ProjectRenderers([string]$AdapterRoot) {
    if (!$AdapterRoot) { return }
    $project = [IO.Path]::GetFullPath((Join-Path $AdapterRoot 'Unreal\BskUnrealRenderer\BskUnrealRenderer.uproject'))
    $pattern = '(?i)(?:^|\s)"?' + [regex]::Escape($project) + '"?(?=\s|$)'
    $renderers = @(Get-CimInstance Win32_Process -Filter "Name = 'UnrealEditor.exe'" -ErrorAction Stop)
    foreach ($renderer in $renderers) {
        if ($renderer.CommandLine -notmatch $pattern -or $renderer.CommandLine -notmatch '(?i)(?:^|\s)-game(?:\s|$)') { continue }
        $process = Get-Process -Id ([int]$renderer.ProcessId) -ErrorAction SilentlyContinue
        $startTicks = Get-SceneProcessStartTicks $process
        if ($null -eq $startTicks) { continue }
        if ([Math]::Abs($startTicks - $renderer.CreationDate.ToUniversalTime().Ticks) -ge 10000) { continue }
        Stop-RecordedProcessTree ([int]$renderer.ProcessId) $startTicks
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
