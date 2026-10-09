param(
    [ValidateSet('Start', 'Stop', 'Status')][string]$Action = 'Start',
    [string]$AdapterRoot,
    [string]$BackendPython,
    [string]$SimulationPython = 'C:\tools\miniconda3\envs\mujoco-dev\python.exe',
    [string]$UnrealRoot = 'C:\Program Files\Epic Games\UE_5.6',
    [string]$Node = 'C:\Program Files\nodejs\node.exe',
    [string]$AuthDatabase,
    [ValidateSet('Platform', 'Demo')][string]$Mode = 'Platform',
    [string]$ModelRoot,
    [string]$PowerShellExe = 'C:\tools\powershell-7.4.13\pwsh.exe',
    [ValidateRange(10, 86400)][int]$Duration = 3600,
    [switch]$NoBrowser
)

$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
. (Join-Path $PSScriptRoot 'scene_process_helpers.ps1')
$runDirectory = Join-Path $root 'run\local-visualization'
$statePath = Join-Path $runDirectory 'state.json'
$stopPath = Join-Path $runDirectory 'stop.request'
$url = 'http://127.0.0.1:18000/'

function Test-RecordedProcess($Record) {
    if (!$Record) { return $false }
    $process = Get-Process -Id $Record.id -ErrorAction SilentlyContinue
    return ($process -and $process.StartTime.ToUniversalTime().Ticks -eq [long]$Record.ticks)
}

if ($Action -ne 'Start') {
    if (!(Test-Path -LiteralPath $statePath)) { Write-Host 'Not running.'; exit 0 }
    $recorded = Get-Content -LiteralPath $statePath -Raw | ConvertFrom-Json
    if ($Action -eq 'Status') {
        Write-Host ('Supervisor running: ' + (Test-RecordedProcess $recorded.supervisor))
        foreach ($record in $recorded.processes) {
            Write-Host ($record.name + ': ' + (Test-RecordedProcess $record))
        }
        Write-Host ('Browser: ' + $url)
        Write-Host ('Logs: ' + $recorded.logs)
        exit 0
    }
    New-Item -ItemType File -Path $stopPath -Force | Out-Null
    $deadline = (Get-Date).AddSeconds(40)
    while ((Test-RecordedProcess $recorded.supervisor) -and (Get-Date) -lt $deadline) {
        Start-Sleep -Milliseconds 250
    }
    if (Test-RecordedProcess $recorded.supervisor) {
        throw 'Supervisor did not stop. Inspect logs before trying again.'
    }
    for ($index = $recorded.processes.Count - 1; $index -ge 0; $index--) {
        $record = $recorded.processes[$index]
        if ($record) { Stop-RecordedProcessTree $record.id ([long]$record.ticks) }
    }
    Write-Host 'Visualization stopped; unrelated applications were left alone.'
    exit 0
}

New-Item -ItemType Directory -Path $runDirectory -Force | Out-Null
$lock = $null
try {
    $lock = [System.IO.File]::Open((Join-Path $runDirectory 'launcher.lock'), 'OpenOrCreate', 'ReadWrite', 'None')
} catch {
    Write-Host ('Already starting or running. Open ' + $url + ' or use stop_local_visualization.cmd.')
    exit 1
}
$script:records = @()
$logDirectory = Join-Path $root ('logs\local-visualization-' + (Get-Date -Format 'yyyyMMdd-HHmmss'))
$supervisor = Get-Process -Id $PID
$script:state = @{supervisor=@{id=$PID; ticks=$supervisor.StartTime.ToUniversalTime().Ticks}; processes=@(); logs=$logDirectory; mode=$Mode}

function Save-State {
    $script:state.processes = @($script:records)
    $temporary = $statePath + '.tmp'
    $script:state | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $temporary -Encoding UTF8
    Move-Item -LiteralPath $temporary -Destination $statePath -Force
}

function Start-Component([string]$Name, [string]$Executable, [string[]]$Arguments, [string]$Directory) {
    $quoted = @($Arguments | ForEach-Object {
        if ($_ -match '"') { throw 'Embedded quotes are not supported in launcher arguments.' }
        '"' + $_ + '"'
    })
    $process = Start-Process -FilePath $Executable -ArgumentList $quoted -WorkingDirectory $Directory -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $logDirectory ($Name + '.out.log')) -RedirectStandardError (Join-Path $logDirectory ($Name + '.err.log'))
    $null = $process.Handle
    $script:records += @{name=$Name; id=$process.Id; ticks=$process.StartTime.ToUniversalTime().Ticks}
    Save-State
    Write-Host ('Started ' + $Name + ' (PID ' + $process.Id + ')')
    return $process
}

function Wait-Port([int]$Port, $Process, [int]$Timeout = 60) {
    $deadline = (Get-Date).AddSeconds($Timeout)
    while ((Get-Date) -lt $deadline) {
        if (Test-Path -LiteralPath $stopPath) { throw 'Stop requested.' }
        if ($Process.HasExited) { throw ('Process exited while waiting for port ' + $Port + '. See ' + $logDirectory) }
        $listener = Get-NetTCPConnection -State Listen -LocalPort $Port -ErrorAction SilentlyContinue
        if ($listener) { return }
        Start-Sleep -Milliseconds 500
    }
    throw ('Timed out waiting for port ' + $Port + '. See ' + $logDirectory)
}

$exitCode = 0
try {
    if (Test-Path -LiteralPath $statePath) {
        $previous = Get-Content -LiteralPath $statePath -Raw | ConvertFrom-Json
        foreach ($record in $previous.processes) {
            if (Test-RecordedProcess $record) { throw 'A previous component is still running. Run stop_local_visualization.cmd first.' }
        }
    }
    if (!$AdapterRoot) { $AdapterRoot = Join-Path (Split-Path $root -Parent) 'space_sim_UE_Adapter' }
    if (!$BackendPython) {
        $BackendPython = Join-Path $root '.venv\Scripts\python.exe'
        if (!(Test-Path -LiteralPath $BackendPython)) {
            $BackendPython = Join-Path ([Environment]::GetFolderPath('Desktop')) 'space_sim_workspace\space_sim_server\.venv\Scripts\python.exe'
        }
    }
    if (!$AuthDatabase) {
        $AuthDatabase = Join-Path $root 'data\auth.sqlite3'
        if (!(Test-Path -LiteralPath $AuthDatabase)) { $AuthDatabase = Join-Path $root 'data\visualization-test-auth.sqlite3' }
    }
    $projectRoot = Join-Path $AdapterRoot 'Unreal\BskUnrealRenderer'
    $editor = Join-Path $UnrealRoot 'Engine\Binaries\Win64\UnrealEditor.exe'
    $project = Join-Path $projectRoot 'BskUnrealRenderer.uproject'
    $catalog = Join-Path $projectRoot 'Saved\AssetImport\cubesat_so101.catalog.json'
    $scenario = Join-Path $projectRoot 'examples\scenario_spacecraft_arm_unreal.py'
    if ($Mode -eq 'Platform') {
        if (!$ModelRoot) { $ModelRoot = Join-Path $root 'model\SARM\platform' }
        $catalog = Join-Path $projectRoot 'Saved\AssetImport\sarm_platform.catalog.json'
        $scenario = Join-Path $root 'simulation\teleop_grasp_unreal.py'
        if (!(Test-Path -LiteralPath $PowerShellExe)) { throw ('PowerShell 7 is required: ' + $PowerShellExe) }
        $env:SPACE_SIM_PYTHON = $SimulationPython
        $condaRoot = Split-Path (Split-Path (Split-Path $SimulationPython -Parent) -Parent) -Parent
        $env:PATH = (Split-Path $SimulationPython -Parent) + ';' + (Join-Path $condaRoot 'Scripts') + ';' + $env:PATH
    }
    foreach ($required in @($BackendPython, $SimulationPython, $Node, $editor, $project, $catalog, $scenario,
        (Join-Path $root 'frontend\dist\index.html'), (Join-Path $root 'signalling\node_modules\ws\package.json'),
        (Join-Path $projectRoot 'Plugins\BskUnrealRuntime\Binaries\Win64\UnrealEditor-BskUnrealRuntime.dll'))) {
        if (!(Test-Path -LiteralPath $required)) { throw ('Missing prerequisite: ' + $required) }
    }
    $listeners = @(Get-NetTCPConnection -State Listen -ErrorAction SilentlyContinue | Where-Object LocalPort -in 5558,8080,8888,18000,18766,18767)
    if ($listeners.Count) {
        $description = ($listeners | ForEach-Object { 'port ' + $_.LocalPort + ' / PID ' + $_.OwningProcess }) -join ', '
        throw ('Ports already occupied: ' + $description + '. No existing processes were stopped.')
    }
    $env:PYTHONPATH = Join-Path $root 'backend'
    & $BackendPython -c 'import space_arm_platform.main, uvicorn'
    if ($LASTEXITCODE -ne 0) { throw 'Backend Python environment check failed.' }
    & $SimulationPython -c 'from Basilisk.simulation import mujoco'
    if ($LASTEXITCODE -ne 0) { throw 'Basilisk environment check failed.' }
    New-Item -ItemType Directory -Path $logDirectory -Force | Out-Null
    Remove-Item -LiteralPath $stopPath -Force -ErrorAction SilentlyContinue
    Save-State
    $randomBytes = New-Object byte[] 48
    $random = [Security.Cryptography.RandomNumberGenerator]::Create()
    $random.GetBytes($randomBytes)
    $env:PS_JWT_SECRET = [Convert]::ToBase64String($randomBytes)
    $env:SPACE_SIM_STREAM_JWT_SECRET = $env:PS_JWT_SECRET
    $env:PS_PLAYER_HOST = '127.0.0.1'
    $env:PS_STREAMER_HOST = '127.0.0.1'
    $env:PS_PLAYER_PORT = '8080'
    $env:PS_STREAMER_PORT = '8888'
    $env:PS_ALLOWED_ORIGINS = '["http://127.0.0.1:18000"]'
    if (!$env:SPACE_SIM_ADMIN_PASSWORD) {
        $random.GetBytes($randomBytes)
        $env:SPACE_SIM_ADMIN_PASSWORD = [Convert]::ToBase64String($randomBytes)
        if (!(Test-Path -LiteralPath $AuthDatabase)) {
            Write-Host ('First-run admin password (save it): ' + $env:SPACE_SIM_ADMIN_PASSWORD)
        }
    }
    $random.Dispose()
    $signal = Start-Component 'signalling' $Node @((Join-Path $root 'signalling\server.mjs')) $root
    Wait-Port 8888 $signal
    Wait-Port 8080 $signal
    $backendArguments = @('-m','space_arm_platform.main','--host','127.0.0.1','--port','18000','--simulation-port','18766','--capture-port','18767','--pixel-streaming-player-port','8080','--pixel-streaming-streamer-id','BskRenderer','--pixel-streaming-signalling-url','ws://127.0.0.1:8080','--auth-database',$AuthDatabase,'--allowed-origin','http://127.0.0.1:18000')
    if ($Mode -eq 'Platform') {
        $backendArguments += @('--runtime-adapter-root',$AdapterRoot,'--runtime-model-root',$ModelRoot,'--runtime-unreal-root',$UnrealRoot,'--runtime-powershell-exe',$PowerShellExe,'--runtime-render-port','5558','--runtime-pixel-streamer-port','8888','--runtime-preview-rate','30')
    }
    $backend = Start-Component 'backend' $BackendPython $backendArguments $root
    Wait-Port 18000 $backend
    $health = Invoke-RestMethod -Uri ($url + 'api/health') -TimeoutSec 10
    if (!$health.ok) { throw 'Backend health check failed.' }
    if ($Mode -eq 'Platform') {
        Write-Host ('Platform ready: ' + $url + ' Log in and click Generate and start scene. No standalone demo is running.')
        Write-Host 'The scene runs until stopped from the browser. Closing a scene does not close the website.'
        if (!$NoBrowser) { Start-Process $url }
        while (!(Test-Path -LiteralPath $stopPath)) {
            if ($signal.HasExited -or $backend.HasExited) { throw ('A platform service exited. See ' + $logDirectory) }
            Start-Sleep -Milliseconds 500
        }
        return
    }
    $renderer = Start-Component 'renderer' $editor @($project,'-game','-RenderOffscreen','-ForceRes','-ResX=1280','-ResY=720','-unattended','-BskListen=127.0.0.1','-BskPort=5558','-PixelStreamingConnectionURL=ws://127.0.0.1:8888','-PixelStreamingID=BskRenderer','-PixelStreamingEncoderCodec=H264','-PixelStreamingWebRTCFps=30') $projectRoot
    Wait-Port 5558 $renderer 180
    $env:PYTHONPATH = (Join-Path $AdapterRoot 'Adapters') + ';' + (Join-Path $AdapterRoot 'python')
    $env:PYTHONUNBUFFERED = '1'
    $simulation = Start-Component 'simulation' $SimulationPython @($scenario,'--model-root',(Join-Path $AdapterRoot 'test\model\spacecraft_and_arm'),'--catalog',$catalog,'--host','127.0.0.1','--port','5558','--duration',([string]$Duration),'--simulation-rate','1') $AdapterRoot
    Write-Host ('Services started. Open ' + $url + ' and log in with your existing account.')
    Write-Host ('Demo duration: ' + $Duration + ' seconds. The website stays available after the demo finishes. Ctrl+C or stop_local_visualization.cmd stops all components.')
    Write-Host 'This is a visual demo, not the full control runtime. Backend simulation-disconnected/runtime-unconfigured labels are expected.'
    Write-Host ('Video is NOT verified by port checks. Logs: ' + $logDirectory)
    if (!$NoBrowser) { Start-Process $url }
    $simulationFinished = $false
    while (!(Test-Path -LiteralPath $stopPath)) {
        foreach ($component in @($signal, $backend, $renderer)) {
            if ($component.HasExited) { throw ('A required service exited: PID ' + $component.Id + '. See ' + $logDirectory) }
        }
        if (!$simulationFinished -and $simulation.HasExited) {
            $simulationFinished = $true
            $simulation.WaitForExit()
            if ($simulation.ExitCode -eq 0) {
                Write-Host 'Demo finished successfully.'
            } else {
                Write-Warning ('Simulation stopped with exit code ' + $simulation.ExitCode + '. See ' + $logDirectory)
            }
            Write-Host 'Website and renderer remain available; physics is no longer advancing. Stop and start again to replay.'
        }
        Start-Sleep -Milliseconds 500
    }
} catch {
    Write-Host ('ERROR: ' + $_.Exception.Message) -ForegroundColor Red
    $exitCode = 1
} finally {
    if ($Mode -eq 'Platform' -and ($script:records | Where-Object name -eq 'backend')) {
        & (Join-Path $PSScriptRoot 'stop_scene_instance.ps1') -Quiet
    }
    for ($index = $script:records.Count - 1; $index -ge 0; $index--) {
        $record = $script:records[$index]
        Stop-RecordedProcessTree $record.id ([long]$record.ticks)
    }
    $lock.Dispose()
}
exit $exitCode
