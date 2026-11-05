param([string]$Python='', [Parameter(Mandatory)][string]$UnrealRoot)
$ErrorActionPreference='Stop'
if ($PSVersionTable.PSVersion.Major -lt 7) { throw 'PowerShell 7 required' }
$projectRoot=Split-Path -Parent $PSScriptRoot
$adapterRoot=Join-Path (Split-Path -Parent $projectRoot) 'space_sim_UE_Adapter'
$powerShellExe=(Get-Process -Id $PID).Path
. (Join-Path $PSScriptRoot 'python_runtime.ps1')
$Python=Resolve-SpaceSimPython -RepositoryRoot $projectRoot -RequestedPython $Python -RequiredModules @('numpy','Basilisk.simulation.mujoco')
$env:SPACE_SIM_PYTHON=$Python
$log=Join-Path $projectRoot 'logs'; New-Item -ItemType Directory -Force $log | Out-Null
& (Join-Path $PSScriptRoot 'start_pixel_streaming.ps1') -UnrealRoot $UnrealRoot -StreamerPort 18888 -PlayerPort 18081 -SfuPort 18889
if($LASTEXITCODE -ne 0){throw 'Signalling startup failed'}
$ids='teleop/camera/spacecraft_overview,teleop/camera/right_sarm_wrist_cam,teleop/camera/left_sarm_wrist_cam'
$streamers='BskDualService__teleop_camera_spacecraft_overview=Global;BskDualService__teleop_camera_right_sarm_wrist_cam=LeftWrist;BskDualService__teleop_camera_left_sarm_wrist_cam=RightWrist'
$arguments=@('-NoProfile','-File',(Join-Path $PSScriptRoot 'run_backend.ps1'),'-Python',$Python,
    '-ApiPort','18080','-ControlPort','18766','-CapturePort','18767','-RendezvousPort','18768',
    '-PixelStreamingPlayerPort','18081','-PixelStreamingId','BskDualService',
    '-PixelStreamingCameraStreamers',$streamers,'-PixelStreamingCameraIds',$ids,
    '-AdapterRoot',$adapterRoot,'-ModelRoot',(Join-Path $projectRoot 'model\SARM\platform'),
    '-UnrealRoot',$UnrealRoot,'-PowerShellExe',$powerShellExe,'-RenderPort','15558','-PixelStreamerPort','18888',
    '-PreviewRate','30','-CaptureRate','5','-DefaultDatasetCapture:$false')
$arguments=@($arguments | ForEach-Object { if ($_ -match '\s') { '"' + $_ + '"' } else { $_ } })
$process=Start-Process -FilePath $powerShellExe -ArgumentList $arguments -WorkingDirectory $projectRoot -WindowStyle Hidden -PassThru `
    -RedirectStandardOutput (Join-Path $log 'dual-service-backend.out.log') -RedirectStandardError (Join-Path $log 'dual-service-backend.err.log')
@{pid=$process.Id;start_ticks=$process.StartTime.ToUniversalTime().Ticks;url='http://127.0.0.1:18080'} | ConvertTo-Json | Set-Content (Join-Path $projectRoot 'run\dual_service_platform.json') -Encoding utf8
Write-Output 'Dual service platform starting at http://127.0.0.1:18080'
