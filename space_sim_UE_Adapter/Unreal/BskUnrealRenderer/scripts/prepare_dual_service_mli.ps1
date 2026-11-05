param([string]$UnrealRoot='')
. (Join-Path $PSScriptRoot 'common.ps1')
$UnrealRoot=Resolve-UnrealRoot $UnrealRoot
$buildScript=Join-Path $PSScriptRoot 'create_dual_service_mli.py'
$log=Join-Path $ProjectRoot 'Saved/AssetImport/dual_service_mli.log'
$arguments=@(('"'+$ProjectFile+'"'),('-ExecutePythonScript="'+$buildScript+'"'),'-unattended','-nop4','-nosplash','-RenderOffscreen',('-abslog="'+$log+'"')) + $BskDerivedDataCacheArguments
$process=Start-Process (Join-Path $UnrealRoot 'Engine/Binaries/Win64/UnrealEditor-Cmd.exe') -ArgumentList $arguments -WindowStyle Hidden -PassThru
$process.WaitForExit()
if ($process.ExitCode -ne 0 -or (Get-Content $log -Raw) -notmatch 'DUAL_SERVICE_MLI_OK' -or (Get-Content $log -Raw) -match 'LogPython: Error:') { throw "MLI preparation failed: $log" }
