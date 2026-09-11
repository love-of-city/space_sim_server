"""Scene supervision/cleanup tests with fake handles plus actual isolated children."""
from __future__ import annotations
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import pytest

ROOT=Path(__file__).resolve().parents[1]
PWSH=shutil.which('pwsh') or r'C:\Program Files\PowerShell\7\pwsh.exe'
pytestmark=pytest.mark.skipif(os.name!='nt' or not Path(PWSH).is_file(),reason='Windows PowerShell 7 required')
HELPERS=ROOT/'scripts/scene_process_helpers.ps1'

def invoke(path,*args):
    return subprocess.run([PWSH,'-NoProfile','-File',str(path),*map(str,args)],capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=25)

@pytest.mark.parametrize('component', ['renderer','simulation','stopped'])
def test_watches_renderer_and_simulation_and_honors_intentional_stop(tmp_path,component):
    script=tmp_path/'watch.ps1'
    script.write_text('''
param($Helpers,$Component)
$ErrorActionPreference='Stop'
. $Helpers
$renderer=[pscustomobject]@{HasExited=$false;ExitCode=3}
$simulation=[pscustomobject]@{HasExited=$false;ExitCode=7}
$script:stopped=$false
function Start-Sleep { param($Milliseconds)
    switch($Component) {
        'renderer' {$renderer.HasExited=$true}
        'simulation' {$simulation.HasExited=$true}
        'stopped' {$script:stopped=$true; $renderer.HasExited=$true}
    }
}
Wait-SceneRuntimeExit $renderer $simulation {$script:stopped} | ConvertTo-Json
''',encoding='utf-8')
    result=invoke(script,HELPERS,component)
    assert result.returncode==0,result.stdout+result.stderr
    out=json.loads(result.stdout)
    assert out['component']==component
    assert out['code']=={'renderer':3,'simulation':7,'stopped':0}[component]

def test_cleanup_does_not_touch_reused_ids_or_older_unrelated_children(tmp_path):
    script=tmp_path/'identities.ps1'
    script.write_text('''
param($Helpers)
$ErrorActionPreference='Stop'
. $Helpers
function Get-Process { param($Id,$ErrorAction)
    $ticks=@{1001=20L;1002=25L;1003=10L;1004=50L}[[int]$Id]
    [pscustomobject]@{StartTime=[DateTime]::new($ticks,[DateTimeKind]::Utc)}
}
function Get-CimInstance { param($ClassName,$Filter,$ErrorAction)
    if($Filter -eq 'ParentProcessId=1001') { return @([pscustomobject]@{ProcessId=1002},[pscustomobject]@{ProcessId=1003}) }
    return @()
}
function Stop-Process {param($Id,[switch]$Force,$ErrorAction) "STOP-$Id"}
Stop-RecordedProcessTree 1004 1
Stop-RecordedProcessTree 1001 20
''',encoding='utf-8')
    result=invoke(script,HELPERS)
    assert result.returncode==0,result.stdout+result.stderr
    assert result.stdout.split()==['STOP-1002','STOP-1001']

def test_real_child_exit_is_detected_and_other_child_is_cleaned(tmp_path):
    short=tmp_path/'renderer.py'; short.write_text('import time; time.sleep(0.5); raise SystemExit(3)',encoding='utf-8')
    long=tmp_path/'simulation.py';long.write_text('import time; time.sleep(60)',encoding='utf-8')
    script=tmp_path/'real.ps1'
    script.write_text('''
param($Helpers,$Python,$Short,$Long)
$ErrorActionPreference='Stop'
. $Helpers
$renderer=$null;$simulation=$null
try {
    $renderer=Start-Process -FilePath $Python -ArgumentList ('"'+$Short+'"') -PassThru -WindowStyle Hidden
    $simulation=Start-Process -FilePath $Python -ArgumentList ('"'+$Long+'"') -PassThru -WindowStyle Hidden
    $outcome=Wait-SceneRuntimeExit $renderer $simulation {$false}
    if($outcome.component -ne 'renderer' -or $outcome.code -ne 3){throw 'Renderer failure was not detected.'}
    Stop-RecordedProcessTree $simulation.Id $simulation.StartTime.ToUniversalTime().Ticks
    if(!$simulation.WaitForExit(5000)){throw 'Simulation process survived cleanup.'}
    'REAL-EXIT-DETECTED-AND-CLEANED'
} finally {
    foreach($process in @($simulation,$renderer)) {
        if($process -and !$process.HasExited){Stop-RecordedProcessTree $process.Id $process.StartTime.ToUniversalTime().Ticks}
    }
}
''',encoding='utf-8')
    result=invoke(script,HELPERS,sys.executable,short,long)
    assert result.returncode==0,result.stdout+result.stderr
    assert 'REAL-EXIT-DETECTED-AND-CLEANED' in result.stdout

def test_stop_marks_intent_before_ending_child_processes():
    text=(ROOT/'scripts/stop_scene_instance.ps1').read_text(encoding='utf-8-sig')
    assert text.index("$state.phase = 'stopped'") < text.index('Stop-RecordedProcessTree ([int]$state.simulation_pid)')
    start=(ROOT/'scripts/start_scene_instance.ps1').read_text(encoding='utf-8-sig')
    assert 'Wait-SceneRuntimeExit $rendererProcess $simulation' in start
    assert 'Stop-RecordedProcessTree $simulation.Id $runtimeState.simulation_start' in start
    assert 'UE renderer exited unexpectedly' in start
