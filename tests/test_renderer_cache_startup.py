"""Persistent, writable UE cache fallback without depending on a healthy Zen."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import pytest

ROOT=Path(__file__).resolve().parents[1]
ADAPTER=Path(os.environ.get('SPACE_SIM_RESET_ADAPTER',str(ROOT.parents[1]/'space_sim_UE_adapter/space_sim_UE_Adapter')))
UE_PROJECT=ADAPTER/'Unreal/BskUnrealRenderer'
PWSH=shutil.which('pwsh')
pytestmark=pytest.mark.skipif(not PWSH or not (UE_PROJECT/'scripts/start_renderer.ps1').is_file(),reason='matching local UE adapter/PowerShell required')


def test_project_keeps_zen_and_makes_installed_graph_local_cache_writable():
    config=(UE_PROJECT/'Config/DefaultEngine.ini').read_text(encoding='utf-8-sig')
    section=config.split('[InstalledDerivedDataBackendGraph]',1)[1].split('\n[',1)[0]
    assert 'DeleteOnly=false' in section
    assert 'Saved/DerivedDataCache' in section
    script=(UE_PROJECT/'scripts/start_renderer.ps1').read_text(encoding='utf-8-sig')
    assert '-DDC=InstalledDerivedDataBackendGraph' in script
    assert '-DDC-ForceMemoryCache' not in script
    assert '-DDC=InstalledNoZenLocalFallback' not in script


def test_launcher_checks_persistent_cache_passes_child_environment_and_hides_window(tmp_path):
    project=tmp_path/'adapter/Unreal/BskUnrealRenderer'
    scripts=project/'scripts';scripts.mkdir(parents=True)
    for name in ['start_renderer.ps1','common.ps1','python_runtime.ps1']:
        shutil.copy2(UE_PROJECT/'scripts'/name,scripts/name)
    ue=tmp_path/'UE';editor=ue/'Engine/Binaries/Win64/UnrealEditor.exe'
    editor.parent.mkdir(parents=True);editor.touch()
    wrapper=tmp_path/'launch.ps1'
    wrapper.write_text('''
param($Starter,$Engine)
$ErrorActionPreference='Stop'
$old=[Environment]::GetEnvironmentVariable('UE-LocalDataCachePath','Process')
function Start-Process {
    param($FilePath,$ArgumentList,[switch]$PassThru,$WindowStyle,$Environment)
    $global:launched=@{args=$ArgumentList;style=$WindowStyle;env=$Environment}
    return [pscustomobject]@{Id=4242}
}
& $Starter -UnrealRoot $Engine -Port 5568
$global:launched['parent_unchanged']=($old -eq [Environment]::GetEnvironmentVariable('UE-LocalDataCachePath','Process'))
'RESULT '+($global:launched|ConvertTo-Json -Compress -Depth 5)
''',encoding='utf-8')
    result=subprocess.run([PWSH,'-NoProfile','-File',str(wrapper),str(scripts/'start_renderer.ps1'),str(ue)],capture_output=True,text=True,encoding='utf-8',timeout=20)
    assert result.returncode==0,result.stdout+result.stderr
    report=json.loads(next(line[7:] for line in result.stdout.splitlines() if line.startswith('RESULT ')))
    assert report['style']=='Hidden' and report['parent_unchanged']
    assert report['env']['UE-LocalDataCachePath']==str(project/'Saved/DerivedDataCache')
    assert '-DDC=InstalledDerivedDataBackendGraph' in report['args']
    assert not list((project/'Saved/DerivedDataCache').glob('.write-probe-*'))
    assert (project/'Saved/BskRenderer.pid').read_text().strip()=='4242'
