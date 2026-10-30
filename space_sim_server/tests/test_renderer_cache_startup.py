"""Persistent, writable UE cache fallback without depending on a healthy Zen."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import pytest

ROOT=Path(__file__).resolve().parents[1]
ADAPTER=Path(os.environ.get('SPACE_SIM_RESET_ADAPTER',str(ROOT.parent/'space_sim_UE_Adapter')))
UE_PROJECT=ADAPTER/'Unreal/BskUnrealRenderer'
PWSH=shutil.which('pwsh')
pytestmark=pytest.mark.skipif(not PWSH or not (UE_PROJECT/'scripts/start_renderer.ps1').is_file(),reason='matching local UE adapter/PowerShell required')


@pytest.fixture
def cache_root():
    # pytest's long descriptive directory can itself exceed UE's 119-char limit.
    with tempfile.TemporaryDirectory(prefix='bsk-cache-') as directory:
        yield Path(directory).resolve()


def test_project_keeps_zen_and_makes_installed_graph_local_cache_writable():
    config=(UE_PROJECT/'Config/DefaultEngine.ini').read_text(encoding='utf-8-sig')
    section=config.split('[InstalledDerivedDataBackendGraph]',1)[1].split('\n[',1)[0]
    assert 'DeleteOnly=false' in section
    assert 'Saved/DerivedDataCache' in section
    script=(UE_PROJECT/'scripts/start_renderer.ps1').read_text(encoding='utf-8-sig')
    assert '-DDC=InstalledDerivedDataBackendGraph' in script
    assert '-DDC-ForceMemoryCache' not in script
    assert '-DDC=InstalledNoZenLocalFallback' not in script


@pytest.mark.parametrize('cache_mode', ['default', 'inherited', 'explicit', 'long-inherited', 'long-explicit'])
def test_launcher_checks_persistent_cache_passes_child_environment_and_hides_window(tmp_path, cache_root, cache_mode, monkeypatch):
    monkeypatch.delenv('SSH_CONNECTION', raising=False)
    project=tmp_path/'adapter/Unreal/BskUnrealRenderer'
    scripts=project/'scripts';scripts.mkdir(parents=True)
    for name in ['start_renderer.ps1','common.ps1','python_runtime.ps1']:
        shutil.copy2(UE_PROJECT/'scripts'/name,scripts/name)
    ue=tmp_path/'UE';editor=ue/'Engine/Binaries/Win64/UnrealEditor.exe'
    editor.parent.mkdir(parents=True);editor.touch()
    wrapper=tmp_path/'launch.ps1'
    wrapper.write_text('''
param($Starter,$Engine,$Cache)
$ErrorActionPreference='Stop'
$old=[Environment]::GetEnvironmentVariable('UE-LocalDataCachePath','Process')
function Start-Process {
    param($FilePath,$ArgumentList,[switch]$PassThru,$WindowStyle,$Environment)
    $global:launched=@{args=$ArgumentList;style=$WindowStyle;env=$Environment}
    return [pscustomobject]@{Id=4242}
}
$options=@{}
if ($Cache) { $options.DerivedDataCachePath=$Cache }
& $Starter -UnrealRoot $Engine -Port 5568 @options
$global:launched['parent_unchanged']=($old -eq [Environment]::GetEnvironmentVariable('UE-LocalDataCachePath','Process'))
'RESULT '+($global:launched|ConvertTo-Json -Compress -Depth 5)
''',encoding='utf-8')
    # Windows normalizes os.environ keys to uppercase; a plain copied dict
    # otherwise retains an inherited mixed-case setting during the default case.
    environment={k:v for k,v in os.environ.items() if k.casefold()!='ue-localdatacachepath'}
    arguments=[PWSH,'-NoProfile','-File',str(wrapper),str(scripts/'start_renderer.ps1'),str(ue)]
    expected=project/'Saved/DerivedDataCache'
    if cache_mode != 'default':
        expected=cache_root/'warm cache'
        environment['UE-LocalDataCachePath']=str(expected)
    if cache_mode == 'explicit':
        expected=cache_root/'explicit cache'
        environment['UE-LocalDataCachePath']=str(cache_root/('ignored-long-cache-'*10))
        arguments.append(str(expected))
    if cache_mode.startswith('long-'):
        expected=cache_root/('too-long-'*16)
        if cache_mode == 'long-explicit':
            arguments.append(str(expected))
        else:
            environment['UE-LocalDataCachePath']=str(expected)
    result=subprocess.run(arguments,env=environment,capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=20)
    if cache_mode.startswith('long-'):
        assert result.returncode != 0 and 'exceeds 119 characters' in result.stderr
        assert not expected.exists()
        assert not (project/'Saved/BskRenderer.pid').exists()
        return
    assert result.returncode==0,result.stdout+result.stderr
    report=json.loads(next(line[7:] for line in result.stdout.splitlines() if line.startswith('RESULT ')))
    assert report['style']=='Hidden' and report['parent_unchanged']
    cache = Path(report['env']['UE-LocalDataCachePath'])
    assert cache.is_dir() and len(str(cache)) <= 119
    if cache_mode == 'default':
        assert cache.parent.name == 'BskDDC'
    else:
        assert cache == expected
    assert '-DDC=InstalledDerivedDataBackendGraph' in report['args']
    assert not list(cache.glob('.write-probe-*'))
    assert (project/'Saved/BskRenderer.pid').read_text().strip()=='4242'


@pytest.mark.parametrize('windows_error', [0, 9])
def test_ssh_renderer_launch_is_detached_and_preserves_cache_arguments(tmp_path, cache_root, windows_error):
    project = tmp_path / 'adapter with spaces/Unreal/BskUnrealRenderer'
    scripts = project / 'scripts'
    scripts.mkdir(parents=True)
    for name in ['start_renderer.ps1', 'common.ps1', 'python_runtime.ps1']:
        shutil.copy2(UE_PROJECT / 'scripts' / name, scripts / name)
    engine = tmp_path / 'Unreal Engine'
    editor = engine / 'Engine/Binaries/Win64/UnrealEditor.exe'
    editor.parent.mkdir(parents=True)
    editor.touch()
    cache = cache_root / 'persistent cache'
    wrapper = tmp_path / 'launch.ps1'
    wrapper.write_text('''
param($Starter,$Engine,$Cache,[int]$WindowsError)
$ErrorActionPreference='Stop'
$env:SSH_CONNECTION='test connection'
function Start-Process { throw 'SSH must not start an attached child' }
function New-CimInstance { param($ClassName,[switch]$ClientOnly,$Property) return $Property }
function Invoke-CimMethod {
    param($ClassName,$MethodName,$Arguments)
    $Arguments | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $PSScriptRoot 'request.json')
    return [pscustomobject]@{ReturnValue=$WindowsError;ProcessId=6789}
}
& $Starter -UnrealRoot $Engine -Port 25558 -DerivedDataCachePath $Cache -PixelStreamingURL ws://127.0.0.1:28888
''', encoding='utf-8')
    result = subprocess.run([PWSH, '-NoProfile', '-File', str(wrapper),
        str(scripts / 'start_renderer.ps1'), str(engine), str(cache), str(windows_error)],
        capture_output=True, text=True, encoding='utf-8', errors='replace', timeout=20)
    pid_file = project / 'Saved/BskRenderer.pid'
    if windows_error:
        assert result.returncode != 0 and not pid_file.exists()
        assert 'Windows error 9' in result.stderr
        return
    assert result.returncode == 0, result.stderr
    request = json.loads((tmp_path / 'request.json').read_text(encoding='utf-8-sig'))
    assert request['ProcessStartupInformation']['ShowWindow'] == 0
    assert request['CurrentDirectory'] == str(project)
    assert f'"{editor}"' in request['CommandLine']
    assert f'"{project / "BskUnrealRenderer.uproject"}"' in request['CommandLine']
    assert f'"-LocalDataCachePath={cache}"' in request['CommandLine']
    assert '-BskPort=25558' in request['CommandLine']
    assert pid_file.read_text().strip() == '6789'
