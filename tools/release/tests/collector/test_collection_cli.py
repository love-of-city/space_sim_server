from pathlib import Path
import importlib.util,json
import subprocess
import sys
from types import SimpleNamespace
import pytest

ROOT=Path(__file__).resolve().parents[2]/'collector'
spec=importlib.util.spec_from_file_location('cli',ROOT/'app/collection_cli.py')
cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)

def test_collector_product_declares_its_supported_subset(capsys):
    from space_arm_platform.task_catalog import COLLECTOR_TASK_CHOICES, TASK_CHOICES, TASK_LABELS

    tasks=cli.collector_tasks()
    assert tuple(tasks)==COLLECTOR_TASK_CHOICES
    assert set(tasks)=={'drawer_cycle','aviation_plug_insert','guide_plug_insert',
                        'cylindrical_plug_insert','aviation_bolt_insert','locking_rod_cycle'}
    assert set(tasks)<set(TASK_CHOICES)
    assert tasks=={name:TASK_LABELS[name] for name in COLLECTOR_TASK_CHOICES}
    assert cli.main(['tasks'])==0
    assert json.loads(capsys.readouterr().out)==tasks

def test_release_module_can_be_imported_without_server_or_site_packages():
    result=subprocess.run([sys.executable,'-I','-S','-c',
        'import importlib.util,sys; '
        'spec=importlib.util.spec_from_file_location("collector_cli",sys.argv[1]); '
        'module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)',
        str(ROOT/'app/collection_cli.py')],text=True,capture_output=True)
    assert result.returncode==0,result.stderr

def test_check_imports_real_recording_validator_before_external_bundle_probe(monkeypatch):
    class BundleProbeReached(Exception):
        pass

    # Do not replace the validator with a fake: this must expose stale imports.
    monkeypatch.delitem(sys.modules,'collection.recording_validation',raising=False)
    def probe(command,**kwargs):
        validator=sys.modules.get('collection.recording_validation')
        assert validator is not None,'check must load the real validator before probing the runtime'
        assert Path(validator.__file__).name=='recording_validation.py'
        assert callable(validator.validate_recording)
        assert command==[sys.executable,str(cli.BASE/'app/check.py')]
        assert kwargs['check'] is True
        raise BundleProbeReached

    monkeypatch.setattr(cli.subprocess,'run',probe)
    with pytest.raises(BundleProbeReached):
        cli.main(['check'])

def test_validate_calls_real_recording_validator(tmp_path):
    missing=tmp_path/'unrecorded-episode'
    with pytest.raises(FileNotFoundError) as error:
        cli.main(['validate','--episode',str(missing),'--task','drawer_cycle'])
    assert Path(error.value.filename)==missing/'metadata.json'

@pytest.mark.parametrize('task',['drawer_open','drawer_close','locking_rod_fold','locking_rod_restore'])
def test_other_repository_tasks_are_outside_collector_product(task,tmp_path):
    from space_arm_platform.task_catalog import TASK_CHOICES

    assert task in TASK_CHOICES
    with pytest.raises(SystemExit) as error:
        cli.main(['validate','--episode',str(tmp_path),'--task',task])
    assert error.value.code==2

@pytest.mark.parametrize('arguments',[
 ['run','--task','drawer_cycle','--output','unused'],
 ['run','--task','drawer_cycle','--output','unused','--model','model.xml'],
 ['run','--task','drawer_cycle','--output','unused','--layout-pool','pool.json','--model','model.xml','--catalog','catalog.json'],
 ['run','--task','drawer_cycle','--output','unused','--layout-pool','pool.json','--range-deg','nan'],
 ['run','--task','drawer_cycle','--output','unused','--layout-pool','pool.json','--count','3','--max-attempts','2'],
])
def test_invalid_requests_fail_before_launch(arguments,monkeypatch):
    def forbidden(*args):raise AssertionError('must not launch')
    monkeypatch.setattr(cli,'run',forbidden)
    with pytest.raises(SystemExit) as e:cli.main(arguments)
    assert e.value.code==2

def test_random_pool_and_capture_settings_forwarded(tmp_path):
    a=SimpleNamespace(task='drawer_cycle',count=20,max_attempts=80,seed=123,range_deg=30,
        randomize_overview_camera=True,randomize_orbit_phase=False,layout_pool=tmp_path/'pool.json',
        model=None,catalog=None,gpu_index=0,port_base=30000)
    cmd=cli.collection_command(a,tmp_path,tmp_path/'scene.json')
    for flag in ['--layout-pool','--offline','--reuse-renderer','--reuse-dynamics','--randomize-overview-camera','--no-randomize-orbit-phase']:assert flag in cmd
    assert '--model' not in cmd
    assert cmd[cmd.index('--overview-camera-sampling')+1]=='presets'
    assert cmd[cmd.index('--range-deg')+1]=='30'
    assert [int(cmd[cmd.index('--'+name+'-port')+1]) for name in ['control','capture','render','signalling']]==[30000,30001,30002,30003]

def test_saved_diagnostic_cannot_silently_disable_requested_randomization(tmp_path):
    scene=tmp_path/'input.json';scene.write_text(json.dumps({'initialization_enabled':False}))
    a=SimpleNamespace(scene=scene,range_deg=30)
    with pytest.raises(ValueError,match='disables joint'):cli.make_scene(a,tmp_path)

def test_default_scene_uses_enabled_initialization_and_current_camera(tmp_path):
    a=SimpleNamespace(scene=None,range_deg=30,seed=123,randomize_overview_camera=True,randomize_orbit_phase=False)
    doc=json.loads(cli.make_scene(a,tmp_path).read_text())
    assert isinstance(doc.get('created_by',{}),dict)
    assert doc['initialization_enabled'] is True
    assert doc['randomize_overview_camera'] is True
    assert doc['overview_camera_sampling']=='presets'
    assert len(doc['randomization']['arm_joint_position_rad'])==8
