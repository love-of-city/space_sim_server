from pathlib import Path
import importlib.util,json
from types import SimpleNamespace
import pytest

ROOT=Path(__file__).resolve().parents[2]/'collector'
spec=importlib.util.spec_from_file_location('cli',ROOT/'app/collection_cli.py')
cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)

def test_all_six_tasks_are_production_names():
    from simulation.scripted_tasks.plug_profiles import PLUG_PROFILES
    from simulation.scripted_tasks.threaded_tasks import THREADED_TASKS
    assert set(cli.TASKS)=={'drawer_cycle',*PLUG_PROFILES,*THREADED_TASKS}

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
