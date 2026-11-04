"""Web/API fixed layouts must reach the actual collector argv unchanged."""
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from fastapi.testclient import TestClient
import pytest

from space_arm_platform.app import PlatformConfig, create_app
from space_arm_platform.collection_inputs import fixed_layout_input, validate_fixed_snapshot
from space_arm_platform.collection_jobs import CollectionCreate, CollectionManager, CollectionTask, TASKS, TERMINAL
from space_arm_platform.collection_layouts import digest
from space_arm_platform.recorder import EpisodeRecorder
from test_collection_layouts import layout_pool  # Existing complete manifest/asset fixture.
from test_socket_colors import seven_hole_model  # Existing seven-hole physical binding fixture.

ROOT = Path(__file__).resolve().parents[1]
OWNER = dict(user_id='owner', username='operator', role='operator')


@pytest.fixture
def fixed_inputs(layout_pool):
    entry = json.loads(layout_pool.read_text())['layouts'][0]
    model, catalog = Path(entry['model']), Path(entry['catalog'])
    ET.ElementTree(seven_hole_model()).write(model)
    manifest = model.with_suffix('.layout.json')
    record = json.loads(manifest.read_text())
    record['model_sha256'] = digest(model)
    manifest.write_text(json.dumps(record))
    model.with_name('attitude_control.json').write_text('{}')
    return model, catalog


@pytest.fixture
def api(tmp_path, monkeypatch, fixed_inputs):
    model, catalog = fixed_inputs
    monkeypatch.setenv('SPACE_SIM_COLLECTION_FIXED_MODEL', str(model))
    monkeypatch.setenv('SPACE_SIM_COLLECTION_FIXED_CATALOG', str(catalog))
    monkeypatch.delenv('SPACE_SIM_COLLECTION_LAYOUT_POOL', raising=False)
    monkeypatch.setattr(EpisodeRecorder, 'prepare', lambda self: None)
    project = tmp_path / 'project'
    project.mkdir()
    (project / 'collector.py').write_text('''import argparse,json,sys
from pathlib import Path
p=argparse.ArgumentParser()
for key in ('output','model','catalog','scene','count','seed','layout-pool'):p.add_argument('--'+key)
a,_=p.parse_known_args()
out=Path(a.output);out.mkdir()
(out/'received.json').write_text(json.dumps(dict(argv=sys.argv[1:],model=a.model,catalog=a.catalog,seed=a.seed,scene=json.loads(Path(a.scene).read_text()))))
(out/'summary.json').write_text(json.dumps(dict(status='complete',accepted_episodes=[str(out/('episode-'+str(i))) for i in range(int(a.count))])))
''')
    for name in ('guide_plug_insert', 'aviation_bolt_insert', 'drawer_cycle'):
        monkeypatch.setitem(TASKS, name, CollectionTask('test', 'collector.py', 'sarm-task-box-20261012'))
    editor = tmp_path / 'editor'
    editor.touch()
    monkeypatch.setattr('space_arm_platform.collection_jobs.unreal_editor', lambda _: editor)
    app = create_app(PlatformConfig(project_root=ROOT, data_root=tmp_path/'episodes',
                                   simulation_port=0, capture_port=0, auth_database=tmp_path/'auth.sqlite3'))
    app.state.collections.project_root = project
    with TestClient(app) as client:
        client.post('/api/auth/login', json=dict(username='admin', password='ChangeMe123!')).raise_for_status()
        yield client, app.state.collections


def wait_job(manager, key):
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        job = manager.jobs[key]
        if job['status'] in TERMINAL:
            return job
        time.sleep(.03)
    pytest.fail('test collector did not finish')


@pytest.mark.parametrize('task', ['guide_plug_insert', 'aviation_bolt_insert'])
def test_api_fixed_layout_reaches_real_child_argv_for_every_worker(api, fixed_inputs, task):
    client, manager = api
    model, catalog = fixed_inputs
    capabilities = client.get('/api/collections/catalog').json()
    assert capabilities['fixed_layout']['available']
    assert next(t for t in capabilities['tasks'] if t['id'] == task)['requires_color_layout']
    response = client.post('/api/collections', json=dict(task=task, workers=2, count=2, max_attempts=2, seed=41))
    assert response.status_code == 201, response.text
    job = wait_job(manager, response.json()['id'])
    assert job['status'] == 'complete', job
    folder = manager.root / job['id']
    saved = json.loads((folder/'fixed-layout.json').read_text())
    assert saved == job['fixed_layout'] == fixed_layout_input(model, catalog)[0]
    assert job['config']['layout_randomization'] is False
    for index in range(2):
        received = json.loads((folder/f'worker-{index}'/'received.json').read_text())
        assert received['model'] == str(model) and received['catalog'] == str(catalog)
        assert '--layout-pool' not in received['argv']
        assert int(received['seed']) == 41 + index
        assert received['scene']['collection_fixed_layout'] == saved
        assert received['scene']['task_box_layout']['model_sha256'] == saved['model_sha256']
        assert received['scene']['collection_layout_randomization'] is False


def test_api_missing_fixed_inputs_rejects_color_task_before_any_worker(api):
    client, manager = api
    manager.fixed_model = manager.fixed_catalog = None
    before = set(manager.root.iterdir())
    capability = client.get('/api/collections/catalog').json()['fixed_layout']
    assert capability['available'] is False and capability['configured'] is False
    response = client.post('/api/collections', json=dict(task='guide_plug_insert', workers=1, count=1, max_attempts=1))
    assert response.status_code == 409
    assert '可区分孔位的固定操作台' in response.json()['detail']
    assert not manager.jobs and set(manager.root.iterdir()) == before


def test_partial_fixed_config_reports_unavailable_and_rejects_before_launch(api):
    client, manager = api
    manager.fixed_catalog = None
    capability = client.get('/api/collections/catalog').json()['fixed_layout']
    assert not capability['available'] and capability['configured']
    response = client.post('/api/collections', json=dict(workers=1, count=1, max_attempts=1))
    assert response.status_code == 409 and '同时配置' in response.json()['detail']
    assert not manager.jobs


def test_fixed_snapshot_rejects_inputs_changed_after_job_creation(api, fixed_inputs, monkeypatch):
    _, manager = api
    run = manager._run
    monkeypatch.setattr(manager, '_run', lambda _: None)
    job = manager.start(CollectionCreate(task='guide_plug_insert', workers=1, count=1, max_attempts=1), OWNER, {})
    catalog = fixed_inputs[1]
    document = json.loads(catalog.read_text())
    document['note'] = 'changed after snapshot'
    catalog.write_text(json.dumps(document))
    run(manager.jobs[job['id']])
    result = manager.jobs[job['id']]
    assert result['status'] == 'failed' and '发生变化' in result['error']
    assert not (manager.root/job['id']/'worker-0').exists()


def test_saved_layout_mismatch_is_not_replaced_by_default_fixed_layout(api):
    _, manager = api
    with pytest.raises(ValueError, match='已保存场景'):
        manager.start(CollectionCreate(workers=1, count=1, max_attempts=1), OWNER,
                      {'task_box_layout': {'model_sha256': 'another-model'}})
    assert not manager.jobs


def test_fixed_validation_preserves_files_and_rejects_monolithic_model(fixed_inputs):
    model, catalog = fixed_inputs
    before = model.read_bytes()
    snapshot, _ = fixed_layout_input(model, catalog)
    validate_fixed_snapshot(snapshot)
    assert model.read_bytes() == before
    model.write_text('<mujoco><asset/><worldbody><body name="asm_board1"/></worldbody></mujoco>')
    manifest = model.with_suffix('.layout.json')
    record = json.loads(manifest.read_text())
    record['model_sha256'] = digest(model)
    manifest.write_text(json.dumps(record))
    with pytest.raises(ValueError, match='binding|missing|requires|Missing'):
        fixed_layout_input(model, catalog)


def test_explicit_random_layout_keeps_pool_semantics_with_fixed_config_present(layout_pool, tmp_path, monkeypatch):
    manager = CollectionManager(tmp_path, tmp_path/'episodes', tmp_path, tmp_path,
                                layout_pool=layout_pool, fixed_model='unused-fixed.xml')
    (tmp_path/'collection').mkdir()
    (tmp_path/'collection/collect_scripted_tasks.py').touch()
    catalog = tmp_path/'Unreal/BskUnrealRenderer/Saved/AssetImport/sarm_20261012.catalog.json'
    catalog.parent.mkdir(parents=True)
    catalog.touch()
    editor = tmp_path/'editor'
    editor.touch()
    monkeypatch.setattr('space_arm_platform.collection_jobs.unreal_editor', lambda _: editor)
    monkeypatch.setattr(manager, '_run', lambda _: None)
    try:
        job = manager.start(CollectionCreate(task='guide_plug_insert', workers=1, count=1,
                            max_attempts=1, layout_randomization=True), OWNER, {})
        assert job['config']['layout_randomization'] is True and job['fixed_layout'] is None
        folder = manager.root/job['id']
        assert json.loads((folder/'layout-pool.json').read_text()) == json.loads(layout_pool.read_text())
        assert not (folder/'fixed-layout.json').exists()
    finally:
        manager.close()
