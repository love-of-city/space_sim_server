import json
from copy import deepcopy

import pytest

from space_arm_platform.imported_scene_library import digest, model_for_instance
from space_arm_platform.library_layouts import select_layout
from space_arm_platform.models import SceneInstanceCreate
from space_arm_platform.scene_runtime import SceneRuntimeManager
from test_imported_scene_library import install_fixture


def pool_fixture(tmp_path):
    entry, initial, _ = install_fixture(tmp_path)
    variants = []
    for i in range(2):
        source = json.loads((tmp_path/'source.json').read_text())
        model = tmp_path/f'model-{i}.xml'
        model.write_text(f'<mujoco model="layout-{i}"/>')
        source['model'] = str(model)
        source['dependencies'] = [dict(path=str(model), sha256=digest(model))]
        source['task_box_layout'] = dict(model_sha256=digest(model), target={'layout_seed':i}, service={'layout_seed':1-i})
        path = tmp_path/f'source-{i}.json'
        path.write_text(json.dumps(source))
        selected = deepcopy(entry)
        selected.update(source_instance=str(path), source_instance_sha256=digest(path))
        variants.append(dict(id=f'layout-{i}', model_sha256=digest(model), entries={entry['id']:selected}))
    pool = tmp_path/'pool.json'
    pool.write_text(json.dumps(dict(schema='dual-taskbox-layout-pool/1',
        base_sources={entry['id']:entry['source_instance_sha256']}, variants=variants)))
    entry.update(task_box_layout_pool=str(pool), task_box_layout_pool_sha256=digest(pool))
    (tmp_path/'run/imported_scene_library.json').write_text(json.dumps(dict(schema='space-arm-platform-library/1',entries=[entry])))
    return entry, initial, pool


def test_reproducible_selection_preserves_dual_initial_state(tmp_path):
    entry, initial, _ = pool_fixture(tmp_path)
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    req = SceneInstanceCreate(template_id=entry['id'], randomize_task_box_layout=True, seed=101, dataset_capture=False)
    a, b = manager.create_instance(req), manager.create_instance(req)
    assert a['seed'] == 101
    assert a['task_box_layout_randomization'] == b['task_box_layout_randomization']
    assert a['randomization'] == initial['randomization']
    assert a['preheld_capture'] == initial['preheld_capture']
    assert a['randomize_overview_camera'] is False
    assert model_for_instance(a) == model_for_instance(b)
    assert len({select_layout(entry, seed)[1]['layout_id'] for seed in range(12)}) == 2


def test_fixed_default_does_not_select_pool(tmp_path):
    entry, initial, _ = pool_fixture(tmp_path)
    a = SceneRuntimeManager(None, project_root=tmp_path).create_instance(SceneInstanceCreate(template_id=entry['id'], dataset_capture=False))
    assert 'task_box_layout_randomization' not in a
    assert a['randomization'] == initial['randomization']


def test_changed_pool_or_model_fails_closed(tmp_path):
    entry, _, pool = pool_fixture(tmp_path)
    selected, _, _ = select_layout(entry, 101)
    source = json.loads(open(selected['source_instance']).read())
    from pathlib import Path
    Path(source['model']).write_text('<changed/>')
    with pytest.raises(ValueError, match='dependency changed'):
        select_layout(entry, 101)
    pool.write_text('{}')
    with pytest.raises(ValueError, match='pool changed'):
        select_layout(entry, 101)


def test_candidate_cannot_be_recorded_or_silently_fall_back(tmp_path):
    entry, _, _ = pool_fixture(tmp_path)
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    with pytest.raises(ValueError, match='关闭数据采集'):
        manager.create_instance(SceneInstanceCreate(template_id=entry['id'], randomize_task_box_layout=True))
    entry.pop('task_box_layout_pool')
    with pytest.raises(ValueError, match='没有已准备'):
        select_layout(entry, 4)
