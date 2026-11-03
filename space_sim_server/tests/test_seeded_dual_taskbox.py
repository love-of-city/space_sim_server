"""Seed generation, complete clones and independent physical seat ownership."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from space_arm_platform import library_layouts
from space_arm_platform.imported_scene_library import digest
from test_library_layouts import pool_fixture
from tools.prepare_seeded_dual_taskbox import role_seed
from simulation.scene_library.taskbox_mount import extract_taskbox, mount_taskbox, detach_taskbox, LOOSE_BODIES
from simulation.plug_retention import model_retention_sockets
from tools.build_dual_taskbox_layout import cross_box_contacts


def test_role_streams_do_not_swap_or_repeat_two_prebuilt_layouts():
    pairs = [(role_seed(s, 'target'), role_seed(s, 'service')) for s in range(20)]
    assert len(set(pairs)) == 20
    assert len({v for pair in pairs for v in pair}) == 40
    assert pairs[5] == (role_seed(5, 'target'), role_seed(5, 'service'))
    assert role_seed(5, 'target', 1) not in pairs[5]
    with pytest.raises(ValueError):
        role_seed(True, 'target')


def test_complete_service_parts_have_independent_free_joints_and_retention(tmp_path):
    source = Path(__file__).resolve().parents[1]/'model/SARM/platform/sarm_20261012.xml'
    root = ET.parse(source).getroot()
    template = extract_taskbox(root)
    detach_taskbox(root)
    target = ET.SubElement(root.find('worldbody'), 'body', name='capture_target', pos='2 0 0')
    ET.SubElement(target, 'freejoint', name='capture_target_free')
    for owner, prefix in (('capture_target', ''), ('cubesat_bus', 'service_')):
        mount_taskbox(root, template, owner, position=[0, 0, 0], rotation=np.eye(3),
                      prefix=prefix, move_loose_parts=True)
        for name in LOOSE_BODIES:
            body = root.find(f'./worldbody/body[@name="{prefix+name}"]')
            assert body is not None
            assert body.find('freejoint') is not None or body.find('joint[@type="free"]') is not None
            assert len(list(body.iter('geom'))) > 0
    path = tmp_path/'complete.xml'
    previous = list(root.find('contact'))
    cross_box_contacts(root)
    added = [n for n in root.find('contact') if n not in previous]
    assert added
    assert all('cubesat_bus' not in n.attrib.values() and 'capture_target' not in n.attrib.values() for n in added)
    for node in added:
        refs = [v for k, v in node.attrib.items() if k.startswith(('geom', 'body'))]
        assert any(v.startswith('service_') for v in refs)
        assert any(not v.startswith('service_') for v in refs)
    ET.ElementTree(root).write(path)
    # Every copied free part must receive the same orbital translation and
    # velocity as its seat. An omitted service part stays near the Earth origin
    # and makes the first local dynamics step diverge.
    from space_arm_platform.task_box import initialize_free_plugs
    class Body:
        def setPosition(self, value): self.position = value
        def setVelocity(self, value): self.velocity = value
    bodies = {node.get('name'): Body() for node in root.findall('.//body[@name]')}
    scene = SimpleNamespace(getBodyNames=lambda: list(bodies), getBody=bodies.__getitem__)
    initialize_free_plugs(scene, path, [-6878136.6, 0, 0], [.01, -4728.5, -5965.9])
    for prefix in ('', 'service_'):
        for name in LOOSE_BODIES:
            body = bodies[prefix+name]
            local = np.fromstring(root.find(f'./worldbody/body[@name="{prefix+name}"]').get('pos'), sep=' ')
            np.testing.assert_allclose(body.position, np.array([-6878136.6, 0, 0])+local, atol=1e-9)
            assert body.velocity == [.01, -4728.5, -5965.9]
    sockets = model_retention_sockets(path, owned=True)
    assert len(sockets) == 6
    for prefix, owner in (('', 'capture_target'), ('service_', 'cubesat_bus')):
        for kind in ('aviation', 'guide', 'cylindrical'):
            seats = sockets[prefix+kind+'_plug']
            initial = [s for s in seats if s.initial]
            assert len(initial) == 1 and initial[0].owner_body == owner
            assert initial[0].name == prefix+kind+'_initial'
            assert len(seats) == 5
            assert {s.owner_body for s in seats} == {'capture_target', 'cubesat_bus'}


def generator_fixture(tmp_path):
    entry, _, old_pool = pool_fixture(tmp_path)
    base = json.loads(Path(entry['source_instance']).read_text())
    cfg = tmp_path/'generator.json'
    cfg.write_text(json.dumps(dict(schema='seeded-dual-taskbox-config/1', dependencies=[],
        cache_root=str(tmp_path/'cache'), base_model_sha256=digest(base['model']), geometry_python='python')))
    entry.update(task_box_layout_generator=str(cfg), task_box_layout_generator_sha256=digest(cfg))
    return entry, old_pool


def test_generation_uses_requested_seed_and_reuses_only_matching_cache(tmp_path, monkeypatch):
    entry, old_pool = generator_fixture(tmp_path)
    calls = []
    def prepare(command, **kwargs):
        seed = int(command[command.index('--seed')+1])
        output = Path(command[command.index('--output')+1]); output.mkdir()
        pool = json.loads(old_pool.read_text())
        chosen = deepcopy(pool['variants'][0]['entries'][entry['id']])
        source = json.loads(Path(chosen['source_instance']).read_text())
        source['task_box_layout'].update(complete_parts_on_both_spacecraft=True)
        path = output/'source.json'; path.write_text(json.dumps(source))
        chosen.update(source_instance=str(path), source_instance_sha256=digest(path))
        pool['variants'] = [dict(entries={entry['id']:chosen})]
        result = output/'pool.json'; result.write_text(json.dumps(pool))
        (output/'pipeline.json').write_text(json.dumps(dict(completed=True, seed=seed, generation_attempt=0,
            role_seeds={r:role_seed(seed,r) for r in ('target','service')}, registered=dict(pool=str(result)),
            result_sha256=digest(result))))
        calls.append(seed)
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(library_layouts.subprocess, 'run', prepare)
    a = library_layouts.select_layout(entry, 101)
    assert library_layouts.select_layout(entry, 101) == a
    others = [library_layouts.select_layout(entry, s) for s in (102, 103)]
    assert calls == [101, 102, 103]
    assert len({x[1]['layout_id'] for x in [a, *others]}) == 3
    assert a[1]['role_seeds']['target'] != a[1]['role_seeds']['service']
    Path(a[0]['source_instance']).write_text('{}')
    with pytest.raises(ValueError, match='evidence changed'):
        library_layouts.select_layout(entry, 101)


def test_generation_failure_never_falls_back_to_old_pool(tmp_path, monkeypatch):
    entry, _ = generator_fixture(tmp_path)
    monkeypatch.setattr(library_layouts.subprocess, 'run', lambda *a, **kw: SimpleNamespace(returncode=1))
    with pytest.raises(ValueError, match='随机生成未通过'):
        library_layouts.select_layout(entry, 101)


def test_status_and_stop_remain_responsive_during_generation(tmp_path, monkeypatch):
    from concurrent.futures import ThreadPoolExecutor
    import threading
    import sys
    from space_arm_platform.scene_runtime import SceneRuntimeManager
    from space_arm_platform.models import SceneInstanceCreate
    manager = SceneRuntimeManager(SimpleNamespace(project_root=tmp_path, python_exe=sys.executable))
    entered, release = threading.Event(), threading.Event()
    def prepare(*args, **kwargs):
        entered.set()
        assert release.wait(5)
        return {}
    monkeypatch.setattr(manager, 'create_instance', prepare)
    with ThreadPoolExecutor(max_workers=2) as executor:
        worker = executor.submit(manager.start, SceneInstanceCreate())
        try:
            assert entered.wait(2)
            assert executor.submit(manager.status).result(timeout=1)['phase'] == 'preparing_layout'
            assert executor.submit(manager.stop).result(timeout=1)['phase'] == 'stopping'
        finally:
            release.set()
        with pytest.raises(RuntimeError, match='已取消'):
            worker.result(timeout=2)
    assert manager._process is None
