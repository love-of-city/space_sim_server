"""Content locking must detect changes even when names, versions and mtimes match."""
from copy import deepcopy
import json
import os
from pathlib import Path

import pytest

from space_arm_platform import benchmark_runtime as runtime


@pytest.fixture
def isolated_runtime(tmp_path, monkeypatch):
    server = tmp_path/'server'
    model = server/'model/SARM/platform'
    model.mkdir(parents=True)
    (model/'sarm_ground_target_self_collision.xml').write_text(
        '<mujoco><compiler meshdir="meshes"/><asset><mesh name="test" file="geometry.bin"/></asset></mujoco>')
    (model/'meshes').mkdir()
    (model/'meshes/geometry.bin').write_bytes(b'original physical mesh')
    (model/'attitude_control.json').write_text('{"gain":1}')
    native = server/'run/native_acceleration'
    native.mkdir(parents=True)
    (native/'stepper.dll').write_bytes(b'native dynamic library')
    (native/'local_mujoco_stepper.json').write_text(json.dumps(dict(library='run/native_acceleration/stepper.dll',
        library_sha256=runtime.digest_file(native/'stepper.dll'))))
    adapter = tmp_path/'space_sim_UE_Adapter/Adapters/bsk_render_adapter'
    adapter.mkdir(parents=True)
    (adapter/'__init__.py').write_text('# renderer adapter\n')
    python = tmp_path/'python'
    python.mkdir()
    (python/'python.exe').write_bytes(b'Python executable')
    (python/'Lib').mkdir()
    (python/'Lib/standard.py').write_text('VALUE = 1\n')
    monkeypatch.setattr(runtime, 'PACKAGES', ())
    monkeypatch.setattr(runtime.sys, 'executable', str(python/'python.exe'))
    monkeypatch.setattr(runtime.sys, 'base_prefix', str(python))
    monkeypatch.setattr(runtime.sysconfig, 'get_path', lambda _: str(python/'Lib'))
    monkeypatch.setattr(runtime.importlib.metadata, 'distributions', lambda: [])
    return server


def test_identity_is_stable_through_json_and_ignores_bytecode_cache(isolated_runtime):
    first = runtime.runtime_identity(isolated_runtime)
    assert not first['errors']
    assert json.loads(json.dumps(first)) == first
    adapter = isolated_runtime.parent/'space_sim_UE_Adapter/Adapters/bsk_render_adapter'
    (adapter/'__pycache__').mkdir()
    (adapter/'__pycache__/__init__.pyc').write_bytes(b'different machine cache')
    assert runtime.runtime_identity(isolated_runtime) == first


@pytest.mark.parametrize('component', ['mesh', 'controller', 'adapter', 'native', 'stdlib', 'added_file'])
def test_content_changes_are_detected_without_relying_on_file_timestamps(isolated_runtime, component):
    first = runtime.runtime_identity(isolated_runtime)
    paths = dict(mesh=isolated_runtime/'model/SARM/platform/meshes/geometry.bin',
                 controller=isolated_runtime/'model/SARM/platform/attitude_control.json',
                 adapter=isolated_runtime.parent/'space_sim_UE_Adapter/Adapters/bsk_render_adapter/__init__.py',
                 native=isolated_runtime/'run/native_acceleration/stepper.dll',
                 stdlib=isolated_runtime.parent/'python/Lib/standard.py',
                 added_file=isolated_runtime.parent/'python/Lib/extra.py')
    path = paths[component]
    old = path.stat() if path.exists() else None
    path.write_bytes(b'changed content')
    if old: os.utime(path, ns=(old.st_atime_ns, old.st_mtime_ns))
    changed = runtime.runtime_identity(isolated_runtime)
    assert changed != first and changed['fingerprint'] != first['fingerprint']
    if component == 'native': assert 'native_library' in changed['errors']


def test_missing_lfs_content_is_an_explicit_dependency_failure(isolated_runtime):
    path = isolated_runtime/'model/SARM/platform/meshes/geometry.bin'
    path.write_text('version https://git-lfs.github.com/spec/v1\n')
    assert 'hydrated LFS' in runtime.runtime_identity(isolated_runtime)['errors']['native_assets']


def test_plan_v2_cannot_drop_its_runtime_lock_but_legacy_archives_remain_readable(isolated_runtime):
    identity = runtime.runtime_identity(isolated_runtime)
    plan = dict(schema_version='spacesim-autonomy-batch-plan/2', runtime_identity=identity)
    assert runtime.plan_runtime_identity(plan) == identity
    for invalid in (None, {}, {'fingerprint': 'hash'}):
        with pytest.raises(ValueError, match='complete runtime identity'):
            runtime.plan_runtime_identity(dict(plan, runtime_identity=invalid))
    assert runtime.plan_runtime_identity({'schema_version': 'spacesim-autonomy-batch-plan/1'}) is None
    with pytest.raises(ValueError, match='unknown'):
        runtime.plan_runtime_identity({'schema_version': 'spacesim-autonomy-batch-plan/99'})
    altered = deepcopy(plan)
    altered['runtime_identity']['files_sha256']['native/library'] = 'changed'
    with pytest.raises(ValueError, match='fingerprint'):
        runtime.plan_runtime_identity(altered)


@pytest.mark.parametrize('defect', ['missing', 'changed', 'unverified'])
def test_completed_trial_cannot_substitute_a_different_runtime(tmp_path, defect):
    from space_arm_platform.benchmark_batch import _completed_trial
    from space_arm_platform.benchmark_contract import load_task, SPEC_DIRECTORY
    from space_arm_platform.benchmark_execution import write_json
    task = load_task(SPEC_DIRECTORY/'m02-gnss-formation-hold-v1.json')
    expected = dict(fingerprint='frozen', files_sha256={'library': 'same'})
    (tmp_path/'run').mkdir()
    manifest = dict(task_fingerprint=task.fingerprint(), source={'source_sha256': {}}, runtime_identity=deepcopy(expected))
    if defect == 'missing': manifest.pop('runtime_identity')
    if defect == 'changed': manifest['runtime_identity']['files_sha256']['library'] = 'replacement'
    write_json(tmp_path/'run/manifest.json', manifest)
    write_json(tmp_path/'run/result.json', dict(seed=201, method='gnss-formation-v1', outcome='success',
                                               runtime_identity_verified=defect != 'unverified'))
    write_json(tmp_path/'status.json', {'outcome': 'success'})
    with pytest.raises(ValueError, match='runtime dependencies'):
        _completed_trial(tmp_path, {'seed': 201}, task, 'gnss-formation-v1', source_sha256={}, runtime_identity=expected)


def test_runtime_change_during_a_failed_method_cannot_be_labeled_method_failure(tmp_path, monkeypatch):
    from space_arm_platform import benchmark_execution as execution
    from space_arm_platform.benchmark_contract import load_task, SPEC_DIRECTORY
    task = load_task(SPEC_DIRECTORY/'d32-cooperative-gnss-v1.json')
    snapshots = iter([dict(fingerprint='first', errors={}), dict(fingerprint='changed', errors={})])
    monkeypatch.setattr(execution, 'runtime_identity', lambda _: next(snapshots))
    monkeypatch.setattr(execution, 'native_environment', lambda *args: (_ for _ in ()).throw(execution.MethodFailure('test failure')))
    result = execution.run_episode(task, 'gnss-pd-v1', 201, tmp_path/'run')
    assert result['outcome'] == 'infrastructure_error'
    assert result['runtime_identity_verified'] is False
    assert 'runtime dependencies changed' in result['detail']
    assert 'runtime_final.json' in result['evidence_sha256']
