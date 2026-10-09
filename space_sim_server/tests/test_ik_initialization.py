"""Scene persistence and isolated worker failure/override semantics."""
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from space_arm_platform.control_defaults import ELBOW_UP_TELEOP_PROFILE, BALANCED_TELEOP_PROFILE
from space_arm_platform.ik_initialization import prepare_initial_posture
from space_arm_platform.models import SceneInstanceCreate
from space_arm_platform.scene_runtime import SceneRuntimeManager


def test_new_scene_persists_selection_but_never_changes_explicit_angles(tmp_path, monkeypatch):
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    model_root = tmp_path / 'model/SARM/platform'
    model_root.mkdir(parents=True)
    # Real limits, but the expensive worker is replaced only in this unit test.
    source = Path(__file__).resolve().parents[1] / 'model/SARM/platform/sarm_platform.xml'
    (model_root / 'sarm_platform.xml').write_bytes(source.read_bytes())
    calls = []
    def prepare(path, randomized):
        calls.append(randomized['arm_joint_position_rad'][:])
        return dict(status='selected', joint_position_rad=[0., -.5, -.3, 0., 0., 0.])
    monkeypatch.setattr('space_arm_platform.ik_initialization.prepare_initial_posture', prepare)
    req = dict(seed=123, template_id='spacecraft-arm-teleop')
    old = manager.create_instance(SceneInstanceCreate(**req, randomization_profile=BALANCED_TELEOP_PROFILE))
    new = manager.create_instance(SceneInstanceCreate(**req, randomization_profile=ELBOW_UP_TELEOP_PROFILE))
    assert len(calls) == 1
    assert new['randomization']['arm_joint_position_rad'][:6] == [0., -.5, -.3, 0., 0., 0.]
    assert new['randomization']['arm_joint_position_rad'][6:] == old['randomization']['arm_joint_position_rad'][6:]
    assert new['randomization']['target_position_m'] == old['randomization']['target_position_m']
    stored = json.loads(Path(new['config_path']).read_text(encoding='utf-8'))
    assert stored['ik_initialization'] == new['ik_initialization']
    assert stored['randomization'] == new['randomization']
    # Loader/reset consumes the saved angles, not a fresh preferred solve.
    import importlib.util
    if importlib.util.find_spec('Basilisk') is not None:
        from simulation.teleop_grasp_unreal import _load_scene_instance
        loaded = _load_scene_instance(Path(new['config_path']))
        assert loaded['randomization']['arm_joint_position_rad'] == new['randomization']['arm_joint_position_rad']
        assert len(calls) == 1
    explicit = manager.create_instance(SceneInstanceCreate(**req, randomization_profile=ELBOW_UP_TELEOP_PROFILE, initial_arm_joint_position_deg=[1,2,3,4,5,6]))
    assert len(calls) == 1
    assert explicit['randomization']['arm_joint_position_rad'][:6] == [math.radians(i) for i in range(1,7)]
    assert explicit['ik_initialization']['reason'] == 'explicit_initial_joints'


def test_worker_off_and_failures_preserve_original_input(monkeypatch, tmp_path):
    import subprocess
    randomized = {'arm_joint_position_rad': [0.]*8}
    monkeypatch.setenv('SPACE_SIM_ELBOW_MODE', 'off')
    def forbidden(*args, **kwargs):
        raise AssertionError('worker must not run')
    monkeypatch.setattr(subprocess, 'run', forbidden)
    assert prepare_initial_posture(tmp_path/'model.xml',randomized)['status'] == 'skipped'
    monkeypatch.setenv('SPACE_SIM_ELBOW_MODE', 'prefer')
    def failed(*args, **kwargs):
        raise subprocess.TimeoutExpired(args[0], 45)
    monkeypatch.setattr(subprocess, 'run', failed)
    assert prepare_initial_posture(tmp_path/'model.xml',randomized)['status'] == 'fallback'
    assert randomized == {'arm_joint_position_rad': [0.]*8}
    monkeypatch.setenv('SPACE_SIM_ELBOW_MODE', 'typo')
    with pytest.raises(ValueError):
        prepare_initial_posture(tmp_path/'model.xml',randomized)


def test_worker_uses_explicit_python_and_rejects_invalid_output(monkeypatch, tmp_path):
    import subprocess
    monkeypatch.setenv('SPACE_SIM_ELBOW_MODE', 'prefer')
    monkeypatch.setenv('SPACE_SIM_POSTURE_PYTHON', 'chosen-python')
    def run(args, **kwargs):
        assert args[0] == 'chosen-python'
        assert kwargs['timeout'] == 45
        assert 'Basilisk' not in json.dumps(args)
        return SimpleNamespace(stdout='{"status":"selected","joint_position_rad":[0]}')
    monkeypatch.setattr(subprocess, 'run', run)
    assert prepare_initial_posture(tmp_path/'model.xml',{})['status'] == 'fallback'


@pytest.mark.parametrize('output', ['[]', 'null', 'not-json', '{"status":"unknown"}',
    '{"status":"selected","joint_position_rad":[true,0,0,0,0,0]}'])
def test_malformed_worker_response_falls_back(monkeypatch, tmp_path, output):
    import subprocess
    monkeypatch.setenv('SPACE_SIM_ELBOW_MODE', 'prefer')
    monkeypatch.setattr(subprocess, 'run', lambda *a, **kw: SimpleNamespace(stdout=output))
    assert prepare_initial_posture(tmp_path/'model.xml', {})['status'] == 'fallback'
