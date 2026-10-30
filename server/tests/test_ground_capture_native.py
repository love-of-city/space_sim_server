"""Real Basilisk MJScene coverage (never import Python MuJoCo here)."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType

import numpy as np
import pytest
from space_arm_platform.models import SceneInstanceCreate
from space_arm_platform.scene_runtime import SceneRuntimeManager
from space_arm_platform.scene_targets import DEFAULT_TEMPLATE, GROUND_TARGET_TEMPLATE, capture_target
from simulation.teleop_grasp_unreal import _load_scene_instance

ROOT = Path(__file__).resolve().parents[1]


def test_loader_accepts_new_scene_and_keeps_zero_pose_validation(tmp_path):
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    instance = manager.create_instance(SceneInstanceCreate(seed=42))
    loaded = _load_scene_instance(Path(instance["config_path"]))
    assert loaded["template_id"] == DEFAULT_TEMPLATE
    assert loaded["randomization"]["target_hinge_position_rad"] == 0.0
    import json
    for bad in (.1, float("nan"), True, "0"):
        loaded["randomization"]["target_hinge_position_rad"] = bad
        path = tmp_path / "invalid.json"
        path.write_text(json.dumps(loaded), encoding="utf-8")
        with pytest.raises(ValueError, match="target_hinge_position_rad"):
            _load_scene_instance(path)


def test_native_scene_has_independent_free_target_and_passive_wing(monkeypatch):
    if "mujoco" in sys.modules and hasattr(sys.modules["mujoco"], "MjModel"):
        pytest.fail("Python MuJoCo and Basilisk MuJoCo must run in separate processes")
    monkeypatch.setitem(sys.modules, "mujoco", ModuleType("mujoco"))
    source = ROOT / "model/SARM/platform/scenarios/scenario_sarm_grasp.py"
    spec = importlib.util.spec_from_file_location("native_ground_target_test", source)
    native = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, native)
    spec.loader.exec_module(native)
    target = capture_target(GROUND_TARGET_TEMPLATE)
    native.MODEL_PATH = target.resolve_model(source.parents[1])
    native.TARGET_POS = np.asarray(target.position_m)
    native.TARGET_QUAT = np.asarray(target.orientation_wxyz)
    native.JointTrajectoryPublisher.reference = classmethod(lambda cls, t: (native.PREGRASP.copy(), np.zeros(8)))
    sim, scene, models, recorders = native._build_simulation()
    native._initialize_state(sim, scene)
    assert len(list(scene.getBodyNames())) == 15
    hinge = scene.getBody(target.hinge_body).getScalarJoint(target.hinge_joint)
    hinge.setVelocity(.1)
    sim.ConfigureStopTime(native.macros.sec2nano(.1))
    sim.ExecuteSimulation()
    assert hinge.stateOutMsg.read().state > .001
    assert np.isfinite(np.asarray(scene.stateOutMsg.read().qpos)).all()
    bus = np.asarray(scene.getBody("cubesat_bus").getOrigin().stateOutMsg.read().r_BN_N)
    body = np.asarray(scene.getBody("capture_target").getOrigin().stateOutMsg.read().r_BN_N)
    np.testing.assert_allclose(body - bus, target.position_m, atol=.002, rtol=0)
    assert sim.attitude_control.enabled
    assert len(native.JOINTS) == 8 and len(native.ACTUATORS) == 8


@pytest.mark.parametrize('initial_hinge_speed', [1., -1.])
def test_native_coarse_self_contact_stops_panel_before_a_full_turn(monkeypatch, initial_hinge_speed):
    if 'mujoco' in sys.modules and hasattr(sys.modules['mujoco'], 'MjModel'):
        pytest.fail('Python MuJoCo and Basilisk MuJoCo must run in separate processes')
    monkeypatch.setitem(sys.modules, 'mujoco', ModuleType('mujoco'))
    source = ROOT / 'model/SARM/platform/scenarios/scenario_sarm_grasp.py'
    spec = importlib.util.spec_from_file_location('native_coarse_self_contact_test', source)
    native = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, native)
    spec.loader.exec_module(native)
    target = capture_target(DEFAULT_TEMPLATE)
    assert target.collision_model == 'coarse_boxes_with_target_self_collision'
    native.MODEL_PATH = target.resolve_model(source.parents[1])
    native.TIME_STEP = .002
    # Keep the target away from the arm so a stop cannot be a robot contact.
    # This is a test fixture only; production initialization is unchanged.
    native.TARGET_POS = np.asarray([2., 0., 0.])
    native.TARGET_QUAT = np.asarray(target.orientation_wxyz)
    native.JointTrajectoryPublisher.reference = classmethod(lambda cls, t: (native.PREGRASP.copy(), np.zeros(8)))
    sim, scene, models, recorders = native._build_simulation()
    native._initialize_state(sim, scene)
    hinge = scene.getBody(target.hinge_body).getScalarJoint(target.hinge_joint)
    hinge.setVelocity(initial_hinge_speed)
    positions, velocities = [], []
    for step in range(1, 101):
        sim.ConfigureStopTime(native.macros.sec2nano(step * .05))
        sim.ExecuteSimulation()
        positions.append(float(hinge.stateOutMsg.read().state))
        velocities.append(float(hinge.stateDotOutMsg.read().state))
        assert np.isfinite(np.asarray(scene.stateOutMsg.read().qpos)).all()
    assert min(positions) > -np.pi and max(positions) < np.pi
    assert max(abs(v) for v in positions) > 1.5  # Actually approached an obstacle.
    assert min(velocities) < 0 < max(velocities)  # Contact reversed relative motion.
    assert len(list(scene.getBodyNames())) == 15
    assert sim.attitude_control.enabled
    assert len(native.JOINTS) == len(native.ACTUATORS) == 8
