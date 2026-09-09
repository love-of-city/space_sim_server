"""Replay validation and the actual orbital position/velocity assignment path."""
import importlib.util
import json
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest
from Basilisk.utilities import RigidBodyKinematics as rbk, simIncludeGravBody

from space_arm_platform.models import SceneInstanceCreate
from space_arm_platform.scene_runtime import SceneRuntimeManager, _DEFAULT_ORBIT
from simulation.teleop_grasp_unreal import _apply_orbital_initial_state, _load_scene_instance


def make_instance(tmp_path, **kwargs):
    return SceneRuntimeManager(None, project_root=tmp_path).create_instance(
        SceneInstanceCreate(seed=42, randomize_orbit_phase=True, **kwargs)
    )


def save_instance(tmp_path, instance):
    path = tmp_path / "replay.json"
    path.write_text(json.dumps(instance), encoding="utf-8")
    return path


@pytest.mark.parametrize("angle", [0, 90, 180, 270, 359.999999])
def test_loader_replays_the_saved_angle_without_resampling(tmp_path, angle):
    instance = make_instance(tmp_path)
    instance["environment"]["orbit"]["true_anomaly_deg"] = angle
    instance["seed"] = 123  # Stored physical state is authoritative, not a new draw.
    loaded = _load_scene_instance(save_instance(tmp_path, instance))
    assert loaded["randomize_orbit_phase"] is True
    assert loaded["environment"]["orbit"]["true_anomaly_deg"] == angle
    assert loaded["randomization"] == instance["randomization"]


def test_legacy_instances_keep_explicit_angles_or_default_to_180(tmp_path):
    instance = make_instance(tmp_path)
    del instance["randomize_orbit_phase"]
    loaded = _load_scene_instance(save_instance(tmp_path, instance))
    assert loaded["randomize_orbit_phase"] is False
    assert loaded["environment"]["orbit"] == instance["environment"]["orbit"]
    del instance["environment"]
    assert _load_scene_instance(save_instance(tmp_path, instance))["environment"]["orbit"] == _DEFAULT_ORBIT


@pytest.mark.parametrize("bad", [None, 0, 1, "true", "false", [], {}])
def test_hand_edited_orbital_flags_are_validated(tmp_path, bad):
    instance = make_instance(tmp_path)
    instance["randomize_orbit_phase"] = bad
    with pytest.raises(ValueError, match="randomize_orbit_phase must be a boolean"):
        _load_scene_instance(save_instance(tmp_path, instance))


@pytest.mark.parametrize("missing", ["environment", "orbit", "true_anomaly_deg"])
def test_randomized_replays_require_the_actual_sampled_angle(tmp_path, missing):
    instance = make_instance(tmp_path)
    if missing == "environment":
        del instance["environment"]
    elif missing == "orbit":
        del instance["environment"]["orbit"]
    else:
        del instance["environment"]["orbit"]["true_anomaly_deg"]
    with pytest.raises(ValueError, match="requires a saved environment.orbit.true_anomaly_deg"):
        _load_scene_instance(save_instance(tmp_path, instance))


@pytest.mark.parametrize("bad", [None, "invalid", float("nan"), float("inf")])
def test_invalid_orbital_angles_are_rejected(tmp_path, bad):
    instance = make_instance(tmp_path)
    instance["environment"]["orbit"]["true_anomaly_deg"] = bad
    with pytest.raises(ValueError, match="true_anomaly_deg"):
        _load_scene_instance(save_instance(tmp_path, instance))


class StateSink:
    def __init__(self):
        self.joints = {}

    def setPosition(self, value):
        self.position = np.asarray(value).copy()

    def setVelocity(self, value):
        self.velocity = np.asarray(value).copy()

    def setAttitude(self, value):
        self.attitude = np.asarray(value).copy()

    def setAttitudeRate(self, value):
        self.spin = np.asarray(value).copy()

    def getScalarJoint(self, name):
        return self.joints.setdefault(name, StateSink())


def expected_circular_state(earth, angle):
    # Independent analytic expectation for the current RAAN=argp=0 circle.
    theta = math.radians(angle)
    inclination = math.radians(_DEFAULT_ORBIT["inclination_deg"])
    radius = earth.radEquator + _DEFAULT_ORBIT["altitude_m"]
    position = radius * np.array([math.cos(theta), math.sin(theta) * math.cos(inclination), math.sin(theta) * math.sin(inclination)])
    velocity = math.sqrt(earth.mu / radius) * np.array([-math.sin(theta), math.cos(theta) * math.cos(inclination), math.cos(theta) * math.sin(inclination)])
    return position, velocity


@pytest.mark.parametrize("angle", [0, 90, 180, 270, 359.999999, None])
def test_body_and_joint_setters_receive_matching_phase_and_preserve_local_state(tmp_path, angle):
    instance = make_instance(tmp_path)
    if angle is not None:
        instance["environment"]["orbit"]["true_anomaly_deg"] = angle
    loaded = _load_scene_instance(save_instance(tmp_path, instance))
    bodies = {name: StateSink() for name in ["cubesat_bus", "capture_target", *(f"joint_body{i}" for i in range(8))]}
    scene = SimpleNamespace(getBody=bodies.__getitem__)
    native = SimpleNamespace(
        COMMON_VELOCITY=[0.01, -0.004, 0.002],
        JOINTS=[(f"joint_body{i}", f"joint{i}") for i in range(8)],
        _quaternion_to_mrp=lambda q: rbk.EP2MRP(q / np.linalg.norm(q)),
    )
    earth = simIncludeGravBody.gravBodyFactory().createEarth()
    randomized = loaded["randomization"]
    joints = np.asarray(randomized["arm_joint_position_rad"])
    summary = _apply_orbital_initial_state(scene, native, earth, loaded["environment"]["orbit"], randomized, joints)
    expected_r, expected_v = expected_circular_state(earth, loaded["environment"]["orbit"]["true_anomaly_deg"])
    bus, target = bodies["cubesat_bus"], bodies["capture_target"]
    np.testing.assert_allclose(bus.position, expected_r, rtol=0, atol=1e-8)
    np.testing.assert_allclose(bus.velocity, expected_v + native.COMMON_VELOCITY, rtol=0, atol=1e-9)
    np.testing.assert_allclose(target.position - bus.position, randomized["target_position_m"], rtol=0, atol=1e-9)
    np.testing.assert_allclose(target.velocity, expected_v + randomized["target_linear_velocity_m_s"], rtol=0, atol=1e-9)
    np.testing.assert_allclose(target.attitude, native._quaternion_to_mrp(np.asarray(randomized["target_orientation_wxyz"])))
    np.testing.assert_array_equal(target.spin, [0, 0, 0])
    for index, (body, joint) in enumerate(native.JOINTS):
        assert bodies[body].joints[joint].position == joints[index]
        assert bodies[body].joints[joint].velocity == 0
    assert summary["initial_altitude_m"] == pytest.approx(500000, abs=1e-8)
    # The pre-existing 1 cm/s common drift is kept; the orbital component is tangent.
    v_orbit = bus.velocity - native.COMMON_VELOCITY
    assert np.dot(bus.position, v_orbit) / (np.linalg.norm(bus.position) * np.linalg.norm(v_orbit)) == pytest.approx(0, abs=1e-14)


@pytest.fixture
def native(monkeypatch):
    # Use the real Basilisk MJScene, never import a second Python MuJoCo DLL.
    if "mujoco" in sys.modules and hasattr(sys.modules["mujoco"], "MjModel"):
        pytest.fail("Do not load Python MuJoCo and Basilisk MuJoCo in one process")
    monkeypatch.setitem(sys.modules, "mujoco", ModuleType("mujoco"))
    path = Path(__file__).resolve().parents[1] / "model/SARM/platform/scenarios/scenario_sarm_grasp.py"
    spec = importlib.util.spec_from_file_location("sarm_orbit_initialization_test", path)
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("angle", [0, 90, 180, 270, None])
def test_native_mjscene_publishes_the_selected_orbit_and_local_layout(tmp_path, native, angle):
    instance = make_instance(tmp_path)
    orbit = instance["environment"]["orbit"]
    if angle is not None:
        orbit["true_anomaly_deg"] = angle
    randomized = instance["randomization"]
    joints = np.asarray(randomized["arm_joint_position_rad"])
    native.JointTrajectoryPublisher.reference = classmethod(lambda cls, t: (joints.copy(), np.zeros(8)))
    sim, scene, models, recorders = native._build_simulation()
    native._initialize_state(sim, scene)
    earth = simIncludeGravBody.gravBodyFactory().createEarth()
    summary = _apply_orbital_initial_state(scene, native, earth, orbit, randomized, joints)
    # A short native dynamics step refreshes the same origin messages read by UE's bridge.
    sim.ConfigureStopTime(native.macros.sec2nano(0.01))
    sim.ExecuteSimulation()
    bus = scene.getBody("cubesat_bus").getOrigin().stateOutMsg.read()
    target = scene.getBody("capture_target").getOrigin().stateOutMsg.read()
    expected_r = np.asarray(summary["initial_position_m"]) + np.asarray(summary["initial_velocity_m_s"]) * 0.01
    np.testing.assert_allclose(bus.r_BN_N, expected_r, rtol=0, atol=0.02)
    np.testing.assert_allclose(np.asarray(target.r_BN_N) - bus.r_BN_N, randomized["target_position_m"], rtol=0, atol=0.002)
    assert all(np.isfinite(j.stateOutMsg.read().state) for j in [scene.getBody(b).getScalarJoint(n) for b, n in native.JOINTS])
