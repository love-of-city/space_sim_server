import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import xml.etree.ElementTree as ET

from space_arm_platform.control_defaults import BALANCED_TELEOP_HOME
from simulation.serial_chain_kinematics import rotation_matrix_to_vector


SCENARIO = Path(__file__).resolve().parents[1] / "simulation" / "teleop_grasp_unreal.py"
SPEC = importlib.util.spec_from_file_location("teleop_grasp_unreal_test", SCENARIO)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class FakeClient:
    def __init__(self, stale: bool = False) -> None:
        self.stale = stale

    def latest_action(self):
        return (
            {
                "deadman": True,
                "server_sequence": "7",
                "end_effector_linear_velocity_body_m_s": [0.02, 0.0, 0.0],
                "end_effector_angular_velocity_body_rad_s": [0.0, 0.0, 0.1],
                "gripper_velocity_m_s": -0.005,
            },
            self.stale,
        )


class CountingKinematics:
    def __init__(self, *, constant_velocity: bool = False) -> None:
        self.jacobian_calls = 0
        self.inverse_calls = 0
        self.constant_velocity = constant_velocity

    def forward(self, position):
        q = np.asarray(position, dtype=float)
        rotation = (
            MODULE.axis_angle_to_matrix(np.array([1.0, 0.0, 0.0]), q[3])
            @ MODULE.axis_angle_to_matrix(np.array([0.0, 1.0, 0.0]), q[4])
            @ MODULE.axis_angle_to_matrix(np.array([0.0, 0.0, 1.0]), q[5])
        )
        return q[:3].copy(), rotation

    def jacobian(self, _position):
        self.jacobian_calls += 1
        return np.eye(6)

    def inverse_velocity_bounded(
        self, _position, twist, *, joint_velocity_limits,
        joint_position_min=None, joint_position_max=None, dt=None,
    ):
        self.inverse_calls += 1
        limits = np.asarray(joint_velocity_limits, dtype=float)
        if self.constant_velocity:
            velocity = np.minimum(limits, 0.1)
            achieved = np.asarray(twist, dtype=float).copy()
        else:
            velocity = np.clip(np.asarray(twist, dtype=float), -limits, limits)
            achieved = velocity.copy()
        return SimpleNamespace(
            joint_velocity_rad_s=velocity,
            achieved_twist=achieved,
            residual_twist=np.asarray(twist, dtype=float) - achieved,
            jacobian_rank=6,
            velocity_scale=1.0,
            minimum_singular_value=1.0,
            condition_number=1.0,
        )


def test_target_integrates_only_fresh_deadman_command() -> None:
    initial = np.asarray(BALANCED_TELEOP_HOME, dtype=float)
    model = Path(__file__).resolve().parents[1] / "model" / "SARM" / "platform" / "sarm_platform.xml"
    kinematics = MODULE.SerialChainKinematics.from_mjcf(
        model,
        base_body="cubesat_bus",
        joint_names=MODULE.ARM_JOINT_NAMES,
        tool_site="sarm_ee",
    )
    target = MODULE.CartesianTeleopTarget(initial, FakeClient(), kinematics)
    target.reference(0.0)
    position, velocity = target.reference(0.002)
    assert position[6:] == pytest.approx([0.01874, 0.01874])
    assert np.linalg.norm(position[:6] - initial[:6]) > 0.0
    assert np.all(np.abs(velocity[:6]) <= MODULE.ARM_JOINT_VELOCITY_LIMIT + 1e-12)
    assert target.applied_sequence == "7"
    assert target.jacobian_rank == np.linalg.matrix_rank(kinematics.jacobian(initial[:6]), tol=1e-5)

    stale_target = MODULE.CartesianTeleopTarget(initial, FakeClient(stale=True), kinematics)
    stale_target.reference(0.0)
    stale_position, stale_velocity = stale_target.reference(0.002)
    assert np.allclose(stale_position, initial)
    assert np.allclose(stale_velocity, 0.0)


def test_cached_reference_never_recomputes_ik() -> None:
    initial = np.array([0.0, -0.1, 0.2, 0.0, 0.0, 0.4, 0.01875, 0.01875])
    kinematics = CountingKinematics()
    target = MODULE.CartesianTeleopTarget(initial, FakeClient(), kinematics)
    target.reset(0.0)
    target.update(0.01)
    assert kinematics.inverse_calls == 1

    expected_position = target.position.copy()
    expected_velocity = target.velocity.copy()
    for stage_time in (0.010, 0.011, 0.011, 0.012):
        position, velocity = target.cached_reference(stage_time)
        assert np.array_equal(position, expected_position)
        assert np.array_equal(velocity, expected_velocity)

    assert kinematics.inverse_calls == 1
    assert target.update_count == 1
    assert target.ik_solve_count == 1


def test_control_model_updates_only_when_scheduled() -> None:
    initial = np.array([0.0, -0.1, 0.2, 0.0, 0.0, 0.4, 0.01875, 0.01875])
    kinematics = CountingKinematics()
    target = MODULE.CartesianTeleopTarget(initial, FakeClient(), kinematics)
    controller = MODULE.CartesianIkControlModel(target)
    controller.Reset(0)
    controller.UpdateState(10_000_000)
    assert kinematics.inverse_calls == 1
    assert target.update_count == 1


def test_controller_limits_match_all_eight_sarm_joints() -> None:
    model = Path(__file__).resolve().parents[1] / "model/SARM/platform/sarm_platform.xml"
    root = ET.parse(model).getroot()
    names = (*MODULE.ARM_JOINT_NAMES, "joint_finger1", "joint_finger2")
    limits = np.array([
        np.fromstring(root.find(f'.//joint[@name="{name}"]').get("range"), sep=" ")
        for name in names
    ])
    assert np.array_equal(MODULE.JOINT_MIN, limits[:, 0])
    assert np.array_equal(MODULE.JOINT_MAX, limits[:, 1])


def test_joint5_and_both_fingers_stop_at_xml_limits() -> None:
    initial = np.zeros(8)
    initial[4] = MODULE.JOINT_MAX[4] - 0.0005
    initial[6:] = 0.00001
    target = MODULE.CartesianTeleopTarget(
        initial, FakeClient(), CountingKinematics(constant_velocity=True)
    )
    target.reset(0.0)
    position, velocity = target.update(0.01)
    assert position[4] == MODULE.JOINT_MAX[4]
    assert velocity[4] == 0.0
    assert np.array_equal(position[6:], [0.0, 0.0])
    assert np.array_equal(velocity[6:], [0.0, 0.0])


@pytest.mark.parametrize("reason", ["stale", "deadman"])
def test_release_or_timeout_holds_last_target_for_arm_and_both_fingers(reason) -> None:
    client = FakeClient()
    target = MODULE.CartesianTeleopTarget(
        np.array([0, -0.1, 0.2, 0, 0, 0, 0.01875, 0.01875]), client, CountingKinematics()
    )
    target.reset(0.0)
    target.update(0.01)
    held_tool_position = target.target_tool_position.copy()
    held_tool_rotation = target.target_tool_rotation.copy()
    held_gripper = target.position[6:].copy()
    action, _ = client.latest_action()
    if reason == "deadman":
        action["deadman"] = False
    client.latest_action = lambda: (action, reason == "stale")
    for time in (0.02, 0.03, 0.04):
        position, velocity = target.update(time)
        assert np.array_equal(target.target_tool_position, held_tool_position)
        assert np.array_equal(target.target_tool_rotation, held_tool_rotation)
        assert np.array_equal(position[6:], held_gripper)
        assert np.array_equal(velocity[6:], np.zeros(2))
    assert target.ik_solve_count == 4


@pytest.mark.parametrize("model_relative,catalog_name", [
    (None, "sarm_platform.catalog.json"),
    ("custom/platform", "sarm_platform.catalog.json"),
    ("legacy/spacecraft_and_arm", "cubesat_so101.catalog.json"),
])
def test_cli_resolves_catalog_from_selected_adapter_and_model(tmp_path, monkeypatch, model_relative, catalog_name):
    project = tmp_path / "server checkout"
    adapter = tmp_path / "adapter checkout"
    model = project / "model/SARM/platform" if model_relative is None else tmp_path / model_relative
    model.mkdir(parents=True)
    catalog = adapter / "Unreal/BskUnrealRenderer/Saved/AssetImport" / catalog_name
    catalog.parent.mkdir(parents=True)
    catalog.write_text("{}", encoding="utf-8")
    arguments = [str(SCENARIO), "--adapter-root", str(adapter)]
    if model_relative is not None:
        arguments.extend(["--model-root", str(model)])
    monkeypatch.setattr("sys.argv", arguments)
    monkeypatch.setattr(MODULE, "PROJECT_ROOT", project)
    calls = []
    monkeypatch.setattr(MODULE, "run", calls.append)
    MODULE.main()
    assert len(calls) == 1
    assert calls[0].model_root == model
    assert calls[0].catalog == catalog


def test_cli_keeps_explicit_catalog_override(tmp_path, monkeypatch):
    model = tmp_path / "model/SARM/platform"
    model.mkdir(parents=True)
    catalog = tmp_path / "custom.catalog.json"
    catalog.write_text("{}", encoding="utf-8")
    monkeypatch.setattr(MODULE, "PROJECT_ROOT", tmp_path)
    monkeypatch.setattr("sys.argv", [str(SCENARIO), "--adapter-root", str(tmp_path), "--catalog", str(catalog)])
    calls = []
    monkeypatch.setattr(MODULE, "run", calls.append)
    MODULE.main()
    assert calls[0].catalog == catalog


class MutableClient:
    def __init__(self, linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0), deadman=True):
        self.linear = list(linear)
        self.angular = list(angular)
        self.deadman = deadman

    def latest_action(self):
        return ({
            "deadman": self.deadman,
            "server_sequence": "11",
            "end_effector_linear_velocity_body_m_s": self.linear,
            "end_effector_angular_velocity_body_rad_s": self.angular,
            "gripper_velocity_m_s": 0.0,
        }, False)


def test_balanced_home_tracks_each_translation_axis_without_attitude_drift() -> None:
    model = Path(__file__).resolve().parents[1] / "model/SARM/platform/sarm_platform.xml"
    kinematics = MODULE.SerialChainKinematics.from_mjcf(
        model,
        base_body="cubesat_bus",
        joint_names=MODULE.ARM_JOINT_NAMES,
        tool_site="sarm_ee",
    )
    initial = np.asarray(BALANCED_TELEOP_HOME, dtype=float)
    for axis in range(3):
        command = [0.0, 0.0, 0.0]
        command[axis] = 0.05
        target = MODULE.CartesianTeleopTarget(initial, MutableClient(linear=command), kinematics)
        target.reset(0.0)
        start_position, start_rotation = kinematics.forward(initial[:6])
        for step in range(1, 101):
            target.update(step * 0.01)
        end_position, end_rotation = kinematics.forward(target.position[:6])
        displacement = end_position - start_position
        target_displacement = target.target_tool_position - start_position
        assert displacement[axis] == pytest.approx(target_displacement[axis], abs=8.0e-4)
        assert displacement[axis] > 0.04
        assert np.linalg.norm(np.delete(displacement, axis)) < 2.5e-4
        attitude_error = rotation_matrix_to_vector(end_rotation @ start_rotation.T)
        assert np.linalg.norm(attitude_error) < np.deg2rad(0.02)
        assert target.velocity_scale == pytest.approx(1.0)


def test_measured_attitude_error_slows_translation_without_rotating_reference() -> None:
    model = Path(__file__).resolve().parents[1] / "model/SARM/platform/sarm_platform.xml"
    kinematics = MODULE.SerialChainKinematics.from_mjcf(
        model,
        base_body="cubesat_bus",
        joint_names=MODULE.ARM_JOINT_NAMES,
        tool_site="sarm_ee",
    )
    initial = np.asarray(BALANCED_TELEOP_HOME, dtype=float)
    client = MutableClient(linear=(0.05, 0.0, 0.0))
    target = MODULE.CartesianTeleopTarget(initial, client, kinematics)
    target.reset(0.0)
    disturbed = initial[:6].copy()
    disturbed[4] += 0.02
    target.bind_joint_state_provider(lambda: disturbed)
    initial_target_position = target.target_tool_position.copy()
    target.update(0.01)
    assert np.linalg.norm(target.orientation_error) > MODULE.ORIENTATION_TRACKING_SLOWDOWN_START_RAD
    assert 0.0 < target.tracking_scale < 1.0
    assert 0.0 < np.linalg.norm(target.target_tool_position - initial_target_position) < 0.0005
    assert np.allclose(target.achieved_twist[3:], 0.0, atol=1e-10)
