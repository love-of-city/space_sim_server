import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import xml.etree.ElementTree as ET


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
    def __init__(self) -> None:
        self.jacobian_calls = 0
        self.inverse_calls = 0

    def jacobian(self, _position):
        self.jacobian_calls += 1
        return np.eye(6)

    def inverse_velocity(self, _position, twist, *, joint_velocity_limits):
        self.inverse_calls += 1
        velocity = np.minimum(np.asarray(joint_velocity_limits), 0.1)
        achieved = np.asarray(twist, dtype=float).copy()
        return SimpleNamespace(
            joint_velocity_rad_s=velocity,
            achieved_twist=achieved,
            residual_twist=np.zeros(6),
            jacobian_rank=6,
        )


def test_target_integrates_only_fresh_deadman_command() -> None:
    initial = np.array([0.0, -0.1, 0.2, 0.0, 0.0, 0.4, 0.01875, 0.01875])
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
    target = MODULE.CartesianTeleopTarget(initial, FakeClient(), CountingKinematics())
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
    held = target.position.copy()
    action, _ = client.latest_action()
    if reason == "deadman":
        action["deadman"] = False
    client.latest_action = lambda: (action, reason == "stale")
    for time in (0.02, 0.03, 0.04):
        position, velocity = target.update(time)
        assert np.array_equal(position, held)
        assert np.array_equal(velocity, np.zeros(8))
    assert target.ik_solve_count == 1


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
