import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np


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
                "gripper_velocity_rad_s": -0.25,
            },
            self.stale,
        )


class CountingKinematics:
    def __init__(self) -> None:
        self.jacobian_calls = 0
        self.inverse_calls = 0

    def jacobian(self, _position):
        self.jacobian_calls += 1
        return np.eye(6, 5)

    def inverse_velocity(self, _position, twist, *, joint_velocity_limits):
        self.inverse_calls += 1
        velocity = np.minimum(np.asarray(joint_velocity_limits), 0.1)
        achieved = np.asarray(twist, dtype=float).copy()
        return SimpleNamespace(
            joint_velocity_rad_s=velocity,
            achieved_twist=achieved,
            residual_twist=np.zeros(6),
            jacobian_rank=5,
        )


def test_target_integrates_only_fresh_deadman_command() -> None:
    initial = np.array([0.0, -0.1, 0.2, 0.0, 0.0, 0.4])
    model = Path(__file__).resolve().parents[2] / "test" / "model" / "spacecraft_and_arm" / "assets" / "cubesat_so101_grasp" / "cubesat_so101_grasp.xml"
    kinematics = MODULE.SerialChainKinematics.from_mjcf(
        model,
        base_body="cubesat_bus",
        joint_names=MODULE.ARM_JOINT_NAMES,
        tool_site="so101_gripperframe",
    )
    target = MODULE.CartesianTeleopTarget(initial, FakeClient(), kinematics)
    target.reference(0.0)
    position, velocity = target.reference(0.002)
    assert position[5] == 0.3995
    assert np.linalg.norm(position[:5] - initial[:5]) > 0.0
    assert np.all(np.abs(velocity[:5]) <= MODULE.ARM_JOINT_VELOCITY_LIMIT + 1e-12)
    assert target.applied_sequence == "7"
    assert target.jacobian_rank == 5

    stale_target = MODULE.CartesianTeleopTarget(initial, FakeClient(stale=True), kinematics)
    stale_target.reference(0.0)
    stale_position, stale_velocity = stale_target.reference(0.002)
    assert np.allclose(stale_position, initial)
    assert np.allclose(stale_velocity, 0.0)


def test_cached_reference_never_recomputes_ik() -> None:
    initial = np.array([0.0, -0.1, 0.2, 0.0, 0.0, 0.4])
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
    initial = np.array([0.0, -0.1, 0.2, 0.0, 0.0, 0.4])
    kinematics = CountingKinematics()
    target = MODULE.CartesianTeleopTarget(initial, FakeClient(), kinematics)
    controller = MODULE.CartesianIkControlModel(target)
    controller.Reset(0)
    controller.UpdateState(10_000_000)
    assert kinematics.inverse_calls == 1
    assert target.update_count == 1
