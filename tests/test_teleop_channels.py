"""Arm/gripper channel isolation and zero-arm-command IK bypass."""
from pathlib import Path
from unittest.mock import patch

import numpy as np
import pytest

from simulation.teleop_grasp_unreal import (
    ARM_JOINT_NAMES, ARM_JOINT_VELOCITY_LIMIT, CartesianTeleopTarget,
    IK_MODES, SerialChainKinematics,
)
from space_arm_platform.control_defaults import BALANCED_TELEOP_HOME

MODEL = Path(__file__).resolve().parents[1] / "model/SARM/platform/sarm_ground_target_self_collision.xml"
SINGULAR = np.array([0., -.1790243, .2159404, -.0368382, 0., 0.])


class Client:
    def __init__(self, twist=None, grip=0., enabled=True):
        self.twist = np.zeros(6) if twist is None else np.asarray(twist, float)
        self.grip, self.enabled, self.stale = grip, enabled, False

    def latest_action(self):
        return {"deadman": self.enabled, "server_sequence": "9",
                "end_effector_linear_velocity_body_m_s": self.twist[:3].tolist(),
                "end_effector_angular_velocity_body_rad_s": self.twist[3:].tolist(),
                "gripper_velocity_m_s": self.grip}, self.stale


def make_target(client, mode="ik_pose"):
    kinematics = SerialChainKinematics.from_mjcf(MODEL, base_body="cubesat_bus",
        joint_names=ARM_JOINT_NAMES, tool_site="sarm_ee")
    target = CartesianTeleopTarget(np.array(BALANCED_TELEOP_HOME), client, kinematics, ik_mode=mode)
    target.reset(0.)
    return target, kinematics


@pytest.mark.parametrize("mode", IK_MODES)
@pytest.mark.parametrize("grip", [0., .005, -.005])
def test_idle_or_gripper_only_at_singularity_never_calls_ik_or_moves_arm(mode, grip):
    client = Client(grip=grip)
    target, kin = make_target(client, mode)
    # A reached singular reference, different from the scene's initial posture.
    # This previously enabled return-to-home nullspace motion on gripper input.
    target.position[:6] = SINGULAR
    target.target_tool_position, target.target_tool_rotation = kin.forward(SINGULAR)
    held = target.position.copy()
    tool = target.target_tool_position.copy(), target.target_tool_rotation.copy()
    measured = SINGULAR.copy()
    calls = []
    def measured_velocity():
        calls.append(1)
        return np.zeros(6)
    target.bind_joint_state_provider(lambda: measured.copy())
    target.bind_joint_velocity_provider(measured_velocity)
    with patch.object(kin, "inverse_velocity_ik_pose", side_effect=AssertionError("idle arm called IK")), \
         patch.object(kin, "inverse_velocity_bounded", side_effect=AssertionError("idle arm called strict IK")), \
         patch("numpy.linalg.svd", side_effect=AssertionError("idle arm called SVD")):
        for step in range(1,101):
            target.update(step*.01)
            assert np.array_equal(target.position[:6], held[:6])
            assert np.array_equal(target.velocity[:6], np.zeros(6))
            assert target.nullspace_correction_norm == 0.
            assert target.solve_time_ms == 0.
            assert target.solver_status == "holding"
    assert target.ik_solve_count == 0
    assert target.update_count == 100
    assert len(calls) == 100  # Passive actual-state telemetry is still sampled.
    assert np.allclose(target.position[6:], held[6:] + grip)
    assert np.array_equal(target.target_tool_position, tool[0])
    assert np.array_equal(target.target_tool_rotation, tool[1])
    assert target.speed_monitor.latest["measurement_valid"]
    assert not target.speed_monitor.latest["linear"]["active"]
    assert not target.speed_monitor.latest["angular"]["warning"]


@pytest.mark.parametrize("mode", IK_MODES)
@pytest.mark.parametrize("axis", range(6))
def test_gripper_addition_never_changes_the_arm_command(mode, axis):
    cmd = np.zeros(6); cmd[axis] = .02 if axis < 3 else .1
    arm, _ = make_target(Client(cmd), mode)
    both, _ = make_target(Client(cmd, grip=.005), mode)
    fingers = arm.position[6:].copy()
    for step in range(1,21):
        arm.update(step*.01); both.update(step*.01)
        assert np.array_equal(arm.position[:6], both.position[:6])
        assert np.array_equal(arm.velocity[:6], both.velocity[:6])
    assert arm.ik_solve_count == both.ik_solve_count == 20
    assert np.array_equal(arm.position[6:], fingers)
    assert np.all(both.position[6:] > fingers)


@pytest.mark.parametrize("mode", IK_MODES)
def test_switch_to_gripper_clears_smoothed_arm_motion_and_resumes_cleanly(mode):
    client = Client([.02,0,0,0,0,0], grip=.005)
    target, kin = make_target(client, mode)
    for step in range(1,21): target.update(step*.01)
    held = target.position[:6].copy()
    tool = target.target_tool_position.copy()
    initial_fingers = target.position[6:].copy()
    client.twist.fill(0.)  # deadman remains True for the gripper.
    for step in range(21,41):
        target.update(step*.01)
        assert np.array_equal(target.position[:6],held)
        assert np.array_equal(target.desired_twist,np.zeros(6))
        assert np.array_equal(target.velocity[:6],np.zeros(6))
    assert target.ik_solve_count == 20
    assert np.array_equal(target.target_tool_position,tool)
    assert np.all(target.position[6:] > initial_fingers)
    client.grip = 0.
    client.twist[0] = -.02
    fingers = target.position[6:].copy()
    for step in range(41,61): target.update(step*.01)
    assert target.ik_solve_count == 40
    assert np.linalg.norm(target.position[:6]-held) > 0.
    assert np.array_equal(target.position[6:], fingers)
    client.stale = True
    stopped = target.position.copy()
    target.update(.61)
    assert np.array_equal(target.position,stopped)
    assert np.array_equal(target.velocity,np.zeros(8))
    assert target.ik_solve_count == 40
    assert target.speed_monitor.latest["command_stale"]


def test_live_ik_at_singularity_is_task_only_not_home_posture():
    client = Client([.02,0,0,0,0,0])
    target, kin = make_target(client)
    target.position[:6] = SINGULAR
    target.target_tool_position, target.target_tool_rotation = kin.forward(SINGULAR)
    with patch.object(kin,"inverse_velocity_ik_pose", wraps=kin.inverse_velocity_ik_pose) as solve:
        target.update(.01)
    assert solve.call_count == 1
    assert solve.call_args.kwargs.get("nullspace_reference") is None
    assert solve.call_args.kwargs.get("nullspace_gains") is None
    expected = kin.inverse_velocity_ik_pose(SINGULAR, target.desired_twist,
        joint_velocity_limits=ARM_JOINT_VELOCITY_LIMIT,
        joint_position_min=target.joint_min[:6], joint_position_max=target.joint_max[:6], dt=.01)
    assert np.array_equal(target.velocity[:6],expected.joint_velocity_rad_s)
    assert target.nullspace_correction_norm == 0.


def test_skipped_solve_clears_previous_diagnostics_without_disabling_measurements():
    target, kin = make_target(Client(grip=.005))
    target.solver_reasons = [{"code":"joint_position_limit","joints":[4]}]
    target.residual_twist[:] = .5
    target.achieved_twist[:] = .5
    target.solve_time_ms = 3.
    target.ik_damping = .05
    target.nullspace_correction_norm = .1
    target.bind_joint_state_provider(lambda: target.position[:6].copy())
    target.bind_joint_velocity_provider(lambda: np.zeros(6))
    target.update(.01)
    assert target.solver_reasons == []
    assert target.solve_time_ms == target.ik_damping == target.nullspace_correction_norm == 0.
    assert np.array_equal(target.achieved_twist,np.zeros(6))
    assert np.array_equal(target.residual_twist,np.zeros(6))
    assert target.ik_solve_count == 0
    assert target.speed_monitor.latest["measurement_valid"]
    assert not target.speed_monitor.latest["enabled"]
