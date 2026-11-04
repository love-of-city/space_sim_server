"""Exercise gamepad commands through safety scaling and the real SARM IK chain."""
import ast
import time
from pathlib import Path
from unittest.mock import patch

import numpy as np
import pytest

from space_arm_platform.control_defaults import BALANCED_TELEOP_HOME
from space_arm_platform.models import OperatorAction
from space_arm_platform.safety import ActionRejected, SafetyController
from simulation.control_client import SimulationControlClient
from simulation.online_elbow_ik import OnlineElbowPreference
from simulation.gamepad_control import add_joint6_command
from simulation.serial_chain_kinematics import IkResult, SerialChainKinematics

MODEL = Path(__file__).resolve().parents[1] / 'model/SARM/platform/sarm_ground_target_self_collision.xml'


def request(**kw):
    values = dict(client_sequence=1, client_time_ns='1', deadman=True,
                  end_effector_linear_velocity=[0., 0., 0.],
                  end_effector_angular_velocity=[0., 0., 0.],
                  angular_control_frame='end_effector', joint6_velocity=0., input_source='gamepad')
    return OperatorAction(**(values | kw))


class Client:
    def __init__(self, action):
        self.action = action.model_dump()
        self.stale = False

    def latest_action(self):
        return self.action, self.stale


def target_for(command, mode='ik_pose', preference=True):
    # Keep pure safety/protocol tests available without the native runtime.
    from simulation.teleop_grasp_unreal import ARM_JOINT_NAMES, CartesianTeleopTarget

    applied = SafetyController().process('test', command, None)
    assert SimulationControlClient._valid_action(applied.model_dump())
    client = Client(applied)
    kin = SerialChainKinematics.from_mjcf(MODEL, base_body='cubesat_bus',
                                         joint_names=ARM_JOINT_NAMES, tool_site='sarm_ee')
    target = CartesianTeleopTarget(np.array(BALANCED_TELEOP_HOME), client, kin, ik_mode=mode,
                                   elbow_preference=OnlineElbowPreference(enabled=preference))
    target.reset(0.)
    return target, client, kin


def test_joint6_torque_trial_preserves_other_limits_and_pid_gains():
    # Keep this configuration regression available in CI without native Basilisk.
    source = Path(__file__).resolve().parents[1] / 'simulation/teleop_grasp_unreal.py'
    settings = {
        node.targets[0].id: ast.literal_eval(node.value.args[0])
        for node in ast.parse(source.read_text(encoding='utf-8')).body
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name)
        and node.targets[0].id in {'TELEOP_ARM_KP', 'TELEOP_ARM_KD', 'TELEOP_ARM_TORQUE_LIMIT'}
    }
    assert settings['TELEOP_ARM_TORQUE_LIMIT'] == [2., 2., 2., 1., 1., .70]
    assert settings['TELEOP_ARM_KP'] == [32., 32., 32., 30., 30., 15.]
    assert settings['TELEOP_ARM_KD'] == [2., 2., 2., .7, .5, .25]
    assert SafetyController.JOINT6_MAX_RAD_S == .50


def test_safety_scales_tool_channel_separately_and_clears_all_on_neutral():
    safety = SafetyController()
    r = request(end_effector_angular_velocity=[2., -.5, 0.], joint6_velocity=-2.)
    a = safety.process('test', r, None)
    assert a.end_effector_angular_velocity_body_rad_s == [0., 0., 0.]
    assert a.end_effector_angular_velocity_tool_rad_s == [.5, -.25, 0.]
    assert a.joint6_velocity_rad_s == -.5 and a.limited
    for stopped in (safety.process('other', r.model_copy(update={'deadman': False}), None),
                    safety.neutral(None, 'emergency_stop')):
        assert stopped.joint6_velocity_rad_s == 0.
        assert stopped.end_effector_angular_velocity_tool_rad_s == [0.] * 3
        assert not stopped.allow_reference_recovery
    recovery = request(deadman=False, joint6_velocity=1., allow_reference_recovery=True)
    assert not SafetyController().process('test', recovery, None).allow_reference_recovery
    prep = request(joint6_velocity=1., end_effector_angular_velocity=[1., 0., 0.],
                   arm_preparation={'request_id': 'prepare', 'joint_position_deg': [0.] * 6})
    a = SafetyController().process('test', prep, None)
    assert a.joint6_velocity_rad_s == 0. and a.end_effector_angular_velocity_tool_rad_s == [0.] * 3


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_invalid_direct_wrist_numbers_are_rejected(value):
    with pytest.raises(ActionRejected):
        SafetyController().process('test', request(joint6_velocity=value), None)


def test_backend_timeout_clears_both_gamepad_channels(monkeypatch):
    now = [10.]
    monkeypatch.setattr('space_arm_platform.safety.time.monotonic', lambda: now[0])
    safety = SafetyController(timeout_s=.25)
    safety.process('test', request(joint6_velocity=1.,
        end_effector_angular_velocity=[1., 1., 0.]), None)
    now[0] += .3
    neutral = safety.timeout_action(None)
    assert neutral.reason == 'input_timeout' and not neutral.deadman
    assert neutral.joint6_velocity_rad_s == 0.
    assert neutral.end_effector_angular_velocity_tool_rad_s == [0.] * 3
    assert safety.timeout_action(None) is None


@pytest.mark.parametrize('sign', [-1, 1])
def test_simultaneous_ik_and_direct_wrist_cannot_exceed_combined_speed_limit(sign):
    velocity = np.array([.1, -.1, .2, 0., 0., sign * .4])
    result = IkResult(velocity.copy(), velocity.copy(), np.zeros(6), 6)
    desired = velocity.copy(); desired[5] += sign * .5
    bounded = add_joint6_command(result, np.eye(6), desired, sign * .5,
        np.zeros(6), np.full(6, .5), np.full(6, -3.), np.full(6, 3.), .01)
    assert bounded.joint_velocity_rad_s[5] == pytest.approx(sign * .5)
    np.testing.assert_allclose(bounded.joint_velocity_rad_s, desired * (.5 / .9))
    np.testing.assert_allclose(bounded.residual_twist, desired - bounded.achieved_twist)
    np.testing.assert_array_equal(result.joint_velocity_rad_s, velocity)
    assert {'code': 'joint_speed_limit', 'joints': [6]} in bounded.limit_reasons


@pytest.mark.parametrize('mode', ['ik_pose', 'strict'])
@pytest.mark.parametrize('sign', [-1, 1])
def test_ab_only_changes_j6_target_even_with_posture_preferences_enabled(mode, sign):
    t, client, kin = target_for(request(joint6_velocity=sign), mode)
    original = t.position.copy()
    with patch.object(kin, 'inverse_velocity_ik_pose', side_effect=AssertionError('A/B called IK')), \
         patch.object(kin, 'inverse_velocity_bounded', side_effect=AssertionError('A/B called IK')):
        for step in range(1, 51):
            t.update(step * .01)
            np.testing.assert_array_equal(t.position[:5], original[:5])
            np.testing.assert_array_equal(t.position[6:], original[6:])
            assert 0. < sign * t.velocity[5] <= .5
    assert sign * (t.position[5] - original[5]) > .1
    assert t.ik_solve_count == 0
    client.action['joint6_velocity_rad_s'] = 0.
    held = t.position.copy()
    t.update(.51)
    np.testing.assert_array_equal(t.position, held)
    np.testing.assert_array_equal(t.velocity, np.zeros(8))


@pytest.mark.parametrize('q6', [0., 1.1, -1.7])
@pytest.mark.parametrize('axis,sign,aim_axis,aim_sign', [(0, -1, 1, 1), (1, 1, 0, 1)])
def test_right_stick_aims_in_current_tool_frame_after_wrist_rotation(q6, axis, sign, aim_axis, aim_sign):
    angular = [0.] * 3; angular[axis] = sign * .2
    t, _, kin = target_for(request(end_effector_angular_velocity=angular), preference=False)
    t.position[5] = q6
    pos, rotation = kin.forward(t.position[:6])
    t.target_tool_position, t.target_tool_rotation = pos.copy(), rotation.copy()
    t.update(.01)
    np.testing.assert_allclose(t.raw_operator_twist[3:], rotation @ (np.array(angular) * .5), atol=1e-12)
    new_pos, new_rotation = kin.forward(t.position[:6])
    # Local +Z is the pointing direction. Right stick right turns it to +Y;
    # right stick up turns it to +X, regardless of wrist roll in the body frame.
    direction_delta = rotation.T @ (new_rotation[:, 2] - rotation[:, 2])
    assert direction_delta[aim_axis] * aim_sign > 0.
    assert np.linalg.norm(new_pos - pos) < 1e-6


@pytest.mark.parametrize('sign', [-1, 1])
def test_direct_joint_limit_stops_outward_motion_and_allows_reverse(sign):
    t, client, _ = target_for(request(joint6_velocity=sign))
    boundary = t.joint_max[5] if sign > 0 else t.joint_min[5]
    t.position[5] = boundary - sign * .0001
    for step in range(1, 20): t.update(step * .01)
    assert t.position[5] == pytest.approx(boundary)
    assert t.velocity[5] == 0.
    assert {'code': 'joint_position_limit', 'joints': [6]} in t.solver_reasons
    client.action['joint6_velocity_rad_s'] *= -1
    for step in range(20, 50): t.update(step * .01)
    assert (boundary - t.position[5]) * sign > .01


def test_direct_joint_still_obeys_tracking_protection():
    t, client, _ = target_for(request(joint6_velocity=1.))
    measured = t.position[:6].copy(); measured[5] -= .06
    t.bind_joint_state_provider(lambda: measured)
    held = t.position.copy()
    t.update(.01)
    np.testing.assert_array_equal(t.position, held)
    assert t.governor_state == 'tracking_limited' and 6 in t.governor_limited_joints
    client.action['joint6_velocity_rad_s'] = -.5
    t.update(.02)
    assert t.velocity[5] < 0.


@pytest.mark.parametrize('stop', ['stale', 'deadman'])
def test_mixed_tool_and_joint_commands_stop_together_and_reset(stop):
    from simulation.teleop_grasp_unreal import ARM_JOINT_VELOCITY_LIMIT

    t, client, kin = target_for(request(joint6_velocity=1., end_effector_angular_velocity=[1., 1., 0.],
                                      end_effector_linear_velocity=[.5, 0., 0.]))
    for step in range(1, 40):
        before = t.position[:6].copy()
        t.update(step * .01)
        assert np.all(np.abs(t.velocity[:6]) <= ARM_JOINT_VELOCITY_LIMIT + 1e-12)
        np.testing.assert_allclose(t.achieved_twist, kin.jacobian(before) @ t.velocity[:6], atol=1e-10)
    held = t.position.copy()
    if stop == 'stale': client.stale = True
    else: client.action['deadman'] = False
    t.update(.4)
    np.testing.assert_array_equal(t.position, held)
    np.testing.assert_array_equal(t.velocity, np.zeros(8))
    assert t.commanded_joint6_velocity == 0.
    t.reset(.4)
    assert t.commanded_joint6_velocity == 0.


def test_native_receiver_checks_new_fields_and_clears_them_on_timeout():
    # Old packets with no new channels retain their existing meaning.
    old = SimulationControlClient._neutral_action()
    del old['end_effector_angular_velocity_tool_rad_s']; del old['joint6_velocity_rad_s']
    assert SimulationControlClient._valid_action(old)
    for bad in [float('nan'), 1., True, '0.1']:
        assert not SimulationControlClient._valid_action(old | {'joint6_velocity_rad_s': bad})
    for bad in [[0., 0.], [0., float('inf'), 0.], [0., True, 0.]]:
        assert not SimulationControlClient._valid_action(old | {'end_effector_angular_velocity_tool_rad_s': bad})
    client = SimulationControlClient('127.0.0.1', 1, 'test')
    client._action = SafetyController().process('test', request(joint6_velocity=1.,
        end_effector_angular_velocity=[.5, 0., 0.]), None).model_dump()
    client._received_monotonic = time.monotonic() - 1.
    action, stale = client.latest_action()
    assert stale and not action['deadman']
    assert action['joint6_velocity_rad_s'] == 0.
    assert action['end_effector_angular_velocity_tool_rad_s'] == [0.] * 3
