import copy
import time

import numpy as np
import pytest

from simulation.arm_preparation import ArmPreparation, SmoothJointSegment
from simulation.preparation_contract import validate_preparation_plan
from space_arm_platform.preparation_planning import preparation_ready_for_recording


def setup(goal=None):
    goal = np.array([.2, -.3, .1, .25, -.2, .4] if goal is None else goal)
    plan = dict(schema='arm-preparation-plan/1', strategy='validated-waypoints-v1',
                initial_joint_position_rad=[0.] * 6, goal_joint_position_rad=goal.tolist(),
                waypoints_rad=[goal.tolist()], model_sha256='0' * 64)
    controller = ArmPreparation(True, goal, np.full(6, -6.3), np.full(6, 6.3), np.ones(6), plan=plan)
    action = dict(deadman=True, arm_preparation=dict(request_id='one', joint_position_deg=np.rad2deg(goal).tolist()))
    return controller, action, plan


def test_six_literal_targets_measured_arrival_and_limits():
    controller, action, _ = setup()
    position = np.zeros(6)
    velocity = np.zeros(6)
    previous = velocity.copy()
    for _ in range(4000):
        result = controller.step(.01, action, False, position, position, velocity)
        position, velocity = result
        assert np.max(np.abs(velocity) - controller.speed_limits) < 1e-8
        assert np.max(np.abs(velocity - previous) / .01 - controller.acceleration_limits) < 1e-6
        previous = velocity.copy()
        if controller.ready:
            break
    assert controller.ready
    np.testing.assert_allclose(position, controller.default_goal, atol=1e-10)
    assert controller.telemetry()['held_joints'] == []
    assert position[0] != 0 and position[5] != 0


def test_reference_finishes_but_measured_lag_never_marks_ready():
    controller, action, _ = setup()
    reference = np.zeros(6)
    for _ in range(3000):
        reference, _ = controller.step(.01, action, False, reference, np.zeros(6), np.zeros(6))
        if controller.status == 'failed':
            break
    assert controller.status == 'failed'
    assert not controller.ready


def test_cancel_brakes_and_cannot_replay_or_resume_from_unchecked_pose():
    controller, action, _ = setup()
    position = velocity = np.zeros(6)
    for _ in range(150):
        position, velocity = controller.step(.01, action, False, position, position, velocity)
    assert np.max(np.abs(velocity)) > .01
    previous = velocity.copy()
    position, velocity = controller.step(.01, {}, True, position, position, velocity)
    assert controller.status == 'pausing'
    assert np.max(np.abs(velocity - previous) / .01 - controller.acceleration_limits) < 1e-6
    for _ in range(200):
        position, velocity = controller.step(.01, action, False, position, position, velocity)
    assert controller.status == 'cancelled'
    retry = copy.deepcopy(action)
    retry['arm_preparation']['request_id'] = 'retry'
    controller.step(.01, retry, False, position, position, velocity)
    assert controller.status == 'failed'
    assert '零位' in controller.reason


def test_changed_goal_and_invalid_saved_plan_are_rejected():
    controller, action, plan = setup()
    action['arm_preparation']['joint_position_deg'][5] += 1
    controller.step(.01, action, False, np.zeros(6), np.zeros(6), np.zeros(6))
    assert controller.status == 'failed' and '路径' in controller.reason
    plan['waypoints_rad'][-1][5] = 0
    with pytest.raises(ValueError, match='goal'):
        validate_preparation_plan(plan, np.rad2deg(controller.default_goal).tolist())


def test_tracking_saturation_slows_shared_path_clock():
    controller, _, _ = setup()
    controller.segment = SmoothJointSegment(np.zeros(6), np.ones(6), controller.speed_limits, controller.acceleration_limits)
    progress = 0.
    for _ in range(100):
        progress += controller._advance_clock(.01, np.full(6, .05), np.zeros(6), {'effort_ratio': np.ones(6)})
    assert 0 < progress < .9
    assert controller.clock_scale < .8
    with pytest.raises(ValueError, match='受阻'):
        for _ in range(250):
            controller._advance_clock(.01, np.full(6, .05), np.zeros(6), {'effort_ratio': np.ones(6)})


def test_recording_requires_fresh_ready_telemetry_for_same_scene():
    instance = {'instance_id': 'scene'}
    status = {'connected': True, 'latest_observation': {
        'wall_time_ns': str(time.time_ns()), 'scene_instance_id': 'scene',
        'arm_preparation': {'status': 'ready', 'ready': True}}}
    assert preparation_ready_for_recording(instance, status)
    for key, value in [('wall_time_ns', '1'), ('scene_instance_id', 'previous'),
                       ('arm_preparation', {'status': 'moving', 'ready': False})]:
        invalid = copy.deepcopy(status)
        invalid['latest_observation'][key] = value
        assert not preparation_ready_for_recording(instance, invalid)
    status['connected'] = False
    assert not preparation_ready_for_recording(instance, status)
