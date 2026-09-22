"""Preparation lifecycle: true measured arrival, literal joints and fail-closed gates."""
import json
from pathlib import Path
import numpy as np
import pytest
from pydantic import ValidationError
from simulation.arm_preparation import ArmPreparation
from space_arm_platform.control_defaults import DEFAULT_OPERATING_JOINT_DEG, ZERO_START_TELEOP_PROFILE
from space_arm_platform.models import SceneInstanceCreate, OperatorAction
from space_arm_platform.scene_runtime import SceneRuntimeManager
from space_arm_platform.safety import SafetyController


def test_default_initializes_directly_at_operating_pose(tmp_path):
    manager=SceneRuntimeManager(None,project_root=tmp_path)
    req=SceneInstanceCreate(seed=7)
    assert req.randomization_profile==ZERO_START_TELEOP_PROFILE
    instance=manager.create_instance(req)
    assert not instance['arm_preparation_required']
    expected = [np.deg2rad(value) for value in DEFAULT_OPERATING_JOINT_DEG]
    np.testing.assert_allclose(instance['randomization']['arm_joint_position_rad'][:6], expected)
    assert instance['operating_arm_joint_position_deg']==list(DEFAULT_OPERATING_JOINT_DEG)
    assert instance['randomization']['arm_joint_position_rad'][6:]==[.01875]*2
    assert 'ik_initialization' not in instance
    stored = json.loads(Path(instance['config_path']).read_text(encoding='utf-8'))
    assert not stored['arm_preparation_required']
    with pytest.raises(ValueError,match='操作姿态'):
        manager.create_instance(SceneInstanceCreate(initial_arm_joint_position_deg=[0.]*6))
    with pytest.raises(ValueError,match='限位'):
        manager.create_instance(SceneInstanceCreate(operating_arm_joint_position_deg=[0.]*5+[361.]))


def test_action_model_and_safety_preparation_heartbeat():
    request=dict(client_sequence=1,client_time_ns='1',deadman=True,
                 end_effector_linear_velocity=[0.]*3,end_effector_angular_velocity=[0.]*3,
                 arm_preparation=dict(request_id='request',joint_position_deg=list(DEFAULT_OPERATING_JOINT_DEG)))
    safety=SafetyController()
    action=safety.process('operator',OperatorAction(**request),None)
    assert action.arm_preparation.joint_position_deg[-1]==0.
    assert safety.neutral(None,'stop').arm_preparation is None
    request['arm_preparation']['joint_position_deg'][0]=float('nan')
    with pytest.raises(ValidationError):OperatorAction(**request)


def test_live_target_waits_and_resets_with_real_chain():
    pytest.importorskip('Basilisk')
    from simulation import teleop_grasp_unreal as teleop
    root=Path(__file__).resolve().parents[1]
    chain=teleop.SerialChainKinematics.from_mjcf(root/'model/SARM/platform/sarm_platform.xml',base_body='cubesat_bus',joint_names=teleop.ARM_JOINT_NAMES,tool_site='sarm_ee')
    class Client:
        def latest_action(self):
            return dict(deadman=True,server_sequence='1',end_effector_linear_velocity_body_m_s=[.1,0,0],end_effector_angular_velocity_body_rad_s=[0,0,.1],gripper_velocity_m_s=.005),False
    initial=np.array([0.]*6+[.01875]*2)
    target=teleop.CartesianTeleopTarget(initial,Client(),chain,preparation_required=True)
    target.bind_joint_state_provider(lambda:np.zeros(6));target.bind_joint_velocity_provider(lambda:np.zeros(6))
    target.reset(0.)
    for i in range(1,11):
        q,v=target.update(.01*i)
        np.testing.assert_array_equal(q,initial);np.testing.assert_array_equal(v,0.)
    assert target.ik_solve_count==0 and not target.preparation.ready
    target.preparation.status='ready'
    target.update(.11)
    assert target.ik_solve_count==1
    target.reset(.12)
    assert target.preparation.status=='waiting'
    np.testing.assert_array_equal(target.position,initial)
