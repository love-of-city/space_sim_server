import pytest
from space_arm_platform.models import OperatorAction, SimulationObservation, SceneInstanceCreate
from space_arm_platform.safety import SafetyController
from space_arm_platform.scene_runtime import SceneRuntimeManager
from space_arm_platform.control_defaults import ZERO_START_TELEOP_PROFILE

def test_angular_action_and_deadman():
    safety=SafetyController(); safety.gripper_type='four_bar_tendon'
    request=OperatorAction(client_sequence=1,client_time_ns='1',deadman=True,gripper_velocity=1.,input_source='keyboard',end_effector_linear_velocity=[0.]*3,end_effector_angular_velocity=[0.]*3)
    action=safety.process('test',request,None)
    assert action.gripper_velocity_m_s == 0
    assert action.gripper_drive_velocity_rad_s == .2
    request.client_sequence=2;request.deadman=False
    assert safety.process('test',request,None).gripper_drive_velocity_rad_s == 0

def test_linkage_observation_rejects_metres():
    values=dict(protocol='space-arm-control/1',type='observation',simulation_id='test',step_id='1',render_frame_id='1',sim_time_ns='1',wall_time_ns='1',applied_action_sequence='0',
        joint_position_rad=[0.]*8,joint_velocity_rad_s=[0.]*8,target_joint_position_rad=[0.]*8,end_effector_position_body_m=[0.]*3,
        arm_joint_position_rad=[0.]*6,arm_joint_velocity_rad_s=[0.]*6,target_arm_joint_position_rad=[0.]*6,
        gripper_type='four_bar_tendon',gripper_drive_position_rad=0.,gripper_drive_velocity_rad_s=0.,target_gripper_drive_position_rad=0.)
    assert SimulationObservation(**values).gripper_position_m is None
    with pytest.raises(ValueError,match='metre'):
        SimulationObservation(**values,gripper_position_m=[0.,0.])

def test_new_scene_uses_linkage_zero_and_direct_start(tmp_path):
    manager=SceneRuntimeManager(None,project_root=tmp_path)
    request=SceneInstanceCreate(template_id='sarm-task-box-20261012',randomization_profile=ZERO_START_TELEOP_PROFILE)
    instance=manager.create_instance(request)
    assert not instance['arm_preparation_required']
    assert instance['randomization']['arm_joint_position_rad'][6:] == [0.,0.]
    request.randomization_profile='teleop-zero-prepare-v2'
    with pytest.raises(ValueError): manager.create_instance(request)
