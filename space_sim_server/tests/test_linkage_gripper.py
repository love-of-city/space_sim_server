from simulation.sarm_model_profile import load_model_profile
from simulation.gripper_control import LinkageGripperCommand
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def test_validation_rejects_disconnected_tendon():
    import pytest
    from tools.validate_sarm_20261012_platform import validate_cycles
    samples=[dict(time_s=i/10,target_rad=.2*min(i/10%10,10-i/10%10),positions=[0.]*8,coupling_error_rad=0.) for i in range(101,401)]
    with pytest.raises(AssertionError,match='tracking'):
        validate_cycles(samples)

def test_linkage_command_limits_and_hold():
    drive = LinkageGripperCommand()
    assert drive.advance(1, 1, True) == .2
    assert drive.advance(-1, 1, False) == .2
    assert drive.advance(1, 100, True) == 1
    assert drive.advance(-1, 100, True) == 0

def test_profile_keeps_passive_joints_out_of_control():
    p = load_model_profile(ROOT/'model/SARM/platform/sarm_20261012.xml')
    assert p.linkage
    assert p.joints[5] == ('ee_base', 'joint6')
    assert p.joint_names[-2:] == ('ee_link11_joint', 'ee_link12_joint')
    assert len(p.arm_actuators) == 6
    assert p.gripper_actuator == 'ee_fingers_position'

def test_live_target_opposes_linkages_and_holds_on_release():
    import numpy as np
    from simulation.teleop_grasp_unreal import CartesianTeleopTarget
    from simulation.serial_chain_kinematics import SerialChainKinematics
    from space_arm_platform.joint_limits import load_joint_limits
    from types import SimpleNamespace
    model=ROOT/'model/SARM/platform/sarm_20261012.xml'
    p=load_model_profile(model)
    k=SerialChainKinematics.from_mjcf(model,base_body='cubesat_bus',joint_names=p.joint_names[:6],tool_site='sarm_ee')
    action=dict(deadman=True,server_sequence='1',end_effector_linear_velocity_body_m_s=[0.]*3,
        end_effector_angular_velocity_body_rad_s=[0.]*3,gripper_type='four_bar_tendon',gripper_drive_velocity_rad_s=.2)
    client=SimpleNamespace(latest_action=lambda:(action,False))
    target=CartesianTeleopTarget(np.zeros(8),client,k,joint_limits=load_joint_limits(model,p.joint_names))
    target.update(0.);q,v=target.update(.01)
    assert q[6] > 0 and q[7] == -q[6]
    held=q.copy();action['deadman']=False
    q,v=target.update(.02)
    assert np.array_equal(q,held) and np.all(v[6:] == 0)
