"""Regression coverage for removing the custom QP and keeping telemetry passive."""
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from simulation.teleop_grasp_unreal import (
    CartesianTeleopTarget, SerialChainKinematics, ARM_JOINT_NAMES,
    SARM_JOINT_NAMES, IK_MODES, ARM_JOINT_VELOCITY_LIMIT,
)
from simulation.serial_chain_kinematics import _uniform_limit_reasons
from space_arm_platform.control_defaults import BALANCED_TELEOP_HOME
from space_arm_platform.joint_limits import load_joint_limits

ROOT = Path(__file__).resolve().parents[1]


class Client:
    def __init__(self, command=(.02, 0, 0, 0, 0, 0)):
        self.command = np.array(command, dtype=float)
        self.deadman, self.stale = True, False

    def latest_action(self):
        return dict(deadman=self.deadman, server_sequence="1",
            end_effector_linear_velocity_body_m_s=self.command[:3].tolist(),
            end_effector_angular_velocity_body_rad_s=self.command[3:].tolist(),
            gripper_velocity_m_s=0.), self.stale


def target_for(command=(.02,0,0,0,0,0), mode="ik_pose"):
    model=ROOT / "model/SARM/platform/sarm_ground_target_self_collision.xml"
    kin=SerialChainKinematics.from_mjcf(model,base_body="cubesat_bus",joint_names=ARM_JOINT_NAMES,tool_site="sarm_ee")
    client=Client(command)
    target=CartesianTeleopTarget(np.array(BALANCED_TELEOP_HOME),client,kin,ik_mode=mode,
                                joint_limits=load_joint_limits(model,SARM_JOINT_NAMES))
    target.reset(0.)
    return target,client


@pytest.mark.parametrize("mode", IK_MODES)
@pytest.mark.parametrize("reason", ["release", "timeout"])
def test_release_holds_joint_reference_despite_large_measured_pose_error(mode,reason):
    target,client=target_for(mode=mode)
    actual=target.position[:6].copy()
    target.bind_joint_state_provider(lambda: actual.copy())
    target.bind_joint_velocity_provider(lambda: np.zeros(6))
    for i in range(1,31): target.update(i*.01)
    held=target.position.copy()
    goal=(target.target_tool_position.copy(),target.target_tool_rotation.copy())
    actual[4] += .4
    if reason == "timeout": client.stale=True
    else: client.deadman=False
    for i in range(31,61):
        target.update(i*.01)
        assert np.array_equal(target.position,held)
        assert np.array_equal(target.velocity[:6],np.zeros(6))
        assert np.array_equal(target.target_tool_position,goal[0])
        assert np.array_equal(target.target_tool_rotation,goal[1])
        assert target.solver_status == "holding"
        assert not target.speed_monitor.latest["linear"]["warning"]
    # Resume must not snap/rebase the goal to the measured, displaced plant.
    client.stale,client.deadman=False,True
    actual[4] -= .8
    client.command[0] = -.02
    target.update(.61)
    assert np.linalg.norm(target.position-held) > 0
    assert np.linalg.norm(target.position-held) < .02
    assert np.linalg.norm(target.target_tool_rotation-goal[1]) < .01


@pytest.mark.parametrize("axis", range(6))
@pytest.mark.parametrize("direction", [-1,1])
def test_measured_error_and_low_speed_warning_never_gate_any_axis(axis,direction):
    cmd=np.zeros(6); cmd[axis]=direction*(.02 if axis<3 else .1)
    free,client=target_for(cmd)
    blocked,_=target_for(cmd)
    measured=blocked.position[:6].copy(); measured[4] += .5
    blocked.bind_joint_state_provider(lambda: measured.copy())
    blocked.bind_joint_velocity_provider(lambda: np.zeros(6))
    for i in range(1,101):
        free.update(i*.01); blocked.update(i*.01)
        assert np.array_equal(free.position,blocked.position)
        assert np.array_equal(free.velocity,blocked.velocity)
        assert np.all(blocked.position >= blocked.joint_min)
        assert np.all(blocked.position <= blocked.joint_max)
    channel="linear" if axis<3 else "angular"
    assert blocked.speed_monitor.latest[channel]["warning"]
    assert not free.speed_monitor.latest["measurement_valid"]
    blocked.reset(0.)
    assert blocked.speed_monitor.latest[channel] is None


def test_combination_and_reversal_have_no_acceleration_or_braking_state_machine():
    target,client=target_for((.01,-.01,.01,.03,-.02,.02))
    for sign in (1.,-1.,1.):
        client.command=np.array([.01,-.01,.01,.03,-.02,.02])*sign
        for _ in range(20):
            target.update(target.last_sim_seconds+.01)
            assert target.solver_status != "braking"
            assert np.max(np.abs(target.velocity[:6])/ARM_JOINT_VELOCITY_LIMIT) <= 1.+1e-10
            assert not ({r["code"] for r in target.solver_reasons} &
                        {"pose_correction","reference_lead_limit","cartesian_lead_limit","solver_braking","joint_acceleration_limit"})


def test_default_no_longer_imports_qp_packages_and_old_override_fails_clearly():
    assert IK_MODES == ("ik_pose","strict")
    # Import and instantiate with OSQP/SciPy explicitly unavailable.
    script = r'''
import sys
sys.path[:0] = ['.', 'backend']
sys.modules['osqp'] = None
sys.modules['scipy'] = None
import numpy as np
from simulation.teleop_grasp_unreal import CartesianTeleopTarget, SerialChainKinematics, ARM_JOINT_NAMES
from space_arm_platform.control_defaults import BALANCED_TELEOP_HOME
kin=SerialChainKinematics.from_mjcf('model/SARM/platform/sarm_platform.xml',base_body='cubesat_bus',joint_names=ARM_JOINT_NAMES,tool_site='sarm_ee')
t=CartesianTeleopTarget(np.array(BALANCED_TELEOP_HOME),None,kin)
assert t.ik_mode == 'ik_pose'
assert not hasattr(t,'constrained_solver')
'''
    result=subprocess.run([sys.executable,"-c",script],cwd=ROOT,capture_output=True,text=True)
    assert result.returncode == 0,result.stderr
    with pytest.raises(ValueError,match="unsupported IK mode"):
        target_for(mode="constrained")


def test_limit_evidence_identifies_actual_outward_step_not_idle_boundary_joint():
    step=np.array([-.3,0.,.2,0.,0.,0.])
    lower=-np.ones(6); upper=np.ones(6)
    lower[0]=0.; lower[1]=0.
    reasons=_uniform_limit_reasons(step,lower,upper,np.ones(6),0.)
    assert reasons == ({"code":"joint_position_limit","joints":[1]},)
    assert _uniform_limit_reasons(np.array([2.,0,0,0,0,0]),-np.ones(6),np.ones(6),np.ones(6),.5) == (
        {"code":"joint_speed_limit","joints":[1]},)


def test_damped_output_explains_reduction_without_changing_legacy_math():
    target,_=target_for()
    q=np.array([0.,-.1790243,.2159404,-.0368382,0.,0.])
    command=np.array([.05,0,0,0,0,0])
    kin=target.kinematics
    result=kin.inverse_velocity_ik_pose(q,command,joint_velocity_limits=ARM_JOINT_VELOCITY_LIMIT)
    J=kin.jacobian(q)
    expected=J.T @ np.linalg.solve(J@J.T+result.damping**2*np.eye(6),command)
    expected*=min(1.,min(ARM_JOINT_VELOCITY_LIMIT/(np.abs(expected)+1e-30)))
    assert np.allclose(result.joint_velocity_rad_s,expected,atol=1e-12)
