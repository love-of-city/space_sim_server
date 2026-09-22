"""Online elbow preference from zero and arbitrary initial states, not home bias."""
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

import numpy as np
import pytest

from simulation.online_elbow_ik import (
    OnlineElbowPreference, ElbowDeviationState, apply_online_elbow_preference,
)
from simulation.serial_chain_kinematics import SerialChainKinematics
from space_arm_platform.control_defaults import BALANCED_TELEOP_HOME
from space_arm_platform.joint_limits import load_joint_limits

MODEL = Path(__file__).resolve().parents[1] / 'model/SARM/platform/sarm_platform.xml'
NAMES = tuple(f'joint{i}' for i in range(1, 7))
SPEEDS = np.array([.7,.7,.7,.9,1,1])


@pytest.fixture(scope='module')
def chain():
    return SerialChainKinematics.from_mjcf(MODEL, base_body='cubesat_bus', joint_names=NAMES, tool_site='sarm_ee')


def options(dt=.01):
    limits = load_joint_limits(MODEL, NAMES)
    return dict(joint_velocity_limits=SPEEDS, joint_position_min=np.array(limits.lower),
                joint_position_max=np.array(limits.upper), dt=dt)


def step(chain, q, command, pref=OnlineElbowPreference(), state=None, **kwargs):
    opts = options(); opts.update(kwargs)
    base = chain.inverse_velocity_ik_pose(q, command, **opts)
    result, diag = apply_online_elbow_preference(chain, q, command, base,
        preference=pref, deviation_state=state, **opts)
    return base, result, diag


@pytest.mark.parametrize('q', [np.zeros(6), np.array(BALANCED_TELEOP_HOME[:6]),
    np.array([.3, .5, -.4, -.2, .1, .2])])
@pytest.mark.parametrize('up', [(0,0,1), (1,2,3)])
def test_analytic_height_gradient_matches_finite_difference(chain,q,up):
    height, gradient = chain.relative_joint_height(q, up_axis=up)
    for i in range(6):
        delta = np.eye(6)[i]*1e-6
        numeric = (chain.relative_joint_height(q+delta,up_axis=up)[0]
                   -chain.relative_joint_height(q-delta,up_axis=up)[0])/(2e-6)
        assert gradient[i] == pytest.approx(numeric,abs=1e-8)
    assert np.isfinite(height)


@pytest.mark.parametrize('q', [np.zeros(6), np.array(BALANCED_TELEOP_HOME[:6]),
    np.array([.3, .5, -.4, -.2, .1, .2])])
def test_online_correction_prefers_height_at_singular_and_full_rank_poses(chain,q):
    command = np.array([.02, 0,0,0,0,0])
    base, result, diag = step(chain,q,command)
    assert diag.status == 'active'
    gradient = chain.relative_joint_height(q)[1]
    assert gradient @ (result.joint_velocity_rad_s-base.joint_velocity_rad_s) > 0
    assert diag.linear_disturbance_m_s <= diag.linear_budget_m_s + 1e-12
    assert diag.angular_disturbance_rad_s <= diag.angular_budget_rad_s + 1e-12
    assert np.max(np.abs(result.joint_velocity_rad_s)) <= 1
    np.testing.assert_allclose(result.achieved_twist,chain.jacobian(q) @ result.joint_velocity_rad_s,atol=1e-12)
    np.testing.assert_allclose(result.residual_twist,command-result.achieved_twist,atol=1e-12)


@pytest.mark.parametrize('wrist_enabled', [False, True])
def test_zero_start_raises_online_without_replacing_initial_joints(chain,wrist_enabled):
    q = np.zeros(6); free = q.copy()
    state = ElbowDeviationState(); command = np.array([.02,0,0,0,0,0.])
    for _ in range(200):
        base,result,diag = step(chain,q,command,state=state,pref=OnlineElbowPreference(wrist_enabled=wrist_enabled,joint3_enabled=False))
        q += .01*result.joint_velocity_rad_s
        free += .01*chain.inverse_velocity_ik_pose(free,command,**options()).joint_velocity_rad_s
        assert diag.position_offset_m <= .002 + 1e-10
        assert diag.orientation_offset_rad <= np.pi/180 + 1e-10
    # Coupled preference shares the correction budget with wrist-down, so it
    # need not raise the elbow as quickly as the previous elbow-only policy.
    assert chain.relative_joint_height(q)[0] > (.025 if wrist_enabled else .05)
    assert chain.relative_joint_height(q)[0] > chain.relative_joint_height(free)[0]+.02
    if wrist_enabled:
        assert chain.relative_joint_height(q,shoulder_joint='joint6',elbow_joint='joint4')[0] > .012


def test_accumulated_budget_limits_persistent_preference_not_reset_per_tick(chain):
    pref = OnlineElbowPreference(maximum_position_offset_m=1e-5,maximum_orientation_offset_rad=1e-4)
    state = ElbowDeviationState(); q = np.zeros(6); command=np.array([.02,0,0,0,0,0.])
    limited = False
    for _ in range(150):
        _, result, diag = step(chain,q,command,pref,state)
        q += .01*result.joint_velocity_rad_s
        limited |= diag.scale < .01
        assert diag.position_offset_m <= pref.maximum_position_offset_m + 1e-10
        assert diag.orientation_offset_rad <= pref.maximum_orientation_offset_rad + 1e-10
        p,r=chain.forward(q)
        assert np.linalg.norm(p-state.position) <= pref.maximum_position_offset_m+1e-10
    assert limited


@pytest.mark.parametrize('command', [np.zeros(6), np.array([.02,0,0,0,0,0])])
def test_disabled_is_exact_legacy_and_zero_command_does_not_self_move(chain,command):
    q = np.zeros(6)
    base, result, diag = step(chain,q,command,OnlineElbowPreference(enabled=False))
    assert result is base
    if not np.any(command):
        base,result,diag=step(chain,q,command)
        np.testing.assert_array_equal(result.joint_velocity_rad_s,np.zeros(6))
        assert diag.status == 'idle'


def test_tiny_commands_cannot_enable_large_nullspace_motion(chain):
    q = np.zeros(6)
    for magnitude in (1e-4,1e-8,1e-12):
        base,result,diag=step(chain,q,np.array([magnitude,0,0,0,0,0]))
        delta=result.joint_velocity_rad_s-base.joint_velocity_rad_s
        assert np.max(np.abs(delta)) <= .15*magnitude/.05 + 1e-14


def test_preferred_height_satisfied_does_not_pull_toward_initial_pose(chain):
    q=np.array([-.8231,-2.1952,-.4447,1.0316,-.9233,4.7959])
    base,result,diag=step(chain,q,np.array([.02,0,0,0,0,0]))
    assert diag.status == 'shape_satisfied'
    assert result is base


def test_limits_are_not_relaxed_by_extra_correction(chain):
    q=np.zeros(6); lo=np.array([-np.pi]*5+[-2*np.pi]);hi=-lo
    lo[1]=0 # raising in this geometry wants negative shoulder velocity
    base,result,diag=step(chain,q,np.array([.02,0,0,0,0,0]),joint_position_min=lo,joint_position_max=hi)
    assert np.all(q+.01*result.joint_velocity_rad_s >= lo-1e-12)
    assert np.all(q+.01*result.joint_velocity_rad_s <= hi+1e-12)
    assert np.all(np.abs(result.joint_velocity_rad_s) <= SPEEDS+1e-12)


@pytest.mark.parametrize('args', [dict(regularization=0),dict(up_axis=(0,0,0)),
    dict(maximum_linear_disturbance_m_s=-1),dict(preferred_height_m=float('nan')),dict(relative_disturbance=2)])
def test_invalid_config_rejected(args):
    with pytest.raises(ValueError): OnlineElbowPreference(**args)


def test_low_level_requires_valid_base_and_timestep(chain):
    q=np.zeros(6);cmd=np.array([.02,0,0,0,0,0]);opts=options()
    base=chain.inverse_velocity_ik_pose(q,cmd,**opts)
    with pytest.raises(ValueError):
        apply_online_elbow_preference(chain,q,cmd,base,**{**opts,'dt':0})
    with pytest.raises(ValueError):
        apply_online_elbow_preference(chain,q,cmd,replace(base,joint_velocity_rad_s=np.ones(6)*100),**opts)


@pytest.mark.parametrize('reason', ['release', 'timeout', 'gripper'])
@pytest.mark.parametrize('joint3_enabled', [False, True])
def test_live_zero_start_release_resume_keeps_budget_and_reset_clears_it(chain,reason,joint3_enabled):
    from simulation.teleop_grasp_unreal import CartesianTeleopTarget
    class Client:
        enabled=True
        stale=False
        command=[.02,0,0,0,0,0]
        grip=0.
        def latest_action(self):
            return dict(deadman=self.enabled,server_sequence='1',
                end_effector_linear_velocity_body_m_s=self.command[:3],
                end_effector_angular_velocity_body_rad_s=self.command[3:],
                gripper_velocity_m_s=self.grip),self.stale
    client=Client(); initial=np.r_[np.zeros(6),.01875,.01875]
    target=CartesianTeleopTarget(initial,client,chain,elbow_preference=OnlineElbowPreference(joint3_enabled=joint3_enabled))
    target.reset(0.)
    np.testing.assert_array_equal(target.position,initial)
    for i in range(1,101): target.update(i*.01)
    assert chain.relative_joint_height(target.position[:6])[0] > (.001 if joint3_enabled else .01)
    if joint3_enabled:
        assert target.position[2] < 0 # New angle preference takes part of the same budget.
    held=target.position.copy()
    anchor=target.elbow_deviation.position.copy()
    if reason == 'release': client.enabled=False
    elif reason == 'timeout': client.stale=True
    else: client.command=[0.]*6;client.grip=.005
    for i in range(101,121):
        target.update(i*.01)
        np.testing.assert_array_equal(target.position[:6],held[:6])
        np.testing.assert_array_equal(target.elbow_deviation.position,anchor)
        assert target.elbow_diagnostics.status == 'idle'
        assert target.elbow_diagnostics.correction_norm_rad_s == 0
    client.enabled=True;client.stale=False;client.command=[.02,0,0,0,0,0]
    target.update(1.21)
    assert target.elbow_deviation.position is not None
    assert np.linalg.norm(target.position[:6]-held[:6]) <= .02
    target.reset(0.)
    np.testing.assert_array_equal(target.position,initial)
    assert target.elbow_deviation.position is None


def test_live_policy_switch_and_strict_bypass(chain,monkeypatch):
    from simulation.teleop_grasp_unreal import CartesianTeleopTarget
    monkeypatch.setenv('SPACE_SIM_ONLINE_ELBOW_MODE','off')
    initial=np.r_[np.zeros(6),.01875,.01875]
    assert not CartesianTeleopTarget(initial,None,chain).elbow_preference.enabled
    monkeypatch.setenv('SPACE_SIM_ONLINE_ELBOW_MODE','bad')
    with pytest.raises(ValueError): CartesianTeleopTarget(initial,None,chain)
    monkeypatch.setenv('SPACE_SIM_ONLINE_ELBOW_MODE','prefer')
    class Client:
        def latest_action(self):
            return dict(deadman=True,server_sequence='1',
                end_effector_linear_velocity_body_m_s=[0,0,.01],
                end_effector_angular_velocity_body_rad_s=[0,0,0],gripper_velocity_m_s=0),False
    target=CartesianTeleopTarget(initial,Client(),chain,ik_mode='strict')
    target.reset(0.)
    with patch('simulation.teleop_grasp_unreal.apply_online_elbow_preference',side_effect=AssertionError):
        target.update(.01)
    assert not target.elbow_diagnostics.enabled


@pytest.mark.parametrize('q', [np.zeros(6), np.array([.3, .5, -.4, -.2, .1, .2]),
    np.array([-.8231,-2.1952,-.4447,3.,-.9233,4.7959])])
@pytest.mark.parametrize('up', [(0,0,1), (1,2,3)])
def test_wrist_drop_gradient_is_joint4_minus_joint6_geometry(chain,q,up):
    drop, gradient=chain.relative_joint_height(q,shoulder_joint='joint6',elbow_joint='joint4',up_axis=up)
    origins=chain.joint_origins(q);axis=np.array(up,float);axis/=np.linalg.norm(axis)
    assert drop == pytest.approx(axis @ (origins[3]-origins[5]))
    for i in range(6):
        delta=np.eye(6)[i]*1e-6
        numeric=(chain.relative_joint_height(q+delta,shoulder_joint='joint6',elbow_joint='joint4',up_axis=up)[0]
            -chain.relative_joint_height(q-delta,shoulder_joint='joint6',elbow_joint='joint4',up_axis=up)[0])/(2e-6)
        assert gradient[i] == pytest.approx(numeric,abs=1e-8)
    # Turning joint6 does not move its own joint centre.
    assert gradient[5] == 0


def test_wrist_keeps_working_when_elbow_already_high(chain):
    q=np.array([-.8231,-2.1952,-.4447,3.,-.9233,4.7959])
    command=np.array([.02,0,0,0,0,0])
    base, old, old_diag=step(chain,q,command,OnlineElbowPreference(wrist_enabled=False,joint3_enabled=False))
    assert old is base and old_diag.status == 'height_satisfied'
    base, result, diag=step(chain,q,command)
    assert diag.height_m > .3 and diag.wrist_drop_m < 0
    assert not diag.elbow_objective_active and diag.wrist_objective_active
    assert diag.status == 'active'
    base_drop=chain.relative_joint_height(q+.01*base.joint_velocity_rad_s,
        shoulder_joint='joint6',elbow_joint='joint4')[0]
    assert diag.predicted_wrist_drop_m > base_drop
    assert diag.predicted_posture_cost < diag.base_posture_cost


def test_wrist_satisfied_does_not_drive_it_ever_lower(chain):
    q=np.array(BALANCED_TELEOP_HOME[:6]);command=np.array([.02,0,0,0,0,0])
    _, elbow_only, _=step(chain,q,command,OnlineElbowPreference(wrist_enabled=False))
    _, coupled, diag=step(chain,q,command)
    assert diag.wrist_drop_m > .05 and not diag.wrist_objective_active
    np.testing.assert_allclose(coupled.joint_velocity_rad_s,elbow_only.joint_velocity_rad_s,atol=1e-12)


def test_two_goals_share_one_quadratic_and_one_scale(chain):
    import json
    q=np.zeros(6);command=np.array([.02,0,0,0,0,0]);pref=OnlineElbowPreference(joint3_enabled=False)
    base,result,diag=step(chain,q,command,pref)
    assert diag.elbow_objective_active and diag.wrist_objective_active
    assert diag.status == 'active'
    J=chain.jacobian(q);J[3:]*=pref.characteristic_length_m
    H=J.T@J+pref.regularization**2*np.eye(6);rhs=np.zeros(6)
    pairs=[('joint2','joint3',pref.preferred_height_m,pref.height_gain_s,
            pref.maximum_height_speed_m_s,pref.posture_weight),
           ('joint6','joint4',pref.preferred_wrist_drop_m,pref.wrist_height_gain_s,
            pref.maximum_wrist_drop_speed_m_s,pref.wrist_weight)]
    for a,b,target,gain,max_rate,weight in pairs:
        h,g=chain.relative_joint_height(q,shoulder_joint=a,elbow_joint=b)
        deficit=min(max_rate,gain*(target-h))-g@base.joint_velocity_rad_s
        H+=weight*np.outer(g,g);rhs+=weight*deficit*g
    raw=np.linalg.solve(H,rhs)
    np.testing.assert_allclose(result.joint_velocity_rad_s-base.joint_velocity_rad_s,diag.scale*raw,atol=1e-12)
    assert diag.linear_disturbance_m_s <= .002+1e-12
    assert diag.angular_disturbance_rad_s <= .01+1e-12
    # The live snapshot sends dataclass fields via vars(); no numpy.bool_ leaks.
    json.dumps(vars(diag),allow_nan=False)


def test_random_valid_steps_improve_combined_shape_cost_and_share_budgets(chain):
    rng=np.random.default_rng(20);active=0
    pref=OnlineElbowPreference()
    def score(q):
        h=chain.relative_joint_height(q)[0]
        d=chain.relative_joint_height(q,shoulder_joint='joint6',elbow_joint='joint4')[0]
        angle_error=pref.joint3_length_scale_m/pref.joint3_angle_scale_rad*max(0,q[2]+pref.joint3_negative_margin_rad)
        return pref.posture_weight*max(0,.3-h)**2+pref.wrist_weight*max(0,.05-d)**2+pref.joint3_weight*angle_error**2
    for _ in range(40):
        q=rng.uniform(-2.5,2.5,6);command=rng.uniform(-.02,.02,6)
        base,result,diag=step(chain,q,command,pref)
        assert np.all(np.abs(result.joint_velocity_rad_s) <= SPEEDS+1e-12)
        if diag.status == 'active':
            active+=1
            assert score(q+.01*result.joint_velocity_rad_s) < score(q+.01*base.joint_velocity_rad_s)
            assert diag.linear_disturbance_m_s <= diag.linear_budget_m_s+1e-12
            assert diag.angular_disturbance_rad_s <= diag.angular_budget_rad_s+1e-12
            assert diag.position_offset_m <= pref.maximum_position_offset_m+1e-12
            assert diag.orientation_offset_rad <= pref.maximum_orientation_offset_rad+1e-12
    assert active > 5


@pytest.mark.parametrize('kwargs', [dict(wrist_enabled=1),dict(preferred_wrist_drop_m=-.1),
    dict(wrist_weight=0),dict(wrist_height_gain_s=float('nan')),dict(maximum_wrist_drop_speed_m_s=0),
    dict(wrist_upper_joint='joint6'),dict(wrist_lower_joint='')])
def test_invalid_wrist_preferences_fail_clearly(kwargs):
    with pytest.raises(ValueError): OnlineElbowPreference(**kwargs)


def test_wrist_only_switch_does_not_disable_elbow_and_invalid_value_fails(chain,monkeypatch):
    from simulation.teleop_grasp_unreal import CartesianTeleopTarget
    q=np.r_[np.zeros(6),.01875,.01875]
    monkeypatch.setenv('SPACE_SIM_ONLINE_ELBOW_MODE','prefer')
    monkeypatch.setenv('SPACE_SIM_ONLINE_WRIST_MODE','off')
    target=CartesianTeleopTarget(q,None,chain)
    assert target.elbow_preference.enabled and not target.elbow_preference.wrist_enabled
    monkeypatch.setenv('SPACE_SIM_ONLINE_WRIST_MODE','prefer')
    assert CartesianTeleopTarget(q,None,chain).elbow_preference.wrist_enabled
    monkeypatch.setenv('SPACE_SIM_ONLINE_WRIST_MODE','oops')
    with pytest.raises(ValueError,match='SPACE_SIM_ONLINE_WRIST_MODE'):
        CartesianTeleopTarget(q,None,chain)


@pytest.mark.parametrize('angle', [0., .2, .8])
def test_joint3_prefers_negative_without_turning_angle_into_hard_limit(chain,angle):
    q=np.array([0,-1.6,angle,-.3,.2,.2]);command=np.array([.02,0,0,0,0,0])
    before=q.copy();opts=options();before_low=opts['joint_position_min'].copy();before_high=opts['joint_position_max'].copy()
    base,result,diag=step(chain,q,command)
    assert diag.status == 'active' and diag.joint3_objective_active
    assert not diag.elbow_objective_active and not diag.wrist_objective_active
    assert result.joint_velocity_rad_s[2] < base.joint_velocity_rad_s[2]
    assert diag.predicted_joint3_rad == pytest.approx(q[2]+.01*result.joint_velocity_rad_s[2])
    assert diag.predicted_posture_cost < diag.base_posture_cost
    np.testing.assert_array_equal(q,before)
    np.testing.assert_array_equal(options()['joint_position_min'],before_low)
    np.testing.assert_array_equal(options()['joint_position_max'],before_high)
    assert before_high[2] > 3 # positive positions remain mechanically legal


def test_joint3_negative_margin_does_not_pull_further_negative(chain):
    q=np.array([0,-1.6,-.4,.2,.2,.2]);cmd=np.array([.02,0,0,0,0,0])
    base,result,diag=step(chain,q,cmd)
    assert result is base and diag.status == 'shape_satisfied'
    assert not diag.joint3_objective_active


def test_three_goals_share_one_normalized_quadratic_and_scale(chain):
    import json
    q=np.zeros(6);cmd=np.array([.02,0,0,0,0,0]);pref=OnlineElbowPreference()
    base,result,diag=step(chain,q,cmd,pref)
    assert diag.elbow_objective_active and diag.wrist_objective_active and diag.joint3_objective_active
    J=chain.jacobian(q);J[3:]*=pref.characteristic_length_m
    H=J.T@J+pref.regularization**2*np.eye(6);rhs=np.zeros(6)
    h,g=chain.relative_joint_height(q)
    d,dg=chain.relative_joint_height(q,shoulder_joint='joint6',elbow_joint='joint4')
    factor=pref.joint3_length_scale_m/pref.joint3_angle_scale_rad
    g3=np.array([0.,0.,-factor,0.,0.,0.])
    goals=[(h,g,pref.preferred_height_m,pref.height_gain_s,pref.maximum_height_speed_m_s,pref.posture_weight),
           (d,dg,pref.preferred_wrist_drop_m,pref.wrist_height_gain_s,pref.maximum_wrist_drop_speed_m_s,pref.wrist_weight),
           (-factor*q[2],g3,factor*pref.joint3_negative_margin_rad,pref.joint3_gain_s,
            factor*pref.maximum_joint3_preference_speed_rad_s,pref.joint3_weight)]
    for value,grad,target,gain,max_rate,weight in goals:
        deficit=min(max_rate,gain*(target-value))-grad@base.joint_velocity_rad_s
        H+=weight*np.outer(grad,grad);rhs+=weight*deficit*grad
    np.testing.assert_allclose(result.joint_velocity_rad_s-base.joint_velocity_rad_s,
        diag.scale*np.linalg.solve(H,rhs),atol=1e-12)
    assert diag.linear_disturbance_m_s <= diag.linear_budget_m_s+1e-12
    assert diag.angular_disturbance_rad_s <= diag.angular_budget_rad_s+1e-12
    json.dumps(vars(diag),allow_nan=False)
    # Same conversion ratio gives exactly the same angle contribution.
    _,rescaled,_=step(chain,q,cmd,replace(pref,joint3_angle_scale_rad=2.,joint3_length_scale_m=.4))
    np.testing.assert_allclose(result.joint_velocity_rad_s,rescaled.joint_velocity_rad_s,atol=1e-12)


def test_zero_start_joint3_changes_sign_preference_without_overriding_initial_pose(chain):
    command=np.array([.02,0,0,0,0,0]);final=[]
    for enabled in (False,True):
        q=np.zeros(6);state=ElbowDeviationState();pref=OnlineElbowPreference(joint3_enabled=enabled)
        for _ in range(100):
            _,result,diag=step(chain,q,command,pref,state)
            q+=.01*result.joint_velocity_rad_s
            assert diag.position_offset_m <= .002+1e-10
            assert diag.orientation_offset_rad <= np.pi/180+1e-10
        final.append(q)
    assert final[0][2] > 0
    assert final[1][2] < 0
    assert chain.relative_joint_height(final[1])[0] > 0
    assert chain.relative_joint_height(final[1],shoulder_joint='joint6',elbow_joint='joint4')[0] > 0


@pytest.mark.parametrize('kwargs', [dict(joint3_enabled=1),dict(joint3_negative_margin_rad=0),
    dict(joint3_negative_margin_rad=float('nan')),dict(joint3_angle_scale_rad=0),
    dict(joint3_length_scale_m=-1),dict(joint3_gain_s=0),dict(joint3_weight=float('inf')),
    dict(maximum_joint3_preference_speed_rad_s=0)])
def test_invalid_joint3_preferences_fail_clearly(kwargs):
    with pytest.raises(ValueError): OnlineElbowPreference(**kwargs)


def test_joint3_switch_and_manual_positive_initialization(chain,monkeypatch):
    from simulation.teleop_grasp_unreal import CartesianTeleopTarget
    monkeypatch.setenv('SPACE_SIM_ONLINE_ELBOW_MODE','prefer')
    monkeypatch.setenv('SPACE_SIM_ONLINE_WRIST_MODE','prefer')
    monkeypatch.setenv('SPACE_SIM_ONLINE_JOINT3_MODE','off')
    q=np.r_[np.array(BALANCED_TELEOP_HOME[:6]),.01875,.01875]
    t=CartesianTeleopTarget(q,None,chain)
    assert t.elbow_preference.enabled and t.elbow_preference.wrist_enabled
    assert not t.elbow_preference.joint3_enabled
    monkeypatch.setenv('SPACE_SIM_ONLINE_JOINT3_MODE','prefer')
    t=CartesianTeleopTarget(q,None,chain);t.reset(0.)
    assert t.elbow_preference.joint3_enabled
    np.testing.assert_array_equal(t.position,q)
    assert t.position[2] > 0
    monkeypatch.setenv('SPACE_SIM_ONLINE_JOINT3_MODE','invalid')
    with pytest.raises(ValueError,match='SPACE_SIM_ONLINE_JOINT3_MODE'):
        CartesianTeleopTarget(q,None,chain)
