import copy
import numpy as np
import pytest
from simulation.arm_preparation import ArmPreparation, SmoothJointSegment
from space_arm_platform.control_defaults import DEFAULT_OPERATING_JOINT_DEG


def setup(goal=None):
    goal = np.deg2rad(DEFAULT_OPERATING_JOINT_DEG) if goal is None else np.asarray(goal)
    c = ArmPreparation(True, goal, [-np.pi]*5+[-2*np.pi], [np.pi]*5+[2*np.pi], [.7,.7,.7,.9,1.,1.])
    q = np.zeros(6)
    a = dict(deadman=True, arm_preparation=dict(request_id='one', joint_position_deg=np.rad2deg(goal).tolist()))
    return c, q, a


def step(c, q, a, velocity=None, stale=False, reference=None):
    return c.step(.01, a, stale, q if reference is None else reference, q,
                  np.zeros(6) if velocity is None else velocity)


def start(c, q, a):
    for _ in range(36): step(c, q, a)
    assert c.status == 'moving'


def test_waiting_gates_input_and_no_idle_motion_or_planner():
    c,q,a=setup()
    np.testing.assert_array_equal(step(c,q,dict(deadman=True))[0],q)
    assert not c.ready and c.status=='waiting'
    assert not hasattr(c,'planner') and not hasattr(c,'future')


def test_fixed_order_literal_angles_speed_and_measured_arrival():
    c,q,a=setup(); start(c,q,a)
    np.testing.assert_allclose(np.rad2deg(c.waypoints), [
        [0,-67.6,30,143.2,0,0], DEFAULT_OPERATING_JOINT_DEG])
    previous_velocity=np.zeros(6)
    for _ in range(6000):
        ref,vel=step(c,q,a,velocity=previous_velocity)
        assert np.all(np.abs(vel) <= c.speed_limits+1e-10)
        assert np.max(np.abs(vel-previous_velocity))/.01 <= np.pi+1e-6
        np.testing.assert_array_equal(ref[[0,5]], [0.,0.])
        np.testing.assert_array_equal(vel[[0,5]], [0.,0.])
        q=ref;previous_velocity=vel
        if c.ready:break
    assert c.ready
    np.testing.assert_allclose(q,np.deg2rad(DEFAULT_OPERATING_JOINT_DEG),atol=1e-8)
    assert q[5] == 0. and q[0] == 0.
    assert len(c.phase_times)==2
    assert step(c,q,{'deadman':True}) is None
    assert step(c,q,a) is not None


def test_each_stage_waits_for_actual_position_and_velocity():
    c,q,a=setup();start(c,q,a)
    for _ in range(600):
        # Give the controller a stationary reference, but don't move the plant.
        step(c,q,a)
    assert not c.ready and c.phase==0 and c.status=='settling'
    for _ in range(1400):step(c,q,a)
    assert c.status=='failed' and '超时' in c.reason


@pytest.mark.parametrize('cause',['stale','neutral','other_request'])
def test_cancel_holds_measured_and_cannot_replay(cause):
    c,q,a=setup();start(c,q,a);q[:]=.01
    action=copy.deepcopy(a)
    if cause=='neutral':action={'deadman':False}
    if cause=='other_request':action['arm_preparation']['request_id']='other'
    ref,_=step(c,q,action,stale=cause=='stale')
    np.testing.assert_array_equal(ref,q)
    assert c.status=='cancelled' and not c.ready
    step(c,q,a);assert c.status=='cancelled'


def test_retry_resumes_current_phase_not_clearance_again():
    c,q,a=setup();start(c,q,a)
    c.phase=1;c._begin_phase(c.waypoints[0]);q=c.waypoints[0].copy();q[5]=2.
    step(c,q,{'deadman':False});assert c.status=='cancelled'
    a['arm_preparation']['request_id']='two';start(c,q,a)
    assert c.phase==1 and c.segment.start[5]==2.
    assert c.segment.end[5]==2. and c.segment.peak[5]==0.
    np.testing.assert_allclose(c.segment.end,c.goal)


@pytest.mark.parametrize('fault',['limit','nan','tracking','invalid_goal'])
def test_bad_measured_state_and_tracking_aborts(fault):
    c,q,a=setup();start(c,q,a)
    if fault=='limit':q[0]=4.
    if fault=='nan':q[0]=np.nan
    if fault=='tracking':q[0]=.2
    if fault=='invalid_goal':
        c.reset();a['arm_preparation']['joint_position_deg'][5]=361.
    step(c,q,a,reference=np.zeros(6))
    assert c.status=='failed' and not c.ready


def test_moving_start_waits_instead_of_rejecting_button():
    c,q,a=setup()
    for _ in range(20):step(c,q,a,velocity=np.ones(6)*.03)
    assert c.status=='arming' and c.stable==0
    for _ in range(36):step(c,q,a)
    assert c.status=='moving'


def test_target_drift_cannot_replan_or_veto_joint_preparation():
    c,q,a=setup();start(c,q,a)
    ref,vel=c.step(.01,a,False,q,q,np.zeros(6),{'target_position_m':[float('nan'),9,9]})
    assert c.status=='moving' and np.isfinite(ref).all()
    assert c.telemetry()['collision_authority']=='running_native_MJScene'


def test_custom_goal_is_literal_and_limits_still_apply():
    goal=np.deg2rad([-25,-60,-80,140,-70,300])
    c,q,a=setup(goal);start(c,q,a)
    np.testing.assert_array_equal(c.waypoints[-1][1:5],goal[1:5])
    np.testing.assert_array_equal(c.waypoints[-1][[0,5]], [0.,0.])
    assert c.waypoints[0][2]==np.deg2rad(30.)


def test_segment_endpoint_derivative_and_velocity_bound():
    s=SmoothJointSegment(np.zeros(6),np.arange(6),np.ones(6)*.5)
    for t in np.linspace(.001,s.duration-.001,200):
        q,v=s.sample(t)
        numerical=(s.sample(t+1e-5)[0]-s.sample(t-1e-5)[0])/2e-5
        np.testing.assert_allclose(v,numerical,atol=1e-8)
        assert np.max(abs(v))<=.5+1e-10
    np.testing.assert_array_equal(s.sample(s.duration)[0],np.arange(6))
    np.testing.assert_array_equal(s.sample(s.duration)[1],np.zeros(6))


def test_reclick_at_goal_does_not_unfold_the_already_rotated_wrist():
    c,q,a=setup();q=c.goal.copy();start(c,q,a)
    assert len(c.waypoints)==1
    np.testing.assert_array_equal(c.waypoints[0],q)
    for _ in range(150):step(c,q,a)
    assert c.ready
    a['arm_preparation']['request_id']='second-click'
    start(c,q,a)
    assert len(c.waypoints)==1 and c.waypoints[0][2]<0


def test_old_six_joint_request_cannot_reintroduce_last_stage():
    c,q,a=setup(np.deg2rad([-30.2,-67.6,-86.6,143.2,-85.5,337.4]))
    start(c,q,a)
    assert len(c.waypoints)==2 and c.telemetry()['phase_count']==2
    np.testing.assert_array_equal(np.asarray(c.waypoints)[:,[0,5]],0.)
    assert c.goal[0]==c.goal[5]==0.


def test_preparation_keeps_nonzero_j1_j6_references_without_wrapping():
    c,q,a=setup();q[[0,5]]=[-.5,5.8]
    start(c,q,a)
    np.testing.assert_array_equal(c.goal[[0,5]],q[[0,5]])
    for waypoint in c.waypoints:
        np.testing.assert_array_equal(waypoint[[0,5]],q[[0,5]])
    np.testing.assert_array_equal(c.segment.peak[[0,5]],0.)
