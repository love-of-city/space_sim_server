"""Planning must preserve measured frames and intentional-contact scope."""
from types import SimpleNamespace
import numpy as np
import pytest
from simulation.scripted_tasks.collision_motion import NativeMotionQuery
from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix


@pytest.mark.parametrize('held',[False,True])
def test_future_queries_reuse_actual_frame_and_only_held_fingers_are_exempt(monkeypatch,held):
    from simulation.scripted_tasks import drawer_initialization
    restored,queried=[],[]
    bus_p=np.array([.9,-.4,.2])
    bus_r=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
    class Probe:
        def __init__(self,*args,**kwargs):self.home=np.zeros(12)
        def use_measured_qpos(self,q,joints):
            self.home[:]=q;restored.append((q.copy(),joints.copy()))
        def contacts(self,q):pass
        def body_pose(self,name):return bus_p,bus_r
        def pose_collision(self,*args,**kwargs):queried.append((args,kwargs))
        def close(self):pass
    monkeypatch.setattr(drawer_initialization,'NativePoseProbe',Probe)
    measured=np.arange(12.)
    joints=np.r_[np.zeros(6),.27,-.27]
    part_p=np.array([.2,.3,.1])
    sample=SimpleNamespace(native_qpos=measured,joints=joints,
        part_position_body_m=part_p,part_rotation_body=np.eye(3))
    task=SimpleNamespace(model_path='private.xml',name='aviation_bolt_insert',
        chain=SimpleNamespace(forward=lambda q:(part_p,np.eye(3))),
        _grasp_transform=(np.zeros(3),np.eye(3)))
    first=NativeMotionQuery(task,sample,held=held)
    first(np.r_[np.zeros(6),.45]);first.close()
    # Future opening samples have no new native snapshot. Computing four-bar
    # offsets against this opening would corrupt the original measured offset.
    future=SimpleNamespace(**vars(sample));future.native_qpos=None
    future.joints=np.r_[np.zeros(6),.4,-.4]
    second=NativeMotionQuery(task,future,held=held)
    second(np.r_[np.zeros(6),.45]);second.close()
    np.testing.assert_array_equal(restored[-1][0],measured)
    np.testing.assert_array_equal(restored[-1][1],joints)
    args,kwargs=queried[-1]
    pose=args[3]['bolt_freejoint']
    np.testing.assert_allclose(pose[:3],bus_p+bus_r@part_p)
    np.testing.assert_allclose(quaternion_wxyz_to_matrix(pose[3:]),bus_r,atol=1e-12)
    assert args[2]==(('bolt',) if held else ())
    assert args[1]==('aviation_bolt_insert' if held else 'free_motion')
    assert kwargs['clearance_m']==.002


def test_searched_carry_does_not_reuse_a_previous_release_opening():
    from simulation.scripted_tasks.collision_motion import _install
    from simulation.scripted_tasks.threaded_motion import ThreadedTaskBase
    task=SimpleNamespace(
        chain=SimpleNamespace(forward=lambda q:(q[:3],np.eye(3)),jacobian=lambda q:np.eye(6)),
        config=SimpleNamespace(arm_speed_rad_s=.35,linear_speed_m_s=.08,
                               angular_speed_rad_s=.3,gripper_speed_rad_s=.3,
                               grasp_approach_speed_m_s=.04),
        reference=np.zeros(8),_path_opening=(.4,.4,None))
    # The preceding regrasp rewind opened the fingers; the actual current
    # command clamps the part. An IK failure must not replay the old opening.
    _install(task,np.array([np.zeros(6),[.01,0,0,0,0,0]]),held=True)
    for progress in (0.,.25,.75,1.):
        q=ThreadedTaskBase.path_reference(task,progress)
        np.testing.assert_array_equal(q[6:],[0.,0.])


def test_detour_goal_does_not_require_an_unrelated_axial_pregrasp(monkeypatch):
    from simulation.scripted_tasks import collision_motion
    requested = []
    def solve(chain,position,rotation,seed,lower,upper):
        requested.append(position.copy())
        return np.r_[position,np.zeros(3)]
    monkeypatch.setattr(collision_motion,'solve_pose',solve)
    task = SimpleNamespace(chain=None, lower=-np.ones(6), upper=np.ones(6),
                           config=SimpleNamespace(pregrasp_distance_m=.045))
    goal = np.array([.2,.1,.3])
    terminal = collision_motion._terminal(task,goal,np.eye(3),np.array([0.,0.,1.]),
                                          .8,np.zeros(6),False)
    assert len(requested) == len(terminal) == 1
    np.testing.assert_array_equal(requested[0],goal)
    np.testing.assert_array_equal(terminal[0],np.r_[goal,np.zeros(3),.8])


def test_proxy_recheck_uses_installed_detour_including_its_fingers():
    from simulation.scripted_tasks.collision_motion import _check_template
    from simulation.scripted_tasks.joint_path import JointPath
    from simulation.scripted_tasks.obstacle_planner import JointRouteSearch
    old = np.zeros(8)
    knots = np.array([np.r_[np.zeros(6),.1],np.r_[np.ones(6)*.1,.8]])
    proxy = SimpleNamespace(_timing=SimpleNamespace(path=JointPath.from_waypoints(knots)),
                            path_reference=lambda u:old)
    checked = []
    def collision(q):
        checked.append(q.copy())
        return {'body1':'finger','body2':'neighbor'} if q[6] > .5 else None
    search = JointRouteSearch(-np.ones(7),np.ones(7),collision)
    assert not _check_template(proxy,search,False)
    assert max(q[6] for q in checked) > .5
    assert search.report['first_collision']['body1'] == 'finger'


@pytest.mark.parametrize('stage',['approach','reapproach','fold_approach','turn_approach'])
def test_clear_straight_approach_is_accepted_without_search(monkeypatch,stage):
    from simulation.scripted_tasks import collision_motion
    from simulation.scripted_tasks.joint_path import JointPath
    from simulation.scripted_tasks.obstacle_planner import JointRouteSearch
    class Query:
        def __init__(self,*args,**kwargs):
            self.bus_p,self.bus_r,self.clearance = np.zeros(3),np.eye(3),.002
        def __call__(self,q):return None
        def close(self):pass
    monkeypatch.setattr(collision_motion,'NativeMotionQuery',Query)
    def unexpected_search(*args,**kwargs):
        pytest.fail('A clear straight approach must not invoke detour search')
    monkeypatch.setattr(JointRouteSearch,'search',unexpected_search)
    path = JointPath.from_waypoints(np.array([np.zeros(6),np.ones(6)*.05]))
    task = SimpleNamespace(reference=np.r_[np.zeros(6),.1,-.1],lower=-np.ones(6),upper=np.ones(6),
                            config=SimpleNamespace(approach_open_rad=.8),
                            _timing=SimpleNamespace(path=path),
                            path_reference=lambda u:np.r_[path.at(u),.1+.7*u,-.1-.7*u],
                            _duration=2.,_timeout=10.,motion_plans=[],events=[])
    report=collision_motion.plan_free_motion(task,None,np.ones(3)*.05,np.eye(3),
                                            np.array([0.,0.,1.]),stage=stage)
    assert report['route'] == 'straight'
    assert report['iterations'] == 0


def test_alternative_retreat_skips_blocked_goal_and_installs_reachable_endpoint(monkeypatch):
    from simulation.scripted_tasks import collision_motion
    queries=[]
    class Query:
        def __init__(self,*args,**kwargs):
            self.bus_p,self.bus_r,self.clearance=np.zeros(3),np.eye(3),.002
            queries.append(self)
        def __call__(self,q):
            return {'body1':'finger','body2':'neighbor'} if q[0]>.4 and q[1]<.2 else None
        def close(self):pass
    monkeypatch.setattr(collision_motion,'NativeMotionQuery',Query)
    monkeypatch.setattr(collision_motion,'solve_pose',
                        lambda chain,p,r,seed,lo,hi:np.r_[p,np.zeros(3)])
    task=SimpleNamespace(
        reference=np.r_[np.zeros(6),.4,-.4],lower=-np.ones(6),upper=np.ones(6),
        chain=SimpleNamespace(forward=lambda q:(q[:3],np.eye(3)),jacobian=lambda q:np.eye(6)),
        config=SimpleNamespace(approach_open_rad=.65,arm_speed_rad_s=.35,
                               linear_speed_m_s=.08,angular_speed_rad_s=.3,
                               gripper_speed_rad_s=.3,grasp_approach_speed_m_s=.04),
        _path=None,_timeout=10.,motion_plans=[],events=[])
    goals=[np.array([.5,0.,.08]),np.array([.2,.3,.08])]
    report=collision_motion.plan_free_motion(
        task,None,goals[0],np.eye(3),np.array([0.,0.,1.]),stage='retract',
        template_error='direct_retracts_blocked',fallback_positions=goals)
    assert len(queries)==1
    assert report['route']=='searched' and report['goal_candidate_index']==1
    assert report['candidate_failures']==['goal_in_collision']
    np.testing.assert_allclose(task._goal,goals[1])
    np.testing.assert_allclose(task._timing.path.at(1.)[:3],goals[1])
    assert report['queries']>300


def test_alternative_retreat_goals_share_one_query_budget(monkeypatch):
    from simulation.scripted_tasks import collision_motion
    from simulation.scripted_tasks.obstacle_planner import JointRouteSearch,PlanningFailure,SearchOptions
    instances=[]
    class Query:
        def __init__(self,*args,**kwargs):
            self.bus_p,self.bus_r,self.clearance=np.zeros(3),np.eye(3),.002
            instances.append(self)
        def __call__(self,q):return {'body1':'finger','body2':'wall'}
        def close(self):pass
    monkeypatch.setattr(collision_motion,'NativeMotionQuery',Query)
    monkeypatch.setattr(collision_motion,'solve_pose',
                        lambda chain,p,r,seed,lo,hi:np.r_[p,np.zeros(3)])
    def bounded_search(*args,**kwargs):
        return JointRouteSearch(*args,**kwargs,options=SearchOptions(query_budget=3))
    monkeypatch.setattr(collision_motion,'JointRouteSearch',bounded_search)
    task=SimpleNamespace(reference=np.zeros(8),lower=-np.ones(6),upper=np.ones(6),
                         config=SimpleNamespace(approach_open_rad=.65),chain=None,
                         _path=None,motion_plans=[])
    goals=[np.array([.1*i,0.,.08]) for i in range(1,7)]
    with pytest.raises(PlanningFailure,match='collision_query_budget_exhausted') as failure:
        collision_motion.plan_free_motion(task,None,goals[0],np.eye(3),np.array([0.,0.,1.]),
                                          template_error='blocked',fallback_positions=goals)
    assert failure.value.report['queries']==3
    assert len(instances)==1
