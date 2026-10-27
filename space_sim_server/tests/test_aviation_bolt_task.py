"""Independent geometry, multi-turn and actuator-only threaded regressions."""

from dataclasses import replace
from pathlib import Path
import math

import numpy as np
import pytest

from simulation.scripted_tasks.aviation_bolt import AviationBoltConfig, AviationBoltTask
from simulation.scripted_tasks.threaded_motion import ThreadedSample
from simulation.serial_chain_kinematics import axis_angle_to_matrix
from space_arm_platform.control_defaults import DEFAULT_OPERATING_JOINT_DEG


MODEL = Path(__file__).resolve().parents[1]/"model/SARM/platform/sarm_20261012.xml"


def make_task():
    task=AviationBoltTask(MODEL)
    q=np.r_[np.deg2rad(DEFAULT_OPERATING_JOINT_DEG),0.,0.]
    sample=ThreadedSample(0.,q,np.zeros(8),task.initial_position,task.initial_rotation,
                          np.zeros(3),np.zeros(3),np.zeros(6),True,(0,0))
    return task,sample


def test_two_start_delivered_thread_phase_and_lead_match_seated_target():
    task,_=make_task()
    male=task.root.find('.//geom[@name="bolt_thread_sdf"]')
    target=task.root.find('.//geom[@name="board_thread_sdf"]')
    assert male is not None and target is not None
    # Actual male SDF datum is head-local 25 mm down; the female datum is
    # 2 mm lower at a head origin of 32 mm. That gap sets seated yaw +45 deg.
    male_z=task.seated_position[2]-float(male.get("pos").split()[2])
    female_z=task.hole[2]
    yaw=2*math.pi*(male_z-female_z)/task.config.thread_pitch_m
    expected=task.hole_rotation@axis_angle_to_matrix(np.array([0.,0.,1.]),yaw)@np.diag([1.,-1.,-1.])
    np.testing.assert_allclose(task.seated_rotation,expected,atol=1e-12)


@pytest.mark.parametrize("direction",[1.,-1.])
def test_raw_rotation_unwrap_measures_full_turns_across_pi(direction):
    task,sample=make_task()
    task.source_unscrewed=direction<0
    task._target_started=direction<0
    task.stage="unscrew" if direction>0 else "screw_in"
    for theta in np.linspace(0.,9.*math.pi,1001):
        r=axis_angle_to_matrix(np.array([0.,0.,1.]),direction*theta)@sample.part_rotation_body
        task._integrate_rotation(replace(sample,part_rotation_body=r))
    measured=task.source_rotation_rad if direction>0 else task.target_rotation_rad
    assert measured==pytest.approx(9.*math.pi,abs=1e-10)


@pytest.mark.parametrize("contacts",[None,(0,0),(1,0),(0,4)])
def test_screw_does_not_start_without_bilateral_physical_grasp(contacts):
    task,sample=make_task()
    sample=replace(sample,finger_part_contacts=contacts)
    task.reference[:]=sample.joints
    task._enter("grip",sample)
    for tick in range(100):
        task.step(replace(sample,time_s=tick*.005))
    assert task.stage=="grip"
    assert not task.source_unscrewed


def test_recording_rejects_missing_regrasp_or_missing_screw_in():
    good=['settle','approach','grip','unscrew','regrasp_release','regrasp_rewind',
          'regrasp_grip','unscrew','carry','screw_in','release','retract','verify']
    assert AviationBoltTask.validate_recorded_stages(good)
    assert not AviationBoltTask.validate_recorded_stages([s for s in good if s!='regrasp_rewind'])
    assert not AviationBoltTask.validate_recorded_stages([s for s in good if s!='screw_in'])


def test_grasp_transform_is_not_reestimated_to_hide_slip():
    task,sample=make_task()
    task.reference[:]=sample.joints
    task._capture_grasp(sample)
    original=tuple(x.copy() for x in task._grasp_transform)
    targets=[]
    task._follow_pose=lambda p,r,*args,**kwargs:targets.append((p.copy(),r.copy()))
    task._track_part(task.initial_position,task.initial_rotation,sample,.005)
    slipped=replace(sample,part_position_body_m=sample.part_position_body_m+[.1,-.2,.04])
    task._track_part(task.initial_position,task.initial_rotation,slipped,.005)
    np.testing.assert_allclose(targets[0][0],targets[1][0])
    np.testing.assert_allclose(targets[0][1],targets[1][1])
    for before,after in zip(original,task._grasp_transform):np.testing.assert_array_equal(before,after)


def test_reference_half_turn_rewind_returns_same_arm_branch():
    task,sample=make_task()
    task.reference[:]=sample.joints
    task._enter("approach",sample)
    task.reference[:]=task.planned_path_reference(task._duration)
    start=task.reference.copy()
    p,r=task.chain.forward(start[:6])
    axis=task.outward
    def twist(u):return p,axis_angle_to_matrix(axis,math.pi*u)@r
    task._plan_pose(p,twist(1.)[1],path_fn=twist,angle=math.pi)
    task.reference[:]=task.planned_path_reference(task._duration)
    task._phase='unscrew'
    task._stroke_turn=math.pi
    sample=replace(sample,joints=task.reference.copy())
    task._enter('regrasp_rewind',sample)
    final=task.planned_path_reference(task._duration)
    np.testing.assert_allclose(final[:6],start[:6],atol=3e-5)


@pytest.mark.parametrize("bad",[0.,-.016,float('nan')])
def test_invalid_lead_cannot_create_a_straight_pull(bad):
    with pytest.raises(ValueError):AviationBoltConfig(thread_pitch_m=bad)


def test_release_backdrive_reduces_actual_turn_progress_and_cannot_pass_verify():
    task,sample=make_task()
    q=sample.joints.copy()
    q[6:]=[.75,-.75]
    seated=replace(sample,joints=q,part_position_body_m=task.seated_position.copy(),
                   part_rotation_body=task.seated_rotation.copy())
    task.reference[:]=q
    task.source_unscrewed=task.ever_lifted=True
    task.source_rotation_rad=task.source_required_rotation_rad
    task._enter("screw_in",seated)
    task.target_rotation_rad=task.target_required_rotation_rad
    task._previous_part_rotation=task.seated_rotation.copy()
    task.stage="release"
    # An ordinary elastic unwind of .4 rad backs this 16 mm lead out only
    # 1.019 mm. The seat/axis checks alone therefore still pass, but the actual
    # accumulated screw-in rotation no longer meets its required tolerance.
    unwind=.4
    displacement=task.config.thread_pitch_m*unwind/(2*math.pi)
    rewound=replace(seated,
                    part_position_body_m=task.seated_position+task.target_outward*displacement,
                    part_rotation_body=axis_angle_to_matrix(task.target_outward,unwind)@task.seated_rotation)
    task._integrate_rotation(rewound)
    assert task.target_rotation_rad==pytest.approx(task.target_required_rotation_rad-unwind)
    assert abs(task.metrics(rewound)["seat_gap_m"])<task.config.seat_tolerance_m
    task.stage="verify"
    task._last_time=0.
    for tick in range(1,26):
        task.step(replace(rewound,time_s=tick*.05))
    assert task.result is None


def test_rewind_follows_extra_physical_turn_during_release_before_regrasp():
    task,sample=make_task()
    task.reference[:]=sample.joints
    task._enter("approach",sample)
    task.reference[:]=task.planned_path_reference(task._duration)
    p,r=task.chain.forward(task.reference[:6])
    turn=math.pi-.02
    distance=task.config.thread_pitch_m*turn/(2*math.pi)
    axis=task.outward
    def helix(u):
        return p+axis*distance*u,axis_angle_to_matrix(axis,turn*u)@r
    task._plan_pose(*helix(1.),path_fn=helix,distance=distance,angle=turn)
    task.reference[:]=task.planned_path_reference(task._duration)
    task._phase="unscrew"
    task._stroke_turn=turn
    actual_turn=turn+.12
    released=replace(sample,joints=task.reference.copy(),
        part_position_body_m=task.initial_position+axis*task.config.thread_pitch_m*actual_turn/(2*math.pi),
        part_rotation_body=axis_angle_to_matrix(axis,actual_turn)@task.initial_rotation)
    task._enter("regrasp_rewind",released)
    final=task.planned_path_reference(task._duration)
    fp,fr=task.chain.forward(final[:6])
    desired,_=task._regrasp_pose(released,fr)
    assert abs(fr[:,0]@released.part_rotation_body[:,1])>math.cos(1e-5)
    np.testing.assert_allclose(fp,desired,atol=2e-5)


def test_preflight_closes_stationary_before_turn_and_transforms_free_pose_to_world(tmp_path,monkeypatch):
    import xml.etree.ElementTree as ET
    from simulation.scripted_tasks import drawer_initialization
    from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix,matrix_to_quaternion_wxyz

    calls=[]
    class GeometryProbe:
        def __init__(self,path): pass
        def grasp_opening(self,candidate,body):
            calls.append((candidate.copy(),body))
            return .30
        def check_threaded_paths(self,name,paths):
            return dict(accepted=True,checked_poses=sum(len(row[1]) for row in paths))
        def close(self): pass
    monkeypatch.setattr(drawer_initialization,"NativePoseProbe",GeometryProbe)
    tree=ET.parse(MODEL)
    root=tree.getroot()
    world_rotation=axis_angle_to_matrix(np.array([0.,0.,1.]),.47)@axis_angle_to_matrix(np.array([1.,0.,0.]),.20)
    world_translation=np.array([.8,-.4,.2])
    for body in root.findall("./worldbody/body"):
        pos=np.fromstring(body.get("pos","0 0 0"),sep=" ")
        rotation=quaternion_wxyz_to_matrix(np.fromstring(body.get("quat","1 0 0 0"),sep=" "))
        body.set("pos"," ".join(map(str,world_translation+world_rotation@pos)))
        body.set("quat"," ".join(map(str,matrix_to_quaternion_wxyz(world_rotation@rotation))))
    path=tmp_path/"rotated_world.xml"
    tree.write(path)
    task=AviationBoltTask(path)
    q=np.r_[np.deg2rad(DEFAULT_OPERATING_JOINT_DEG),0.,0.]
    paths=task.preflight_paths(q)
    assert len(calls)==1 and calls[0][1]=="bolt"
    assert [row[0] for row in paths[:4]]==["approach","source_grip","source_finish_envelope","source_turn"]
    approach,grip,envelope,turn=paths[:4]
    assert grip[1][0,6]==pytest.approx(1.) and grip[1][-1,6]==pytest.approx(.30)
    np.testing.assert_allclose(grip[1][:,:6],np.broadcast_to(approach[1][-1,:6],grip[1][:,:6].shape),atol=2e-5)
    grip_poses=grip[3]["bolt_freejoint"]
    np.testing.assert_allclose(grip_poses,np.broadcast_to(grip_poses[0],grip_poses.shape),atol=1e-12)
    original_bolt=root.find('./worldbody/body[@name="bolt"]')
    np.testing.assert_allclose(grip_poses[0,:3],np.fromstring(original_bolt.get("pos"),sep=" "),atol=1e-12)
    np.testing.assert_allclose(quaternion_wxyz_to_matrix(grip_poses[0,3:]),
        quaternion_wxyz_to_matrix(np.fromstring(original_bolt.get("quat"),sep=" ")),atol=1e-12)
    np.testing.assert_allclose(turn[1][:,6],.30,atol=1e-12)
    # The extra wrist branch checks another .5 rad of held rotation, then
    # restores the nominal branch before opening and rewinding. Its temporary
    # freejoint motion must never leak into the subsequent private path.
    turn_poses=turn[3]["bolt_freejoint"]
    envelope_poses=envelope[3]["bolt_freejoint"]
    np.testing.assert_allclose(turn_poses[0],envelope_poses[0],atol=2e-5)
    axis_world=world_rotation@task.outward
    envelope_travel=(envelope_poses[-1,:3]-envelope_poses[0,:3])@axis_world
    turn_travel=(turn_poses[-1,:3]-turn_poses[0,:3])@axis_world
    assert envelope_travel==pytest.approx(task.config.thread_pitch_m*(math.pi+.5)/(2*math.pi),abs=3e-5)
    assert turn_travel==pytest.approx(task.config.thread_pitch_m/2,abs=3e-5)
    assert paths[4][0]=="source_open"
    np.testing.assert_allclose(paths[4][3]["bolt_freejoint"][0],turn_poses[-1],atol=2e-5)


def test_preflight_reports_missing_native_grasp_geometry(monkeypatch):
    from simulation.scripted_tasks import drawer_initialization

    def unavailable(path):
        raise RuntimeError("Native contact geometry is unavailable")
    monkeypatch.setattr(drawer_initialization,"NativePoseProbe",unavailable)
    task,sample=make_task()
    with pytest.raises(RuntimeError,match="Native contact geometry is unavailable"):
        task.preflight_paths(sample.joints)


@pytest.mark.parametrize("stage",["unscrew","screw_in"])
def test_final_partial_turn_keeps_grasp_without_relaxing_rotation_success(stage):
    task,sample=make_task()
    source=stage=="unscrew"
    required=task.source_required_rotation_rad if source else task.target_required_rotation_rad
    actual=required-.24
    if source:
        task.source_rotation_rad=actual
        position=task.initial_position+task.outward*task.config.thread_pitch_m*actual/(2*math.pi)
    else:
        task.source_unscrewed=task.ever_lifted=True
        task.target_rotation_rad=actual
        position=task.seated_position+task.target_outward*task.config.thread_pitch_m*.24/(2*math.pi)
    sample=replace(sample,part_position_body_m=position)
    task.reference[:]=sample.joints
    task._enter(stage,sample)
    task._stroke_start=actual-math.pi
    task._stroke_turn=math.pi
    task._track_part=lambda *args,**kwargs:None
    transitions=[]
    task._enter=lambda next_stage,sample:transitions.append(next_stage)
    task._spin(sample,.005)
    # Earlier strokes still regrasp, but a last .24 rad keeps supporting the
    # part and does not claim completion before the original .15 rad tolerance.
    assert not transitions
    assert task.stage==stage
    assert (task.source_rotation_rad if source else task.target_rotation_rad)==actual


@pytest.mark.parametrize("failure",["ik","collision"])
def test_carry_uses_equivalent_phase_when_first_branch_is_unreachable_or_collides(monkeypatch,failure):
    from simulation.scripted_tasks import drawer_initialization

    task,sample=make_task()
    selected=[]
    inspected=[]
    def paths(sample,phase):
        if phase==0. and failure=="ik":raise ValueError("unreachable wrist branch")
        return phase
    class GeometryProbe:
        def __init__(self,path): pass
        def check_threaded_paths(self,name,phase):
            inspected.append(phase)
            return dict(accepted=phase!=0.,reason="approach_collision")
        def close(self): pass
    monkeypatch.setattr(drawer_initialization,"NativePoseProbe",GeometryProbe)
    task._carry_candidate_paths=paths
    task._plan_carry_phase=lambda sample,phase:selected.append(phase)
    task._plan_carry(sample)
    assert selected==[math.pi]
    assert task.carry_thread_symmetry_rad==math.pi
    assert not task.carry_branch_checks[0]["accepted"]
    assert task.carry_branch_checks[-1]["accepted"]
    assert math.pi in inspected


def test_preflight_checks_both_carry_branches_and_keeps_reachable_branch(monkeypatch):
    from simulation.scripted_tasks import drawer_initialization

    task,sample=make_task()
    inspected=[]
    selected=[]
    class GeometryProbe:
        def __init__(self,path): pass
        def check_threaded_paths(self,name,phase):
            inspected.append(phase)
            return dict(accepted=phase==0.,reason="approach_collision")
        def close(self): pass
    monkeypatch.setattr(drawer_initialization,"NativePoseProbe",GeometryProbe)
    task._carry_candidate_paths=lambda sample,phase:phase
    task._plan_carry_phase=lambda sample,phase:selected.append(phase)
    task._plan_carry(sample,check_all=True)
    assert inspected==[0.,math.pi] and selected==[0.]
    assert task.carry_branch_checks[0]["accepted"]
    assert not task.carry_branch_checks[1]["accepted"]


def test_carry_rejects_all_unsafe_equivalent_branches(monkeypatch):
    from simulation.scripted_tasks import drawer_initialization

    task,sample=make_task()
    class GeometryProbe:
        def __init__(self,path): pass
        def check_threaded_paths(self,name,paths):
            return dict(accepted=False,reason="mechanism_environment_collision")
        def close(self): pass
    monkeypatch.setattr(drawer_initialization,"NativePoseProbe",GeometryProbe)
    task._carry_candidate_paths=lambda sample,phase:phase
    with pytest.raises(ValueError,match="No collision-free reachable"):
        task._plan_carry(sample)


@pytest.mark.parametrize("tip_clearance,remaining,can_carry",[(-.00042,.04,False),
                                                           (.00005,.04,False),
                                                           (.0002,.06,False),
                                                           (.0002,.04,True)])
def test_source_handover_requires_real_tip_clearance_and_strict_actual_turn(tip_clearance,remaining,can_carry):
    task,sample=make_task()
    tip=task.pickup+task.outward*tip_clearance
    position=tip-sample.part_rotation_body[:,2]*task.config.shaft_tip_local_m
    sample=replace(sample,part_position_body_m=position)
    task.reference[:]=sample.joints
    task.source_rotation_rad=task.source_required_rotation_rad-remaining
    task._enter("unscrew",sample)
    task._track_part=lambda *args,**kwargs:None
    transitions=[]
    task._enter=lambda stage,sample:transitions.append(stage)
    task._spin(sample,.005)
    assert ("carry" in transitions)==can_carry
    assert task.metrics(sample)["source_tip_clearance_m"]==pytest.approx(tip_clearance)


def test_private_carry_geometry_uses_measured_jaws_while_runtime_keeps_preload_command(monkeypatch):
    task,sample=make_task()
    task.reference[:]=sample.joints
    actual=sample.joints.copy()
    actual[6:]=[.2994,-.2996]
    sample=replace(sample,joints=actual)
    planned=[]
    def plan(probe,sample,phase):
        planned.append(probe.reference[6:].copy())
    def record(probe,stage,body_pose,**kwargs):
        refs=np.broadcast_to(probe.reference,(2,8)).copy()
        return (stage,refs,("bolt",),{"bolt_freejoint":np.zeros((2,7))}),body_pose
    monkeypatch.setattr(AviationBoltTask,"_plan_carry_phase",plan)
    monkeypatch.setattr(AviationBoltTask,"_record_preflight_path",record)
    monkeypatch.setattr(AviationBoltTask,"_thread_preflight_paths",lambda *args:([],args[-1]))
    monkeypatch.setattr(AviationBoltTask,"_release_retract_preflight_paths",lambda *args:[])
    paths=task._carry_candidate_paths(sample,0.)
    np.testing.assert_array_equal(planned[0],actual[6:])
    np.testing.assert_array_equal(paths[0][1][:,6:],np.broadcast_to(actual[6:],(2,2)))
    np.testing.assert_array_equal(task.reference[6:],[0.,0.])


def test_release_retract_verify_handoff_never_replays_partial_release_opening(monkeypatch):
    task,sample=make_task()
    def plan_retract(sample):
        p,r=task.chain.forward(task.reference[:6])
        task._plan_pose(p+.10*task.target_outward,r,opening=task.config.release_open_rad)
    monkeypatch.setattr(task,"_plan_retract",plan_retract)
    actual=sample.joints.copy()
    actual[6:]=[.30,-.30]
    sample=replace(sample,joints=actual)
    task.reference[:]=actual
    task._enter("release",sample)
    task._advance_opening(.10)
    assert task._gripper_timing is not None
    assert task.reference[6]<task.config.release_open_rad
    task._enter("retract",sample)
    assert task._gripper_timing is None
    task.reference[:]=task.planned_path_reference(task._duration)
    assert task.reference[6]==pytest.approx(task.config.release_open_rad)
    task._enter("verify",replace(sample,joints=task.reference.copy()))
    for _ in range(30):
        task._advance_opening(.005)
        assert task.reference[6]==pytest.approx(task.config.release_open_rad)


@pytest.mark.parametrize("remaining,can_release",[(.15,False),(.021,False),(.019,True)])
def test_target_finishes_actual_turn_before_release_to_preserve_final_unwind_margin(remaining,can_release):
    task,sample=make_task()
    task.source_unscrewed=task.ever_lifted=True
    sample=replace(sample,part_position_body_m=task.seated_position.copy(),
                   part_rotation_body=task.seated_rotation.copy())
    task.reference[:]=sample.joints
    task.target_rotation_rad=task.target_required_rotation_rad-remaining
    task._enter("screw_in",sample)
    task._track_part=lambda *args,**kwargs:None
    transitions=[]
    task._enter=lambda stage,sample:transitions.append(stage)
    task._spin(sample,.005)
    assert ("release" in transitions)==can_release
    assert task.config.rotation_tolerance_rad==.15


def test_retract_restores_measured_geometry_and_tries_only_upward_routes(monkeypatch):
    from simulation.scripted_tasks import drawer_initialization

    task,sample=make_task()
    task.reference[:]=sample.joints
    baseline=np.arange(12,dtype=float)
    sample=replace(sample,native_qpos=baseline)
    restored=[]
    inspected=[]
    class GeometryProbe:
        def __init__(self,path):pass
        def use_measured_qpos(self,qpos,joints):
            restored.append((qpos.copy(),joints.copy()))
        def check_threaded_paths(self,name,paths):
            inspected.append(paths)
            return dict(accepted=len(inspected)>1,
                        reason="approach_collision",
                        contact=dict(penetration_m=.00016))
        def close(self):pass
    monkeypatch.setattr(drawer_initialization,"NativePoseProbe",GeometryProbe)
    start,r=task.chain.forward(task.reference[:6])
    task._plan_retract(sample)
    end,_=task.chain.forward(task.planned_path_reference(task._duration)[:6])
    assert len(inspected)==2
    np.testing.assert_array_equal(restored[0][0],baseline)
    np.testing.assert_array_equal(restored[0][1],sample.joints)
    assert not task.retract_branch_checks[0]["accepted"]
    assert task.retract_branch_checks[1]["accepted"]
    assert (end-start)@task.target_outward>.070
    assert np.linalg.norm(end-start)==pytest.approx(.10,abs=2e-5)
    for paths in inspected:
        # Actual native world poses must remain in the private baseline, never
        # be replaced with the nominal bus/model transform after orbit drift.
        assert paths[0][0]=="retract" and paths[0][3]=={}


def test_release_and_retract_are_both_in_complete_nominal_preflight(monkeypatch):
    task,sample=make_task()
    task.reference[:]=sample.joints
    task._enter("approach",sample)
    task.reference[:]=task.planned_path_reference(task._duration)
    body_pose=(task.initial_position.copy(),task.initial_rotation.copy())
    def plan_retract(sample):
        p,r=task.chain.forward(task.reference[:6])
        task._plan_pose(p+.10*task.target_outward,r,opening=task.config.release_open_rad)
    monkeypatch.setattr(task,"_plan_retract",plan_retract)
    paths=task._release_retract_preflight_paths(body_pose)
    assert [p[0] for p in paths]==["target_release","retract"]
    assert paths[-1][1][-1,6]==pytest.approx(task.config.release_open_rad)
    for path in paths:
        poses=path[3]["bolt_freejoint"]
        np.testing.assert_allclose(poses,np.broadcast_to(poses[0],poses.shape))
