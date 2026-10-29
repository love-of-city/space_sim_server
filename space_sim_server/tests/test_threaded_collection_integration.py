"""Task dispatch, articulated read-only feedback and physical dataset rejection."""

from copy import deepcopy
from dataclasses import asdict
from types import SimpleNamespace

import numpy as np
import pytest

from simulation.scripted_tasks.native_state import STATE_READERS, ThreadedState
from simulation.scripted_tasks.threaded_tasks import THREADED_TASKS, threaded_task_class
from space_arm_platform.collection_jobs import CollectionCreate, TASKS
from collection.collect_scripted_tasks import validate_threaded_recording


@pytest.mark.parametrize("name", THREADED_TASKS)
def test_web_collection_dispatches_latest_controller(name):
    assert CollectionCreate(task=name).task == name
    assert TASKS[name].script == "collection/collect_scripted_tasks.py"
    assert TASKS[name].arguments == ("--task", name)
    assert STATE_READERS[name] is ThreadedState
    assert threaded_task_class(name).name == name


@pytest.mark.parametrize("name", THREADED_TASKS)
@pytest.mark.parametrize("profile", [None, "default", "fast_collection"])
def test_private_preflight_uses_the_executed_collection_profile(
    tmp_path, monkeypatch, name, profile
):
    from pathlib import Path
    from simulation.scripted_tasks import drawer_initialization as initialization
    from simulation.scripted_tasks.randomization import write_model_variant
    from simulation.scripted_tasks.threaded_motion import ThreadedSample, model_body_pose
    from space_arm_platform.control_defaults import DEFAULT_OPERATING_JOINT_DEG

    source = Path(__file__).resolve().parents[1] / "model/SARM/platform/sarm_20261012.xml"
    scene = {
        "scripted_task_name": name,
        "randomization": {
            "arm_joint_position_rad": [*np.deg2rad(DEFAULT_OPERATING_JOINT_DEG), 0., 0.]
        },
    }
    if profile is not None:
        scene["collection_config"] = {"threaded_speed_profile": profile}
    model = write_model_variant(source, tmp_path / "model.xml", scene)
    factory = initialization.create_threaded_task
    options = {} if profile is None else {"speed_profile": profile}
    runtime = factory(name, model, **options)

    def initial_approach(task, joints):
        position, rotation = model_body_pose(task.root, task.config.part_body)
        scalars = {
            joint: float(task.root.find(f'.//joint[@name="{joint}"]').get("ref", "0"))
            for joint in getattr(task, "_required_scalars", ())
        }
        sample = ThreadedSample(
            0., joints, np.zeros(8), position, rotation,
            np.zeros(3), np.zeros(3), np.zeros(6), False, scalar_joints=scalars,
        )
        task.reference[:] = joints
        task._enter("approach", sample)
        candidates = np.array([
            task.planned_path_reference(t) for t in np.linspace(0., task._duration, 9)
        ])
        return [("approach", candidates, (task.config.part_body,))]

    joints = np.array(scene["randomization"]["arm_joint_position_rad"])
    expected = initial_approach(runtime, joints)[0][1]
    created = []

    def private_factory(task_name, task_model, **received):
        created.append((received, factory(task_name, task_model, **received)))
        return created[-1][1]

    monkeypatch.setattr(initialization, "create_threaded_task", private_factory)
    # Exercise the real approach planner and factory, while isolating the
    # profile contract from the remaining mechanism paths and native contacts.
    monkeypatch.setattr(type(runtime), "preflight_paths", initial_approach)
    checked = []
    probe = object.__new__(initialization.NativePoseProbe)
    probe.home = np.zeros(8)
    probe.q = probe.home.copy()
    probe.pose_collision = lambda candidate, *args: checked.append(candidate.copy())
    result = probe.validate(model, scene, name)

    assert result["accepted"]
    assert created[0][0] == options
    assert asdict(created[0][1].config) == asdict(runtime.config)
    np.testing.assert_allclose(checked[1:], expected, atol=1e-12)
    assert expected[-1, 6] == pytest.approx(.45 if profile == "fast_collection" else 1.)
    np.testing.assert_array_equal(probe.home, np.zeros(8))


def test_threaded_duration_can_complete_real_turns_without_changing_global_recording_limits():
    from simulation.scripted_tasks.threaded_tasks import task_default_duration_s
    from space_arm_platform.recording_config import RecordingConfig
    from collection.record_scripted_task import task_recording_settings

    original = RecordingConfig()
    settings, override = task_recording_settings(original, "aviation_bolt_insert", 900.)
    assert task_default_duration_s("aviation_bolt_insert") == 900.
    assert task_default_duration_s("drawer_cycle") == 360.
    assert original.max_frames == 18000 and settings.max_frames == 45001
    assert override["original_max_frames"] == 18000
    assert override["effective_max_frames"] == 45001
    shorter, _ = task_recording_settings(original, "locking_rod_cycle", 400.)
    assert shorter.max_frames == 20001
    unchanged, override = task_recording_settings(original, "drawer_cycle", 450.)
    assert unchanged is original and override is None


def test_native_grasp_opening_matches_delivered_head_and_measured_finger_geometry():
    pytest.importorskip("Basilisk")
    from pathlib import Path
    from simulation.scripted_tasks.aviation_bolt import AviationBoltTask
    from simulation.scripted_tasks.drawer_initialization import NativePoseProbe
    from simulation.scripted_tasks.threaded_motion import ThreadedSample
    from space_arm_platform.control_defaults import DEFAULT_OPERATING_JOINT_DEG

    model=Path(__file__).resolve().parents[1]/"model/SARM/platform/sarm_20261012.xml"
    task=AviationBoltTask(model)
    q=np.r_[np.deg2rad(DEFAULT_OPERATING_JOINT_DEG),0.,0.]
    sample=ThreadedSample(0.,q,np.zeros(8),task.initial_position,task.initial_rotation,
                          np.zeros(3),np.zeros(3),np.zeros(6),True)
    task.reference[:]=q
    task._enter("approach",sample)
    candidate=task.planned_path_reference(task._duration)
    probe=NativePoseProbe(model)
    try:
        opening=probe.grasp_opening(candidate,"bolt")
        # This height seats the original head tabs in the finger notches.
        # The outer-head grasp at the old height contacted near .302 rad.
        assert .25<opening<.28
    finally:
        probe.close()


def test_child_fastener_reads_actual_body_pose_scalar_addresses_and_contact_force():
    rb = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    relative_position = np.array([1., 2., 3.])
    base_velocity = np.array([7., 8., 9.])
    base_omega = np.array([0., 0., .2])
    base = SimpleNamespace(r_BN_N=np.array([10., 20., 30.]), v_BN_N=base_velocity,
                           sigma_BN=[0., 0., np.tan(np.pi/8)], omega_BN_B=base_omega)
    part = SimpleNamespace(r_BN_N=base.r_BN_N+rb@relative_position,
                           v_BN_N=base_velocity+rb@(np.array([.3, 0., 0.])+np.cross(base_omega, relative_position)),
                           sigma_BN=base.sigma_BN, omega_BN_B=base_omega+[.1, .2, .3])
    scalar_positions = {"locking_rod_hinge": 2, "fastener_slide": 5, "fastener_spin": 8}
    scalar_rates = {"locking_rod_hinge": 3, "fastener_slide": 6, "fastener_spin": 9}

    class Body:
        def __init__(self, name): self.name = name
        def getOrigin(self):
            return SimpleNamespace(stateOutMsg=SimpleNamespace(read=lambda: base if self.name == "cubesat_bus" else part))
        def getScalarJoint(self, name):
            if self.name != "fastener": raise ValueError("not the owner")
            return SimpleNamespace(getQposAdr=lambda: scalar_positions[name], getQvelAdr=lambda: scalar_rates[name])

    names = ["cubesat_bus", "fastener", "hook", "limit_clip", "locking_rod", "ee_finger1", "ee_finger2"]
    scene = SimpleNamespace(getBody=lambda name: Body(name), getBodyNames=lambda: names)
    state, rate = np.arange(12, dtype=float), np.arange(12, dtype=float)/10
    before_state, before_rate = state.copy(), rate.copy()
    rows = np.array([[0, 1, 1, 5, -.00001, 3., 0, 0, 0, 0, 0],
                     [0, 1, 1, 6, -.00001, 4., 0, 0, 0, 0, 0],
                     [0, 1, 1, 2, -.00001, 2.5, 0, 0, 0, 0, 0],
                     [0, 1, 4, 3, -.00001, 1.5, 0, 0, 0, 0, 0]])
    # Contact geometry is in a different local frame from the orbital body
    # messages. The latch must be classified using the matching native pose.
    local_part = np.array([-.7, .8, -.9])
    geometry = rows.copy()
    for index, offset in enumerate(([0., .01965, .009], [0., -.01965, .009],
                                    [0., .02165, .014], [0., 0., 0.])):
        geometry[index, 5:8] = local_part+rb@offset
    from simulation.scripted_tasks.threaded_motion import ThreadedTaskBase
    stepper = SimpleNamespace(nbody=len(names), _name=lambda kind, index: names[index],
                              qpos=state, qvel=rate, contact_force_snapshot=lambda: rows,
                              contact_geometry_snapshot=lambda: geometry,
                              body_geometry_pose=lambda body: (local_part, rb))
    task = SimpleNamespace(name="locking_rod_cycle", config=SimpleNamespace(part_body="fastener"),
                           is_latch_contact=ThreadedTaskBase.is_latch_contact)
    sample = ThreadedState(task, scene, stepper).sample(1., np.zeros(8), np.zeros(8),
                 {"force_n": [0., 0., 0.], "torque_nm": [0., 0., 0.], "valid": True})
    np.testing.assert_allclose(sample.part_position_body_m, relative_position)
    np.testing.assert_allclose(sample.part_rotation_body, np.eye(3), atol=1e-15)
    np.testing.assert_allclose(sample.part_velocity_body_m_s, [.3, 0., 0.], atol=1e-14)
    np.testing.assert_allclose(sample.part_angular_velocity_body_rad_s, [.1, .2, .3], atol=1e-15)
    assert sample.scalar_joints["fastener_slide"] == 5.
    assert sample.scalar_joints["fastener_spin_velocity"] == .9
    assert sample.scalar_joints["fastener_hook_normal_force_n"] == 2.5
    assert sample.scalar_joints["locking_rod_clip_normal_force_n"] == 1.5
    assert sample.finger_part_contacts == (1, 1)
    assert sample.finger_latch_contacts == (1, 1)
    np.testing.assert_array_equal(sample.native_qpos, state)
    assert not np.shares_memory(sample.native_qpos, state)
    np.testing.assert_array_equal(state, before_state)
    np.testing.assert_array_equal(rate, before_rate)


@pytest.mark.parametrize("arm_body",["link5","wrist_ft_tool"])
def test_preflight_updates_only_private_mechanism_state_and_keeps_arm_collision_check(monkeypatch,arm_body):
    from simulation.scripted_tasks import drawer_initialization as initialization

    candidates = np.zeros((2, 8))
    task = SimpleNamespace(lower=np.full(6, -1.), upper=np.ones(6),
             preflight_paths=lambda q: [("lower", candidates, ("fastener",),
                                         {"locking_rod_hinge": [0., -1.4]})])
    monkeypatch.setattr(initialization, "create_threaded_task", lambda *args: task)
    probe = object.__new__(initialization.NativePoseProbe)
    probe.home = np.zeros(12)
    probe.q = probe.home.copy()
    probe.address = lambda name: 10
    probe.names = {0: "ee_finger1", 1: "fastener", 2: arm_body}
    measured = []

    def contacts(candidate):
        measured.append(float(probe.q[10]))
        if probe.q[10] < -1:
            return np.array([[0, 1, 2, 1, -.001]])
        return np.zeros((0, 5))

    probe.contacts = contacts
    result = probe.validate(None, {"randomization": {"arm_joint_position_rad": [0.]*8}}, "locking_rod_cycle")
    assert not result["accepted"] and result["reason"] == "approach_collision"
    assert result["contact"]["body1"] == arm_body
    assert measured == [0., 0., -1.4]
    np.testing.assert_array_equal(probe.home, np.zeros(12))


@pytest.mark.parametrize("moving,environment",[("locking_rod","task_box_base"),
                                              ("fastener","task_box_base"),
                                              ("locking_rod","layout_plug_start"),
                                              ("fastener","layout_drawer")])
def test_threaded_preflight_rejects_mechanism_sweep_into_base_or_foreign_module(monkeypatch,moving,environment):
    from simulation.scripted_tasks import drawer_initialization as initialization

    candidates=np.zeros((2,8))
    task=SimpleNamespace(lower=np.full(6,-1.),upper=np.ones(6),
             preflight_paths=lambda q:[("fold",candidates,("fastener",),
                                        {"locking_rod_hinge":[0.,-1.6]})])
    monkeypatch.setattr(initialization,"create_threaded_task",lambda *args:task)
    probe=object.__new__(initialization.NativePoseProbe)
    probe.home=np.zeros(12)
    probe.q=probe.home.copy()
    probe.address=lambda name:10
    probe.names={0:moving,1:environment}
    probe.contacts=lambda candidate:(np.array([[0,1,0,1,-.0003]])
                                    if probe.q[10]<-1 else np.zeros((0,5)))
    result=probe.validate(None,{"randomization":{"arm_joint_position_rad":[0.]*8}},"locking_rod_cycle")
    assert not result["accepted"]
    assert result["reason"]=="mechanism_environment_collision" and result["stage"]=="fold"
    assert result["contact"]["body1"]==moving and result["contact"]["body2"]==environment
    assert result["contact"]["penetration_m"]==pytest.approx(.0003)
    np.testing.assert_array_equal(probe.home,np.zeros(12))


@pytest.mark.parametrize("pair",[("locking_rod","fastener"),("fastener","hook"),
                                  ("locking_rod","hook"),("locking_rod","limit_clip"),
                                  ("fastener","limit_clip")])
def test_threaded_preflight_preserves_real_internal_rod_and_clip_contacts(monkeypatch,pair):
    from simulation.scripted_tasks import drawer_initialization as initialization

    task=SimpleNamespace(lower=np.full(6,-1.),upper=np.ones(6),
             preflight_paths=lambda q:[("seat",np.zeros((2,8)),("fastener",))])
    monkeypatch.setattr(initialization,"create_threaded_task",lambda *args:task)
    probe=object.__new__(initialization.NativePoseProbe)
    probe.home=np.zeros(12)
    probe.q=probe.home.copy()
    probe.names=dict(enumerate(pair))
    probe.contacts=lambda candidate:np.array([[0,1,0,1,-.0005]])
    result=probe.validate(None,{"randomization":{"arm_joint_position_rad":[0.]*8}},"locking_rod_cycle")
    assert result["accepted"]


@pytest.mark.parametrize("other_geom,accepted",[("board_thread_sdf",True),
                                               ("bolt_storage_thread_sdf",True),
                                               ("unrelated_board_convex",False)])
def test_bolt_preflight_allows_exact_mating_geoms_without_exempting_whole_board(monkeypatch,other_geom,accepted):
    from simulation.scripted_tasks import drawer_initialization as initialization

    task=SimpleNamespace(lower=np.full(6,-1.),upper=np.ones(6),
             preflight_paths=lambda q:[("turn",np.zeros((2,8)),("bolt",))])
    monkeypatch.setattr(initialization,"create_threaded_task",lambda *args:task)
    probe=object.__new__(initialization.NativePoseProbe)
    probe.home=np.zeros(12)
    probe.q=probe.home.copy()
    probe.names={0:"bolt",1:"asm_board1"}
    probe.geom_name=lambda geom:{0:"bolt_thread_sdf",1:other_geom}[int(geom)]
    probe.contacts=lambda candidate:np.array([[0,1,0,1,-.0003]])
    result=probe.validate(None,{"randomization":{"arm_joint_position_rad":[0.]*8}},"aviation_bolt_insert")
    assert result["accepted"] is accepted
    if not accepted:
        assert result["reason"]=="mechanism_environment_collision"
        assert result["contact"]["geom2"]=="unrelated_board_convex"


@pytest.mark.parametrize("stage,accepted",[("target_turn",True),("target_release",False),("retract",False)])
def test_bolt_withdrawal_rejects_shallow_nonfinger_obstruction(stage,accepted):
    from simulation.scripted_tasks.drawer_initialization import NativePoseProbe

    probe=object.__new__(NativePoseProbe)
    probe.home=np.zeros(12)
    probe.q=probe.home.copy()
    probe.names={0:"ee_link21",1:"guide_plug"}
    probe.contacts=lambda candidate:np.array([[0,1,0,1,-.00016]])
    paths=[(stage,np.zeros((2,8)),("bolt",),{})]
    result=probe.check_threaded_paths("aviation_bolt_insert",paths)
    assert result["accepted"] is accepted
    if not accepted:
        assert result["contact"]["penetration_m"]==pytest.approx(.00016)


def test_private_measured_baseline_preserves_actual_passive_linkage_offsets():
    from simulation.scripted_tasks.drawer_initialization import NativePoseProbe

    probe=object.__new__(NativePoseProbe)
    probe.home=np.zeros(9)
    probe.q=probe.home.copy()
    probe.passive_addresses={"passive":8}
    probe.addresses=np.arange(8)
    probe.v=np.zeros(9)
    probe.handle=None
    probe.ptr=lambda array:array
    forwarded=[]
    probe.lib=SimpleNamespace(
        lms_set_state=lambda handle,q,v:forwarded.append(q.copy()) or 0,
        lms_contacts=lambda *args:0,
    )
    probe.gripper_kinematics=SimpleNamespace(passive_positions=lambda jaws:{"passive":2.*jaws[0]})
    joints=np.r_[np.zeros(6),.30,-.30]
    measured=np.r_[joints,.62]
    probe.use_measured_qpos(measured,joints)
    np.testing.assert_array_equal(probe.home,measured)
    assert probe.passive_position_offsets["passive"]==pytest.approx(.02)
    probe.contacts(joints)
    np.testing.assert_array_equal(forwarded[-1],measured)
    opened=joints.copy()
    opened[6:]=[.40,-.40]
    probe.contacts(opened)
    assert forwarded[-1][8]==pytest.approx(.82)
    np.testing.assert_array_equal(measured,np.r_[joints,.62])


def bolt_recording():
    from simulation.scripted_tasks.aviation_bolt import AviationBoltConfig, AviationBoltTask

    params = asdict(AviationBoltConfig())
    stages = ["settle", "approach", "grip", "unscrew", "carry", "screw_in", "release", "retract", "verify"]
    sample = dict(task_version=AviationBoltTask.version, task_parameters=params, result=None,
                  reference_motion_quality=valid_motion_quality(),
                  finger_latch_contacts=[1, 1], latch_engaged=True,
                  part_body="bolt", target_site="bolt_assembly_hole", finger_part_contacts=[1, 1],
                  source_unscrewed=True, ever_lifted=True, source_rotation_rad=24., target_rotation_rad=24.,
                  source_required_rotation_rad=22.8, target_required_rotation_rad=23.6,
                  source_displacement_m=.0585, source_tip_clearance_m=.0005,
                  radial_error_m=0., seat_gap_m=0., axis_error_rad=0.,
                  part_velocity_body_m_s=[0., 0., 0.], part_angular_velocity_body_rad_s=[0., 0., 0.])
    observations = []
    for stage in stages[:-1] + ["verify"]*55:
        row = dict(sample, stage=stage)
        if stage in ("release", "retract", "verify"): row["finger_part_contacts"] = [0, 0]
        observations.append(dict(sim_time_ns=str(len(observations)*20_000_000), scripted_task=row,
                                 gripper_opening_normalized=.8))
    observations[-1]["scripted_task"]["result"] = "success"
    return observations, stages


def test_recording_acceptance_requires_actual_turns_and_every_physical_operation():
    observations, stages = bolt_recording()
    validate_threaded_recording(observations, stages, "aviation_bolt_insert")
    incomplete_turn = deepcopy(observations)
    incomplete_turn[-1]["scripted_task"]["target_rotation_rad"] = 1.
    with pytest.raises(AssertionError):
        validate_threaded_recording(incomplete_turn, stages, "aviation_bolt_insert")
    with pytest.raises(AssertionError):
        validate_threaded_recording(observations, [s for s in stages if s != "unscrew"], "aviation_bolt_insert")


@pytest.mark.parametrize("field,value",[("source_tip_clearance_m",-.00042),
                                       ("source_rotation_rad",22.74)])
def test_bolt_recording_rejects_carry_before_actual_thread_tip_disengages(field,value):
    observations,stages=bolt_recording()
    carry=next(obs["scripted_task"] for obs in observations if obs["scripted_task"]["stage"]=="carry")
    carry[field]=value
    with pytest.raises(AssertionError):
        validate_threaded_recording(observations,stages,"aviation_bolt_insert")


def rod_recording():
    from simulation.scripted_tasks.locking_rod import LockingRodConfig,LockingRodTask
    from simulation.model_preparation.captive_fastener import LEAD

    config=LockingRodConfig()
    seated=.0055
    spin=seated*2*np.pi/LEAD
    stages=list(LockingRodTask.stages)
    template=dict(task_version=LockingRodTask.version,task_parameters=asdict(config),
                  reference_motion_quality=valid_motion_quality(),
                  finger_latch_contacts=[1, 1],latch_engaged=True,
                  result=None,part_body="fastener",finger_part_contacts=[1,1],
                  loosen_verified=True,folded_verified=True,returned_verified=True,tightened_verified=True,
                  released_stable_seconds=config.verify_hold_s,hinge_rad=0.,
                  fastener_slide_m=seated,fastener_spin_rad=spin,fastener_helix_error_m=0.,
                  fastener_hook_contacts=0,fastener_hook_normal_force_n=0.,
                  locking_rod_clip_contacts=0,fastener_clip_contacts=0,
                  tighten_seated_slide_m=seated,tighten_seated_spin_rad=spin,
                  tighten_peak_hook_force_n=2.,fastener_thread_lead_m=LEAD,
                  part_velocity_body_m_s=[0.,0.,0.],part_angular_velocity_body_rad_s=[0.,0.,0.],
                  scalar_joints={"fastener_hook_contacts":0,"locking_rod_hinge_velocity":0.,
                                 "fastener_spin_velocity":0.})
    observations=[]
    for stage in stages[:-1]+["verify"]*55:
        row=deepcopy(template)
        row["stage"]=stage
        if stage in ("loosen","lower"):
            row.update(fastener_slide_m=0.,fastener_spin_rad=0.)
        if stage in ("release_down","clear_down","reapproach"):
            row.update(hinge_rad=config.folded_hinge_rad,locking_rod_clip_contacts=1,
                       fastener_slide_m=0.,fastener_spin_rad=0.)
        if stage=="tighten":
            row.update(fastener_hook_contacts=1,fastener_hook_normal_force_n=2.)
        if stage in ("release","retract","verify"):
            row["finger_part_contacts"]=[0,0]
        observations.append(dict(sim_time_ns=str(len(observations)*20_000_000),scripted_task=row,
                                 gripper_opening_normalized=.8))
    observations[-1]["scripted_task"]["result"]="success"
    return observations,stages


def valid_motion_quality():
    return dict(samples=1000, max_speed_rad_s=.2, max_acceleration_rad_s2=.6,
                max_jerk_rad_s3=8., max_stage_velocity_jump_rad_s=.003,
                rotation_samples=500, rotation_latch_samples=500,
                rotation_max_wrist_axis_error_rad=.001)


@pytest.mark.parametrize("field,value", [("max_speed_rad_s", .8),
                                        ("max_acceleration_rad_s2", 40.),
                                        ("max_stage_velocity_jump_rad_s", .233),
                                        ("rotation_latch_samples", 0),
                                        ("rotation_max_wrist_axis_error_rad", np.pi/3)])
def test_recording_rejects_jitter_or_wrong_turning_grasp_even_after_task_success(field, value):
    observations, stages = bolt_recording()
    observations[-1]["scripted_task"]["reference_motion_quality"][field] = value
    with pytest.raises(AssertionError):
        validate_threaded_recording(observations, stages, "aviation_bolt_insert")


def test_general_head_contact_during_rotation_does_not_substitute_for_latch():
    observations, stages = bolt_recording()
    for observation in observations:
        row = observation["scripted_task"]
        if row["stage"] == "screw_in":
            row.update(latch_engaged=False, finger_latch_contacts=[0, 0])
    with pytest.raises(AssertionError):
        validate_threaded_recording(observations, stages, "aviation_bolt_insert")


def test_rod_recording_accepts_naturally_unloaded_release_after_real_tightening():
    observations,stages=rod_recording()
    assert observations[-1]["scripted_task"]["fastener_hook_normal_force_n"]==0.
    validate_threaded_recording(observations,stages,"locking_rod_cycle")


@pytest.mark.parametrize("factory,task", [(bolt_recording, "aviation_bolt_insert"),
                                         (rod_recording, "locking_rod_cycle")])
@pytest.mark.parametrize("release_opening", [.45, .75])
def test_released_recording_requires_configured_jaw_clearance_during_entire_hold(factory, task, release_opening):
    observations, stages = factory()
    for obs in observations:
        obs["scripted_task"]["task_parameters"]["release_open_rad"] = release_opening
        obs["gripper_opening_normalized"] = release_opening
    validate_threaded_recording(observations, stages, task)
    insufficient = deepcopy(observations)
    insufficient[-1]["gripper_opening_normalized"] = release_opening-.02
    with pytest.raises(AssertionError):
        validate_threaded_recording(insufficient, stages, task)
    # A final open frame cannot hide a reclosed jaw during the released hold.
    insufficient = deepcopy(observations)
    insufficient[-10]["gripper_opening_normalized"] = release_opening-.02
    with pytest.raises(AssertionError):
        validate_threaded_recording(insufficient, stages, task)
    still_touching = deepcopy(observations)
    still_touching[-10]["scripted_task"]["finger_part_contacts"] = [1, 0]
    with pytest.raises(AssertionError):
        validate_threaded_recording(still_touching, stages, task)


def test_release_target_cannot_be_lowered_below_swept_physical_head_clearance():
    observations, stages = bolt_recording()
    for obs in observations:
        obs["scripted_task"]["task_parameters"]["release_open_rad"] = .3
    with pytest.raises(AssertionError):
        validate_threaded_recording(observations, stages, "aviation_bolt_insert")


@pytest.mark.parametrize("failure",["no_seating_contact","misaligned_seating_contact"])
def test_rod_recording_rejects_missing_real_aligned_tightening_even_with_peak_claim(failure):
    observations,stages=rod_recording()
    tight=next(obs["scripted_task"] for obs in observations if obs["scripted_task"]["stage"]=="tighten")
    if failure=="no_seating_contact":
        tight.update(fastener_hook_contacts=0,fastener_hook_normal_force_n=0.)
    else:
        tight["hinge_rad"]=.2
    with pytest.raises(AssertionError):
        validate_threaded_recording(observations,stages,"locking_rod_cycle")


def test_rod_recording_rejects_excess_backoff_from_physically_reached_seat():
    observations,stages=rod_recording()
    final=observations[-1]["scripted_task"]
    backoff=final["task_parameters"]["maximum_seat_backoff_m"]+.00001
    for obs in observations:
        row=obs["scripted_task"]
        if row["stage"]=="verify":
            row["fastener_slide_m"]-=backoff
            row["fastener_spin_rad"]-=backoff*2*np.pi/final["fastener_thread_lead_m"]
    # Both the broad seating interval and the helix remain satisfied. Only
    # the actual achieved-seat comparison distinguishes this unscrewed state.
    assert final["fastener_slide_m"]>=final["task_parameters"]["tight_slide_min_m"]
    with pytest.raises(AssertionError):
        validate_threaded_recording(observations,stages,"locking_rod_cycle")
