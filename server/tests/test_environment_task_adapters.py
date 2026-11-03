"""New planners must preserve local contact controllers and fixed protections."""
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from simulation.scripted_tasks.environment_tasks import EnvironmentPlugTask
from simulation.scripted_tasks.motion_timing import time_joint_path

MODEL=Path(__file__).resolve().parents[1]/'model/SARM/platform/sarm_20261012.xml'


def test_planning_adapter_keeps_validated_cylinder_controller_and_limits(monkeypatch):
    monkeypatch.setattr('simulation.serial_chain_kinematics.SerialChainKinematics.enable_native_geometry',lambda self:None)
    task=EnvironmentPlugTask(MODEL)
    assert task.config.force_limit_n==40 and task.config.torque_limit_nm==4
    assert callable(task._advance_insertion_goal) and callable(task._unload_before_release)
    assert task.environment_planning


def test_path_reference_keeps_held_fingers_after_searched_carry():
    timing=time_joint_path(np.array([np.zeros(6),np.ones(6)*.01]),.01,0.,joint_speed=.08,
        linear_speed=.025,angular_speed=.2,linear_acceleration=.15,angular_acceleration=.4,joint_acceleration=.6)
    task=SimpleNamespace(_timing=timing,reference=np.r_[np.zeros(6),.4,-.4])
    for u in np.linspace(0,1,8):
        q=EnvironmentPlugTask.path_reference(task,u)
        assert q[6:]==pytest.approx([.4,-.4])
        assert q[:6]==pytest.approx(timing.path.at(u))


def test_native_sample_carries_measured_full_scene_state():
    from simulation.scripted_tasks.scene_plug import PlugSample
    from simulation.scripted_tasks.drawer_lift import DrawerSample
    assert 'native_qpos' in PlugSample.__dataclass_fields__
    assert 'native_qpos' in DrawerSample.__dataclass_fields__


def test_drawer_retract_fingers_do_not_advance_when_arm_clock_is_paused():
    from simulation.scripted_tasks.drawer_cycle import DrawerCycleTask
    timing=SimpleNamespace(path=SimpleNamespace(coefficients=np.zeros((1,6))))
    finger=SimpleNamespace(duration=5.,joint_state=lambda t:(np.array([t/5]),None,None,None))
    task=SimpleNamespace(stage='retract',environment_planning=True,_timing=timing,_gripper_timing=finger,
        _gripper_motion_s=.5,_retract_gripper_entry_s=.5,_motion_s=0.,reference=np.zeros(8))
    for _ in range(5):DrawerCycleTask._step_gripper(task,1.,.1)
    assert task._gripper_motion_s==pytest.approx(.5)
    assert task.reference[6]==pytest.approx(.1)


def test_drawer_failed_template_reaches_search_without_endpoint_timing(monkeypatch):
    from dataclasses import replace
    from test_scripted_drawer_cycle import setup_cycle
    from simulation.scripted_tasks.drawer_lift import DrawerLiftTask
    task,sample=setup_cycle()
    task.environment_planning=True
    sample=replace(sample,native_qpos=np.zeros(40))
    def failed_template(self,name,sample):
        self._timing=None
        self._duration=1.
        self._goal=np.zeros(3)
        self._rotation=np.eye(3)
        raise ValueError('template IK failed')
    class SearchReached(Exception):pass
    def search(*args,**kwargs):
        assert kwargs['error']=='template IK failed'
        raise SearchReached
    monkeypatch.setattr(DrawerLiftTask,'_enter',failed_template)
    monkeypatch.setattr('simulation.scripted_tasks.environment_motion.plan',search)
    with pytest.raises(SearchReached):task._enter('retract',sample)


def test_grip_uses_native_contact_opening_instead_of_zero_preload(monkeypatch):
    from dataclasses import replace
    from scene_plug_fixtures import task_sample
    monkeypatch.setattr('simulation.serial_chain_kinematics.SerialChainKinematics.enable_native_geometry',lambda self:None)
    _,sample=task_sample();task=EnvironmentPlugTask(MODEL)
    task.step(sample)
    monkeypatch.setattr('simulation.scripted_tasks.environment_tasks.plug_hold_opening',lambda task,sample:.37,raising=False)
    task._enter('grip',replace(sample,native_qpos=np.zeros(1)))
    assert task._closed_reference==pytest.approx(.37)


def test_geometry_preloaded_grasp_can_finish_without_zero_command(monkeypatch):
    from dataclasses import replace
    from scene_plug_fixtures import task_sample
    task,sample=task_sample();task._enter('grip',sample)
    task._closed_reference=.37;task.reference[6:]=[.37,-.37]
    q=sample.joints.copy();q[6:]=[.43,-.43]
    completed=[]
    monkeypatch.setattr(task,'_next',lambda sample:completed.append(True))
    task.step(replace(sample,time_s=6.1,joints=q,finger_plug_contacts=(1,1)))
    assert completed and task.result is None


def test_runner_protection_cannot_exceed_requested_40N_4Nm():
    from collection.run_scripted_task import benchmark_wrist_protection
    assert benchmark_wrist_protection(80.)==dict(force_limit_n=40.,torque_limit_nm=4.)
    assert benchmark_wrist_protection(30.)==dict(force_limit_n=30.,torque_limit_nm=4.)


def test_plug_operation_preflight_uses_same_zero_penetration_as_audit(monkeypatch):
    from simulation.scripted_tasks import operation_clearance
    from scene_plug_fixtures import task_sample
    task,sample=task_sample();task.model_path=MODEL;task.motion_plans=[]
    task.stage='insert';task._seat_z=.03
    class Probe:
        home=np.zeros(1)
        def __init__(self,*args,**kwargs):pass
        def contacts(self,*args):return []
        def body_pose(self,*args):return np.zeros(3),np.eye(3)
        def pose_collision(self,*args,**kwargs):
            assert kwargs.get('arm_penetration_m')==0.
            assert kwargs.get('mechanism_clearance_m')==0.
        def close(self):pass
    monkeypatch.setattr(operation_clearance,'acquire_probe',lambda *args,**kwargs: Probe())
    monkeypatch.setattr(operation_clearance,'solve_pose',lambda *args:np.zeros(6))
    assert operation_clearance.check_operation(task,sample)['accepted']


def test_collection_and_scene_experts_keep_distinct_motion_and_completion_contracts():
    from collection.run_scripted_task import uses_scene_expert
    from simulation.scripted_tasks.plug_insert import PlugInsertConfig as CollectionPlugConfig
    from simulation.scripted_tasks.scene_plug import PlugInsertConfig as ScenePlugConfig
    from simulation.scripted_tasks.drawer_cycle import DrawerCycleTask, DrawerCycleConfig
    from simulation.scripted_tasks.scene_drawer import SceneDrawerCycleTask, SceneDrawerCycleConfig
    assert not uses_scene_expert(SimpleNamespace())
    assert uses_scene_expert(SimpleNamespace(scene_expert=True))
    assert uses_scene_expert(SimpleNamespace(replay_trajectory=Path("frozen.json")))
    collection, scene = CollectionPlugConfig(), ScenePlugConfig()
    assert collection.insert_speed_m_s == .025 and not collection.overload_stop_enabled
    assert scene.insert_speed_m_s == .002
    assert DrawerCycleConfig().verify_hold_s == .3
    assert SceneDrawerCycleConfig().verify_hold_s == 2.
    current = DrawerCycleTask(MODEL)
    benchmark = SceneDrawerCycleTask(MODEL)
    current.stage = benchmark.stage = "grip"
    assert current._stable_time() == .15
    assert benchmark._stable_time() == 1.05
    from tools.diagnostics.validate_task_trajectory_contacts import ContactPolicy
    assert "extract" in ContactPolicy(MODEL, "aviation_plug_insert").required_stages
    assert "extract" not in ContactPolicy(MODEL, "aviation_plug_insert", scene_expert=True).required_stages


def test_scene_plug_preparation_keeps_immutable_scene_without_collection_colours(tmp_path):
    from simulation.scripted_tasks.randomization import prepare_episode, write_model_variant
    scene = {"scripted_task_name": "guide_plug_insert", "collection_config": {"controller_family": "scene_expert"}}
    prepare_episode(scene, collection_features=False)
    assert "episode_color_assignment" not in scene["collection_config"]
    assert "color_randomization" not in scene["collection_config"]
    assert write_model_variant(MODEL, tmp_path / "variant.xml", scene, collection_features=False) == MODEL
    assert not (tmp_path / "variant.xml").exists()


def test_scene_dynamics_randomization_does_not_initialize_collection_mechanisms(tmp_path):
    import xml.etree.ElementTree as ET
    from simulation.scripted_tasks.randomization import prepare_episode, write_model_variant
    scene = {"seed": 42, "scripted_task_name": "locking_rod_cycle",
             "collection_randomization": {"dynamics": {"enabled": True}}}
    prepare_episode(scene, collection_features=False)
    result = write_model_variant(MODEL, tmp_path / "variant.xml", scene, collection_features=False)
    record = scene["collection_config"]["model_randomization"]
    assert record["scope"] == "all_bodies_and_contact_geoms"
    assert record["task_initialization"] == {}
    before = ET.parse(MODEL).find('.//joint[@name="fastener_slide"]')
    after = ET.parse(result).find('.//joint[@name="fastener_slide"]')
    assert after.attrib == before.attrib
