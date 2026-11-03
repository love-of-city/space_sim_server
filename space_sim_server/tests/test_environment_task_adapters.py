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
    from simulation.scripted_tasks.plug_insert import PlugSample
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
    from test_scripted_plug_task import task_sample
    monkeypatch.setattr('simulation.serial_chain_kinematics.SerialChainKinematics.enable_native_geometry',lambda self:None)
    _,sample=task_sample();task=EnvironmentPlugTask(MODEL)
    task.step(sample)
    monkeypatch.setattr('simulation.scripted_tasks.environment_tasks.plug_hold_opening',lambda task,sample:.37,raising=False)
    task._enter('grip',replace(sample,native_qpos=np.zeros(1)))
    assert task._closed_reference==pytest.approx(.37)


def test_geometry_preloaded_grasp_can_finish_without_zero_command(monkeypatch):
    from dataclasses import replace
    from test_scripted_plug_task import task_sample
    task,sample=task_sample();task._enter('grip',sample)
    task._closed_reference=.37;task.reference[6:]=[.37,-.37]
    q=sample.joints.copy();q[6:]=[.43,-.43]
    completed=[]
    monkeypatch.setattr(task,'_next',lambda sample:completed.append(True))
    task.step(replace(sample,time_s=6.1,joints=q,finger_plug_contacts=(1,1)))
    assert completed and task.result is None


def test_runner_protection_cannot_exceed_requested_40N_4Nm():
    from tools.run_scripted_task import benchmark_wrist_protection
    assert benchmark_wrist_protection(80.)==dict(force_limit_n=40.,torque_limit_nm=4.)
    assert benchmark_wrist_protection(30.)==dict(force_limit_n=30.,torque_limit_nm=4.)


def test_plug_operation_preflight_uses_same_zero_penetration_as_audit(monkeypatch):
    from simulation.scripted_tasks import operation_clearance
    from test_scripted_plug_task import task_sample
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
    monkeypatch.setattr(operation_clearance,'NativePoseProbe',Probe)
    monkeypatch.setattr(operation_clearance,'solve_pose',lambda *args:np.zeros(6))
    assert operation_clearance.check_operation(task,sample)['accepted']
