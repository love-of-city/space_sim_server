"""All collection task adapters must plan the actual body and finger motion."""

from types import SimpleNamespace
import numpy as np
import pytest
from simulation.scripted_tasks import environment_motion as motion
from simulation.scripted_tasks.motion_timing import time_joint_path


@pytest.mark.parametrize(
    "name,body,joint",
    [
        ("aviation_plug_insert", "aviation_plug", "aviation_plug_free"),
        ("guide_plug_insert", "guide_plug", "guide_plug_free"),
        ("cylindrical_plug_insert", "cylindrical_plug", "cylindrical_plug_free"),
        ("drawer_cycle", "drawer_base", None),
        ("locking_rod_cycle", "fastener", None),
    ],
)
def test_adapter_preserves_actual_scene_and_held_body(
    monkeypatch, tmp_path, name, body, joint
):
    model = tmp_path / "model.xml"
    model.write_text(
        f'<mujoco><worldbody><body name="{body}"><freejoint name="{joint}"/></body></worldbody></mujoco>'
    )
    config = SimpleNamespace(
        part_body=body,
        plug_body=body,
        arm_speed_rad_s=0.35,
        linear_speed_m_s=0.1,
        carry_speed_m_s=0.05,
        gripper_speed_rad_s=0.2,
    )
    task = SimpleNamespace(
        name=name,
        config=config,
        model_path=model,
        path_reference=lambda u: np.zeros(8),
        drawer_origin=np.zeros(3),
        axis=np.array([0.0, 0.0, 1.0]),
        drawer_rotation=np.eye(3),
        _required_scalars=("fastener_slide",),
        motion_plans=[],
        stage="lift",
    )
    actual = np.arange(20.0)
    sample = SimpleNamespace(
        native_qpos=actual,
        joints=np.zeros(8),
        drawer_m=0.12,
        buckle_rad=0.2,
        plug_position_body_m=np.ones(3),
        plug_rotation_body=np.eye(3),
        part_position_body_m=np.ones(3),
        part_rotation_body=np.eye(3),
        scalar_joints={"fastener_slide": 0.04},
    )
    captured = []

    def plan(proxy, observed, *args, **kw):
        captured.append((proxy, observed, kw))
        proxy.motion_plans.append({"route": "template"})
        return {"route": "template"}

    monkeypatch.setattr(motion, "plan_free_motion", plan)
    motion.plan(
        task,
        sample,
        np.ones(3),
        np.eye(3),
        np.array([0.0, 0.0, 1.0]),
        held=name.endswith("_insert"),
    )
    proxy, observed, kw = captured[0]
    assert proxy.planning_part_body == body
    assert proxy.planning_part_joint == joint
    assert observed.native_qpos is actual
    assert len(task.motion_plans) == 1
    if name.endswith("_insert"):
        assert proxy.config.linear_speed_m_s == 0.05
    elif name == "drawer_cycle":
        assert proxy.planning_scalars == {
            "drawer_base_slide": 0.12,
            "drawer_buckle_joint": 0.2,
        }
    else:
        assert proxy.planning_scalars == {"fastener_slide": 0.04}


def timing(points):
    return time_joint_path(
        points,
        0.0,
        0.0,
        joint_speed=0.2,
        linear_speed=0.1,
        angular_speed=0.2,
        linear_acceleration=0.2,
        angular_acceleration=0.4,
        joint_acceleration=0.4,
    )


def test_release_sweep_tracks_continuing_opening_and_never_mutates_clock():
    from simulation.scripted_tasks.plug_insert import PlugInsertTask

    arm = timing([np.zeros(6), np.ones(6) * 0.1])
    fingers = timing([[0.65], [0.25]])
    task = SimpleNamespace(
        _timing=arm,
        _gripper_timing=fingers,
        _release_opening_s=0.6,
        _gripper_reverse=True,
        _opening_at_entry=0.3,
        stage="retract",
    )
    for u in np.linspace(0.0, 1.0, 21):
        q = PlugInsertTask.path_reference(task, u)
        seconds = arm.seconds_at(u)
        assert arm.at(seconds) == pytest.approx(u, abs=1e-8)
        expected = fingers.joint_state(fingers.duration - 0.6 - seconds)[0][0]
        assert q[6] == pytest.approx(expected, abs=1e-14)
        assert q[7] == -q[6]
    assert task._release_opening_s == 0.6


def test_searched_retract_handoff_holds_open_fingers_instead_of_replaying_release():
    from dataclasses import replace
    from pathlib import Path
    from simulation.scripted_tasks.plug_insert import PlugInsertTask, PlugSample
    from space_arm_platform.control_defaults import DEFAULT_OPERATING_JOINT_DEG

    model = Path(__file__).resolve().parents[1] / 'model/SARM/platform/sarm_20261012.xml'
    task = PlugInsertTask(model)
    q = np.r_[np.deg2rad(DEFAULT_OPERATING_JOINT_DEG), .65, -.65]
    sample = PlugSample(0., q, np.zeros(8), np.array([.3695,.33003,.032]),
                        np.diag([1.,-1.,-1.]), np.zeros(3), np.zeros(3), np.zeros(6), True)
    task.step(sample)
    task.stage = 'retract'
    task._gripper_timing = timing([[.65], [.25]])
    task._gripper_reverse = True
    task._release_unloaded = True
    task._release_opening_s = .6
    # A searched 7D path has already finished opening; its predecessor's
    # independent release clock stayed frozen while this path owned the jaws.
    task._timing = timing([np.r_[q[:6], .4], q[:7]])
    task._duration = task._motion_s = task._timing.duration
    task.reference[:] = q
    task._enter('verify', sample)
    for tick in range(1, 5):
        reference, velocity = task.step(replace(sample,time_s=tick*.005))
        np.testing.assert_allclose(reference[6:], [.65,-.65], atol=1e-12)
        np.testing.assert_allclose(velocity[6:], [0.,0.], atol=1e-12)


def test_socket_exemption_does_not_exempt_arm_or_other_obstacles():
    from simulation.scripted_tasks.drawer_initialization import NativePoseProbe
    probe = NativePoseProbe.__new__(NativePoseProbe)
    probe.handle = None
    probe.q = np.zeros(1)
    probe.home = np.zeros(1)
    probe.names = {1:'aviation_plug',2:'asm_board2',3:'ee_finger1',4:'fastener'}
    probe.geom_name = lambda geom: 'holder_geom'
    candidate = np.zeros(8)
    row = [1,2,1,2,-.001]
    probe.contacts = lambda q:[row]
    assert probe.pose_collision(candidate,'aviation_plug_insert',axial_socket=('aviation',True)) is None
    row[:] = [1,2,3,2,-.001]
    assert probe.pose_collision(candidate,'aviation_plug_insert',('aviation_plug',),axial_socket=('aviation',True))
    row[:] = [1,2,1,4,-.001]
    assert probe.pose_collision(candidate,'aviation_plug_insert',axial_socket=('aviation',True))


def test_search_adapter_installs_selected_retreat_goal_for_runtime_completion(monkeypatch):
    task=SimpleNamespace(name='locking_rod_cycle',stage='retract',
        config=SimpleNamespace(part_body='fastener',arm_speed_rad_s=.35,
                               linear_speed_m_s=.1,gripper_speed_rad_s=.2),
        _required_scalars=(),path_reference=lambda u:np.zeros(8))
    sample=SimpleNamespace(joints=np.zeros(8),scalar_joints={},
                            part_position_body_m=np.zeros(3),part_rotation_body=np.eye(3))
    target=np.array([.1,.2,.12])
    def searched(proxy,observed,*args,**kwargs):
        assert kwargs['fallback_positions'] is goals
        proxy._path=np.array([np.zeros(7),np.r_[target,np.zeros(3),.65]])
        proxy._timing=timing(proxy._path)
        proxy._duration=proxy._timing.duration
        proxy._goal,proxy._rotation=target.copy(),np.eye(3)
        return {'route':'searched','goal_candidate_index':1}
    goals=[np.array([.1,.2,0.]),target]
    monkeypatch.setattr(motion,'plan_free_motion',searched)
    motion.plan(task,sample,goals[0],np.eye(3),np.array([0.,0.,1.]),fallback_positions=goals)
    np.testing.assert_allclose(task._goal,target)
    np.testing.assert_allclose(task._timing.path.at(1.)[:3],target)
