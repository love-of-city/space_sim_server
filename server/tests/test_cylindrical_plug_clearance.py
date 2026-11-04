from collection.recorded_task_contracts import recorded_stages
"""Regress the long-shaft grasp and the blocked side retreat with real geometry."""

from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from simulation.scripted_tasks.plug_insert import PlugInsertTask, PlugSample
from simulation.scripted_tasks.plug_motion_config import PlugInsertConfig
from simulation.scripted_tasks.plug_clearance import cylindrical_hold_opening, plan_cylindrical_retract
from simulation.scripted_tasks.motion import solve_pose
from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix, matrix_to_quaternion_wxyz


MODEL = Path(__file__).resolve().parents[1] / "model/SARM/platform/sarm_20261012.xml"


def initial_task(*, native_geometry=True):
    task = PlugInsertTask(MODEL)
    if native_geometry:
        task.chain.enable_native_geometry()
    element = ET.parse(MODEL).getroot().find('.//body[@name="cylindrical_plug"]')
    position = np.fromstring(element.get("pos"), sep=" ")
    rotation = quaternion_wxyz_to_matrix(np.fromstring(element.get("quat"), sep=" "))
    q = np.r_[np.deg2rad([90., -60., 60., 0., -90., 0.]), .45, -.45]
    sample = PlugSample(0., q, np.zeros(8), position, rotation,
                        np.zeros(3), np.zeros(3), np.zeros(6), True)
    task.step(sample)
    return task, sample


def test_cylindrical_height_and_version_do_not_change_other_connectors_or_explicit_offsets():
    assert PlugInsertConfig().grasp_offset_m == (0., 0., -.006)
    assert PlugInsertConfig(plug_body="aviation_plug").grasp_offset_m == (0., 0., -.003)
    assert PlugInsertConfig(plug_body="guide_plug").grasp_offset_m == (0., 0., -.003)
    assert PlugInsertConfig(grasp_offset_m=(0., 0., .001)).grasp_offset_m == (0., 0., .001)
    version = PlugInsertTask.task_version("cylindrical_plug_insert")
    assert version == PlugInsertTask.version
    assert recorded_stages('cylindrical_plug_insert', version) == recorded_stages('cylindrical_plug_insert', 'plug-insert/11-all-latch-aligned')
    assert recorded_stages('cylindrical_plug_insert', 'plug-insert/12-cylindrical-clearance') == recorded_stages('cylindrical_plug_insert', version)
    assert recorded_stages('cylindrical_plug_insert', 'plug-insert/12-round-plug-release') == recorded_stages('cylindrical_plug_insert', version)
    from simulation.scripted_tasks.task_initialization import randomized_scene
    scene = randomized_scene({"randomization": {"arm_joint_position_rad": [0.] * 8}}, 42,
                             task_name="cylindrical_plug_insert")
    assert scene["collection_config"]["task_version"] == version
    for historical in ('plug-insert/13-cylindrical-clearance-gentle-release', 'plug-insert/13-straight-approach'):
        assert recorded_stages('cylindrical_plug_insert', historical) == recorded_stages('cylindrical_plug_insert', version)


@pytest.mark.parametrize("clearance_result", [False, True])
def test_native_release_queries_geometry_before_fixed_aperture_and_gates_retreat(monkeypatch, clearance_result):
    # The geometry query below is controlled by this state-machine test.
    task, sample = initial_task(native_geometry=False)
    task.stage = "release"
    task._release_unloaded = True
    task._release_start_opening = .45
    task._gripper_timing = None
    task._path = None
    task._entered = 0.
    task._timeout = 90.
    sample = replace(sample, finger_plug_contacts=(0, 0), native_qpos=np.zeros(1))
    monkeypatch.setattr(task, "_hold_release_pose", lambda *_: None)
    calls, transitions = [], []
    monkeypatch.setattr(task, "_next", lambda observed: transitions.append(observed.time_s))

    def clearance(_task, observed):
        calls.append(observed.time_s)
        # Actual fingers are clear, but the old +.08-rad gate is not met.
        assert min(observed.joints[6], -observed.joints[7]) < .53
        return clearance_result

    monkeypatch.setattr("simulation.scripted_tasks.plug_clearance.cylindrical_release_clearance", clearance)
    for tick in range(1, 5):
        task.step(replace(sample, time_s=tick*.02))
    assert not calls and not transitions
    task.step(replace(sample, time_s=.12))
    assert calls == [.12]
    assert bool(transitions) is clearance_result


@pytest.mark.parametrize("unloaded,contacts,native", [
    (False, (0, 0), True),
    (True, (1, 0), True),
    (True, (0, 0), False),
])
def test_release_geometry_cannot_bypass_unloading_contacts_or_non_native_aperture(monkeypatch, unloaded, contacts, native):
    task, sample = initial_task(native_geometry=False)
    task.stage = "release"
    task._release_unloaded = unloaded
    task._release_start_opening = .45
    task._gripper_timing = None
    task._path = None
    task._entered = 0.
    task._timeout = 90.
    sample = replace(sample, finger_plug_contacts=contacts,
                     native_qpos=np.zeros(1) if native else None)
    monkeypatch.setattr(task, "_hold_release_pose", lambda *_: None)
    calls, transitions = [], []
    monkeypatch.setattr(task, "_next", lambda observed: transitions.append(observed.time_s))
    monkeypatch.setattr("simulation.scripted_tasks.plug_clearance.cylindrical_release_clearance",
                        lambda *_: calls.append(True) or True)
    for tick in range(1, 11):
        task.step(replace(sample, time_s=tick*.02))
    assert not calls and not transitions


def test_native_grasp_centers_both_original_tabs_and_bounds_servo_preload():
    pytest.importorskip("Basilisk")
    from simulation.scripted_tasks.native_geometry_probe import NativePoseProbe

    task, sample = initial_task()
    goal, rotation = task._tracked_grasp(sample)
    task.reference[:6] = solve_pose(task.chain, goal, rotation, sample.joints[:6], task.lower, task.upper)
    probe = NativePoseProbe(MODEL)
    try:
        probe.contacts(sample.joints)
        sample = replace(sample, native_qpos=probe.q.copy())
        before = sample.native_qpos.copy()
        contact = probe.grasp_opening(task.reference.copy(), "cylindrical_plug")
        opening = cylindrical_hold_opening(task, sample)
        assert contact - opening == pytest.approx(.06)
        assert opening > .2
        np.testing.assert_array_equal(sample.native_qpos, before)
        probe.contacts(np.r_[task.reference[:6], contact-.001, -contact+.001])
        count = probe.lib.lms_contact_geometry(probe.handle, None, 0)
        rows = np.empty((count, 11))
        probe.check(probe.lib.lms_contact_geometry(probe.handle, probe.ptr(rows), count))
        fingers = set()
        for row in rows:
            names = {probe.names[int(row[2])], probe.names[int(row[3])]}
            if "cylindrical_plug" not in names or row[4] > 1e-5:
                continue
            local = sample.plug_rotation_body.T @ (row[5:8]-sample.plug_position_body_m)
            # Independent original CAD tab region, not the hex flats or its rim.
            if abs(local[0]) < .004 and .020 < abs(local[1]) < .0225 and .005 < local[2] < .010:
                fingers.update(names.intersection({"ee_finger1", "ee_finger2"}))
        assert fingers == {"ee_finger1", "ee_finger2"}
    finally:
        probe.close()


def test_rising_retreat_avoids_neighbor_that_blocks_old_horizontal_path():
    pytest.importorskip("Basilisk")
    from simulation.scripted_tasks.native_geometry_probe import NativePoseProbe

    task, sample = initial_task()
    part_position = task.hole + [0., 0., task.config.flange_local_m]
    part_rotation = task.hole_rotation @ np.diag([1., -1., -1.])
    sample = replace(sample, plug_position_body_m=part_position, plug_rotation_body=part_rotation)
    goal, rotation = task._tracked_grasp(sample)
    q = np.r_[solve_pose(task.chain, goal, rotation, sample.joints[:6], task.lower, task.upper), .45, -.45]
    task.reference[:] = q
    task.stage = "retract"
    task._opening_at_entry = .45
    tangent = rotation[:, 1].copy()
    tangent[2] = 0.
    tangent /= np.linalg.norm(tangent)
    if tangent[0] < 0:
        tangent = -tangent
    probe = NativePoseProbe(MODEL)
    try:
        for body, position in (("cylindrical_plug", part_position),
                               ("aviation_plug", part_position+.085*tangent+.025*rotation[:, 0])):
            address = probe.address(body+"_freejoint")
            probe.q[address:address+7] = np.r_[position, matrix_to_quaternion_wxyz(part_rotation)]
        probe.contacts(q)
        sample = replace(sample, joints=q, native_qpos=probe.q.copy())
        before = sample.native_qpos.copy()
        probe.use_measured_qpos(sample.native_qpos, q)
        task._plan(goal+.12*tangent, rotation)
        old_hits = [probe.pose_collision(np.r_[task._timing.path.at(u), .65, -.65], task.name)
                    for u in np.linspace(0., 1., 81)]
        assert any(hit and "aviation_plug" in (hit["body1"], hit["body2"]) for hit in old_hits)
        plan_cylindrical_retract(task, sample)
        assert task.retract_plan["height_m"] >= .08
        for opening in (.45, .65):
            for u in np.linspace(0., 1., 161):
                assert probe.pose_collision(np.r_[task._timing.path.at(u), opening, -opening], task.name,
                                            arm_penetration_m=.0002) is None
        np.testing.assert_array_equal(sample.native_qpos, before)
    finally:
        probe.close()


def test_retreat_checks_timed_opening_and_continues_after_positive_gap_failure(monkeypatch, tmp_path):
    """A safe later heading must survive a stricter clearance check of the first."""
    model = tmp_path / "model.xml"
    model.write_text('<mujoco><worldbody><body name="cylindrical_plug">'
                     '<freejoint name="cylindrical_plug_free"/></body></worldbody></mujoco>')
    reference = np.r_[np.zeros(6), .38, -.38]
    measured = np.r_[np.zeros(6), .37, -.375]
    active, calls = {}, []

    class Path:
        def at(self, progress):
            return np.array([.02*progress, 0., 0., 0., 0., 0.])

        def derivative_bounds(self):
            return np.full((3, 6), .02)

    class Query:
        closed = False

        def __init__(self, proxy, observed, *, clearance_m):
            assert clearance_m == .002
            assert proxy._planning_stage == "retract"
            assert proxy.planning_part_body == "cylindrical_plug"
            assert proxy.planning_part_joint == "cylindrical_plug_free"
            np.testing.assert_array_equal(observed.joints, measured)

        def __call__(self, configuration):
            progress = configuration[0]/.02
            opening = configuration[6]
            calls.append((active["candidate"], progress, opening))
            if .2 < progress < .8 and active["candidate"] == 0:
                # The old penetration-only gate would accept this 1.8-mm gap.
                return {"body1": "ee_finger1", "body2": "cylindrical_plug",
                        "separation_m": .0018, "penetration_m": -.0018}
            if progress > 1e-9 and opening < .38+.27*progress-1e-12:
                raise AssertionError("Frozen initial aperture rejects a safe timed opening")
            return None

        def close(self):
            Query.closed = True

    task = SimpleNamespace(model_path=model,
                           config=SimpleNamespace(plug_body="cylindrical_plug", release_open_rad=.65),
                           reference=reference.copy(), _release_clearance_probe=None)

    def plan(position, _rotation):
        active["candidate"] = int(position[0])
        task._timing = SimpleNamespace(path=Path())

    task._plan = plan
    task.path_reference = lambda progress: np.r_[task._timing.path.at(progress),
                                                 .38+.27*progress, -.38-.27*progress]
    observed = SimpleNamespace(joints=measured.copy(), native_qpos=np.zeros(1),
                               plug_position_body_m=np.zeros(3), plug_rotation_body=np.eye(3))
    candidates = [dict(position=np.array([index, 0., 0.]), rotation=np.eye(3),
                       height_m=height, angle_rad=angle, direction_body=[1., 0., 0.])
                  for index, height, angle in ((0, .08, 0.), (1, .12, np.pi))]
    monkeypatch.setattr("simulation.scripted_tasks.collision_motion.NativeMotionQuery", Query)
    plan_cylindrical_retract(task, observed, candidates)
    assert task.retract_plan["height_m"] == .12
    assert task.retract_plan["attempts"][0]["collision"]["separation_m"] == .0018
    assert task.retract_plan["attempts"][1]["collision"] is None
    assert task.retract_plan["start_opening_rad"] == .37
    assert task.retract_plan["clearance_m"] == .002
    assert any(candidate == 1 and 0. < progress < 1. and .38 < opening < .65
               for candidate, progress, opening in calls)
    assert any(candidate == 1 and 0. < progress < 1. and opening == .65
               for candidate, progress, opening in calls)
    np.testing.assert_array_equal(task.reference, reference)
    np.testing.assert_array_equal(observed.joints, measured)
    assert Query.closed
