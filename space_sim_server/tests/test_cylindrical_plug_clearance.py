"""Regress the long-shaft grasp and the blocked side retreat with real geometry."""

from dataclasses import replace
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from simulation.scripted_tasks.plug_insert import PlugInsertConfig, PlugInsertTask, PlugSample
from simulation.scripted_tasks.plug_clearance import cylindrical_hold_opening, plan_cylindrical_retract
from simulation.scripted_tasks.motion import solve_pose
from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix, matrix_to_quaternion_wxyz


MODEL = Path(__file__).resolve().parents[1] / "model/SARM/platform/sarm_20261012.xml"


def initial_task():
    task = PlugInsertTask(MODEL)
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
    assert version == PlugInsertTask.integrated_cylindrical_version
    assert PlugInsertTask.recorded_stages(version) == PlugInsertTask.recorded_stages(PlugInsertTask.latch_version)
    assert PlugInsertTask.recorded_stages(PlugInsertTask.cylindrical_clearance_version) == PlugInsertTask.recorded_stages(version)
    assert PlugInsertTask.recorded_stages(PlugInsertTask.gentle_release_version) == PlugInsertTask.recorded_stages(version)
    from simulation.scripted_tasks.drawer_initialization import randomized_scene
    scene = randomized_scene({"randomization": {"arm_joint_position_rad": [0.] * 8}}, 42,
                             task_name="cylindrical_plug_insert")
    assert scene["collection_config"]["task_version"] == version


def test_native_grasp_centers_both_original_tabs_and_bounds_servo_preload():
    pytest.importorskip("Basilisk")
    from simulation.scripted_tasks.drawer_initialization import NativePoseProbe

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
    from simulation.scripted_tasks.drawer_initialization import NativePoseProbe

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
