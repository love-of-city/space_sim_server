"""Task targets and dry-friction frames must follow independently rotated seats."""
from dataclasses import replace
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from simulation.plug_retention import model_retention_sockets
from simulation.scripted_tasks.drawer_lift import DrawerLiftTask
from simulation.scripted_tasks.plug_insert import PlugInsertConfig, PlugInsertTask, PlugSample
from simulation.scripted_tasks.plug_profiles import PLUG_PROFILES
from simulation.serial_chain_kinematics import (
    axis_angle_to_matrix, matrix_to_quaternion_wxyz, quaternion_wxyz_to_matrix,
)
from space_arm_platform.control_defaults import DEFAULT_OPERATING_JOINT_DEG

MODEL = Path(__file__).resolve().parents[1] / 'model/SARM/platform/sarm_20261012.xml'


def write_quat(node, rotation):
    node.set('quat', ' '.join(map(str, matrix_to_quaternion_wxyz(rotation))))


@pytest.mark.parametrize('task_name', PLUG_PROFILES)
def test_independent_pickup_and_target_yaws_control_grasp_seating_and_acceptance(tmp_path, task_name):
    profile = PLUG_PROFILES[task_name]
    root = ET.parse(MODEL).getroot()
    pickup_r = axis_angle_to_matrix(np.array([0., 0., 1.]), -.65)
    hole_r = axis_angle_to_matrix(np.array([0., 0., 1.]), 1.2)
    write_quat(root.find(f'.//site[@name="{profile.prefix}_pickup"]'), pickup_r)
    write_quat(root.find(f'.//site[@name="{profile.target_site}"]'), hole_r)
    body = root.find(f'./worldbody/body[@name="{profile.body}"]')
    old_r = quaternion_wxyz_to_matrix(np.fromstring(body.get('quat'), sep=' '))
    write_quat(body, pickup_r @ old_r)
    model = tmp_path / 'rotated.xml'
    ET.ElementTree(root).write(model)
    task = PlugInsertTask(model, PlugInsertConfig(plug_body=profile.body))
    sample = PlugSample(0., np.r_[np.deg2rad(DEFAULT_OPERATING_JOINT_DEG), 0., 0.], np.zeros(8),
                        np.fromstring(body.get('pos'), sep=' '), pickup_r @ old_r,
                        np.zeros(3), np.zeros(3), np.zeros(6), True)
    task.step(sample)
    np.testing.assert_allclose(task.plug_rotation, hole_r @ old_r, atol=1e-14)
    assert abs(np.dot(task.grasp_rotation[:, 0], pickup_r[:, 0])) == pytest.approx(1.)
    seated = replace(sample, plug_position_body_m=task.hole+[0., 0., profile.flange_m],
                     plug_rotation_body=hole_r @ old_r)
    assert task.metrics(seated)['orientation_error_rad'] < 1e-12
    if profile.keyed:
        assert task.metrics(replace(seated, plug_rotation_body=pickup_r @ old_r))['orientation_error_rad'] > 1.
    sockets = model_retention_sockets(model)[profile.body]
    np.testing.assert_allclose(sockets[0].rotation_body, pickup_r, atol=1e-14)
    seat = next(s for s in sockets if s.name == profile.target_site)
    np.testing.assert_allclose(seat.plug_rotation_body, hole_r @ old_r, atol=1e-14)


@pytest.mark.parametrize('angle', [-np.pi, -.7, .8, np.pi/2])
def test_drawer_grip_frame_rotates_and_original_slide_coordinate_stays_vertical(tmp_path, angle):
    original = DrawerLiftTask(MODEL)
    root = ET.parse(MODEL).getroot()
    drawer = root.find('.//body[@name="drawer_base"]')
    yaw = axis_angle_to_matrix(np.array([0., 0., 1.]), angle)
    write_quat(drawer, yaw @ original.drawer_rotation)
    model = tmp_path / 'rotated.xml'
    ET.ElementTree(root).write(model)
    task = DrawerLiftTask(model)
    np.testing.assert_allclose(task.grip_origin-task.drawer_origin,
                               yaw @ (original.grip_origin-original.drawer_origin), atol=1e-14)
    np.testing.assert_allclose(task.axis, [0., 0., 1.], atol=1e-14)
