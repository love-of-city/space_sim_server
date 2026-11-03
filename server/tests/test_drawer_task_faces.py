"""Rigid mounting changes must rotate the calibrated drawer mechanism as a unit."""
from dataclasses import replace
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from simulation.scripted_tasks.drawer_cycle import DrawerCycleTask
from simulation.scripted_tasks.drawer_lift import DrawerLiftTask, DrawerSample
from simulation.serial_chain_kinematics import (
    axis_angle_to_matrix, matrix_to_quaternion_wxyz, quaternion_wxyz_to_matrix,
)

MODEL=Path(__file__).resolve().parents[1]/'model/SARM/platform/sarm_20261012.xml'


def rotated_drawer(tmp_path, rotation, local_axis='0 0 -1'):
    root=ET.parse(MODEL).getroot()
    drawer=root.find('.//body[@name="drawer_base"]')
    position=np.fromstring(drawer.get('pos'),sep=' ')
    original=quaternion_wxyz_to_matrix(np.fromstring(drawer.get('quat'),sep=' '))
    drawer.set('pos',' '.join(map(str,rotation@position)))
    drawer.set('quat',' '.join(map(str,matrix_to_quaternion_wxyz(rotation@original))))
    drawer.find('joint[@name="drawer_base_slide"]').set('axis',local_axis)
    path=tmp_path/'drawer.xml';ET.ElementTree(root).write(path)
    return path


@pytest.mark.parametrize('task_class',[DrawerLiftTask,DrawerCycleTask])
@pytest.mark.parametrize('angle',[0.,np.pi/2,-np.pi/2,.6])
def test_drawer_waypoints_rotate_with_the_mount(tmp_path,task_class,angle):
    rotation=axis_angle_to_matrix(np.array([0.,1.,0.]),angle)
    original=task_class(MODEL)
    task=task_class(rotated_drawer(tmp_path,rotation))
    sample=DrawerSample(0.,np.zeros(8),np.zeros(8),0.,0.,0.,np.zeros(6),True)
    task.step(sample)
    np.testing.assert_allclose(task.axis,rotation@original.axis,atol=1e-12)
    np.testing.assert_allclose(task.grip_origin,rotation@original.grip_origin,atol=1e-12)
    np.testing.assert_allclose(task.grasp_rotation[:,2],-task.axis,atol=1e-12)
    tangent=task.drawer_rotation[:,1]
    start=task.grasp-.1*task.axis+.03*tangent
    goal,_=task._stage_goal('raise',start,np.eye(3))
    np.testing.assert_allclose(goal-start,(task.config.clearance_m+.1)*task.axis,atol=1e-12)
    lifted,_=task._stage_goal('lift',start,np.eye(3))
    np.testing.assert_allclose(lifted-task.grasp,task.config.lift_m*task.axis,atol=1e-12)
    if task_class is DrawerCycleTask:
        assert task._stage_goal('raise',task.hover+.01*task.axis,np.eye(3))[0] is None
        returned,_=task._stage_goal('return',start,np.eye(3))
        np.testing.assert_allclose(returned-task.grasp,-task.config.seating_overtravel_m*task.axis,atol=1e-12)


def test_mount_update_moves_grasp_and_origin_in_same_reference_frame():
    task=DrawerCycleTask(MODEL)
    sample=DrawerSample(0.,np.zeros(8),np.zeros(8),0.,0.,0.,np.zeros(6),True)
    task.step(sample)
    task.result='success'
    rotation=axis_angle_to_matrix(np.array([1.,0.,0.]),np.pi/2)
    old_axis=task.axis.copy();old_grip=task.grip_origin.copy()
    origin=rotation@task.drawer_origin+np.array([.3,.2,.1])
    task.step(replace(sample,drawer_mount=(origin,rotation@task.drawer_rotation)))
    np.testing.assert_allclose(task.drawer_origin,origin)
    np.testing.assert_allclose(task.grip_origin,rotation@old_grip+[.3,.2,.1])
    np.testing.assert_allclose(task.axis,rotation@old_axis)
    np.testing.assert_allclose(task.hover-task.grasp,task.axis*task.config.clearance_m,atol=1e-12)


def test_changed_internal_rail_axis_still_requires_calibration(tmp_path):
    with pytest.raises(ValueError,match='local drawer slide axis'):
        DrawerCycleTask(rotated_drawer(tmp_path,np.eye(3),local_axis='1 0 0'))
