"""Task-plane invariance, opposed spacecraft faces, and socket ownership.

Pure geometry/controller checks; no native solver or UE process is launched.
"""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from simulation.plug_retention import model_retention_sockets
from simulation.scripted_tasks.plug_clearance import cylindrical_retract_candidates
from simulation.scripted_tasks.scene_plug import PlugInsertConfig, PlugInsertTask, PlugSample
from simulation.serial_chain_kinematics import axis_angle_to_matrix, matrix_to_quaternion_wxyz

MODEL = Path(__file__).resolve().parents[1]/'model/SARM/platform/sarm_20261012.xml'
FLIP = np.diag([1., -1., -1.])


def oriented_model(tmp_path, rotation):
    root = ET.parse(MODEL).getroot()
    box = root.find('.//body[@name="task_box_base"]')
    pos = np.fromstring(box.get('pos'), sep=' ')
    box.set('pos', ' '.join(map(str, rotation @ pos)))
    box.set('quat', ' '.join(map(str, matrix_to_quaternion_wxyz(rotation))))
    path = tmp_path/'oriented.xml'
    ET.ElementTree(root).write(path)
    return path


def sample_at(position, rotation):
    return PlugSample(0., np.zeros(8), np.zeros(8), position, rotation,
                      np.zeros(3), np.zeros(3), np.zeros(6), True)


ROTATIONS = [np.eye(3),
             axis_angle_to_matrix(np.array([0., 1., 0.]), np.pi/2),
             axis_angle_to_matrix(np.array([0., 1., 0.]), -np.pi/2),
             axis_angle_to_matrix(np.array([1., 2., 3.])/np.sqrt(14), 1.2)]


@pytest.mark.parametrize('rotation', ROTATIONS)
def test_extraction_and_insertion_follow_socket_normals_without_vertical_division(tmp_path, rotation):
    task = PlugInsertTask(oriented_model(tmp_path, rotation),
                          PlugInsertConfig(plug_body='guide_plug', transport_clearance_m=.2))
    donor_normal = task.pickup_rotation[:, 2]
    sample = sample_at(task.pickup+.027*donor_normal, task.pickup_rotation @ FLIP)
    task._plug_goal = task.pickup+.1*donor_normal
    task._advance_extraction_goal(sample, .005)
    delta = task._plug_goal-sample.plug_position_body_m
    np.testing.assert_allclose(delta, donor_normal*.001, atol=1e-12)
    task._advance_extraction_goal(replace(sample,wrench=np.array([31.,0.,0.,0.,0.,0.])), .005)
    np.testing.assert_allclose(task._plug_goal, sample.plug_position_body_m, atol=1e-12)

    normal = task.hole_rotation[:, 2]
    inserted = sample_at(task.hole+.05*normal, task.hole_rotation @ FLIP)
    task._seat_z = task._axis_coordinate(task.hole)+task.config.flange_local_m+task.config.seat_clearance_m
    task._plug_goal = inserted.plug_position_body_m.copy()
    task._advance_insertion_goal(inserted,.005)
    np.testing.assert_allclose(task._plug_goal-inserted.plug_position_body_m,
                               -normal*task.config.insert_speed_m_s*.005,atol=1e-12)
    seated = replace(inserted,plug_position_body_m=task.hole+.025*normal)
    assert task.metrics(seated)['seat_gap_m'] == pytest.approx(0.,abs=1e-12)
    assert task.metrics(seated)['radial_error_m'] < 1e-12
    assert task.config.force_limit_n == 40. and task.config.torque_limit_nm == 4.


def test_nonhorizontal_faces_require_explicit_clearance(tmp_path):
    model = oriented_model(tmp_path, ROTATIONS[1])
    with pytest.raises(ValueError,match='explicit transport_clearance'):
        PlugInsertTask(model)


def test_opposed_faces_use_separate_outward_clearances_not_a_shared_global_plane():
    task = PlugInsertTask(MODEL, PlugInsertConfig(transport_clearance_m=.16))
    task.pickup = np.array([1., .2, .3])
    task.pickup_rotation = ROTATIONS[2]  # donor points -X into the gap
    task.hole = np.array([.4, .2, .3])
    task.hole_rotation = ROTATIONS[1]  # recipient points +X into the gap
    np.testing.assert_allclose(task._axis_goal(task._transport_coordinate(pickup=True),pickup=True),
                               [.84,.2,.3])
    np.testing.assert_allclose(task._axis_goal(task._transport_coordinate()), [.56,.2,.3])
    # A signed projection is allowed to be negative; it is not a negative clearance.
    assert task._transport_coordinate(pickup=True) < 0


def test_retract_candidates_rise_from_receiver_plane_and_keep_tangential_clearance():
    task = PlugInsertTask(MODEL, PlugInsertConfig(transport_clearance_m=.16))
    task.reference = np.zeros(8)
    task.hole_rotation = ROTATIONS[1]
    start, _ = task.chain.forward(task.reference[:6])
    normal = task.hole_rotation[:,2]
    for candidate in cylindrical_retract_candidates(task):
        delta = candidate['position']-start
        assert delta @ normal == pytest.approx(candidate['height_m'])
        assert np.linalg.norm(delta-normal*(delta@normal)) == pytest.approx(.12)


def test_goal_tracks_rigid_rotation_of_its_actual_owner():
    task=PlugInsertTask(MODEL,PlugInsertConfig(transport_clearance_m=.16))
    task.stage='insert'
    task.result='success'  # Exercise measurement/frame update without commanding an arm.
    task.reference=np.zeros(8)
    offset=np.array([.001,-.002,.05])
    task._plug_goal=task.hole+offset
    task._seat_z=task.hole[2]+.02525
    delta=np.array([.3,-.2,.1])
    rotation=ROTATIONS[1]
    old_hole=task.hole.copy()
    changed=sample_at(rotation@old_hole+delta,rotation@FLIP)
    changed=replace(changed,site_poses={
        'hole':(rotation@old_hole+delta,rotation),
        'pickup':(task.pickup.copy(),task.pickup_rotation.copy()),
    })
    task.step(changed)
    np.testing.assert_allclose(task._plug_goal,rotation@(old_hole+offset)+delta,atol=1e-12)
    assert task._seat_z==pytest.approx(float(task.hole@rotation[:,2])+.02525)


def test_named_service_receiver_resolves_same_type_and_actual_owner(tmp_path):
    root = ET.parse(MODEL).getroot()
    service = root.find('.//body[@name="cubesat_bus"]')
    target = root.find('./worldbody/body[@name="capture_target"]')
    # Move original box under an independent target; service receives a complete copy.
    box = root.find('.//body[@name="task_box_base"]')
    duplicate = deepcopy(box)
    for node in duplicate.iter():
        if node.get('name'):
            node.set('name','service_'+node.get('name'))
    duplicate.set('pos','0.2 0.3 0.4')
    duplicate.set('quat',' '.join(map(str,matrix_to_quaternion_wxyz(ROTATIONS[1]))))
    service.append(duplicate)
    service.remove(box)
    target.append(box)
    equality = root.find('equality')
    ET.SubElement(equality,'weld',name='preheld_capture_lock',body1='gripper',body2='capture_target')
    path=tmp_path/'two_boxes.xml';ET.ElementTree(root).write(path)
    sockets=model_retention_sockets(path,owned=True)
    for body, seats in sockets.items():
        assert len(seats)==5
        assert sum(s.initial for s in seats)==1
        assert seats[0].owner_body=='capture_target'
        receiving=[s for s in seats if s.name.startswith('service_')]
        assert len(receiving)==2 and all(s.owner_body=='cubesat_bus' for s in receiving)
        for seat in receiving:
            np.testing.assert_allclose(seat.rotation_body[:,2],[1.,0.,0.],atol=1e-12)
        assert all(not s.name.endswith('pickup') for s in seats)
    cfg=PlugInsertConfig(plug_body='guide_plug',target_site='service_guide_insert_40p5',transport_clearance_m=.16)
    task=PlugInsertTask(path,cfg)
    assert task.hole_owner=='cubesat_bus'
    with pytest.raises(ValueError,match='forward assembly hole'):
        PlugInsertConfig(plug_body='guide_plug',target_site='service_aviation_insert_40p5')
