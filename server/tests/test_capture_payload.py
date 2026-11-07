"""Prevent a seated payload from being left behind or accepted without transport."""
import xml.etree.ElementTree as ET
import numpy as np
import pytest


def test_capture_reset_moves_only_target_owned_free_parts():
    from simulation.scripted_tasks.capture_payload import initial_payload_poses
    from simulation.serial_chain_kinematics import axis_angle_to_matrix
    root = ET.fromstring('''<mujoco><worldbody>
      <body name="cubesat_bus"><freejoint name="cubesat_free"/>
        <site name="guide_pickup"/></body>
      <body name="capture_target" pos="1 2 3"><freejoint name="target_free"/>
        <site name="aviation_pickup"/></body>
      <body name="aviation_plug" pos="1.1 2.2 3.3"><freejoint name="plug_free"/></body>
      <body name="guide_plug" pos="0 0 0"><freejoint name="guide_free"/></body>
    </worldbody></mujoco>''')
    target_position = np.array([4.,-2.,1.])
    target_rotation = axis_angle_to_matrix(np.array([0.,0.,1.]),.7)
    poses = initial_payload_poses(root,target_position,target_rotation)
    assert set(poses) == {'aviation_plug'}
    p,r = poses['aviation_plug']
    np.testing.assert_allclose(target_rotation.T@(p-target_position),[.1,.2,.3])
    np.testing.assert_allclose(target_rotation.T@r,np.eye(3),atol=1e-14)
    assert root.find('.//body[@name="aviation_plug"]').get('pos') == '1.1 2.2 3.3'


def test_capture_preparation_disables_saved_ideal_lock_without_mutating_source(tmp_path):
    from simulation.model_preparation.capture_grasp_model import prepare_capture_model
    source=tmp_path/'source.xml'
    text='''<mujoco><compiler/><worldbody><body name="gripper"/>
      <body name="capture_target"><freejoint name="target_free"/></body></worldbody>
      <equality><weld name="preheld_capture_lock" body1="gripper" body2="capture_target"/>
      </equality></mujoco>'''
    source.write_text(text)
    prepared=prepare_capture_model(source,tmp_path/'capture.xml')
    root=ET.parse(prepared).getroot()
    assert root.find('./equality/weld').get('active')=='false'
    assert root.find('.//body[@name="capture_target"]/freejoint') is not None
    assert source.read_text()==text


def test_capture_preparation_restores_scaled_sun_sensor_without_moving_other_parts(tmp_path):
    from pathlib import Path
    from simulation.model_preparation.capture_grasp_model import prepare_capture_model
    original = Path(__file__).resolve().parents[1] / 'model/SARM/platform/sarm_dual_capture26.xml'
    root = ET.parse(original).getroot()
    target = root.find('.//body[@name="capture_target"]')
    mount = target.find('geom[@name="target_instance_0077_part_034_color_00"]')
    target.remove(mount)
    scale, offset = 2.1, np.array([-.317515, -.054725, -.046286570143])
    for geom in target.findall('geom'):
        position = np.fromstring(geom.get('pos', '0 0 0'), sep=' ')
        geom.set('pos', ' '.join(map(str, position * scale + offset)))
    anchor = target.find('geom[@name="target_instance_0078_part_035_color_00"]')
    mesh = root.find(f'./asset/mesh[@name="{anchor.get("mesh")}"]')
    mesh.set('scale', '2.1 2.1 2.1')
    before = {g.get('name'): dict(g.attrib) for g in target.findall('geom')}
    source = tmp_path / 'source.xml'
    ET.ElementTree(root).write(source)
    source_bytes = source.read_bytes()
    output = prepare_capture_model(source, tmp_path / 'prepared.xml')
    prepared = ET.parse(output).getroot()
    restored = prepared.find(f'.//geom[@name="{mount.get("name")}"]')
    np.testing.assert_allclose(np.fromstring(restored.get('pos'), sep=' '),
                               np.fromstring(mount.get('pos'), sep=' ') * scale + offset)
    for key in ('quat', 'rgba', 'mass', 'contype', 'conaffinity', 'group'):
        assert restored.get(key) == mount.get(key)
    restored_mesh = prepared.find(f'./asset/mesh[@name="{restored.get("mesh")}"]')
    np.testing.assert_allclose(np.fromstring(restored_mesh.get('scale'), sep=' '), [2.1] * 3)
    for geom in prepared.findall('.//body[@name="capture_target"]/geom'):
        if geom.get('name') in before:
            assert geom.attrib == before[geom.get('name')]
    again = ET.parse(prepare_capture_model(output, tmp_path / 'again.xml')).getroot()
    assert len(again.findall(f'.//geom[@name="{mount.get("name")}"]')) == 1
    assert source.read_bytes() == source_bytes


def test_capture_sun_sensor_restoration_rejects_inconsistent_assembly():
    from pathlib import Path
    from simulation.model_preparation.capture_grasp_model import restore_capture_sun_sensor_mount
    original = Path(__file__).resolve().parents[1] / 'model/SARM/platform/sarm_dual_capture26.xml'
    root = ET.parse(original).getroot()
    target = root.find('.//body[@name="capture_target"]')
    target.remove(target.find('geom[@name="target_instance_0077_part_034_color_00"]'))
    target.find('geom[@name="target_instance_0079_part_036_color_00"]').set('pos', '0 0 0')
    with pytest.raises(ValueError, match='assembly transform'):
        restore_capture_sun_sensor_mount(root)


def test_handoff_acceptance_rejects_grasp_only_or_wrong_delivery_pose():
    from collection.validate_capture_grasp import validate_handoff
    final=dict(stage='handoff_hold',grasp_hold_s=20.,carry_plan={'accepted':True},
               handoff=dict(target_position_error_m=.001,target_angle_error_rad=.001))
    validate_handoff(final,{'pull','handoff_hold'})
    with pytest.raises(AssertionError):validate_handoff(final,{'hold'})
    with pytest.raises(AssertionError):
        validate_handoff({**final,'handoff':dict(target_position_error_m=.01,
            target_angle_error_rad=.001)},{'pull','handoff_hold'})


@pytest.mark.parametrize('version', ['capture-target-grasp/4-physical-handoff',
                                   'capture-target-grasp/5-preferred-handoff-branch'])
def test_both_handoff_versions_require_delivery_and_stable_support(version):
    from collection.validate_capture_grasp import validate_task_rows
    final = dict(task_version=version, result='success', stage='handoff_hold',
        grasp_hold_s=20., stable_hold_s=20., self_contact=False, peak_contact_force_n=5.,
        carry_plan={'accepted': True},
        handoff=dict(target_position_error_m=.001, target_angle_error_rad=.001),
        feedback=dict(jaw_support_count=4, jaw_force_n=5.))
    rows = [dict(sim_time_ns=str(i*20_000_000), scripted_task=dict(final),
                 gripper_type='capture26_four_jaw', joint_position_rad=[0.]*12)
            for i in range(1001)]
    stages = {'settle','align','approach','close','tighten','hold','pull','handoff_hold'}
    validate_task_rows(rows, stages)
    rows[-1]['scripted_task'] = {**final, 'handoff':dict(
        target_position_error_m=.01, target_angle_error_rad=.001)}
    with pytest.raises(AssertionError):
        validate_task_rows(rows, stages)


def test_bolt_seat_observation_cancels_owner_translation_and_rotation():
    from types import SimpleNamespace
    from simulation.scripted_tasks.capture_grasp_transport import bolt_seat_observation
    from simulation.serial_chain_kinematics import axis_angle_to_matrix, matrix_to_quaternion_wxyz
    rotation = axis_angle_to_matrix(np.array([0., 1., 0.]), .7)
    origin = np.array([8., -3., 4.])
    seat = np.array([.1, .2, .3])
    state = {
        'capture_target': np.r_[origin, matrix_to_quaternion_wxyz(rotation)],
        'bolt': np.r_[origin+rotation@seat, matrix_to_quaternion_wxyz(rotation)],
    }
    stepper = SimpleNamespace(body_detail=state.__getitem__)
    socket = dict(owner_body='capture_target', position_body_m=seat.tolist(),
                  plug_rotation_body=np.eye(3).tolist())
    result = bolt_seat_observation(stepper, socket)
    assert result['seat_position_error_m'] < 1e-14
    assert result['seat_axis_error_rad'] < 1e-7
    state['bolt'][:3] += rotation @ np.array([.02, 0., 0.])
    assert bolt_seat_observation(stepper, socket)['seat_position_error_m'] == pytest.approx(.02)
