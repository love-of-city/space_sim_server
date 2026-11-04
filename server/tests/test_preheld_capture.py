from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from simulation.components import preheld_capture


def test_hold_gain_configuration_cannot_raise_actuator_limit():
    assert preheld_capture.hold_gains()['kp'] == 1500.
    assert preheld_capture.hold_gains(dict(kp=2500., kd=80., torque_limit_nm=80.))['kp'] == 2500.
    for values in (dict(kp=2500., kd=80., torque_limit_nm=81.),
                   dict(kp=float('nan'), kd=80., torque_limit_nm=80.)):
        with pytest.raises(ValueError, match='80 Nm'):
            preheld_capture.hold_gains(values)


@pytest.mark.parametrize('binding,valid', [('gripper', True), ('cubesat_bus', False)])
def test_target_is_independent_and_bound_to_capture_gripper(tmp_path, monkeypatch, binding, valid):
    monkeypatch.setattr(preheld_capture, 'CaptureArmTarget', lambda _: SimpleNamespace(
        lower=np.full(12, -1.), upper=np.full(12, 1.)))
    model = tmp_path / 'model.xml'
    model.write_text(f'<mujoco><worldbody><body name="capture_target"><freejoint/></body></worldbody>'
                     f'<equality><weld name="preheld_capture_lock" body1="{binding}" '
                     'body2="capture_target"/></equality></mujoco>')
    if valid:
        np.testing.assert_array_equal(preheld_capture.validate_preheld(model, [0.] * 12), np.zeros(12))
        for values in ([0.] * 6, [float('nan')] * 12, [2.] * 12):
            with pytest.raises(ValueError, match='joint references'):
                preheld_capture.validate_preheld(model, values)
    else:
        with pytest.raises(ValueError, match='independent free body'):
            preheld_capture.validate_preheld(model, [0.] * 12)


def test_legacy_cross_materializer_cannot_create_deck_mounted_target(tmp_path):
    from simulation.scene_library.materialize import materialize
    with pytest.raises(ValueError, match='current-controller'):
        materialize(tmp_path, tmp_path / 'library', geometry_python='unused',
                    scene_ids=['04_cross_transfer/guide_plug_insert'])


def test_preheld_initial_geometry_is_in_service_frame(tmp_path):
    from simulation.scene_library.frames import mounted_body_pose
    xml = ('<mujoco><worldbody><body name="cubesat_bus" pos="1 0 0"/>'
           '<body name="capture_target" pos="2 3 0"><freejoint/>'
           '<body name="target_mount" pos="0 0 1"/></body></worldbody>'
           '<equality><weld name="preheld_capture_lock" body1="gripper" '
           'body2="capture_target"/></equality></mujoco>')
    position, _ = mounted_body_pose(ET.fromstring(xml), 'target_mount', allow_preheld=True)
    np.testing.assert_allclose(position, [1, 3, 1])
    with pytest.raises(ValueError, match='Movable mounting'):
        mounted_body_pose(ET.fromstring(xml.replace('body1="gripper"', 'body1="cubesat_bus"')),
                          'target_mount', allow_preheld=True)


def test_socket_owners_and_local_coordinates_follow_actual_mount(tmp_path):
    from simulation.plug_retention import model_retention_sockets
    model = tmp_path / 'sockets.xml'
    model.write_text('<mujoco><worldbody><body name="cubesat_bus"><freejoint name="bus_free"/>'
        '<site name="guide_insert_40p5" pos=".4 0 0"/></body>'
        '<body name="capture_target" pos="1 2 0"><freejoint name="target_free"/>'
        '<site name="guide_pickup" pos=".1 0 0"/></body>'
        '<body name="guide_plug" pos="1.1 2 .025" quat="0 1 0 0"><freejoint name="plug_free"/></body>'
        '</worldbody><equality><weld name="preheld_capture_lock" body1="gripper" body2="capture_target"/></equality></mujoco>')
    donor, receiver = model_retention_sockets(model, owned=True)['guide_plug']
    assert donor.owner_body == 'capture_target' and receiver.owner_body == 'cubesat_bus'
    np.testing.assert_allclose(donor.position_body_m, [.1, 0, .025])
    np.testing.assert_allclose(receiver.position_body_m, [.4, 0, .025])
