"""Known-load native verification; never imports Python mujoco."""

import numpy as np
import pytest
from test_local_mujoco_stepper import MJCF, build, run_to
from Basilisk.architecture import messaging
from simulation.wrench_sensors import WrenchSensor

SENSOR = WrenchSensor("test", "load_site", "load_force", "load_torque", "mount", "tool")


def load_model(tmp_path, site_quat="1 0 0 0"):
    # Static cantilever isolates sign, COM offset moment and frame rotation.
    extra = f'''<body name="mount" pos="0 0 3">
      <body name="tool"><inertial pos="0.2 0 0" mass="1" diaginertia=".01 .01 .01"/>
        <geom type="sphere" pos=".2 0 0" size=".02" contype="0" conaffinity="0"/>
        <site name="load_site" quat="{site_quat}"/>
      </body></body>'''
    text = MJCF.replace(
        '<compiler angle="radian"/>', '<compiler angle="radian" fusestatic="false"/>'
    )
    text = text.replace("</worldbody>", extra + "</worldbody>")
    text = text.replace(
        "</mujoco>",
        '<sensor><force name="load_force" site="load_site"/><torque name="load_torque" site="load_site"/></sensor></mujoco>',
    )
    path = tmp_path / "load.xml"
    path.write_text(text, encoding="utf-8")
    return path


@pytest.mark.parametrize("substeps", [1, 5])
@pytest.mark.parametrize("rotated", [False, True])
def test_native_wrench_known_force_moment_frame_and_time(tmp_path, substeps, rotated):
    quat = "0.7071067811865476 0 0 0.7071067811865476" if rotated else "1 0 0 0"
    sim, scene, stepper, keep = build(
        load_model(tmp_path, quat), "local", substeps=substeps
    )
    force = messaging.CmdForceInertialMsg()
    torque = messaging.CmdTorqueBodyMsg()
    force.write(messaging.CmdForceInertialMsgPayload(forceRequestInertial=[0, 10, 0]))
    torque.write(messaging.CmdTorqueBodyMsgPayload(torqueRequestBody=[1, 2, 3]))
    stepper.add_body_wrench_input("tool", force, torque)
    keep += [force, torque]
    run_to(sim, 0)
    initial = SENSOR.observation(stepper, 0)
    assert initial["valid"] is False
    run_to(sim, 0.05)
    values, sample, end = stepper.read_wrench("load_force", "load_torque")
    # MuJoCo reaction at the parent cut opposes external load; r x F adds 2 Nm.
    expected_force = np.array([0, -10, 0])
    expected_torque = np.array([-1, -2, -5])
    if rotated:
        rotation = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
        expected_force = rotation @ expected_force
        expected_torque = rotation @ expected_torque
    np.testing.assert_allclose(
        values, np.r_[expected_force, expected_torque], atol=1e-8
    )
    assert 0 < end - sample <= 4_166_667
    assert end - sample == pytest.approx(4_166_667 / substeps, abs=1)
    observed = SENSOR.observation(stepper, end)
    assert observed["valid"] and int(observed["sample_time_ns"]) == sample
    with pytest.raises(RuntimeError, match="sensor pair"):
        stepper.read_wrench("load_force", "missing")
    assert stepper.read_wrench("missing", "absent") is None
    stepper.Reset(0)
    assert SENSOR.observation(stepper, 0)["valid"] is False


def test_common_acceleration_does_not_create_spurious_interface_load(tmp_path):
    path = load_model(tmp_path)
    text = path.read_text(encoding="utf-8").replace(
        '<body name="mount" pos="0 0 3">',
        '<body name="mount" pos="0 0 3"><freejoint/><inertial pos="0 0 0" mass="2" diaginertia=".1 .1 .1"/>',
    )
    path.write_text(text, encoding="utf-8")
    sim, scene, stepper, keep = build(path, "local")
    for name, mass in [("mount", 2), ("tool", 1)]:
        msg = messaging.CmdForceInertialMsg()
        msg.write(
            messaging.CmdForceInertialMsgPayload(forceRequestInertial=[0, 10 * mass, 0])
        )
        stepper.add_body_wrench_input(name, msg, source=name)
        keep.append(msg)
    run_to(sim, 0.05)
    values, _, _ = stepper.read_wrench("load_force", "load_torque")
    np.testing.assert_allclose(values, 0, atol=1e-8)


def test_contact_load_reaches_sensor_and_lever_arm(tmp_path):
    path = load_model(tmp_path)
    text = path.read_text(encoding="utf-8")
    text = text.replace(
        'pos=".2 0 0" size=".02" contype="0" conaffinity="0"', 'pos=".2 0 0" size=".02"'
    )
    text = text.replace('name="ball" pos="1.5 0 0"', 'name="ball" pos=".2 0 3.065"')
    path.write_text(text, encoding="utf-8")
    sim, scene, stepper, keep = build(path, "local")
    run_to(sim, 0.004167)
    values, _, _ = stepper.read_wrench("load_force", "load_torque")
    assert values[2] > 0.01
    assert values[4] == pytest.approx(-0.2 * values[2], rel=1e-7)


def test_sensor_reading_is_passive_for_identical_model(tmp_path):
    path = load_model(tmp_path)
    sim, scene, stepper, keep = build(path, "local")
    run_to(sim, 0.1)
    state = stepper.qpos.copy(), stepper.qvel.copy()
    import xml.etree.ElementTree as ET

    root = ET.parse(path).getroot()
    root.remove(root.find("sensor"))
    other = tmp_path / "without_sensor.xml"
    other.write_bytes(ET.tostring(root))
    sim2, scene2, stepper2, keep2 = build(other, "local")
    run_to(sim2, 0.1)
    np.testing.assert_array_equal(state[0], stepper2.qpos)
    np.testing.assert_array_equal(state[1], stepper2.qvel)


def test_completed_sample_survives_state_rebase(tmp_path):
    from simulation.local_mujoco_stepper import _pointer

    path = load_model(tmp_path)
    path.write_text(
        path.read_text(encoding="utf-8").replace('contype="0" conaffinity="0"', ""),
        encoding="utf-8",
    )
    sim, scene, stepper, keep = build(path, "local")
    run_to(sim, 0.05)
    before = stepper.read_wrench("load_force", "load_torque")
    # lms_set_state runs mj_forward, including sensors. This state places the
    # free ball against the tool, so an uncached sensor would change to contact.
    q = stepper.qpos.copy()
    v = stepper.qvel.copy()
    q[-7:-4] = [0.2, 0, 3.065]
    stepper._check(
        stepper._lib.lms_set_state(stepper._handle, _pointer(q), _pointer(v)),
        "test rebase",
    )
    after = stepper.read_wrench("load_force", "load_torque")
    np.testing.assert_array_equal(before[0], after[0])
    assert before[1:] == after[1:]
