"""Force thresholds, no remote capture, and actual local-core free-body dynamics."""
import math
from pathlib import Path

import numpy as np
import pytest

from simulation.plug_retention import (
    PlugFriction, RetentionParameters, RetentionSocket, axial_backstop_force,
    load_retention_parameters, model_retention_sockets, paired_com_wrenches,
)
from simulation.serial_chain_kinematics import axis_angle_to_matrix


def seat(name="initial", position=(0., 0., .3), initial=True):
    return RetentionSocket(name, np.array(position), np.eye(3), np.eye(3), initial)


def test_no_axial_spring_anywhere_in_the_sliding_zone_and_bounded_backstop():
    p = RetentionParameters()
    for z in np.linspace(0, 2*p.friction_travel_m, 40):
        assert axial_backstop_force(z, p) == 0
    assert axial_backstop_force(-1, p) == p.backstop_force_limit_n
    with pytest.raises(ValueError):
        RetentionParameters(static_friction_n=math.nan)
    with pytest.raises(ValueError):
        RetentionParameters(static_friction_n=3, sliding_friction_n=4)


def test_config_can_disable_retention_and_rejects_misspelled_keys(tmp_path):
    path = tmp_path / "retention.json"
    path.write_text('{"enabled": false, "parameters": {"static_friction_n": 29}}')
    assert load_retention_parameters(path) is None
    path.write_text('{"enabled": true, "parameters": {"static_friction_n": 29}}')
    assert load_retention_parameters(path).static_friction_n == 29
    path.write_text('{"enabled": true, "paramters": {}}')
    with pytest.raises(ValueError):
        load_retention_parameters(path)


def test_round_socket_allows_axial_roll_but_requires_axis_alignment():
    from dataclasses import replace
    socket = replace(seat(initial=False), keyed=False)
    model = PlugFriction((socket,), RetentionParameters())
    yaw = axis_angle_to_matrix(np.array([0., 0., 1.]), 2.)
    _, torque = model.wrench([0., 0., .306], yaw, [0., 0., -.001], [0., 0., 1.])
    assert model.active is socket
    np.testing.assert_allclose(
        torque, [0., 0., -RetentionParameters().torsional_damping_nms_rad], atol=1e-12
    )
    _, stopped_torque = model.wrench(
        [0., 0., .306], yaw, [0., 0., 0.], [0., 0., 0.]
    )
    np.testing.assert_allclose(stopped_torque, 0, atol=1e-12)
    model.reset()
    tilt = axis_angle_to_matrix(np.array([1., 0., 0.]), .2) @ yaw
    model.wrench([0., 0., .306], tilt, [0., 0., -.001], [0., 0., 0.])
    assert model.active is None


def test_released_plug_is_not_attracted_from_distance_or_wrong_key_orientation():
    p = RetentionParameters()
    for position, rotation in [([0, 0, .35], np.eye(3)),
                               ([.01, 0, .305], np.eye(3)),
                               ([0, 0, .305], axis_angle_to_matrix(np.array([0, 0, 1.]), .2)),
                               ([0, 0, .305], np.diag([1., -1., -1.])),
                               ([0, 0, .305], np.diag([-1., -1., 1.]))]:
        model = PlugFriction((seat(),), p)
        force, torque = model.wrench(position, rotation, [0, 0, -.01], [0, 0, 0])
        np.testing.assert_array_equal(force, np.zeros(3))
        np.testing.assert_array_equal(torque, np.zeros(3))
        assert model.status == "free"


def test_release_reentry_and_target_socket_use_actual_motion_without_pose_reset():
    p = RetentionParameters()
    target = seat("assembly", (.1, 0, .3), False)
    model = PlugFriction((seat(), target), p)
    model.wrench([0, 0, .3], np.eye(3), np.zeros(3), np.zeros(3))
    assert model.active.name == "initial"
    model.wrench([0, 0, .31], np.eye(3), [0, 0, .01], np.zeros(3))
    assert model.active is None
    force, _ = model.wrench([.1, 0, .306], np.eye(3), [0, 0, -.001], np.zeros(3))
    assert model.active.name == "assembly"
    assert force[2] == 0  # dry friction is solved by the native physics port
    model.wrench([.1, 0, .3], np.eye(3), np.zeros(3), np.zeros(3))
    assert model.status == "retained"


def test_pair_conserves_force_and_angular_momentum_with_rotated_offset_coms():
    rotation = axis_angle_to_matrix(np.array([.3, .2, .1]), .8)
    point = np.array([.3, -.2, .1])
    plug_com, base_com = point+[.01, .02, .03], np.array([0., .4, -.3])
    f, tp, fb, tb = paired_com_wrenches(np.array([2., -3., 4.]), np.array([.01, .02, -.03]),
                                       point, plug_com, base_com, rotation, rotation.T)
    np.testing.assert_allclose(f+fb, 0, atol=1e-14)
    total_moment = np.cross(plug_com, f)+rotation@tp+np.cross(base_com, fb)+rotation.T@tb
    np.testing.assert_allclose(total_moment, 0, atol=1e-14)


def test_com_wrenches_match_reference_cross_products_element_by_element():
    rng = np.random.default_rng(20261027)
    for _ in range(256):
        force, torque, point, plug_com, base_com = rng.normal(size=(5, 3))
        rotations = [axis_angle_to_matrix(rng.normal(size=3), rng.uniform(-math.pi, math.pi))
                     for _ in range(2)]
        expected = (force, rotations[0].T @ (torque + np.cross(point-plug_com, force)),
                    -force, rotations[1].T @ (-torque + np.cross(point-base_com, -force)))
        actual = paired_com_wrenches(force, torque, point, plug_com, base_com, *rotations)
        for value, reference in zip(actual, expected, strict=True):
            np.testing.assert_array_equal(value, reference)


def test_production_socket_frames_read_existing_geometry_without_editing_it():
    import hashlib
    path = Path(__file__).resolve().parents[1]/"model/SARM/platform/sarm_20261012.xml"
    before = hashlib.sha256(path.read_bytes()).hexdigest()
    sockets = model_retention_sockets(path)
    assert set(sockets) == {"guide_plug", "aviation_plug", "cylindrical_plug"}
    for entries in sockets.values():
        assert len(entries) == 3
        assert entries[0].initial
        assert not entries[1].initial
        assert np.linalg.norm(entries[0].position_body_m-entries[1].position_body_m) > .05
    assert hashlib.sha256(path.read_bytes()).hexdigest() == before


MJCF = """<mujoco><option gravity="0 0 0"/>
<worldbody><body name="bus"><freejoint name="bus_free"/>
<geom type="box" size=".1 .1 .1" mass="10" contype="0" conaffinity="0"/></body>
<body name="plug" pos="0 0 .3"><freejoint name="plug_free"/>
<geom type="sphere" size=".02" mass=".1" contype="0" conaffinity="0"/></body></worldbody>
</mujoco>"""


def build_pair(tmp_path, *, initial_z=.3, initial_speed=0., publish_every=8, substeps=1,
               parameters=None, rotation=None, offset_com=False, sockets=None, articulated_base=False):
    pytest.importorskip("Basilisk")
    from Basilisk.architecture import messaging
    from Basilisk.simulation import mujoco
    from Basilisk.utilities import SimulationBaseClass, macros
    from simulation.assembly import AssemblyContext, SessionConfig
    from simulation.components.plug_retention import PlugRetentionComponent
    from simulation.local_mujoco_stepper import LocalMujocoStepper
    from simulation.physics_ports import PhysicsPorts
    from simulation.serial_chain_kinematics import matrix_to_quaternion_wxyz, quaternion_wxyz_to_matrix

    path = tmp_path/"pair.xml"
    xml = MJCF
    if offset_com:
        xml = xml.replace('<geom type="sphere"', '<geom pos=".001 .002 .003" type="sphere"')
        xml = xml.replace('<geom type="box"', '<geom pos=".01 .02 .03" type="box"')
    if articulated_base:
        xml = xml.replace('</body>', '<body name="link" pos=".1 0 0"><joint name="link_hinge" type="hinge" axis="0 1 0"/>'
                          '<geom pos=".03 0 0" type="sphere" size=".02" mass=".2" '
                          'contype="0" conaffinity="0"/></body></body>', 1)
    path.write_text(xml)
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("p")
    process.addTask(sim.CreateNewTask("t", macros.sec2nano(.0025)))
    process.addTask(sim.CreateNewTask("scene_init", 10**18))
    scene = mujoco.MJScene.fromFile(str(path))
    sim.AddModelToTask("scene_init", scene)
    stepper = LocalMujocoStepper(scene, path, (), publish_every=publish_every, substeps=substeps)
    sim.AddModelToTask("t", stepper, 1000)
    force = messaging.CmdForceInertialMsg()
    stepper.add_body_wrench_input("plug", force, source="test_drive")
    ctx = AssemblyContext(sim, scene, config=SessionConfig(model_path=path, physics_task="t", publish_stride=publish_every),
                          physics_ports=PhysicsPorts(scene, stepper=stepper))
    # These force-driven regression cases use explicit 5/4 N thresholds.
    component = PlugRetentionComponent(model_path=path, parameters=parameters or
                                       RetentionParameters(static_friction_n=5., sliding_friction_n=4.),
                                       sockets={"plug": sockets or (seat(),)}, base="bus")
    ctx.install([component])
    sim.InitializeSimulation()
    rotation = np.eye(3) if rotation is None else rotation
    quaternion = matrix_to_quaternion_wxyz(rotation)
    for name in ("bus", "plug"):
        scene.getBody(name).setAttitude(quaternion[1:] / (1+quaternion[0]))
    scene.getBody("plug").setPosition(rotation @ np.array([0., 0., initial_z]))
    scene.getBody("plug").setVelocity(rotation @ np.array([0., 0., initial_speed]))

    def run(seconds, force_n):
        force.write(messaging.CmdForceInertialMsgPayload(forceRequestInertial=rotation[:, 2]*force_n))
        sim.ConfigureStopTime(macros.sec2nano(seconds))
        sim.ExecuteSimulation()
        a = scene.getBody("plug").getFreeJoint().getQposAdr()
        b = scene.getBody("bus").getFreeJoint().getQposAdr()
        base_rotation = quaternion_wxyz_to_matrix(stepper.qpos[b+3:b+7])
        return float((base_rotation.T @ (stepper.qpos[a:a+3]-stepper.qpos[b:b+3]))[2]-.3)

    return run, component, stepper, ctx, sim, scene, force


def test_native_small_load_holds_large_load_releases_and_free_plug_keeps_drifting(tmp_path):
    run, component, core, *_keep = build_pair(tmp_path)
    try:
        held = run(.5, 1.)
        assert abs(held) < 1e-10
        assert component.models["plug"].detent.active is not None
        run(.8, 8.)
        assert component.models["plug"].detent.status == "free"
        distance = run(1., 0.)
        assert run(1.2, 0.) > distance > .008
        assert np.isfinite(core.qpos).all()
        np.testing.assert_array_equal(component.models["plug"].last_force_n, np.zeros(3))
        # A released channel reports no friction force while the native core
        # continues to expose the plug's actual outward velocity.
        row = component.telemetry()["plug"]
        assert row["friction_mode"] == "free"
        assert row["axial_velocity_m_s"] > 0.
        np.testing.assert_array_equal(row["force_inertial_n"], np.zeros(3))
    finally:
        core.close()


@pytest.mark.parametrize("substeps", [1, 4])
@pytest.mark.parametrize("load", [-4.9, -1., 0., 1., 4.9])
def test_native_static_friction_holds_partially_inserted_without_creep(tmp_path, substeps, load):
    run, component, core, *_keep = build_pair(tmp_path, initial_z=.304, substeps=substeps)
    try:
        assert run(1., load) == pytest.approx(.004, abs=1e-10)
        # A 1e-6 m/s solver tolerance permits sub-micrometre accumulated error.
        assert run(2., 0.) == pytest.approx(.004, abs=1e-6)
        row = component.telemetry()["plug"]
        assert row["friction_mode"] == "static"
        assert row["status"] == "held"
        assert abs(row["axial_friction_force_n"]) <= 5
    finally:
        core.close()


@pytest.mark.parametrize("depth", [.001, .004, .007])
@pytest.mark.parametrize("speed", [-.4, -.2, .2, .4])
def test_native_sliding_force_is_constant_across_depth_speed_and_direction(tmp_path, depth, speed):
    run, component, core, *_keep = build_pair(tmp_path, initial_z=.3+depth, initial_speed=speed)
    try:
        run(.0025, 0.)
        row = component.telemetry()["plug"]
        assert row["friction_mode"] == "sliding"
        assert row["axial_friction_force_n"] == pytest.approx(-math.copysign(4., speed), abs=1e-8)
    finally:
        core.close()


@pytest.mark.parametrize("direction", [-1., 1.])
def test_native_release_during_motion_stops_mid_stroke_without_axial_return(tmp_path, direction):
    run, component, core, *_keep = build_pair(tmp_path, initial_z=.304)
    try:
        start = run(.1, direction)
        moved = run(.11, 6.*direction)
        stopped = run(.2, 0.)
        final = run(1.2, 0.)
        assert direction*(moved-start) > .0001
        assert 0 < stopped < .008
        assert final == pytest.approx(stopped, abs=1e-10)
        assert component.telemetry()["plug"]["friction_mode"] == "static"
    finally:
        core.close()


def test_native_rotated_interface_with_offset_coms_conserves_internal_momentum(tmp_path):
    rotation = axis_angle_to_matrix(np.array([.3, -.2, .1]), .8)
    errors = []
    for substeps in (1, 8):
        run, component, core, *_keep = build_pair(tmp_path, initial_z=.304, initial_speed=.2,
                                                 rotation=rotation, offset_com=True, substeps=substeps)
        try:
            run(0., 0.)
            before = core.momentum()
            run(.005, 0.)
            after = core.momentum()
            error = [np.linalg.norm(after[key]-before[key])
                     for key in ("linear_kg_m_s", "angular_about_com_kg_m2_s")]
            # COM-offset rotation has the usual first-order integration error;
            # finer stepping must reduce it, rather than accumulate momentum.
            assert error[0] < 1e-5
            assert error[1] < 3e-6
            errors.append(error)
            assert np.isfinite(component.telemetry()["plug"]["force_inertial_n"]).all()
        finally:
            core.close()
    assert np.all(np.asarray(errors[1]) < np.asarray(errors[0])*.6)


def test_native_initial_capture_uses_final_scene_pose_instead_of_reset_message(tmp_path):
    sockets = (seat(), seat("assembly", (0., 0., .4), False))
    run, component, core, *_keep = build_pair(tmp_path, initial_z=.404, sockets=sockets)
    try:
        assert run(.5, 1.) == pytest.approx(.104, abs=1e-10)
        assert component.telemetry()["plug"]["socket"] == "assembly"
        assert component.telemetry()["plug"]["status"] == "held"
    finally:
        core.close()


@pytest.mark.parametrize("load", [-4.9, 4.9])
def test_native_stiction_prevents_midpoint_drift_against_an_articulated_base(tmp_path, load):
    run, component, core, *_keep = build_pair(tmp_path, initial_z=.304, offset_com=True,
                                             articulated_base=True)
    try:
        assert run(1., load) == pytest.approx(.004, abs=2e-6)
        stopped = run(1.2, 0.)
        assert run(2., 0.) == pytest.approx(stopped, abs=2e-6)
        assert component.telemetry()["plug"]["status"] == "held"
    finally:
        core.close()
