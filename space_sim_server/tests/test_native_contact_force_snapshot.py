"""Physical contact reactions must be measurable without stepping/editing state."""

import numpy as np

from test_local_mujoco_stepper import MJCF, build, run_to


def test_contact_force_matches_ball_acceleration_and_is_read_only(tmp_path):
    path = tmp_path / "contact_force.xml"
    # The left face is clear of the arm: this ball has one external reaction,
    # no gravity, damping, actuator or other force. Its F = m a is independent
    # evidence for the new raw solver-force accessor.
    path.write_text(MJCF.replace('name="ball" pos="1.5 0 0"',
                                 'name="ball" pos="-0.24 0 0"'), encoding="utf-8")
    sim, scene, stepper, keep = build(path, "local")
    try:
        run_to(sim, .0042)
        before = [array.copy() for array in (stepper.qpos, stepper.qvel, stepper.qacc,
                                             stepper.ctrl, stepper.actuator_force, stepper.stats)]
        before_steps = stepper.step_count
        contacts = stepper.contact_snapshot()
        forces = stepper.contact_force_snapshot()
        geometry = stepper.contact_geometry_snapshot()
        assert forces.shape == (len(contacts), 11) and len(forces) > 0
        np.testing.assert_array_equal(forces[:, :5], contacts)
        assert all({stepper._name(1, int(row[2])), stepper._name(1, int(row[3]))}
                   == {"bus", "ball"} for row in forces)
        assert np.isfinite(forces).all() and forces[:, 5].sum() > .1
        dof = scene.getBody("ball").getFreeJoint().getQvelAdr()
        np.testing.assert_allclose(forces[:, 5].sum(), -.2*stepper.qacc[dof], rtol=1e-10, atol=1e-10)
        # A small tangential reaction is legitimate because the bus also carries
        # the moving test arm. Wrench magnitude still equals the sphere's m a.
        assert len(forces) == 1
        np.testing.assert_array_equal(geometry[:, :5], contacts)
        np.testing.assert_allclose(np.linalg.norm(geometry[:, 8:], axis=1), 1., atol=1e-12)
        ball, ball_rotation = stepper.body_geometry_pose("ball")
        bus, _ = stepper.body_geometry_pose("bus")
        # MuJoCo places a sphere/plane contact halfway between the two
        # penetrating surfaces, at the same phase as its stored body pose.
        np.testing.assert_allclose(np.linalg.norm(geometry[0, 5:8]-ball),
                                   .05+geometry[0, 4]/2, atol=1e-10)
        np.testing.assert_allclose(ball_rotation.T@ball_rotation, np.eye(3), atol=1e-12)
        direction = ball-bus
        if stepper._name(1, int(geometry[0, 2])) == "ball":
            direction = -direction
        assert geometry[0, 8:]@direction > 0
        np.testing.assert_allclose(np.linalg.norm(forces[0, 5:8]),
                                   .2*np.linalg.norm(stepper.qacc[dof:dof+3]), rtol=1e-10, atol=1e-10)
        np.testing.assert_allclose(forces[:, 8:], 0., atol=1e-10)
        np.testing.assert_array_equal(forces, stepper.contact_force_snapshot())
        np.testing.assert_array_equal(geometry, stepper.contact_geometry_snapshot())
        np.testing.assert_array_equal(ball, stepper.body_geometry_pose("ball")[0])
        for expected, actual in zip(before, (stepper.qpos, stepper.qvel, stepper.qacc,
                                             stepper.ctrl, stepper.actuator_force, stepper.stats)):
            np.testing.assert_array_equal(expected, actual)
        assert stepper.step_count == before_steps
    finally:
        stepper.close()
