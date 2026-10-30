"""Compare the native law against the independent Python reference at transitions."""
import math
from pathlib import Path

import numpy as np
import pytest

from simulation.native_plug_retention import NativePlugFriction
from simulation.plug_retention import PlugFriction, RetentionParameters, RetentionSocket, paired_com_wrenches
from simulation.serial_chain_kinematics import axis_angle_to_matrix, quaternion_wxyz_to_matrix


@pytest.mark.parametrize("keyed", [True, False])
@pytest.mark.parametrize("seed", [5, 41, 728])
def test_guidance_matches_reference_across_capture_release_and_reentry(keyed, seed):
    if not (Path(__file__).resolve().parents[1] / "run/native_acceleration/local_mujoco_stepper.json").exists():
        pytest.skip("build the local_mujoco_stepper helper first")
    rng = np.random.default_rng(seed)
    parameters = RetentionParameters()
    rotation = axis_angle_to_matrix(rng.normal(size=3), .6)
    sockets = tuple(RetentionSocket(
        str(i), rng.uniform(-.5, .5, 3), rotation,
        rotation @ np.diag([1., -1., -1.]), i == 0, keyed,
    ) for i in range(3))
    reference = PlugFriction(sockets, parameters)
    native = NativePlugFriction(sockets, parameters)
    for trial in range(120):
        if trial % 5 == 0:
            reference.reset()
            native.reset()
        socket = sockets[trial % 3]
        # Capture, seated backstop, slide, release by depth/side/angle, then
        # reenter a different socket. Large errors also exercise torque limits.
        cases = [(.006, .0001, .01, -.001), (0, 0, 0, 0),
                 (-.003, .0002, .04, -.1), (.004, .0015, .1, .002),
                 (.009, 0, 0, .001), (.004, .021, 0, -.001),
                 (.004, 0, math.pi, -.001), (.006, 0, 0, -.001)]
        for z, lateral, angle, axial_speed in cases:
            position = socket.position_body_m + socket.rotation_body @ np.array([lateral, 0, z])
            actual = axis_angle_to_matrix(rng.normal(size=3), angle) @ socket.plug_rotation_body
            velocity = socket.rotation_body @ np.array([.001, -.002, axial_speed])
            omega = rng.normal(size=3)
            expected = reference.wrench(position, actual, velocity, omega)
            result = native.wrench(position, actual, velocity, omega)
            assert reference.active is native.active
            assert reference.status == native.status
            assert reference.first_sample == native.first_sample
            assert reference.displacement_m == pytest.approx(native.displacement_m, abs=1e-14)
            np.testing.assert_allclose(result, expected, atol=2e-12, rtol=2e-11)
    reference.reset()
    native.reset()
    assert native.active is None and native.first_sample and native.displacement_m is None


def test_native_guidance_rejects_nonfinite_sample():
    if not (Path(__file__).resolve().parents[1] / "run/native_acceleration/local_mujoco_stepper.json").exists():
        pytest.skip("build the local_mujoco_stepper helper first")
    socket = RetentionSocket("seat", np.zeros(3), np.eye(3), np.eye(3), True)
    native = NativePlugFriction((socket,), RetentionParameters())
    with pytest.raises(ValueError, match="finite"):
        native.wrench([math.nan, 0, 0], np.eye(3), np.zeros(3), np.zeros(3))


@pytest.mark.parametrize("orbital_offset", [0., 6.9e6])
def test_native_body_transforms_and_com_reactions_match_message_path(orbital_offset):
    if not (Path(__file__).resolve().parents[1] / "run/native_acceleration/local_mujoco_stepper.json").exists():
        pytest.skip("build the local_mujoco_stepper helper first")
    rng = np.random.default_rng(420)
    for _ in range(200):
        q = rng.normal(size=14)
        q[:3] += orbital_offset
        q[7:10] = q[:3] + rng.uniform(-.5, .5, 3)
        v = rng.normal(size=12) * .001
        coms = rng.uniform(-.05, .05, (2, 3))
        br, pr = [quaternion_wxyz_to_matrix(q[i+3:i+7]) for i in (0, 7)]
        bp, pp = q[:3], q[7:10]
        bw, pw = br @ v[3:6], pr @ v[9:12]
        position = br.T @ (pp-bp)
        rotation = br.T @ pr
        socket = RetentionSocket("seat", position + [0, 0, -.002], np.eye(3), rotation,
                                 True, bool(rng.integers(2)))
        reference = PlugFriction((socket,), RetentionParameters())
        native = NativePlugFriction((socket,), RetentionParameters())
        force, torque = reference.wrench(position, rotation,
            br.T @ (v[6:9]-v[:3]-np.cross(bw, pp-bp)), br.T @ (pw-bw))
        expected = paired_com_wrenches(br @ force, br @ torque, pp,
            pp + pr @ coms[1], bp + br @ coms[0], pr, br)
        actual = native.paired_wrenches(q, v, (0, 0, coms[0]), (7, 6, coms[1]))
        assert native.active is socket
        assert native.status == reference.status
        np.testing.assert_allclose(actual, expected, atol=2e-10, rtol=1e-10)
