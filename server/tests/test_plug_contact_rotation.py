"""Round-shaft spin must not hide tilt or weaken keyed connector control."""

from dataclasses import replace

import numpy as np
import pytest

from simulation.serial_chain_kinematics import axis_angle_to_matrix, rotation_matrix_to_vector
from test_independent_plug_tasks import make_task


def _prepare_contact_stage(task, sample, stage):
    task.stage = stage
    task._path = task._timing = task._gripper_timing = None
    task._entered = 0.
    task._timeout = 90.
    task._plug_goal = sample.plug_position_body_m.copy()
    task._seat_z = task.hole[2] + task.config.flange_local_m
    task._contact_feedback.reset(task.reference[:6])
    position, rotation = task.chain.forward(sample.joints[:6])
    task._grasp_transform = (
        rotation.T @ (sample.plug_position_body_m-position),
        rotation.T @ sample.plug_rotation_body,
    )


@pytest.mark.parametrize("name,axis,angle,allowed", [
    ("cylindrical_plug_insert", [0., 0., 1.], .2, True),
    ("cylindrical_plug_insert", [1., 0., 0.], .08, False),
    ("aviation_plug_insert", [0., 0., 1.], .2, False),
    ("guide_plug_insert", [0., 0., 1.], .2, False),
])
def test_extraction_ignores_round_spin_but_holds_tilt_and_key_errors(monkeypatch, name, axis, angle, allowed):
    task, sample = make_task(name)
    desired = task.pickup_rotation @ np.diag([1., -1., -1.])
    task._extract_rotation = desired
    task._extract_z = task.pickup[2] + task.profile.tip_m + .012
    sample = replace(
        sample, plug_position_body_m=np.r_[task.pickup[:2], task._extract_z],
        plug_rotation_body=axis_angle_to_matrix(np.asarray(axis), angle) @ desired,
    )
    _prepare_contact_stage(task, sample, "extract")
    velocities, transitions = [], []
    monkeypatch.setattr(task, "_part_feedback", lambda *_args, **kwargs: velocities.append(kwargs["axial_velocity"]))
    monkeypatch.setattr(task, "_next", lambda observed: transitions.append(observed.time_s))
    task.step(replace(sample, time_s=.02))
    assert bool(velocities[0] > 0.) is allowed
    assert bool(transitions) is allowed


@pytest.mark.parametrize("name,tilt,yaw,angular_velocity,allowed", [
    ("cylindrical_plug_insert", 0., .4, [0., 0., .5], True),
    ("cylindrical_plug_insert", .02, .4, [0., 0., .5], False),
    ("cylindrical_plug_insert", 0., .4, [.04, 0., .5], False),
    ("aviation_plug_insert", 0., .08, [0., 0., 0.], False),
    ("aviation_plug_insert", 0., 0., [0., 0., .5], False),
])
def test_held_seating_excludes_only_round_axial_spin(monkeypatch, name, tilt, yaw, angular_velocity, allowed):
    task, sample = make_task(name)
    desired = task.hole_rotation @ np.diag([1., -1., -1.])
    actual = (axis_angle_to_matrix(np.array([1., 0., 0.]), tilt)
              @ axis_angle_to_matrix(np.array([0., 0., 1.]), yaw) @ desired)
    sample = replace(
        sample, plug_position_body_m=task.hole + [0., 0., task.config.flange_local_m],
        plug_rotation_body=actual,
        plug_angular_velocity_body_rad_s=np.asarray(angular_velocity),
    )
    _prepare_contact_stage(task, sample, "insert")
    transitions = []
    monkeypatch.setattr(task, "_part_feedback", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(task, "_next", lambda observed: transitions.append(observed.time_s))
    for tick in range(1, 10):
        task.step(replace(sample, time_s=tick*.02))
    assert bool(transitions) is allowed


@pytest.mark.parametrize("name,retains_spin", [
    ("cylindrical_plug_insert", True),
    ("aviation_plug_insert", False),
    ("guide_plug_insert", False),
])
def test_contact_rotation_corrects_round_tilt_with_shortest_swing(name, retains_spin):
    task, _ = make_task(name)
    desired = np.diag([1., -1., -1.])
    spun = axis_angle_to_matrix(np.array([0., 0., 1.]), .6) @ desired
    actual = axis_angle_to_matrix(np.array([1., 0., 0.]), .02) @ spun
    corrected = task._contact_rotation(actual, desired)
    np.testing.assert_allclose(corrected[:, 2], desired[:, 2], atol=1e-12)
    if retains_spin:
        np.testing.assert_allclose(corrected, spun, atol=1e-12)
        assert np.linalg.norm(rotation_matrix_to_vector(corrected @ actual.T)) == pytest.approx(.02)
    else:
        np.testing.assert_allclose(corrected, desired, atol=1e-12)


@pytest.mark.parametrize("name,retains_spin", [
    ("cylindrical_plug_insert", True),
    ("aviation_plug_insert", False),
])
def test_contact_feedback_preserves_only_round_yaw(monkeypatch, name, retains_spin):
    task, sample = make_task(name)
    desired = task.hole_rotation @ np.diag([1., -1., -1.])
    actual = axis_angle_to_matrix(np.array([0., 0., 1.]), .3) @ desired
    sample = replace(sample, plug_rotation_body=actual)
    _prepare_contact_stage(task, sample, "insert")
    before = task.reference[:6].copy()
    seen = []
    original = task._tool_for_plug

    def tool_for_plug(observed, position, rotation):
        seen.append(rotation.copy())
        return original(observed, position, rotation)

    monkeypatch.setattr(task, "_tool_for_plug", tool_for_plug)
    # Isolate rotational feedback from the separately tested axial preload.
    task._contact_feedback.friction_force_n = 0.
    task._part_feedback(sample, .005, desired_rotation=desired)
    np.testing.assert_allclose(seen[0], actual if retains_spin else desired, atol=1e-12)
    if retains_spin:
        np.testing.assert_allclose(task.reference[:6], before, atol=1e-12)
    else:
        assert np.linalg.norm(task.reference[:6]-before) > 1e-6
