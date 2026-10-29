"""Threaded task execution must preserve the direct, coordinated approach."""

from dataclasses import replace
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from simulation.scripted_tasks.threaded_motion import ThreadedSample
from simulation.scripted_tasks.threaded_tasks import create_threaded_task
from simulation.serial_chain_kinematics import axis_angle_to_matrix, rotation_matrix_to_vector
from space_arm_platform.control_defaults import DEFAULT_OPERATING_JOINT_DEG
from simulation.model_preparation.threaded_task_initialization import apply_locking_cycle_initial_state


MODEL = Path(__file__).resolve().parents[1] / "model/SARM/platform/sarm_20261012.xml"


def observed(task, joints, *, scalars=None):
    if scalars is None:
        position, rotation = task.initial_position, task.initial_rotation
    else:
        position, rotation = task.mechanism_pose(*(scalars[k] for k in task._required_scalars))
    return ThreadedSample(
        0., joints.copy(), np.zeros(8), position.copy(), rotation.copy(),
        np.zeros(3), np.zeros(3), np.zeros(6), True,
        scalar_joints={} if scalars is None else scalars,
    )


def check_approach(task, sample):
    """Check the FK of the timed command, not just the Cartesian helper."""
    profile = task._path_opening[2]
    start, start_rotation = task.chain.forward(task.reference[:6])
    end, _ = task.chain.forward(task._path[-1])
    references = []
    times = np.linspace(0., task._duration, 401)
    for time in times:
        progress = task._timing.at(time)
        q = task.planned_path_reference(time)
        position, rotation = task.chain.forward(q[:6])
        _, expected_rotation = profile.pose(progress)
        np.testing.assert_allclose(position, start + (end-start)*progress, atol=2e-5)
        assert np.linalg.norm(rotation_matrix_to_vector(expected_rotation @ rotation.T)) < 2e-5
        assert q[6] == pytest.approx(profile.opening(progress))
        assert q[7] == -q[6]
        references.append(q)
    references = np.asarray(references)
    opening_delta = profile.goal_opening-profile.initial_opening
    assert np.all(np.diff(references[:, 6])*np.sign(opening_delta) >= -1e-12)
    midpoint = task.path_reference(profile.blend.finish / 2)
    position, rotation = task.chain.forward(midpoint[:6])
    assert np.linalg.norm(position-start) > 1e-4
    if abs(opening_delta) > 1e-6:
        assert min(references[0, 6], references[-1, 6]) < midpoint[6] < max(references[0, 6], references[-1, 6])
    if profile.angle > 1e-4:
        assert np.linalg.norm(rotation_matrix_to_vector(rotation @ start_rotation.T)) > 1e-4
    # Arm and opening share the same execution clock when tracking lags.
    task._motion_s = task._duration * .3
    task.reference[:] = task.planned_path_reference(task._motion_s)
    lagged = task.reference.copy()
    lagged[0] -= .1
    before = task.reference.copy()
    task._path_step(replace(sample, joints=lagged), .005)
    np.testing.assert_allclose(task.reference, before, atol=1e-12)


@pytest.mark.parametrize("speed_profile", ["default", "fast_collection"])
def test_bolt_runtime_approach_is_direct_with_shared_arm_and_jaw_clock(speed_profile):
    task = create_threaded_task("aviation_bolt_insert", MODEL, speed_profile=speed_profile)
    joints = np.r_[np.deg2rad(DEFAULT_OPERATING_JOINT_DEG), 0., 0.]
    sample = observed(task, joints)
    task.reference[:] = joints
    task._enter("approach", sample)
    check_approach(task, sample)


@pytest.fixture(scope="module", params=["default", "fast_collection"])
def rod_paths(request, tmp_path_factory):
    root = ET.parse(MODEL).getroot()
    apply_locking_cycle_initial_state(root)
    model = tmp_path_factory.mktemp("direct-rod") / "model.xml"
    ET.ElementTree(root).write(model)
    task = create_threaded_task("locking_rod_cycle", model, speed_profile=request.param)
    # Contact geometry belongs to the native integration tests. Here replay
    # its nominal grasp width so all real reapproach stage starts are covered.
    task._contact_opening = lambda *args: .33
    initial = np.r_[np.deg2rad(DEFAULT_OPERATING_JOINT_DEG), 0., 0.]
    paths = task.preflight_paths(initial)
    return model, request.param, {name: (refs, scalars) for name, refs, _, scalars in paths}


@pytest.mark.parametrize("stage", ["approach", "fold_approach", "reapproach", "turn_approach"])
def test_rod_runtime_and_preflight_reapproaches_are_direct(rod_paths, stage):
    model, speed_profile, paths = rod_paths
    references, scalar_path = paths[stage]
    task = create_threaded_task("locking_rod_cycle", model, speed_profile=speed_profile)
    sample = observed(task, references[0], scalars={k: float(v[0]) for k, v in scalar_path.items()})
    task.reference[:] = sample.joints
    task._enter(stage, sample)
    check_approach(task, sample)
    positions = np.array([task.chain.forward(q[:6])[0] for q in references])
    direction = positions[-1]-positions[0]
    distance = np.linalg.norm(direction)
    deviations = np.linalg.norm(np.cross(positions-positions[0], direction/distance), axis=1)
    assert deviations.max() < 2e-5


@pytest.mark.parametrize("lateral_distance", [.015, -.015])
def test_short_lateral_reapproach_is_checked_by_ik_without_an_axial_detour(lateral_distance):
    task = create_threaded_task("aviation_bolt_insert", MODEL)
    joints = np.r_[np.deg2rad(DEFAULT_OPERATING_JOINT_DEG), .1, -.1]
    sample = observed(task, joints)
    task.reference[:] = joints
    position, rotation = task.chain.forward(joints[:6])
    goal = position + rotation[:, 0]*lateral_distance
    goal_rotation = axis_angle_to_matrix(rotation[:, 2], .07) @ rotation
    task._plan_pose(goal, goal_rotation, approach=True)
    check_approach(task, sample)


@pytest.mark.parametrize("task_name", ["aviation_plug_insert", "guide_plug_insert", "cylindrical_plug_insert"])
def test_searched_plug_approach_owns_execution_but_grip_still_tracks_drift(task_name, monkeypatch):
    from test_independent_plug_tasks import make_task
    from simulation.scripted_tasks import environment_motion
    from simulation.scripted_tasks.collision_motion import _install

    task, sample = make_task(task_name)
    task._enter("approach", sample)
    # Supply a checked-search result with a different jaw law and a small
    # arm detour. Collision search itself is covered in test_collision_motion;
    # this verifies that the actual plug controller executes its installed 7D
    # result instead of retaining the straight candidate or a second tracker.
    progress = np.linspace(0., 1., len(task._path))
    knots = np.column_stack([task._path.copy(), progress**2*(3.-2.*progress)])
    knots[:, 0] += .02*np.sin(np.pi*progress)**2
    def install_search(proxy, *args, **kwargs):
        _install(proxy, knots, held=False)
        return {"route": "searched"}

    monkeypatch.setattr(environment_motion, "plan_free_motion", install_search)
    environment_motion.plan(task, sample, task._goal, task._rotation, [0., 0., 1.])
    task._motion_s = task._duration * .35
    task.reference[:] = task.planned_path_reference(task._motion_s)
    task._last_time = sample.time_s
    for tick in range(1, 41):
        moved = replace(sample, time_s=tick*.005, joints=task.reference.copy(),
                        plug_position_body_m=sample.plug_position_body_m+[.003, -.002, 0.])
        expected = task.planned_path_reference(task._motion_s+.005)
        reference, _ = task.step(moved)
        np.testing.assert_allclose(reference, expected, atol=1e-12)
    assert task.stage == "approach"

    task.reference[:] = task.planned_path_reference(task._duration)
    at_grasp = replace(sample, time_s=.2, joints=task.reference.copy())
    task._enter("grip", at_grasp)
    initial_position, _ = task.chain.forward(task.reference[:6])
    shifted_goal = initial_position + [.003, 0., 0.]
    for tick in range(1, 81):
        moved = replace(at_grasp, time_s=.2+tick*.005, joints=task.reference.copy(),
                        plug_position_body_m=sample.plug_position_body_m+[.003, 0., 0.])
        task.step(moved)
    final_position, _ = task.chain.forward(task.reference[:6])
    assert task.stage == "grip"
    assert np.linalg.norm(final_position-shifted_goal) < .001
    assert np.linalg.norm(final_position-initial_position) > .002
