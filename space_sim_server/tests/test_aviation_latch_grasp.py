"""Aviation grasps must follow the tabs, not merely touch the hexagonal head."""

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from test_independent_plug_tasks import make_task
from simulation.scripted_tasks.motion import solve_pose
from simulation.scripted_tasks.plug_insert import PlugInsertTask
from simulation.serial_chain_kinematics import axis_angle_to_matrix


@pytest.mark.parametrize("task_name", ["aviation_plug_insert", "guide_plug_insert", "cylindrical_plug_insert"])
def test_grasp_axis_matches_latch_geometry_in_delivered_stl(task_name):
    task, sample = make_task(task_name)
    mesh = (Path(__file__).resolve().parents[1] / "model/SARM_mjcf_20261012/meshes"
            / f"{task.profile.body}.STL")
    triangles = np.fromfile(mesh, offset=84, dtype=np.dtype([
        ("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attr", "<u2")]))
    vertices = np.unique(triangles["vertices"].reshape(-1, 3), axis=0)
    # Measured rounded latch edges occupy x=+/-3 mm, z=4.5..10.5 mm.
    # Select the CAD feature independently of the configured closing axis.
    tabs = vertices[(np.abs(vertices[:, 0]) <= 3.01)
                    & (np.linalg.norm(vertices[:, :2], axis=1) > 20.)
                    & (vertices[:, 2] >= 4.49) & (vertices[:, 2] <= 10.51)]
    centers = []
    for sign in [-1, 1]:
        side = tabs[sign*tabs[:, 1] > 0.]
        assert len(side) >= 20
        np.testing.assert_allclose(side[:, 2].min(), 4.5, atol=.01)
        np.testing.assert_allclose(side[:, 2].max(), 10.5, atol=.01)
        centers.append(side.mean(axis=0))
    physical_axis = centers[1]-centers[0]
    physical_axis /= np.linalg.norm(physical_axis)
    assert abs(np.dot(task.profile.grasp_closing_axis, physical_axis)) > .999
    # Test the commanded grasp frame against geometry, not its own config.
    world_axis = sample.plug_rotation_body @ physical_axis
    assert abs(task.grasp_rotation[:, 0] @ world_axis) > .999


@pytest.mark.parametrize("yaw", [-2.4, -0.4, 0.6, 2.7])
@pytest.mark.parametrize("task_name", ["aviation_plug_insert", "guide_plug_insert", "cylindrical_plug_insert"])
def test_measured_latch_axis_tracks_drift_independently_of_socket(yaw, task_name):
    task, sample = make_task(task_name)
    rotation = axis_angle_to_matrix(np.array([0., 0., 1.]), yaw)
    moved = replace(sample, plug_rotation_body=rotation @ sample.plug_rotation_body,
                    plug_position_body_m=sample.plug_position_body_m + [.003, -.002, 0.])
    position, grasp_r = task._tracked_grasp(moved)
    np.testing.assert_allclose(position, moved.plug_position_body_m
                               + moved.plug_rotation_body @ task.config.grasp_offset_m)
    assert abs(grasp_r[:, 0] @ moved.plug_rotation_body[:, 1]) == pytest.approx(1.)
    np.testing.assert_allclose(grasp_r[:, 2], moved.plug_rotation_body[:, 2])
    np.testing.assert_allclose(grasp_r.T @ grasp_r, np.eye(3), atol=1e-12)
    assert np.linalg.det(grasp_r) == pytest.approx(1.)


def at_grasp(task, sample, wrong_heading=False):
    rotation = task.grasp_rotation
    if wrong_heading:
        rotation = rotation @ axis_angle_to_matrix(np.array([0., 0., 1.]), np.pi / 2)
    q = sample.joints.copy()
    q[:6] = solve_pose(task.chain, task.grasp, rotation, q[:6], task.lower, task.upper)
    q[6:] = [.15, -.15]
    return replace(sample, joints=q, finger_plug_contacts=(1, 1))


@pytest.mark.parametrize("wrong_heading", [False, True])
@pytest.mark.parametrize("task_name", ["aviation_plug_insert", "guide_plug_insert", "cylindrical_plug_insert"])
def test_bilateral_contact_on_wrong_faces_does_not_start_extraction(task_name, wrong_heading):
    task, sample = make_task(task_name)
    sample = at_grasp(task, sample, wrong_heading)
    task.reference[:] = sample.joints
    task.reference[6:] = 0.
    task._enter("grip", sample)
    for tick in range(1, 91):
        task.step(replace(sample, time_s=tick * .005))
    assert task.stage == ("grip" if wrong_heading else "extract")


@pytest.mark.parametrize("task_name", ["aviation_plug_insert", "guide_plug_insert", "cylindrical_plug_insert"])
def test_straight_approach_rotates_and_opens_without_stopping(task_name):
    from simulation.scripted_tasks.drawer_cycle import DrawerCycleConfig
    from simulation.serial_chain_kinematics import rotation_matrix_to_vector
    task, sample = make_task(task_name)
    drawer = DrawerCycleConfig()
    assert task.config.linear_speed_m_s == drawer.linear_speed_m_s
    assert task.config.arm_speed_rad_s == drawer.arm_speed_rad_s
    assert task.config.grasp_approach_speed_m_s == drawer.approach_speed_m_s
    assert task.stages[:3] == ("settle", "approach", "grip")
    start_p, start_r = task.chain.forward(sample.joints[:6])
    task._enter("approach", sample)
    np.testing.assert_allclose(task._goal, task.grasp)
    positions, rotations, references = [], [], []
    times = np.linspace(0., task._duration, 1001)
    for t in times:
        ref = task.planned_path_reference(t)
        position, rotation = task.chain.forward(ref[:6])
        u = task._timing.at(t)
        np.testing.assert_allclose(position, task._approach_profile.pose(u)[0], atol=2e-5)
        np.testing.assert_allclose(position, start_p+(task.grasp-start_p)*u, atol=2e-5)
        assert ref[6] == pytest.approx(task._approach_profile.opening(u))
        assert ref[7] == -ref[6]
        positions.append(position)
        rotations.append(rotation)
        references.append(ref)
    dt = times[1]-times[0]
    assert np.max(np.linalg.norm(np.diff(positions, axis=0), axis=1)/dt) <= drawer.linear_speed_m_s*1.001
    assert np.max(np.abs(np.diff(references, axis=0)[:, :6])/dt) <= drawer.arm_speed_rad_s*1.001
    assert np.max(np.abs(np.diff(references, axis=0)[:, 6])/dt) <= drawer.gripper_speed_rad_s*1.001
    omega = [np.linalg.norm(rotation_matrix_to_vector(b@a.T))/dt
             for a, b in zip(rotations[:-1], rotations[1:])]
    assert max(omega) <= drawer.angular_speed_rad_s*1.001
    assert np.linalg.norm(rotation_matrix_to_vector(rotations[250] @ start_r.T)) > .001
    assert 0 < references[250][6] < 1
    # Execution uses the same finger path as collision preflight, even if arm
    # tracking pauses the shared clock.
    task.step(replace(sample, time_s=.02))
    np.testing.assert_allclose(task.reference[6:], task.planned_path_reference(task._motion_s)[6:])
    clock = task._motion_s
    task.reference[0] = sample.joints[0] + .1
    task.step(replace(sample, time_s=.04))
    assert task._motion_s == clock
    np.testing.assert_allclose(task.reference[6:], task.planned_path_reference(clock)[6:])
    boundary = task._approach_profile.blend.finish
    crossing = [task._timing.state(t) for t in times
                if abs(task._timing.at(t)-boundary) < .005]
    assert crossing and all(state[1] > 0 for state in crossing)
    # Finishing rotation/opening never changes the TCP line of approach.
    for u in np.linspace(boundary, 1., 20):
        p, r = task._approach_profile.pose(u)
        np.testing.assert_allclose(p, start_p+(task.grasp-start_p)*u, atol=1e-12)
        np.testing.assert_allclose(r, task.grasp_rotation, atol=1e-12)
        assert task._approach_profile.opening(u) == 1.


@pytest.mark.parametrize("task_name", ["aviation_plug_insert", "guide_plug_insert", "cylindrical_plug_insert"])
def test_measured_drift_does_not_override_checked_approach(task_name):
    task, sample = make_task(task_name)
    task._enter("approach", sample)
    task._last_time = sample.time_s
    # A persistent shifted target used to modify the already collision-checked
    # arm reference with a second tracker, including on searched approaches.
    for tick in range(1, 81):
        moved = replace(sample, time_s=tick*.005, joints=task.reference.copy(),
                        plug_position_body_m=sample.plug_position_body_m+[.003, -.002, 0.])
        reference, _ = task.step(moved)
        np.testing.assert_allclose(reference, task.planned_path_reference(task._motion_s), atol=1e-12)


def test_extract_keeps_grasp_heading_and_clears_pin_tips_before_turning():
    task, sample = make_task("aviation_plug_insert")
    sample = at_grasp(task, sample)
    task.reference[:] = sample.joints
    task._enter("extract", sample)
    np.testing.assert_allclose(task._plug_goal[:2], task.pickup[:2])
    np.testing.assert_allclose(task._extract_rotation, sample.plug_rotation_body)
    resulting_tip_z = task._extract_z - task.profile.tip_m
    assert resulting_tip_z >= task.pickup[2] + .012 - 1e-12


def test_extraction_recenters_before_advancing_and_does_not_use_target_yaw():
    task, sample = make_task("aviation_plug_insert")
    sample = at_grasp(task, sample)
    task.reference[:] = sample.joints
    task.plug_rotation = axis_angle_to_matrix(np.array([0., 0., 1.]), .6) @ sample.plug_rotation_body
    task._enter("extract", sample)
    initial_z = task._plug_goal[2]
    drifted = replace(sample, time_s=.005,
                      plug_position_body_m=sample.plug_position_body_m + [.002, 0., 0.])
    task.step(drifted)
    assert task._plug_goal[2] == initial_z
    np.testing.assert_allclose(task._extract_rotation, sample.plug_rotation_body)
    task.step(replace(sample, time_s=.01))
    assert initial_z < task._plug_goal[2] <= initial_z + .002


def test_insert_pauses_axial_advance_when_key_is_misaligned():
    task, sample = make_task("aviation_plug_insert")
    sample = at_grasp(task, sample)
    task.reference[:] = sample.joints
    task._enter("extract", sample)  # establish the measured grasp transform
    task._enter("insert", sample)
    before = task._plug_goal.copy()
    task.step(replace(sample, time_s=.005))
    assert task._plug_goal[2] == before[2]


def test_other_plugs_share_continuous_sequence_with_their_latch_axis():
    for name in ("guide_plug_insert", "cylindrical_plug_insert"):
        task, _ = make_task(name)
        assert "open" not in task.stages
        assert "extract" in task.stages
        assert task.coordinated_grasp
        assert task.profile.grasp_closing_axis == (0., 1., 0.)
        assert task.config.insert_speed_m_s == .025
        assert task.config.carry_speed_m_s == .05
        assert task.config.shaft_tip_local_m == .095


def test_contact_load_triggers_small_backoff_before_hard_stop():
    task, sample = make_task("aviation_plug_insert")
    task.config = replace(task.config, overload_stop_enabled=True)
    sample = at_grasp(task, sample)
    task.reference[:] = sample.joints
    task._enter("extract", sample)
    task._enter("insert", sample)
    loaded = replace(sample, time_s=.005, wrench=np.array([0., 0., 20., 3., 0., 0.]))
    task.step(loaded)
    assert task.result is None
    assert task._contact_recovery
    assert task._contact_recoveries == 1
    assert task._plug_goal[2] == pytest.approx(sample.plug_position_body_m[2] + .001)
    assert task.config.force_limit_n == 40.
    assert task.config.torque_limit_nm == 4.


def test_default_aviation_does_not_backoff_or_abort_due_to_load_thresholds():
    task, sample = make_task("aviation_plug_insert")
    sample = at_grasp(task, sample)
    task.reference[:] = sample.joints
    task._enter("extract", sample)
    task._enter("insert", sample)
    for i in range(1, 41):
        task.step(replace(sample, time_s=i * .005,
                          wrench=np.array([0., 0., 50., 5., 0., 0.])))
    assert task.result is None
    assert not task._contact_recovery
    assert task._contact_recoveries == 0


def test_opening_releases_fingers_without_driving_wrist_down_to_seat():
    task, sample = make_task("aviation_plug_insert")
    sample = at_grasp(task, sample)
    task.reference[:] = sample.joints
    task._enter("release", sample)
    wrist_before = task.reference[:6].copy()
    opening_before = task.reference[6]
    task.step(replace(sample, time_s=.005))
    np.testing.assert_array_equal(task.reference[:6], wrist_before)
    assert task.reference[6] > opening_before


def test_release_unwinds_insertion_preload_before_finishing_opening():
    task, sample = make_task("aviation_plug_insert")
    sample = at_grasp(task, sample)
    task.reference[:] = sample.joints
    task.reference[1] -= .10  # preload exceeds the normal 0.06 rad tracking gate
    task.reference[6:] = 0.
    before = task.reference.copy()
    task._enter("release", sample)
    measured_p, measured_r = task.chain.forward(sample.joints[:6])
    np.testing.assert_allclose(task._goal, measured_p)
    np.testing.assert_allclose(task._rotation, measured_r)
    np.testing.assert_array_equal(task._path[0], before[:6])
    for i in range(1, int((task._duration + .1) / .005) + 1):
        task.step(replace(sample, time_s=i * .005))
    assert task.stage == "release"  # arm path complete, fingers still closing/opening
    np.testing.assert_allclose(task.reference[:6], sample.joints[:6], atol=2e-5)


def test_collection_version_and_historical_stage_validation():
    from simulation.scripted_tasks.drawer_initialization import randomized_scene
    source = {"randomization": {"arm_joint_position_rad": [0.] * 8}}
    scene = randomized_scene(source, 42, task_name="aviation_plug_insert")
    version = scene["collection_config"]["task_version"]
    assert version == PlugInsertTask.straight_approach_version
    assert "extract" in PlugInsertTask.recorded_stages(version)
    assert "open" in PlugInsertTask.recorded_stages("plug-insert/2")
    assert "raise" in PlugInsertTask.recorded_stages("plug-insert/3-latch-grasp")
    assert "raise" not in PlugInsertTask.recorded_stages(version)
    assert "descend" in PlugInsertTask.recorded_stages("plug-insert/4-coordinated-approach")
    assert "descend" not in PlugInsertTask.recorded_stages(version)
    with pytest.raises(ValueError, match="Unsupported"):
        PlugInsertTask.recorded_stages("future-unknown")


def test_actual_pickup_following_does_not_chase_socket_vibration():
    task, sample = make_task("aviation_plug_insert")
    sample = at_grasp(task, sample)
    sample = replace(sample, finger_plug_contacts=(0, 0))
    task.reference[:] = sample.joints
    task._enter("grip", sample)
    velocities = []
    for tick in range(1, 601):
        t = tick * .005
        wave = np.sin(2*np.pi*21.5*t)
        rotation = axis_angle_to_matrix(np.array([0., 1., 0.]), .008*wave)
        moved = replace(sample, time_s=t, joints=task.reference.copy(),
                        plug_position_body_m=sample.plug_position_body_m + [.00027*wave, 0., 0.],
                        plug_rotation_body=rotation @ sample.plug_rotation_body)
        _, velocity = task.step(moved)
        if t > 1.:
            velocities.append(velocity[:6])
    assert task.stage == "grip" and task.result is None
    assert np.max(np.abs(velocities)) < .01


@pytest.mark.parametrize("stage", ["lift", "transfer", "above_hole"])
def test_carrying_has_gentle_speed_acceleration_and_jerk(stage):
    task, sample = make_task("aviation_plug_insert")
    task.stage = stage
    task._opening_at_entry = .3
    p, r = task.chain.forward(task.reference[:6])
    goal = p + np.array([.02, .01, -.08])
    task._plan(goal, r)
    positions = []
    times = np.linspace(0, task._duration, 4001)
    for t in times:
        positions.append(task.chain.forward(task.planned_path_reference(t)[:6])[0])
    dt = times[1]-times[0]
    velocity = np.diff(positions, axis=0)/dt
    acceleration = np.diff(velocity, axis=0)/dt
    jerk = np.diff(acceleration, axis=0)/dt
    assert np.linalg.norm(velocity, axis=1).max() <= .0501
    assert np.linalg.norm(acceleration, axis=1).max() <= .1501
    assert np.linalg.norm(jerk, axis=1).max() <= .8001
    # The first second cannot surge straight to cruise speed.
    assert np.linalg.norm(velocity[times[1:] < .1], axis=1).max() < .01
    assert task.config.linear_speed_m_s == .1


def test_gentle_carry_keeps_v5_recordings_compatible():
    assert PlugInsertTask.recorded_stages("plug-insert/5-smooth-tracking") == PlugInsertTask.continuous_carry_stages


@pytest.mark.parametrize("yaw", [0., .5])
@pytest.mark.parametrize("task_name", ["aviation_plug_insert", "guide_plug_insert", "cylindrical_plug_insert"])
def test_continuous_carry_does_not_stop_or_replan_at_phase_boundaries(task_name, yaw):
    task, sample = make_task(task_name)
    sample = at_grasp(task, sample)
    start = sample.plug_position_body_m + [0., 0., .093]
    q = sample.joints.copy()
    q[:6] = solve_pose(task.chain, start + sample.plug_rotation_body @ task.config.grasp_offset_m,
                       task.grasp_rotation, q[:6], task.lower, task.upper)
    sample = replace(sample, joints=q, plug_position_body_m=start)
    task.reference[:] = q
    task.plug_rotation = axis_angle_to_matrix(np.array([0., 0., 1.]), yaw) @ task.plug_rotation
    task._enter("lift", sample)
    timing = task._timing
    times = np.linspace(0., timing.duration, 3001)
    states = [timing.joint_state(t) for t in times]
    positions = np.array([task.chain.forward(s[0])[0] for s in states])
    dt = times[1]-times[0]
    speed = np.linalg.norm(np.diff(positions, axis=0), axis=1)/dt
    assert speed.max() <= .051
    assert speed.max() > .035
    acceleration = np.diff(positions, n=2, axis=0)/dt**2
    jerk = np.diff(positions, n=3, axis=0)/dt**3
    assert np.linalg.norm(acceleration, axis=1).max() < .151
    assert np.linalg.norm(jerk, axis=1).max() < .81
    assert np.abs(np.array([s[1] for s in states])).max() <= task.config.arm_speed_rad_s*1.001
    assert np.abs(np.array([s[2] for s in states])).max() <= .701
    assert np.abs(np.array([s[3] for s in states])).max() <= 8.01
    for index, stage in enumerate(("transfer", "above_hole")):
        boundary = task._carry_profile.boundaries[index]
        t = times[np.argmin([abs(timing.at(t)-boundary) for t in times])]
        assert timing.state(t)[1]*task._carry_profile.length > .025
        task._motion_s = t
        task._enter(stage, sample)
        assert task._timing is timing
        assert task._motion_s == t
    end_p, end_r = task.chain.forward(task.planned_path_reference(timing.duration)[:6])
    offset, relative_r = task._grasp_transform
    np.testing.assert_allclose(end_p+end_r @ offset, [*task.hole[:2], task.hole[2]+task.profile.tip_m+.012], atol=2e-5)
    np.testing.assert_allclose(end_r @ relative_r, task.plug_rotation, atol=2e-5)
    assert PlugInsertTask.recorded_stages("plug-insert/6-gentle-carry") == task.continuous_carry_stages


def test_contact_speed_is_not_limited_to_position_gain_times_two_mm():
    task, sample = make_task("aviation_plug_insert")
    sample = at_grasp(task, sample)
    task.reference[:] = sample.joints
    task._enter("extract", sample)
    # Evaluate steady moving contact, without inventing an instantaneous
    # zero-inertia robot around the velocity-feedback controller.
    jac = task.chain.jacobian(sample.joints[:6])
    desired = np.array([0., 0., .025, 0., 0., 0.])
    qd = np.linalg.solve(jac, desired)
    sample = replace(sample, time_s=.005, joint_velocities=np.r_[qd, 0., 0.])
    task._axial_speed = task._axial_measured_velocity = .025
    _, velocity = task.step(sample)
    commanded = task.chain.jacobian(task.reference[:6])[:3] @ velocity[:6]
    assert commanded[2] == pytest.approx(.025, abs=.001)
    assert task.stage == "extract"
    assert task.config.insert_speed_m_s == .025
    assert "align_hole" not in task.stages and "seat" not in task.stages
    assert "align_hole" in task.recorded_stages("plug-insert/7-continuous-carry")


@pytest.mark.parametrize("task_name", ["aviation_plug_insert", "guide_plug_insert", "cylindrical_plug_insert"])
def test_release_unloads_finger_preload_with_arm_before_opening(task_name):
    task, sample = make_task(task_name)
    sample = at_grasp(task, sample)
    task.reference[:] = sample.joints
    task.reference[1] -= .10
    task.reference[6:] = 0.
    task._enter("release", sample)
    for i in range(1, int(.8*task._duration/.005)):
        task.step(replace(sample, time_s=i*.005))
        assert 0. <= task.reference[6] <= sample.joints[6]
    for i in range(int(.8*task._duration/.005), int((task._duration+.4)/.005)+1):
        task.step(replace(sample, time_s=i*.005))
    assert task.reference[6] > sample.joints[6] + .001


@pytest.mark.parametrize("contacts", [(0, 0), (1, 0), (0, 1)])
@pytest.mark.parametrize("task_name", ["aviation_plug_insert", "guide_plug_insert", "cylindrical_plug_insert"])
def test_release_retracts_before_full_open_only_after_both_fingers_clear(task_name, contacts):
    task, sample = make_task(task_name)
    sample = at_grasp(task, sample)
    task.reference[:] = sample.joints
    task._enter("release", sample)
    timing = task._gripper_timing
    for tick in range(1, 1501):
        # An ideal tracking fixture isolates the stage handoff from dynamics.
        moving = replace(sample, time_s=tick*.005, joints=task.reference.copy(),
                         finger_plug_contacts=contacts)
        task.step(moving)
        if task.stage == "retract":
            break
    if any(contacts):
        assert task.stage == "release"
        return
    assert task.stage == "retract"
    assert .23 <= moving.joints[6] < task.config.release_open_rad-.1
    assert task._gripper_timing is timing
    before = task.reference.copy()
    elapsed = task._release_opening_s
    for extra in range(1, 21):
        task.step(replace(moving, time_s=moving.time_s+extra*.005,
                          joints=task.reference.copy()))
    assert task._release_opening_s > elapsed
    assert task.reference[6] > before[6]
    assert np.linalg.norm(task.reference[:6]-before[:6]) > 1e-6
    assert task.recorded_stages("plug-insert/8-continuous-contact") == task.latch_stages


@pytest.mark.parametrize("gap", [.0005, .002])
def test_seating_uses_actual_assembly_not_lagging_virtual_target(gap):
    task, sample = make_task("aviation_plug_insert")
    position = task.hole + [0., 0., task.config.flange_local_m+gap]
    rotation = task.plug_rotation
    q = sample.joints.copy()
    q[:6] = solve_pose(task.chain, position+rotation @ task.config.grasp_offset_m,
                       task.grasp_rotation, q[:6], task.lower, task.upper)
    q[6:] = [.15, -.15]
    sample = replace(sample, joints=q, plug_position_body_m=position,
                     plug_rotation_body=rotation, finger_plug_contacts=(1, 1))
    task.reference[:] = q
    task.reference[6:] = 0.
    task._enter("extract", sample)
    task._enter("insert", sample)
    task._plug_goal[2] = task._seat_z+.002
    for i in range(1, 34):
        task.step(replace(sample, time_s=i*.005))
    assert task.stage == ("release" if gap < .001 else "insert")


def test_blocked_contact_does_not_accumulate_unbounded_servo_preload():
    task, sample = make_task("aviation_plug_insert")
    sample = at_grasp(task, sample)
    task.reference[:] = sample.joints
    task._enter("extract", sample)
    for i in range(1, 801):
        task.step(replace(sample, time_s=i*.005))
    assert task.result is None
    assert np.max(np.abs(task.reference[:6]-sample.joints[:6])) <= .060001
    assert not task.config.overload_stop_enabled
