"""Task-5 release provenance, actuator-only behavior and real-contact gates."""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from simulation.scripted_tasks.drawer_initialization import randomized_scene
from simulation.scripted_tasks.floating_capture import (
    FLOATING_TASKS, FloatingCaptureConfig, create_floating_task,
)
from simulation.scripted_tasks.floating_initialization import FloatingInitialConfig
from simulation.scripted_tasks.native_state import FloatingSample
from simulation.scripted_tasks.randomization import write_model_variant
from simulation.serial_chain_kinematics import (
    quaternion_wxyz_to_matrix, matrix_to_quaternion_wxyz, axis_angle_to_matrix,
)

MODEL = Path(__file__).resolve().parents[1]/"model/SARM/platform/sarm_20261012.xml"


def source_scene():
    return dict(template_id="sarm-task-box-20261012", randomization=dict(
        arm_joint_position_rad=[0.]*8, target_position_m=[3., 0., 0.],
        target_orientation_wxyz=[1., 0., 0., 0.],
        target_linear_velocity_m_s=[.01, -.004, .002],
        target_angular_velocity_rad_s=[0., 0., 0.]))


def realized(tmp_path, name="floating_bolt_capture", seed=71):
    scene = randomized_scene(source_scene(), seed, 30., name)
    scene["scripted_task_name"] = name
    path = write_model_variant(MODEL, tmp_path/f"{name}.xml", scene)
    return scene, path


@pytest.mark.parametrize("name", FLOATING_TASKS)
def test_release_replays_and_preserves_team_geometry(tmp_path, name):
    original = MODEL.read_bytes()
    scene, path = realized(tmp_path, name)
    state = deepcopy(scene["collection_config"]["floating_initial_state"])
    replay = write_model_variant(MODEL, tmp_path/"replay.xml", scene)
    assert replay.read_bytes() == path.read_bytes()
    assert MODEL.read_bytes() == original
    assert scene["collection_config"]["floating_initial_state"] == state
    assert .12 <= state["camera_to_grasp_distance_m"] <= .75
    np.testing.assert_allclose(np.linalg.norm(np.subtract(state["grasp_position_body_m"], state["camera_position_body_m"])),
                               state["camera_to_grasp_distance_m"], atol=1e-12)
    assert 0 < np.linalg.norm(state["velocity_body_m_s"]) <= .001
    assert 0 < np.linalg.norm(state["angular_velocity_body_rad_s"]) <= .003
    before, after = ET.fromstring(original), ET.parse(path).getroot()
    body = after.find(f'./worldbody/body[@name="{FLOATING_TASKS[name].body}"]')
    original_body = before.find(f'.//body[@name="{FLOATING_TASKS[name].body}"]')
    assert body is not None
    assert body.find("freejoint") is not None or body.find('joint[@type="free"]') is not None
    inertial, original_inertial = body.find("inertial"), original_body.find("inertial")
    if original_inertial is None:
        assert inertial is None  # Source mass/inertia are computed from geoms.
    else:
        assert inertial.attrib == original_inertial.attrib
    physical = lambda g: {key: value for key, value in g.attrib.items() if key not in ("rgba", "material")}
    assert [physical(g) for g in body.findall("geom")] == [physical(g) for g in original_body.findall("geom")]
    assert not after.findall("./equality/weld")
    assert before.find('.//camera[@name="sarm_wrist_cam"]').attrib == after.find('.//camera[@name="sarm_wrist_cam"]').attrib
    if name == "floating_panel_capture":
        assert after.find('.//joint[@name="outer_panel_hinge"]') is None


def test_configuration_and_saved_identity_reject_invalid_inputs(tmp_path):
    for values in ({"min_distance_m": .119}, {"max_distance_m": .751},
                   {"min_distance_m": .7, "max_distance_m": .2}, {"max_speed_m_s": float("nan")},
                   {"max_spin_rad_s": -.1}):
        with pytest.raises(ValueError):
            FloatingInitialConfig(**values)
    scene, _ = realized(tmp_path)
    scene["collection_config"]["floating_initial_state"]["task"] = "floating_panel_capture"
    with pytest.raises(ValueError, match="does not match"):
        write_model_variant(MODEL, tmp_path/"invalid.xml", scene)
    with pytest.raises(ValueError):
        FloatingCaptureConfig(arm_speed_rad_s=1.)


@pytest.mark.parametrize("key,value", [("position_body_m", [1., 2., 3.]),
    ("velocity_body_m_s", [.1, 0., 0.]), ("orientation_wxyz", [0., 0., 0., 0.]),
    ("camera_to_grasp_distance_m", float("nan")), ("ik_joint_position_rad", [0.]*6)])
def test_replay_rejects_corrupt_physical_state(tmp_path, key, value):
    scene, _ = realized(tmp_path)
    scene["collection_config"]["floating_initial_state"][key] = value
    with pytest.raises(ValueError, match="floating release"):
        write_model_variant(MODEL, tmp_path/"corrupt.xml", scene)


@pytest.mark.parametrize("limit,tilt", [(0., 10.), (8., 12.), (20., 25.)])
def test_replay_rejects_out_of_bounds_tilt_even_with_consistent_pose_and_ik(tmp_path, limit, tilt):
    from simulation.scripted_tasks.motion import solve_pose
    source = source_scene()
    source["floating_capture"] = {"max_tilt_deg": limit}
    scene = randomized_scene(source, 73, 0., "floating_bolt_capture")
    model = write_model_variant(MODEL, tmp_path/"valid.xml", scene)
    assert write_model_variant(MODEL, tmp_path/"replay.xml", scene).read_bytes() == model.read_bytes()
    task = create_floating_task("floating_bolt_capture", model)
    initial = scene["randomization"]["arm_joint_position_rad"][:6]
    _, initial_rotation = task.chain.forward(initial)
    desired = axis_angle_to_matrix(np.array([0., 0., 1.]), np.deg2rad(tilt))@initial_rotation
    asset = task.asset
    basis = np.column_stack((asset.closing_axis, np.cross(asset.approach_axis, asset.closing_axis), asset.approach_axis))
    rotation = desired@basis.T
    release = scene["collection_config"]["floating_initial_state"]
    point = np.asarray(release["grasp_position_body_m"])
    goal = solve_pose(task.chain, point, desired, initial, task.lower, task.upper)
    release.update(position_body_m=(point-rotation@asset.grip_point).tolist(),
        orientation_wxyz=matrix_to_quaternion_wxyz(rotation).tolist(), ik_joint_position_rad=goal.tolist())
    with pytest.raises(ValueError, match="physical configuration"):
        write_model_variant(MODEL, tmp_path/"outside_bounds.xml", scene)


@pytest.mark.parametrize("distance", [.12, .75])
def test_declared_distance_endpoints_are_reachable(tmp_path, distance):
    source = source_scene()
    source["floating_capture"] = dict(min_distance_m=distance, max_distance_m=distance)
    for name in FLOATING_TASKS:
        scene = randomized_scene(source, 71, 0, name)
        write_model_variant(MODEL, tmp_path/f"{name}.xml", scene)
        assert scene["collection_config"]["floating_initial_state"]["camera_to_grasp_distance_m"] == distance


def sample_at_grasp(task, scene, stamp=0., contacts=(0, 0)):
    state = scene["collection_config"]["floating_initial_state"]
    q = np.r_[state["ik_joint_position_rad"], 1., -1.]
    return FloatingSample(stamp, q, np.zeros(8), np.asarray(state["position_body_m"]),
        quaternion_wxyz_to_matrix(state["orientation_wxyz"]), np.zeros(3), np.zeros(3),
        np.zeros(6), True, contacts, finger_normal_force_n=(1., 1.))


def test_position_agreement_alone_cannot_succeed(tmp_path):
    scene, path = realized(tmp_path)
    task = create_floating_task("floating_bolt_capture", path)
    sample = sample_at_grasp(task, scene)
    for i in range(2200):
        task.step(replace(sample, time_s=i*.01))
    assert task.result == "failed"
    assert task.reason == "bilateral_contact_not_established"


def test_contact_hold_is_continuous_and_does_not_modify_measurements(tmp_path):
    scene, path = realized(tmp_path)
    task = create_floating_task("floating_bolt_capture", path)
    sample = sample_at_grasp(task, scene, contacts=(1, 1))
    measured = sample.joints.copy()
    for i in range(180):
        task.step(replace(sample, time_s=i*.01))
    assert task.stage == "hold" and task.result is None
    task.step(replace(sample, time_s=1.8, finger_plug_contacts=(1, 0)))
    assert task.metrics(sample)["held_seconds"] == 0.
    for i in range(181, 400):
        task.step(replace(sample, time_s=i*.01))
    assert task.result is None
    for i in range(400, 510):
        task.step(replace(sample, time_s=i*.01))
    assert task.result == "success"
    np.testing.assert_array_equal(sample.joints, measured)
    assert all(event["stage"] != "success" or event["sim_time_s"] >= 4.8 for event in task.events)


def test_unexpected_contact_and_target_loss_fail(tmp_path):
    scene, path = realized(tmp_path)
    task = create_floating_task("floating_bolt_capture", path)
    sample = sample_at_grasp(task, scene, contacts=(1, 1))
    task.step(replace(sample, unexpected_contacts=[{"body1": "link3", "body2": "bolt"}]))
    assert task.result == "failed" and task.reason == "unexpected_collision"
    task = create_floating_task("floating_bolt_capture", path)
    for i in range(200):
        contacts = (1, 1) if i < 100 else (0, 0)
        task.step(replace(sample, time_s=i*.01, finger_plug_contacts=contacts))
    assert task.result == "failed" and task.reason == "target_lost"


@pytest.mark.parametrize("contacts,forces", [((1,), (1., 1.)), ((1, 1), (1.,)),
    ((1, -1), (1., 1.)), ((1, .5), (1., 1.)), ((1, float("nan")), (1., 1.)),
    ((1, 1), (1., -1.))])
def test_malformed_contact_channels_cannot_establish_a_grasp(tmp_path, contacts, forces):
    scene, path = realized(tmp_path)
    task = create_floating_task("floating_bolt_capture", path)
    sample = replace(sample_at_grasp(task, scene), finger_plug_contacts=contacts,
                     finger_normal_force_n=forces)
    task.step(sample)
    assert task.result == "failed" and task.reason == "invalid_contact_observation"


def test_existing_collection_registry_exposes_floating_tasks():
    from space_arm_platform.collection_jobs import CollectionCreate, TASKS
    for name in FLOATING_TASKS:
        plan = CollectionCreate(task=name)
        assert plan.range_deg == 0 and TASKS[name].script == "collection/collect_scripted_tasks.py"
    with pytest.raises(ValueError, match="固定相机"):
        CollectionCreate(task="floating_bolt_capture", randomize_overview_camera=True)


def test_preflight_does_not_exempt_target_environment_or_initial_finger_collisions():
    from simulation.scripted_tasks.drawer_initialization import NativePoseProbe
    probe = NativePoseProbe.__new__(NativePoseProbe)
    probe.home = probe.q = np.zeros(8)
    probe.names = {1: "satellite_outer_panel", 2: "ee_finger1", 3: "task_box_base"}
    probe.contacts = lambda q: np.array([[0, 1, 1, 3, -.002]])
    contact = probe.pose_collision(np.zeros(8), "floating_panel_capture",
                                   allowed_contact_bodies=("satellite_outer_panel",))
    assert contact["collision_kind"] == "mechanism_environment_collision"
    probe.contacts = lambda q: np.array([[0, 1, 1, 2, -.002]])
    assert probe.pose_collision(np.zeros(8), "floating_panel_capture") is not None
    assert probe.pose_collision(np.zeros(8), "floating_panel_capture",
                                allowed_contact_bodies=("satellite_outer_panel",)) is None


def test_recording_rejects_a_success_flag_with_an_interrupted_hold():
    from collection.collect_scripted_tasks import validate_floating_recording
    from simulation.scripted_tasks.floating_capture import FloatingCaptureTask
    state = dict(task_version=FloatingCaptureTask.version, target_body="bolt", result="success",
        task_parameters={"hold_s": 3.}, held_seconds=3., stage="hold",
        feedback_source="authoritative_simulation_state", finger_target_contacts=[1, 1],
        finger_normal_force_n=[1., 1.], slip_m=0., angular_slip_rad=0., relative_speed_m_s=0.)
    steps = [dict(sim_time_ns=str(i*20_000_000), scripted_task=deepcopy(state)) for i in range(160)]
    validate_floating_recording(steps, list(FloatingCaptureTask.stages), "floating_bolt_capture")
    steps[50]["scripted_task"]["finger_normal_force_n"] = [1., 0.]
    with pytest.raises(AssertionError):
        validate_floating_recording(steps, list(FloatingCaptureTask.stages), "floating_bolt_capture")


@pytest.mark.parametrize("contacts,forces", [([1], [1.]), ([1, 1], [1.]),
    ([1, -1], [1., 1.]), ([1, .5], [1., 1.]), ([1, float("nan")], [1., 1.])])
def test_recording_requires_two_valid_finger_channels(contacts, forces):
    from collection.collect_scripted_tasks import validate_floating_recording
    from simulation.scripted_tasks.floating_capture import FloatingCaptureTask
    state = dict(task_version=FloatingCaptureTask.version, target_body="bolt", result="success",
        task_parameters={"hold_s": 3.}, held_seconds=3., stage="hold",
        feedback_source="authoritative_simulation_state", finger_target_contacts=contacts,
        finger_normal_force_n=forces, slip_m=0., angular_slip_rad=0., relative_speed_m_s=0.)
    steps = [dict(sim_time_ns=str(i*20_000_000), scripted_task=deepcopy(state)) for i in range(160)]
    with pytest.raises(AssertionError):
        validate_floating_recording(steps, list(FloatingCaptureTask.stages), "floating_bolt_capture")
