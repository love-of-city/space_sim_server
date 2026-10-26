"""Overview selection, coordinate conventions and immutable episode provenance."""
import copy
import json
import math
import random
from pathlib import Path

import numpy as np
import pytest
from pydantic import ValidationError

from bsk_render_adapter.mjcf_assets import parse_mjcf_scene_metadata
from simulation.scripted_tasks.drawer_initialization import randomized_scene
from simulation.scripted_tasks.randomization import prepare_episode
from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix
from space_arm_platform.collection_jobs import CollectionCreate
from space_arm_platform.models import EpisodeStart, EpisodeStop, SceneInstanceCreate
from space_arm_platform.overview_camera import (
    CAMERA_ID, LEGACY_SCHEMA, MIN_ARM_BASE_MOUNT_AXIS_DISTANCE_M, PRESETS, SCHEMA,
    V2_PRESETS, V2_SCHEMA, V3_PRESETS, V3_SCHEMA, camera_pose, sample_overview_camera,
    validate_episode_overview_camera, validate_overview_camera,
    validate_overview_capture,
)
from space_arm_platform.recorder import EpisodeRecorder
from space_arm_platform.scene_runtime import SceneRuntimeManager

ROOT = Path(__file__).resolve().parents[1]


def test_selection_is_reproducible_independent_and_covers_all_presets(tmp_path):
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    base = dict(template_id="sarm-task-box-20261012", seed=42)
    state = random.getstate()
    fixed = manager.create_instance(SceneInstanceCreate(**base))
    first = manager.create_instance(SceneInstanceCreate(**base, randomize_overview_camera=True))
    second = manager.create_instance(SceneInstanceCreate(**base, randomize_overview_camera=True))
    assert random.getstate() == state
    assert first["overview_camera"] == second["overview_camera"]
    assert validate_overview_camera(first) == first["overview_camera"]
    assert "overview_camera" not in fixed
    for key in ("randomization", "environment", "runtime"):
        assert first[key] == fixed[key]
    assert {sample_overview_camera(seed)["preset_id"] for seed in range(100)} == set(PRESETS)


@pytest.mark.parametrize("key", PRESETS)
@pytest.mark.parametrize("schema", [LEGACY_SCHEMA, V2_SCHEMA, V3_SCHEMA, SCHEMA])
def test_wire_axes_aim_and_table_coverage(key, schema):
    pose = camera_pose(key, schema=schema)
    rotation = quaternion_wxyz_to_matrix(pose["orientation_body_from_camera_wxyz"])
    position = np.array(pose["position_body_m"])
    np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
    if schema == SCHEMA:
        assert position[2] == .30  # Height above the deck, not distance to centre.
    if schema == LEGACY_SCHEMA and key == "P0":
        parsed = parse_mjcf_scene_metadata(ROOT / "model/SARM/platform/sarm_20261012.xml", "teleop")
        camera = next(c for c in parsed.cameras if c["camera_id"] == CAMERA_ID)
        for field in ("position_body_m", "orientation_body_from_camera_wxyz", "field_of_view_rad"):
            np.testing.assert_allclose(pose[field], camera[field], atol=1e-14)
    else:
        delta = np.array(pose["look_at_body_m"]) - position
        np.testing.assert_allclose(rotation[:, 0], delta / np.linalg.norm(delta), atol=1e-12)
        assert rotation[2, 2] > 0  # Upright image, no camera roll.
    # Check the full deck plus an 18 cm tall prism, not just its centre.
    for x in (.125, .575):
        for y in (.065, .615):
            for z in (0., .18):
                forward, left, up = rotation.T @ (np.array([x, y, z]) - position)
                assert forward > 0
                assert max(abs(left), abs(up)) / forward < .85 * math.tan(pose["field_of_view_rad"] / 2)


def test_mounts_and_their_footprints_are_on_the_actual_satellite_stl():
    # Intersect a vertical ray with the delivered physical mesh, not its AABB.
    # STL vertices are mm; the MJCF asset applies scale=0.001.
    path = ROOT / "model/SARM_mjcf_20261012/meshes/base_link_mujoco.stl"
    dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    with path.open("rb") as stream:
        stream.seek(80)
        count = int.from_bytes(stream.read(4), "little")
        triangles = np.fromfile(stream, dtype=dtype, count=count)["vertices"].astype(float) * .001
    normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    surface = triangles[(normals[:, 2] > 1e-12) & np.all(abs(triangles[:, :, 2]) < 1e-7, axis=1)]
    assert len(surface) > 0
    a = surface[:, 0, :2]
    v, w = surface[:, 1, :2]-a, surface[:, 2, :2]-a
    det = v[:, 0]*w[:, 1]-v[:, 1]*w[:, 0]

    def on_surface(point):
        d = np.asarray(point)-a
        u = (d[:, 0]*w[:, 1]-d[:, 1]*w[:, 0])/det
        t = (v[:, 0]*d[:, 1]-v[:, 1]*d[:, 0])/det
        return np.any((u >= -1e-8) & (t >= -1e-8) & (u+t <= 1+1e-8))

    for key in PRESETS:
        pose = camera_pose(key)
        mount = np.array(pose["mount_position_body_m"])
        np.testing.assert_allclose(mount, [*pose["position_body_m"][:2], 0.])
        assert pose["parent_id"] == "teleop/cubesat_bus"
        assert pose["height_above_mount_m"] == .30
        assert on_surface(mount[:2])
        for angle in np.linspace(0, 2*math.pi, 32, endpoint=False):
            offset = .015*np.array([math.cos(angle), math.sin(angle)])
            assert on_surface(mount[:2]+offset), (key, angle)
    # The previous P4 camera's projection was outside the real satellite.
    assert not on_surface(camera_pose("P4", schema=V2_SCHEMA)["position_body_m"][:2])


def test_new_preset_outside_mounting_surface_is_rejected(monkeypatch):
    monkeypatch.setitem(PRESETS, "P4", V2_PRESETS["P4"])
    with pytest.raises(ValueError, match="mounting surface"):
        camera_pose("P4")


def test_new_mounts_avoid_the_arm_base_and_reject_the_previous_p3(monkeypatch):
    for key in PRESETS:
        x, y, _ = camera_pose(key)["mount_position_body_m"]
        assert math.hypot(x, y) >= MIN_ARM_BASE_MOUNT_AXIS_DISTANCE_M
    assert math.hypot(*camera_pose("P3")["position_body_m"][:2]) > .45
    monkeypatch.setitem(PRESETS, "P3", V3_PRESETS["P3"])
    with pytest.raises(ValueError, match="arm base"):
        camera_pose("P3")


def test_saved_v3_p3_replays_its_real_recording_calibration(tmp_path):
    # Golden record from the 20 s UE P3 evaluation, before relocating P3.
    saved = dict(
        schema=V3_SCHEMA, enabled=True, stream="space-arm-overview-camera-presets-v3",
        selection_seed=2, distribution="uniform_discrete", camera_id=CAMERA_ID,
        preset_id="P3", label="−Y 内侧安装点 · 30 cm", parent_id="teleop/cubesat_bus",
        position_body_m=[-.10000000000000003, -.04999999999999999, .3],
        orientation_body_from_camera_wxyz=[.9197956508781218, -.06654281150275157,
                                           .1783830205501667, .34311443111941603],
        camera_axes="+X forward,+Y left,+Z up", deck_center_body_m=[.35, .34, 0.],
        position_relative_to_deck_center_m=[-.45, -.39, .3], look_at_body_m=[.35, .34, .06],
        field_of_view_rad=1.5707963267948966,
        mount_position_body_m=[-.10000000000000003, -.04999999999999999, 0.],
        height_above_mount_m=.3, mounting_footprint_radius_m=.015,
    )
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    scene = manager.create_instance(SceneInstanceCreate(
        template_id="sarm-task-box-20261012", seed=2, randomize_overview_camera=True))
    scene["overview_camera"] = saved
    assert validate_overview_camera(scene) == saved
    prepare_episode(scene)
    assert scene["overview_camera"] == saved
    from simulation.teleop_grasp_unreal import _load_scene_instance
    path = tmp_path / "v3.json"
    path.write_text(json.dumps(scene))
    assert _load_scene_instance(path)["overview_camera"] == saved
    new = randomized_scene(scene, 2, 0)["overview_camera"]
    assert new["schema"] == SCHEMA
    assert new["preset_id"] == "P3"
    np.testing.assert_allclose(new["position_body_m"], [-.18, .42, .30])


def test_saved_v2_replays_while_new_scenes_use_mounted_presets(tmp_path):
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    scene = manager.create_instance(SceneInstanceCreate(
        template_id="sarm-task-box-20261012", seed=1, randomize_overview_camera=True))
    scene["overview_camera"] = sample_overview_camera(1, schema=V2_SCHEMA)
    assert scene["overview_camera"]["preset_id"] == "P4"
    np.testing.assert_allclose(scene["overview_camera"]["position_body_m"], [.05, .86, .30])
    assert validate_overview_camera(scene) == scene["overview_camera"]
    prepare_episode(scene)
    assert scene["overview_camera"]["schema"] == V2_SCHEMA
    assert randomized_scene(scene, 1, 0)["overview_camera"]["schema"] == SCHEMA


def test_saved_v1_record_replays_without_switching_to_30cm(tmp_path):
    # Golden calibration from the previous real UE P4 recording (seed=9).
    legacy = dict(
        schema=LEGACY_SCHEMA, enabled=True, stream="space-arm-overview-camera-presets-v1",
        selection_seed=9, distribution="uniform_discrete", camera_id=CAMERA_ID,
        preset_id="P4", label="+Y 侧高俯视", parent_id="teleop/cubesat_bus",
        position_body_m=[-.030000000000000027, .54, .72],
        orientation_body_from_camera_wxyz=[.853358127986967, .11436846898923304,
                                           .4628596078613583, -.21085716044295874],
        camera_axes="+X forward,+Y left,+Z up", deck_center_body_m=[.35, .34, 0.],
        position_relative_to_deck_center_m=[-.38, .2, .72],
        look_at_body_m=[.35, .34, .06], field_of_view_rad=1.4066130234496284,
    )
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    scene = manager.create_instance(SceneInstanceCreate(
        template_id="sarm-task-box-20261012", seed=9, randomize_overview_camera=True))
    assert scene["overview_camera"]["schema"] == SCHEMA
    scene["overview_camera"] = legacy
    assert validate_overview_camera(scene) == legacy
    prepare_episode(scene)
    assert scene["overview_camera"] == legacy
    from simulation.teleop_grasp_unreal import _load_scene_instance
    path = tmp_path / "legacy.json"
    path.write_text(json.dumps(scene))
    assert _load_scene_instance(path)["overview_camera"] == legacy
    # An explicit new collection attempt upgrades to the current preset version.
    assert randomized_scene(scene, 9, 0)["overview_camera"] == sample_overview_camera(9)
    for invalid in (None, "space-arm-overview-camera/99", SCHEMA):
        altered = copy.deepcopy(scene)
        altered["overview_camera"]["schema"] = invalid
        with pytest.raises(ValueError):
            validate_overview_camera(altered)


def test_attempt_resamples_but_recording_preparation_and_reload_preserve_pose(tmp_path):
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    source = manager.create_instance(SceneInstanceCreate(
        template_id="sarm-task-box-20261012", seed=42, randomize_overview_camera=True))
    original = copy.deepcopy(source)
    first = randomized_scene(source, 11, 0)
    assert first["overview_camera"] == sample_overview_camera(11)
    assert first["collection_config"]["overview_camera_randomization"] == first["overview_camera"]
    assert source == original
    assert first == randomized_scene(source, 11, 0)
    disabled = copy.deepcopy(source)
    disabled["randomize_overview_camera"] = False
    baseline = randomized_scene(disabled, 11, 0)
    for key in ("randomization", "environment"):
        assert first[key] == baseline[key]
    assert first["collection_config"]["realized_randomization"] == baseline["collection_config"]["realized_randomization"]
    reloaded = json.loads(json.dumps(first))
    prepare_episode(reloaded)
    assert reloaded == first
    from simulation.teleop_grasp_unreal import _load_scene_instance
    path = tmp_path / "saved.json"
    path.write_text(json.dumps(first))
    assert _load_scene_instance(path)["overview_camera"] == first["overview_camera"]
    recorder = EpisodeRecorder(tmp_path / "episodes")
    episode = recorder.start(EpisodeStart(scene_instance=first))
    recorder.stop(EpisodeStop())
    directory = tmp_path / "episodes" / episode["episode_id"]
    assert validate_episode_overview_camera(directory, first["overview_camera"])["passed"]
    wrong = copy.deepcopy(first["overview_camera"])
    wrong["preset_id"] = "bad"
    with pytest.raises(ValueError, match="differs"):
        validate_episode_overview_camera(directory, wrong)


@pytest.mark.parametrize("value", [None, 1, "true", [], {}])
def test_randomization_switch_requires_a_boolean(value):
    with pytest.raises(ValidationError):
        SceneInstanceCreate(randomize_overview_camera=value)
    with pytest.raises(ValidationError):
        CollectionCreate(randomize_overview_camera=value)


@pytest.mark.parametrize("position_delta", [.1, 1e-8])
def test_invalid_or_unsupported_saved_poses_fail_closed(tmp_path, position_delta):
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    with pytest.raises(ValidationError):
        SceneInstanceCreate(template_id="sarm-task-box-20261012-demo", randomize_overview_camera=True)
    instance = manager.create_instance(SceneInstanceCreate(
        template_id="sarm-task-box-20261012", seed=12, randomize_overview_camera=True))
    for field, value in (("seed", 99), ("overview_camera", None), ("randomize_overview_camera", False)):
        changed = copy.deepcopy(instance)
        changed[field] = value
        with pytest.raises(ValueError):
            validate_overview_camera(changed)
    instance["overview_camera"]["position_body_m"][0] += position_delta
    with pytest.raises(ValueError):
        validate_overview_camera(instance)


def test_capture_validation_accounts_for_satellite_rotation_and_rejects_old_ue_pose():
    # Quarter-turn about inertial Z, with the active local axes coinciding
    # with inertial axes. The camera stays rigidly attached to the satellite.
    body_rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    camera = camera_pose("P4")
    expected_rotation = body_rotation @ quaternion_wxyz_to_matrix(camera["orientation_body_from_camera_wxyz"])
    metadata = dict(camera_id=CAMERA_ID,
                    intrinsics=dict(fov_x_rad=camera["field_of_view_rad"],
                                    fov_y_rad=camera["field_of_view_rad"]),
                    extrinsics=dict(c_NC=expected_rotation.ravel().tolist(),
                                    position_L_m=(body_rotation @ camera["position_body_m"]).tolist()),
                    floating_origin=dict(c_LN=np.eye(3).ravel().tolist()))
    observation = dict(attitude_control=dict(orientation_inertial_wxyz=[math.sqrt(.5), 0, 0, math.sqrt(.5)]))
    validate_overview_capture(metadata, observation, camera)
    old_pose = camera_pose("P0")
    with pytest.raises(AssertionError):
        validate_overview_capture(metadata, observation, old_pose)
    metadata["intrinsics"]["fov_y_rad"] = math.radians(80.5929896518)
    with pytest.raises(AssertionError, match="field of view"):
        validate_overview_capture(metadata, observation, camera)
