"""Historical continuous records replay unchanged; new attempts use fixed presets."""
import copy
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

from simulation.scripted_tasks.drawer_initialization import randomized_scene
from simulation.scripted_tasks.randomization import prepare_episode
from space_arm_platform.models import EpisodeStart, EpisodeStop, SceneInstanceCreate
from space_arm_platform.overview_camera import (
    CAMERA_ID, LEGACY_SCHEMA, SCHEMA, V2_SCHEMA, V3_SCHEMA, V4_SCHEMA,
    initialize_overview_camera, sample_overview_camera,
    validate_episode_overview_camera, validate_overview_camera,
)
from space_arm_platform.recorder import EpisodeRecorder
from space_arm_platform.scene_runtime import SceneRuntimeManager

ROOT = Path(__file__).resolve().parents[1]
V5_SCHEMA = "space-arm-overview-camera/5"

def saved_v5_camera():
    saved = dict(
            schema="space-arm-overview-camera/5", enabled=True,
            stream="space-arm-overview-camera-continuous-v5", selection_seed=91,
            distribution="uniform_continuous_rejection", camera_id=CAMERA_ID,
            sampling_attempts=2,
            sampling_bounds_body_xy_m=[[-.185, -.065], [-.095, .585]],
            mounting_surface_bounds_body_xy_m=[[-.2, -.08], [-.08, .6]],
            minimum_arm_base_mount_axis_distance_m=.18,
            workspace_view_bounds_body_m=[[.125, .065, 0.], [.575, .615, .18]],
            workspace_frame_margin_fraction=.15,
            label="连续随机安装点 · 30 cm", parent_id="teleop/cubesat_bus",
            position_body_m=[-.17131819071978002, .5773227009052124, .3],
            orientation_body_from_camera_wxyz=[.9581055244132237, .04177877679128892,
                                               .19261003178334435, -.2078213500945089],
            camera_axes="+X forward,+Y left,+Z up", deck_center_body_m=[.35, .34, 0.],
            position_relative_to_deck_center_m=[-.52131819071978, .2373227009052124, .3],
            look_at_body_m=[.35, .34, .06], field_of_view_rad=1.5707963267948966,
            mount_position_body_m=[-.17131819071978002, .5773227009052124, 0.],
            height_above_mount_m=.3, mounting_footprint_radius_m=.015,
        )
    return saved


def test_saved_v5_record_replays_its_frozen_calibration(tmp_path):
    # This complete record came from the old continuous sampler, not the
    # current implementation, so altered historical geometry cannot pass.
    saved = saved_v5_camera()
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    scene = manager.create_instance(SceneInstanceCreate(
        template_id="sarm-task-box-20261012", seed=91, randomize_overview_camera=True))
    scene["overview_camera"] = copy.deepcopy(saved)
    scene["overview_camera_sampling"] = "continuous"
    assert validate_overview_camera(scene) == saved
    prepare_episode(scene)
    assert scene["overview_camera"] == saved
    assert scene["overview_camera_sampling"] == "continuous"
    original = copy.deepcopy(scene)
    from simulation.teleop_grasp_unreal import _load_scene_instance
    path = tmp_path / "v5.json"
    path.write_text(json.dumps(scene), encoding="utf-8")
    assert _load_scene_instance(path)["overview_camera"] == saved
    recorder = EpisodeRecorder(tmp_path / "episodes")
    episode = recorder.start(EpisodeStart(scene_instance=scene))
    recorder.stop(EpisodeStop())
    assert validate_episode_overview_camera(
        tmp_path / "episodes" / episode["episode_id"], saved,
    ) == dict(passed=True, schema=V5_SCHEMA, position_body_m=saved["position_body_m"])
    next_scene = randomized_scene(scene, 91, 0)
    assert next_scene["overview_camera_sampling"] == "presets"
    assert next_scene["overview_camera"] == sample_overview_camera(91)
    assert scene == original


@pytest.mark.parametrize("schema", [LEGACY_SCHEMA, V2_SCHEMA, V3_SCHEMA, V4_SCHEMA, SCHEMA])
def test_saved_sampling_preference_does_not_enable_continuous_new_attempts(schema):
    record = sample_overview_camera(23, schema=schema)
    scene = dict(template_id="sarm-task-box-20261012", seed=23,
                 randomize_overview_camera=True, overview_camera_sampling="continuous",
                 overview_camera=record)
    original = copy.deepcopy(scene)
    assert validate_overview_camera(scene) == record
    assert scene == original
    initialize_overview_camera(scene)
    assert scene["overview_camera_sampling"] == "presets"
    assert scene["overview_camera"] == sample_overview_camera(23)


def test_new_sampling_rejects_continuous_schema():
    with pytest.raises(ValueError, match="unsupported overview camera preset version"):
        sample_overview_camera(91, schema=V5_SCHEMA)


@pytest.mark.parametrize("field,value", [
    ("sampling_bounds_body_xy_m", [[-.2, -.08], [-.08, .6]]),
    ("minimum_arm_base_mount_axis_distance_m", 0.),
    ("workspace_frame_margin_fraction", 0.),
    ("position_body_m", [0., 0., .30]),
    ("selection_seed", 92), ("sampling_attempts", 0),
])
def test_saved_continuous_constraints_and_pose_cannot_be_forged(field, value):
    camera = saved_v5_camera()
    camera[field] = value
    scene = dict(template_id="sarm-task-box-20261012", seed=91,
                 randomize_overview_camera=True, overview_camera=camera)
    with pytest.raises(ValueError, match="saved seed"):
        validate_overview_camera(scene)


@pytest.mark.parametrize("mode", [None, True, 1, "CONTINUOUS", {}, []])
def test_invalid_sampling_mode_fails_closed(mode):
    with pytest.raises(ValueError, match="overview_camera_sampling"):
        initialize_overview_camera(dict(overview_camera_sampling=mode))


def test_historical_continuous_pose_reaches_renderer_without_a_preset_id(monkeypatch):
    monkeypatch.setitem(sys.modules, "simulation.assembly", SimpleNamespace(
        AssemblyContext=object, Slot=SimpleNamespace(OUTPUT=0)))
    spec = importlib.util.spec_from_file_location("historical_camera_render_probe", ROOT / "simulation/components/render.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    camera = saved_v5_camera()
    actual = SimpleNamespace(camera_id=CAMERA_ID, display_name="Overview")
    lamps = []
    bridge = SimpleNamespace(_cameras=[actual], add_mj_scene=lambda *a, **kw: None,
                             add_lights=lambda lamp: lamps.append(lamp),
                             set_scene_settings=lambda *_: None)
    monkeypatch.setitem(sys.modules, "bsk_render_adapter", SimpleNamespace(
        BasiliskRenderBridge=lambda **kw: bridge, SceneSettings=SimpleNamespace,
        VisualElement=SimpleNamespace))
    monkeypatch.setitem(sys.modules, "simulation.teleop_grasp_unreal", SimpleNamespace(
        _register_celestial_bodies=lambda *a: None))
    ctx = SimpleNamespace(scene=object(), extra={"template_id": "sarm-task-box-20261012"},
                          add=lambda *a, **kw: None, keep_alive=lambda *a: None)
    module.RenderComponent(model_path="unused.xml", catalog={}, host="localhost", port=5558,
                           render_rate_hz=50, capture_rate_hz=50, capture_state_provider=None,
                           overview_camera=camera).install(ctx)
    for key in ("parent_id", "position_body_m", "orientation_body_from_camera_wxyz", "field_of_view_rad"):
        assert getattr(actual, key) == camera[key]
    assert actual.display_name == "Overview (Continuous)"
    assert len(lamps) == 1 and lamps[0].parent_id == CAMERA_ID
