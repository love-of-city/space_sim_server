"""Wrist camera selection survives scene reload and dataset provenance."""
import json
import math
from pathlib import Path

import pytest
from pydantic import ValidationError

from bsk_render_adapter.descriptors import CameraVisual
from space_arm_platform.models import SceneInstanceCreate, EpisodeStart, EpisodeStop
from space_arm_platform.recorder import EpisodeRecorder
from space_arm_platform.scene_runtime import SceneRuntimeManager


@pytest.mark.parametrize("mode", ["rgb", "fisheye", "monochrome"])
def test_mode_roundtrip_and_recording_provenance(tmp_path, mode):
    from simulation.teleop_grasp_unreal import _load_scene_instance

    manager = SceneRuntimeManager(None, project_root=tmp_path)
    baseline = manager.create_instance(SceneInstanceCreate(seed=7))
    instance = manager.create_instance(SceneInstanceCreate(seed=7, wrist_camera_mode=mode))
    assert instance["randomization"] == baseline["randomization"]
    assert instance["environment"] == baseline["environment"]
    assert _load_scene_instance(Path(instance["config_path"]))["wrist_camera_mode"] == mode
    recorder = EpisodeRecorder(tmp_path / "episodes")
    episode = recorder.start(EpisodeStart(scene_instance=instance))
    recorder.stop(EpisodeStop())
    metadata = json.loads((tmp_path / "episodes" / episode["episode_id"] / "metadata.json").read_text(encoding="utf-8"))
    assert metadata["scene_instance"]["wrist_camera_mode"] == mode


def test_legacy_scene_defaults_to_rgb(tmp_path):
    from simulation.teleop_grasp_unreal import _load_scene_instance

    instance = SceneRuntimeManager(None, project_root=tmp_path).create_instance(SceneInstanceCreate())
    instance.pop("wrist_camera_mode")
    path = tmp_path / "legacy.json"
    path.write_text(json.dumps(instance))
    assert _load_scene_instance(path)["wrist_camera_mode"] == "rgb"


@pytest.mark.parametrize("mode", ["unknown", "RGB", None, True, 1, {}])
def test_invalid_mode_rejected_in_api_and_saved_scene(tmp_path, mode):
    from simulation.teleop_grasp_unreal import _load_scene_instance

    with pytest.raises(ValidationError):
        SceneInstanceCreate(wrist_camera_mode=mode)
    instance = SceneRuntimeManager(None, project_root=tmp_path).create_instance(SceneInstanceCreate())
    instance["wrist_camera_mode"] = mode
    path = tmp_path / "invalid.json"
    path.write_text(json.dumps(instance))
    with pytest.raises(ValueError, match="wrist_camera_mode"):
        _load_scene_instance(path)


def test_adapter_validates_fisheye_aperture_and_preserves_rgb_compatibility():
    assert CameraVisual("wrist").to_payload()["camera_mode"] == "rgb"
    camera = CameraVisual("wrist", camera_mode="fisheye", field_of_view_rad=math.radians(160))
    assert camera.to_payload()["camera_mode"] == "fisheye"
    for fov in (0, -1, math.pi, math.inf, math.nan):
        camera.field_of_view_rad = fov
        with pytest.raises(ValueError, match="field_of_view_rad"):
            camera.to_payload()
