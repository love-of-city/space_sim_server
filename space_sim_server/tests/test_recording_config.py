import json

import pytest
from fastapi.testclient import TestClient

from space_arm_platform.app import PlatformConfig, create_app
from space_arm_platform.recording_config import (
    H264_ENCODER_PRESETS, NVENC_ENCODER_PRESETS, VideoEncodingConfig, load_recording_config,
)


@pytest.mark.parametrize("enabled", [False, True])
def test_backend_applies_recording_file_to_new_episodes(tmp_path, enabled):
    project = tmp_path / "project"
    settings = project / "config" / "recording.json"
    settings.parent.mkdir(parents=True)
    (project / "frontend").mkdir()
    settings.write_text(json.dumps({"save_raw_camera_images": enabled}), encoding="utf-8-sig")
    data = tmp_path / "episodes"
    app = create_app(PlatformConfig(project_root=project, data_root=data, simulation_port=0, capture_port=0))
    with TestClient(app) as client:
        assert client.post("/api/auth/login", json={"username": "admin", "password": "ChangeMe123!"}).status_code == 200
        response = client.post("/api/episodes/start", json={"camera_ids": []})
        assert response.status_code == 200, response.text
        episode = response.json()
        assert episode["raw_camera_images_saved"] is enabled
        assert episode["storage_profile"] == "compact"
        assert (data / episode["episode_id"] / "cameras").exists() is enabled
        assert client.post("/api/episodes/stop", json={}).status_code == 200


def test_missing_recording_file_defaults_to_no_image_copies(tmp_path):
    assert load_recording_config(tmp_path / "missing.json").save_raw_camera_images is False
    assert load_recording_config(tmp_path / "missing.json").storage_profile == "compact"
    assert load_recording_config(tmp_path / "missing.json").save_dynamics_trace is False


@pytest.mark.parametrize("document", [
    '{"save_raw_camera_images": "false"}',
    '{"save_raw_camera_images": 1}',
    '{"save_raw_camera_images": null}',
    '{"save_raw_camera_image": true}',
    '{"storage_profile": "unknown"}',
    '{"save_dynamics_trace": "false"}',
    '{"scripted_native_geometry": "true"}',
    '{"capture_backpressure_poll_s": 0.0}',
    '{"capture_backpressure_poll_s": 1.0}',
    '{"capture_rate_hz": 15}',
    '{"capture_rate_hz": 30}',
    '{"max_frames": 0}',
    '{"camera_ids": ["a/b", "a_b"]}',
    '{"camera_ids": [""]}',
    '{"capture_products": ["depth"]}',
    '{"capture_products": []}',
    '{"video": {"codec": "unknown"}}',
    '{"video": {"encoder_threads": 0}}',
    '{"video": {"streaming_encoding": "false"}}',
    '{"video": {"encoder_queue_maxsize": 0}}',
    '{"video": {"streaming_encoding": true, "quality": 52}}',
    '{"video": {"streaming_encoding": true, "keyframe_interval": 0}}',
    '{"video": {"streaming_encoding": true, "codec": "h264_nvenc", "keyframe_interval": 1}}',
    '{"video": {"quality": 25}}',
    '{"video": {"typo": true}}',
    '{"video": {"streaming_encoding": true, "encoder_preset": "p1"}}',
    '{"video": {"streaming_encoding": true, "encoder_preset": "unknown"}}',
    '{"video": {"streaming_encoding": true, "codec": "h264_nvenc", "encoder_preset": "veryfast"}}',
    '{"video": {"streaming_encoding": true, "codec": "h264_nvenc", "encoder_preset": "p8"}}',
    '{"video": {"encoder_preset": "veryfast"}}',
    '{"video": {"streaming_encoding": true, "encoder_preset": 1}}',
    '{broken',
])
def test_bad_recording_configuration_fails_explicitly(tmp_path, document):
    path = tmp_path / "recording.json"
    path.write_text(document, encoding="utf-8")
    with pytest.raises(ValueError, match="Invalid recording configuration"):
        load_recording_config(path)


def test_streaming_performance_settings_roundtrip(tmp_path):
    path = tmp_path / "recording.json"
    path.write_text(json.dumps({
        "scripted_native_geometry": True, "capture_backpressure_poll_s": 0.002,
        "video": {"streaming_encoding": True, "codec": "h264_nvenc", "quality": 24,
                  "keyframe_interval": 2, "encoder_queue_maxsize": 8, "encoder_preset": "p2"},
    }), encoding="utf-8")
    config = load_recording_config(path)
    assert config.scripted_native_geometry and config.capture_backpressure_poll_s == .002
    assert config.video.quality == 24 and config.video.keyframe_interval == 2
    assert config.video.encoder_preset == "p2"


@pytest.mark.parametrize("codec,presets", [("h264", H264_ENCODER_PRESETS), ("h264_nvenc", NVENC_ENCODER_PRESETS)])
def test_codec_specific_presets_are_supported(codec, presets):
    assert VideoEncodingConfig(codec=codec).encoder_preset is None
    for preset in presets:
        config = VideoEncodingConfig(codec=codec, streaming_encoding=True, encoder_preset=preset)
        assert config.model_dump()["encoder_preset"] == preset


def test_file_defaults_reach_scene_and_episode_requests(tmp_path, monkeypatch):
    project = tmp_path / "project"
    (project / "config").mkdir(parents=True)
    (project / "frontend").mkdir()
    settings = {"capture_rate_hz": 10, "dataset_capture": False, "camera_ids": ["wrist"],
                "max_frames": 123, "video": {"encoder_threads": 1}}
    (project / "config" / "recording.json").write_text(json.dumps(settings), encoding="utf-8")
    app = create_app(PlatformConfig(project_root=project, data_root=tmp_path / "episodes",
                                   simulation_port=0, capture_port=0))
    scenes = app.state.scenes
    monkeypatch.setattr(scenes, "catalog", lambda: {"defaults": {}})
    monkeypatch.setattr(scenes, "create_instance", lambda payload, creator: payload.model_dump())
    monkeypatch.setattr(scenes, "start", lambda payload, creator: payload.model_dump())
    async def accept_recording(payload):
        return payload.model_dump()
    monkeypatch.setattr(app.state.capture_lifecycle, "start", accept_recording)
    with TestClient(app) as client:
        client.post("/api/auth/login", json={"username": "admin", "password": "ChangeMe123!"})
        effective = client.get("/api/recording/config").json()
        assert effective["capture_rate_hz"] == 10 and effective["video"]["encoder_threads"] == 1
        assert client.get("/api/scenes/catalog").json()["defaults"] == {
            "capture_rate_hz": 10, "dataset_capture": False}
        for path in ("/api/scenes/instances", "/api/scenes/start"):
            default = client.post(path, json={})
            assert default.status_code == 200, default.text
            assert default.json()["capture_rate_hz"] == 10 and default.json()["dataset_capture"] is False
            explicit = client.post(path, json={"capture_rate_hz": 5, "dataset_capture": True}).json()
            assert explicit["capture_rate_hz"] == 5 and explicit["dataset_capture"] is True
        episode = client.post("/api/episodes/start", json={}).json()
        assert episode["fps"] == 10 and episode["camera_ids"] == ["wrist"] and episode["max_frames"] == 123
        explicit = client.post("/api/episodes/start", json={"fps": 5, "camera_ids": [], "max_frames": 9}).json()
        assert explicit["fps"] == 5 and explicit["camera_ids"] == [] and explicit["max_frames"] == 9
        # Saved scene rate wins over new file defaults, but explicit mismatches fail.
        monkeypatch.setattr(scenes, "status", lambda: {"phase": "running", "instance": {
            "runtime": {"capture_rate_hz": 25, "dataset_capture": True}}})
        inherited = client.post("/api/episodes/start", json={})
        assert inherited.status_code == 200, inherited.text
        assert inherited.json()["fps"] == 25
        assert client.post("/api/episodes/start", json={"fps": 10}).status_code == 409


def test_legacy_launcher_defaults_without_file(tmp_path):
    (tmp_path / "frontend").mkdir()
    app = create_app(PlatformConfig(project_root=tmp_path, data_root=tmp_path / "episodes",
                                   simulation_port=0, capture_port=0, runtime_capture_rate=5,
                                   runtime_default_dataset_capture=False))
    with TestClient(app) as client:
        client.post("/api/auth/login", json={"username": "admin", "password": "ChangeMe123!"})
        settings = client.get("/api/recording/config").json()
        assert settings["capture_rate_hz"] == 5 and settings["dataset_capture"] is False
