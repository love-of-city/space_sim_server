"""Exercise live callbacks -> official v3 writer -> official v3 reader, offline."""
import io
import json
import threading
import time

import numpy as np
import pytest
from PIL import Image
from lerobot.datasets.lerobot_dataset import LeRobotDataset

from space_arm_platform.models import EpisodeStart, EpisodeStop, SimulationObservation
from space_arm_platform.recorder import EpisodeRecorder


def observation(index=1, session="session-a"):
    return SimulationObservation(
        protocol="space-arm-control/1", type="observation", simulation_id="test",
        render_session_id=session, step_id=str(index), render_frame_id=str(index),
        sim_time_ns=str(index * 100_000_000), wall_time_ns="1", applied_action_sequence="0",
        joint_position_rad=[.01 * index] * 6 + [.02, .02],
        joint_velocity_rad_s=[0.] * 8, target_joint_position_rad=[.02 * index] * 6 + [.03, .03],
        end_effector_position_body_m=[1., 2., 3.],
    )


def packet(index=1, camera="wrist", session="session-a"):
    blob = io.BytesIO()
    Image.fromarray(np.full((32, 32, 3), index * 20, dtype=np.uint8)).save(blob, format="PNG")
    products = {"rgb": blob.getvalue()}
    meta = dict(protocol="bsk-capture/1", camera_id=camera, session_id=session,
                capture_sequence=str(index), source_frame_id=str(index),
                sim_time_ns=str(index * 100_000_000), stream_kind="authoritative", state_kind="authoritative",
                dataset_format="lerobot-v3", sampling_fps=10, sample_index=str(index), resolution=[32, 32],
                products=[{"name": name, "file_name": name + ".png"}
                          for name in products])
    return meta, products


def make_recorder(tmp_path, cameras=("wrist",), products=("rgb",)):
    recorder = EpisodeRecorder(tmp_path, drain_timeout_s=.15)
    meta = recorder.start(EpisodeStart(fps=10, camera_ids=list(cameras), capture_products=list(products)))
    return recorder, tmp_path / meta["episode_id"]


def test_live_v3_roundtrip_rgb_only_and_two_cameras(tmp_path):
    recorder, root = make_recorder(tmp_path, ("overview", "wrist"), ("rgb",))
    for index in range(1, 4):
        # Both network arrival orders must work, with real encoded products.
        recorder.record_authoritative_capture(*packet(index, "wrist"))
        recorder.record_observation(observation(index), None)
        recorder.record_authoritative_capture(*packet(index, "overview"))
    assert recorder.wait_for_writes()
    assert recorder.sync_status()["dataset_frame_count"] == 3
    result = recorder.stop(EpisodeStop(outcome="success"))
    assert result["dataset_status"] == "complete", result
    assert result["dataset_frame_count"] == 3
    info = json.loads((root / "lerobot/meta/info.json").read_text())
    assert info["codebase_version"] == "v3.0"
    assert info["fps"] == 10 and info["total_frames"] == 3
    loaded = LeRobotDataset(f"local/{root.name}", root=root / "lerobot", video_backend="pyav")
    assert len(loaded) == 3
    for index in range(3):
        row = loaded[index]
        assert row["observation.images.wrist"].shape == (3, 32, 32)
        np.testing.assert_allclose(row["observation.end_effector_pose_body"][:3], [1., 2., 3.])
        np.testing.assert_allclose(row["action"][:6], [.02 * (index + 1)] * 6)
        np.testing.assert_allclose(row["timestamp"], index / 10, atol=1e-6)
        assert row["observation.images.overview"].shape == (3, 32, 32)
    assert not any("segmentation" in key or "depth" in key for key in info["features"])
    assert not (root / "lerobot/auxiliary").exists()
    assert not list(root.rglob("*.pfm"))
    assert len(list((root / "cameras").rglob("*.png"))) == 6
    metadata = json.loads((root / "metadata.json").read_text(encoding="utf-8"))
    assert metadata["dataset_capture_products"] == ["rgb"]
    assert (root / "lerobot/meta/tasks.parquet").is_file()
    assert list((root / "lerobot/meta/episodes").rglob("*.parquet"))
    assert len(list((root / "lerobot/videos").rglob("*.mp4"))) == 2


def test_stop_drains_late_camera_without_accepting_new_observations(tmp_path):
    recorder, _ = make_recorder(tmp_path)
    recorder.record_observation(observation(), None)
    results = []
    worker = threading.Thread(target=lambda: results.append(recorder.stop(EpisodeStop())))
    worker.start()
    for _ in range(100):
        if recorder.sync_status()["finalizing"]:
            break
        time.sleep(.001)
    recorder.record_observation(observation(2), None)
    recorder.record_authoritative_capture(*packet())
    worker.join(timeout=20)
    assert not worker.is_alive()
    assert results[0]["dataset_status"] == "complete", results
    assert results[0]["step_count"] == 1


def test_missing_camera_is_failed_not_silently_dropped(tmp_path):
    recorder, _ = make_recorder(tmp_path, ("overview", "wrist"))
    recorder.record_observation(observation(), None)
    recorder.record_authoritative_capture(*packet())
    result = recorder.stop(EpisodeStop())
    assert result["status"] == "failed"
    assert result["incomplete_sample_count"] == 1
    assert "timed out" in result["dataset_error"]


@pytest.mark.parametrize("fault", ["legacy", "fps", "time", "corrupt", "missing_product"])
def test_invalid_camera_prevents_valid_dataset(tmp_path, fault):
    recorder, _ = make_recorder(tmp_path)
    recorder.record_observation(observation(), None)
    meta, data = packet()
    if fault == "legacy": meta.pop("dataset_format")
    if fault == "fps": meta["sampling_fps"] = 30
    if fault == "time": meta["sample_index"] = "9"
    if fault == "corrupt": data["rgb"] = b"not PNG"
    if fault == "missing_product": data.clear()
    recorder.record_authoritative_capture(meta, data)
    result = recorder.stop(EpisodeStop())
    assert result["status"] == "failed", result


def test_frame_gap_is_rejected_not_retimestamped(tmp_path):
    recorder, _ = make_recorder(tmp_path, ())
    recorder.record_observation(observation(1), None)
    recorder.record_observation(observation(3), None)
    result = recorder.stop(EpisodeStop())
    assert result["status"] == "failed"
    assert "missing/non-monotonic" in result["dataset_error"]


def test_sampling_uses_simulation_time_and_preserves_source_time(tmp_path):
    recorder, root = make_recorder(tmp_path, ())
    recorder.record_observation(observation().model_copy(update={"sim_time_ns": "33333333"}), None)
    assert recorder.sync_status()["dataset_frame_count"] == 0
    recorder.record_observation(observation(1), None)
    result = recorder.stop(EpisodeStop())
    assert result["status"] == "complete", result
    row = LeRobotDataset(f"local/{root.name}", root=root / "lerobot", video_backend="pyav")[0]
    assert row["observation.sim_time_ns"].item() == 100_000_000
    assert row["timestamp"].item() == 0


def test_empty_episode_is_not_advertised_as_valid(tmp_path):
    recorder, _ = make_recorder(tmp_path)
    result = recorder.stop(EpisodeStop())
    assert result["status"] == result["dataset_status"] == "empty"


def test_new_episode_starts_clean_after_failure(tmp_path):
    recorder, _ = make_recorder(tmp_path)
    recorder.record_observation(observation(), None)
    assert recorder.stop(EpisodeStop())["status"] == "failed"
    recorder.start(EpisodeStart(camera_ids=[]))
    recorder.record_observation(observation(), None)
    assert recorder.stop(EpisodeStop())["status"] == "complete"

def test_changed_render_session_is_failed_even_for_numeric_recording(tmp_path):
    recorder, _ = make_recorder(tmp_path, ())
    recorder.record_observation(observation(1), None)
    recorder.record_observation(observation(2, session="reset"), None)
    result = recorder.stop(EpisodeStop())
    assert result["status"] == "failed"
    assert "render session changed" in result["dataset_error"]


def test_duplicate_camera_before_bundle_complete_is_failed(tmp_path):
    recorder, _ = make_recorder(tmp_path, ("wrist", "overview"))
    recorder.record_observation(observation(), None)
    recorder.record_authoritative_capture(*packet())
    recorder.record_authoritative_capture(*packet())
    result = recorder.stop(EpisodeStop())
    assert result["status"] == "failed"
    assert "duplicate camera" in result["dataset_error"]


def test_encoder_failure_cannot_be_marked_complete(tmp_path, monkeypatch):
    recorder, _ = make_recorder(tmp_path, ())
    recorder.record_observation(observation(), None)
    def fail():
        raise OSError("disk full")
    assert recorder.wait_for_writes()
    monkeypatch.setattr(recorder._dataset.dataset, "save_episode", fail)
    result = recorder.stop(EpisodeStop(outcome="success"))
    assert result["outcome"] == "success"  # operator outcome is independent
    assert result["status"] == "failed" and "disk full" in result["dataset_error"]
    assert recorder.episode_id is None


def test_joint_layout_change_fails_recording(tmp_path):
    recorder, _ = make_recorder(tmp_path, ())
    recorder.record_observation(observation(), None)
    second = observation(2).model_copy(update={
        "joint_position_rad": [0.] * 6, "joint_velocity_rad_s": [0.] * 6,
        "target_joint_position_rad": [0.] * 6})
    recorder.record_observation(second, None)
    assert recorder.stop(EpisodeStop())["status"] == "failed"


def test_api_finishes_native_dataset_and_schedules_only_valid_archive(tmp_path):
    from fastapi.testclient import TestClient
    from space_arm_platform.app import PlatformConfig, create_app
    project_root = tmp_path / 'project'
    (project_root / 'frontend').mkdir(parents=True)
    app = create_app(PlatformConfig(project_root=project_root,
                                   data_root=tmp_path / "episodes", simulation_port=0, capture_port=0))
    with TestClient(app) as client:
        assert client.post("/api/auth/login", json={"username": "admin", "password": "ChangeMe123!"}).status_code == 200
        start = client.post("/api/episodes/start", json={"camera_ids": [], "instruction": "API recording"})
        assert start.status_code == 200
        app.state.recorder.record_observation(observation(), None)
        stop = client.post("/api/episodes/stop", json={"outcome": "success"})
        assert stop.status_code == 200
        assert stop.json()["dataset_status"] == "complete", stop.text
        assert "archive_job" in stop.json()
        root = tmp_path / "episodes" / start.json()["episode_id"] / "lerobot"
        assert len(LeRobotDataset("local/api", root=root, video_backend="pyav")) == 1
        client.post("/api/episodes/start", json={"camera_ids": []})
        empty = client.post("/api/episodes/stop", json={"outcome": "success"})
        assert empty.json()["dataset_status"] == "empty"
        assert "archive_job" not in empty.json()


def test_invalid_recording_configuration_rejected():
    from pydantic import ValidationError
    from space_arm_platform.models import SceneInstanceCreate
    for fields in [
        {"fps": 24}, {"camera_ids": ["a/b", "a_b"]},
        {"capture_products": ["depth"]}, {"capture_products": ["rgb", "depth"]},
        {"capture_products": ["segmentation"]}, {"capture_products": ["rgb", "segmentation"]},
    ]:
        with pytest.raises(ValidationError):
            EpisodeStart(**fields)
    with pytest.raises(ValidationError):
        SceneInstanceCreate(capture_rate_hz=24, dataset_capture=True)
    assert SceneInstanceCreate(capture_rate_hz=24, dataset_capture=False).capture_rate_hz == 24


def test_audit_action_matched_by_applied_sequence_not_latest(tmp_path):
    from space_arm_platform.models import AppliedAction
    recorder, root = make_recorder(tmp_path, ())
    def action(seq):
        return AppliedAction(server_sequence=str(seq), server_time_ns="1", client_sequence="1",
                             client_time_ns="1", deadman=True, input_source="keyboard",
                             end_effector_linear_velocity_body_m_s=[seq * .01, 0., 0.],
                             end_effector_angular_velocity_body_rad_s=[0.] * 3, gripper_velocity_rad_s=0.)
    recorder.record_action(action(1))
    recorder.record_action(action(2))
    obs = observation().model_copy(update={"applied_action_sequence": "1"})
    recorder.record_observation(obs, action(2))
    assert recorder.stop(EpisodeStop())["status"] == "complete"
    row = json.loads((root / "steps.jsonl").read_text(encoding="utf-8").splitlines()[0])
    assert row["applied_action"]["server_sequence"] == "1"


def test_default_recording_is_rgb_only():
    request = EpisodeStart()
    assert request.capture_products == ["rgb"]
    assert len(request.camera_ids) == 2
    assert request.fps == 30


@pytest.mark.parametrize("unsupported", ["depth", "segmentation"])
@pytest.mark.parametrize("metadata_only", [False, True])
def test_old_multimodal_peer_is_rejected_without_writing_assets(tmp_path, unsupported, metadata_only):
    recorder, root = make_recorder(tmp_path)
    recorder.record_observation(observation(), None)
    meta, products = packet()
    meta["products"].append({"name": unsupported, "file_name": unsupported + ".bin"})
    if not metadata_only:
        products[unsupported] = b"old peer payload"
    with pytest.raises(ValueError, match="RGB-only"):
        recorder.record_authoritative_capture(meta, products)
    result = recorder.stop(EpisodeStop())
    assert result["dataset_status"] == "failed"
    assert result["capture_count"] == 0
    assert not any(p.is_file() for p in (root / "cameras").rglob("*"))


def test_30fps_rgb_roundtrip_uses_absolute_physics_timestamps(tmp_path):
    from space_arm_platform.sampling import tick_time_ns
    recorder = EpisodeRecorder(tmp_path, drain_timeout_s=.15)
    cameras = ["overview", "wrist"]
    result = recorder.start(EpisodeStart(camera_ids=cameras))
    root = tmp_path / result["episode_id"] / "lerobot"
    # More than a full second exercises both alternating rounded intervals.
    # Start at one day of simulation time to detect epoch-dependent drift.
    base_tick = 86400 * 30
    count = 33
    for frame in range(count):
        tick = base_tick + frame
        stamp = str(tick_time_ns(tick * 8, 240))
        obs = observation(frame + 1).model_copy(update={"sim_time_ns": stamp})
        recorder.record_observation(obs, None)
        for camera in cameras:
            meta, products = packet(1, camera)
            meta.update(source_frame_id=str(frame + 1), sim_time_ns=stamp,
                        capture_sequence=str(frame + 1), sampling_fps=30, sample_index=str(tick))
            recorder.record_authoritative_capture(meta, products)
    result = recorder.stop(EpisodeStop(outcome="success"))
    assert result["dataset_status"] == "complete", result
    assert result["dataset_frame_count"] == count
    info = json.loads((root / "meta/info.json").read_text(encoding="utf-8"))
    assert info["fps"] == 30 and info["total_frames"] == count
    loaded = LeRobotDataset("local/rgb30", root=root, video_backend="pyav")
    for index in (0, 1, 2, 29, 30, 32):
        row = loaded[index]
        assert row["observation.sim_time_ns"].item() == tick_time_ns(base_tick + index, 30)
        np.testing.assert_allclose(row["timestamp"], index / 30, atol=1e-6)
        for camera in cameras:
            assert row[f"observation.images.{camera}"].shape == (3, 32, 32)
    assert not any("depth" in key or "segmentation" in key for key in info["features"])


def test_full_sync_buffer_waits_for_rgb_instead_of_losing_state(tmp_path):
    recorder = EpisodeRecorder(tmp_path, drain_timeout_s=2.)
    recorder.start(EpisodeStart(fps=10, camera_ids=["wrist"]))
    for index in range(1, 129):
        recorder.record_observation(observation(index), None)
    worker = threading.Thread(target=recorder.record_observation, args=(observation(129), None))
    worker.start()
    time.sleep(.05)
    assert worker.is_alive()
    assert recorder.sync_status()["pending_dataset_samples"] == 128
    assert recorder.sync_status()["dataset_error"] is None
    # This real camera packet initializes the writer and frees one bounded slot.
    recorder.record_authoritative_capture(*packet(1))
    worker.join(timeout=10)
    assert not worker.is_alive()
    assert recorder.wait_for_writes()
    sync = recorder.sync_status()
    assert sync["dataset_frame_count"] == 1
    assert sync["pending_dataset_samples"] == 128
    assert sync["dataset_error"] is None
    recorder.stop(EpisodeStop(outcome="aborted"))


def test_stop_wakes_observation_blocked_by_capture_backpressure(tmp_path):
    recorder = EpisodeRecorder(tmp_path, drain_timeout_s=.2)
    recorder.start(EpisodeStart(fps=10, camera_ids=["wrist"]))
    for index in range(1, 129):
        recorder.record_observation(observation(index), None)
    worker = threading.Thread(target=recorder.record_observation, args=(observation(129), None))
    worker.start()
    time.sleep(.03)
    assert worker.is_alive()
    result = recorder.stop(EpisodeStop(outcome="aborted"))
    worker.join(timeout=2)
    assert not worker.is_alive()
    assert result["step_count"] == 128  # blocked state did not cross the stop cutoff
    assert result["dataset_status"] == "failed"  # absent RGB is never fabricated


def test_missing_rgb_eventually_fails_with_actionable_backpressure_error(tmp_path):
    recorder = EpisodeRecorder(tmp_path, drain_timeout_s=.02)
    recorder.start(EpisodeStart(fps=10, camera_ids=["wrist"]))
    for index in range(1, 130):
        recorder.record_observation(observation(index), None)
    sync = recorder.sync_status()
    assert sync["pending_dataset_samples"] == 128
    assert "RGB capture backpressure" in sync["dataset_error"]
    assert recorder.stop(EpisodeStop())["dataset_status"] == "failed"


def test_stop_allows_bounded_backlog_to_drain_while_frames_make_progress(tmp_path):
    recorder = EpisodeRecorder(tmp_path, drain_timeout_s=.4)
    recorder.start(EpisodeStart(fps=10, camera_ids=["wrist"]))
    recorder.record_observation(observation(1), None)
    recorder.record_authoritative_capture(*packet(1))  # initialize the real writer first
    recorder.record_observation(observation(2), None)
    recorder.record_observation(observation(3), None)
    result = []
    worker = threading.Thread(target=lambda: result.append(recorder.stop(EpisodeStop())))
    worker.start()
    for _ in range(100):
        if recorder.sync_status()["finalizing"]:
            break
        time.sleep(.005)
    time.sleep(.25)
    recorder.record_authoritative_capture(*packet(2))
    time.sleep(.25)  # total drain exceeds .4s, but no progress gap does
    assert worker.is_alive()
    recorder.record_authoritative_capture(*packet(3))
    worker.join(timeout=10)
    assert not worker.is_alive()
    assert result[0]["dataset_status"] == "complete", result
    assert result[0]["dataset_frame_count"] == 3
