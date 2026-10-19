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
from space_arm_platform.recording_config import VideoEncodingConfig


def observation(index=1, session="session-a"):
    return SimulationObservation(
        protocol="space-arm-control/1", type="observation", simulation_id="test",
        render_session_id=session, step_id=str(index), render_frame_id=str(index),
        sim_time_ns=str(index * 100_000_000), wall_time_ns="1", applied_action_sequence="0",
        joint_position_rad=[.01 * index] * 6 + [.02, .02],
        joint_velocity_rad_s=[0.] * 8, target_joint_position_rad=[.02 * index] * 6 + [.03, .03],
        end_effector_position_body_m=[1., 2., 3.],
    )


def packet(index=1, camera="wrist", session="session-a", size=(32, 32)):
    blob = io.BytesIO()
    Image.fromarray(np.full((size[1], size[0], 3), index * 20, dtype=np.uint8)).save(blob, format="PNG")
    products = {"rgb": blob.getvalue()}
    meta = dict(protocol="bsk-capture/1", camera_id=camera, session_id=session,
                capture_sequence=str(index), source_frame_id=str(index),
                sim_time_ns=str(index * 100_000_000), stream_kind="authoritative", state_kind="authoritative",
                dataset_format="lerobot-v3", sampling_fps=10, sample_index=str(index), resolution=list(size),
                products=[{"name": name, "file_name": name + ".png"}
                          for name in products])
    return meta, products


def make_recorder(tmp_path, cameras=("wrist",), products=("rgb",), video_encoding=None):
    recorder = EpisodeRecorder(tmp_path, drain_timeout_s=.15, video_encoding=video_encoding)
    meta = recorder.start(EpisodeStart(fps=10, camera_ids=list(cameras), capture_products=list(products)))
    return recorder, tmp_path / meta["episode_id"]


@pytest.mark.parametrize("platform,frames,shape,parallel", [
    ("win32", 135, (360, 640, 3), False),
    ("win32", 900, (360, 640, 3), True),
    ("win32", 135, (2160, 3840, 3), True),
    ("linux", 135, (360, 640, 3), True),
])
def test_finish_avoids_windows_spawn_only_for_small_videos(tmp_path, monkeypatch, platform, frames, shape, parallel):
    from types import SimpleNamespace
    from unittest.mock import Mock
    from space_arm_platform import lerobot_capture
    monkeypatch.setattr(lerobot_capture.sys, "platform", platform)
    keys = ("observation.sim_time_ns", "observation.source_frame_id", "observation.applied_action_sequence",
            "observation.wrist_wrench_valid", "observation.wrist_wrench_sample_time_ns",
            "observation.wrist_wrench_interval_end_time_ns")
    dataset = SimpleNamespace(episode_buffer={key: [np.array([7], dtype=np.int64)] for key in keys},
                              save_episode=Mock(), finalize=Mock(), stop_image_writer=Mock())
    writer = object.__new__(lerobot_capture.LiveLeRobotWriter)
    writer.dataset, writer.root, writer.frames = dataset, tmp_path, frames
    writer.wrench_stream = lerobot_capture.WrenchStreamWriter(tmp_path)
    writer.sidecars = None
    writer.camera_shapes = {"overview": shape, "wrist": shape}
    (tmp_path / "meta").mkdir()
    (tmp_path / "meta/platform.json").write_text('{}')
    writer.finish("aborted", "performance check")
    dataset.save_episode.assert_called_once_with(parallel_encoding=parallel)
    dataset.finalize.assert_called_once()
    dataset.stop_image_writer.assert_called_once()
    assert all(dataset.episode_buffer[key][0].shape == () for key in keys)
    assert json.loads((tmp_path / "meta/platform.json").read_text())["episode_result"] == {
        "outcome": "aborted", "note": "performance check"}


@pytest.mark.parametrize("streaming", [False, True])
def test_live_v3_roundtrip_rgb_only_and_two_cameras(tmp_path, streaming):
    video = VideoEncodingConfig(streaming_encoding=streaming, encoder_threads=1, image_writer_threads=1)
    recorder, root = make_recorder(tmp_path, ("overview", "wrist"), ("rgb",), video_encoding=video)
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
    assert not (root / "cameras").exists()
    captures = [json.loads(line) for line in (root / "captures.jsonl").read_text(encoding="utf-8").splitlines()]
    assert len(captures) == 6 and all(row["files"] == {} for row in captures)
    metadata = json.loads((root / "metadata.json").read_text(encoding="utf-8"))
    assert metadata["raw_camera_images_saved"] is False
    assert metadata["dataset_capture_products"] == ["rgb"]
    assert (root / "lerobot/meta/tasks.parquet").is_file()
    assert list((root / "lerobot/meta/episodes").rglob("*.parquet"))
    assert len(list((root / "lerobot/videos").rglob("*.mp4"))) == 2
    assert metadata["video_encoding"] == video.model_dump()


@pytest.mark.process_writer
def test_spawned_writer_receives_video_configuration(tmp_path):
    video = VideoEncodingConfig(streaming_encoding=True, encoder_threads=1, encoder_queue_maxsize=8)
    recorder = EpisodeRecorder(tmp_path, video_encoding=video)
    try:
        metadata = recorder.start(EpisodeStart(fps=10, camera_ids=["wrist"]))
        for index in range(1, 4):
            recorder.record_observation(observation(index), None)
            recorder.record_authoritative_capture(*packet(index))
        result = recorder.stop(EpisodeStop())
        assert result["dataset_status"] == "complete", result
        root = tmp_path / metadata["episode_id"] / "lerobot"
        platform = json.loads((root / "meta/platform.json").read_text(encoding="utf-8"))
        assert platform["video_encoding"] == video.model_dump()
        assert not list(root.rglob("*.png"))
        loaded = LeRobotDataset(f"local/{root.parent.name}", root=root, video_backend="pyav")
        assert len(loaded) == 3
        for index in range(3):
            row = loaded[index]
            assert row["observation.images.wrist"].shape == (3, 32, 32)
            np.testing.assert_allclose(row["timestamp"], index / 10, atol=1e-6)
    finally:
        recorder.close()


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
    def fail(**kwargs):
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
    assert request.fps == 50


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
    result = recorder.start(EpisodeStart(camera_ids=cameras, fps=30))
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


@pytest.mark.parametrize("size", [(640, 360), (1024, 1024)])
def test_recording_dimensions_follow_rgb_and_metadata_without_changing_state_schema(tmp_path, size):
    from space_arm_platform.lerobot_capture import LiveLeRobotWriter
    from space_arm_platform.models import EpisodeStart
    root = tmp_path / "lerobot"
    writer = LiveLeRobotWriter(root, EpisodeStart(fps=10, camera_ids=["overview", "wrist"]))
    for i in range(1, 3):
        writer.append(observation(i), {camera: packet(i, camera, size=size) for camera in ("overview", "wrist")})
    writer.finish()
    info = json.loads((root / "meta/info.json").read_text())
    assert info["features"]["observation.state"]["shape"] == [8]
    assert info["features"]["action"]["shape"] == [8]
    assert info["fps"] == 10
    loaded = LeRobotDataset("local/dimensions", root=root, video_backend="pyav")
    for camera in ("overview", "wrist"):
        assert info["features"][f"observation.images.{camera}"]["shape"] == [size[1], size[0], 3]
        assert tuple(loaded[0][f"observation.images.{camera}"].shape) == (3, size[1], size[0])
    np.testing.assert_allclose(loaded[0]["action"][:6], [.02] * 6)


def linkage_observation(index=1, actual=0.25, target=0.8):
    values = observation(index).model_dump()
    values.update(
        gripper_type="four_bar_tendon",
        joint_position_rad=[.01 * index] * 6 + [actual, -actual + .001],
        joint_velocity_rad_s=[0.] * 6 + [.1, -.1],
        target_joint_position_rad=[.02 * index] * 6 + [target, -target],
        arm_joint_position_rad=[.01 * index] * 6,
        arm_joint_velocity_rad_s=[0.] * 6,
        target_arm_joint_position_rad=[.02 * index] * 6,
        gripper_drive_position_rad=actual,
        gripper_drive_velocity_rad_s=.1,
        target_gripper_drive_position_rad=target,
    )
    return SimulationObservation.model_validate(values)


def test_linkage_7d_roundtrip_with_raw_diagnostics_and_square_images(tmp_path):
    recorder, root = make_recorder(tmp_path)
    actuals = [-.002, .25, 1.002]
    targets = [0., .8, 1.]
    for index, (actual, target) in enumerate(zip(actuals, targets), 1):
        recorder.record_observation(linkage_observation(index, actual, target), None)
        recorder.record_authoritative_capture(*packet(index, size=(1024, 1024)))
    assert recorder.wait_for_writes()
    result = recorder.stop(EpisodeStop())
    assert result["dataset_status"] == "complete", result
    info = json.loads((root / "lerobot/meta/info.json").read_text())
    semantics = json.loads((root / "lerobot/meta/platform.json").read_text(encoding="utf-8"))
    assert semantics["schema"] == "space-arm-lerobot/2"
    assert semantics["gripper_normalization"]["closed"] == 0.
    assert semantics["gripper_normalization"]["opened"] == 1.
    for feature in ("observation.state", "action"):
        assert info["features"][feature]["shape"] == [7]
        assert info["features"][feature]["names"][-1] == "gripper_opening_normalized"
    assert info["features"]["observation.joint_velocity"]["shape"] == [8]
    loaded = LeRobotDataset(f"local/{root.name}", root=root / "lerobot", video_backend="pyav")
    steps = [json.loads(line) for line in (root / "steps.jsonl").read_text(encoding="utf-8").splitlines()]
    for index, (actual, target) in enumerate(zip(actuals, targets)):
        row = loaded[index]
        expected = min(1., max(0., actual))
        np.testing.assert_allclose(row["observation.state"], [.01 * (index+1)] * 6 + [expected])
        np.testing.assert_allclose(row["action"], [.02 * (index+1)] * 6 + [target])
        np.testing.assert_allclose(row["observation.raw_joint_position"][-2:], [actual, -actual+.001])
        np.testing.assert_allclose(row["observation.raw_target_joint_position"][-2:], [target, -target])
        assert row["observation.images.wrist"].shape == (3, 1024, 1024)
        audit = steps[index]["observation"]
        assert audit["gripper_opening_normalized"] == expected
        assert audit["target_gripper_opening_normalized"] == target
        assert audit["joint_position_rad"][6] == actual
        platform = json.loads(row["observation.platform_json"])
        assert platform == audit


def test_normalized_opening_serialization_cannot_override_actual_state():
    obs = linkage_observation(actual=.35, target=.9)
    values = obs.model_dump()
    values["gripper_opening_normalized"] = 1.
    values["target_gripper_opening_normalized"] = 0.
    restored = SimulationObservation.model_validate(values)
    assert restored.model_dump_json().count('"gripper_opening_normalized":') == 1
    encoded = json.loads(restored.model_dump_json())
    assert encoded["gripper_opening_normalized"] == .35
    assert encoded["target_gripper_opening_normalized"] == .9
