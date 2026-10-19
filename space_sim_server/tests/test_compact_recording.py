"""Compact recordings stay trainable and preserve geometry and sensor clocks."""
import json

import numpy as np
import pyarrow.parquet as pq
import pytest
from lerobot.datasets.lerobot_dataset import LeRobotDataset

from space_arm_platform.compact_storage import ParquetChunks, read_chunks, read_observations, read_wrench
from space_arm_platform.dataset_view import Store
from space_arm_platform.models import EpisodeStart, EpisodeStop, SimulationObservation
from space_arm_platform.recorder import EpisodeRecorder
from test_lerobot_recording import observation, packet
from test_multirate_wrench import sample


@pytest.mark.parametrize("raw", [False, True])
def test_compact_live_training_viewer_and_native_force_roundtrip(tmp_path, raw):
    recorder = EpisodeRecorder(tmp_path, storage_profile="compact", save_raw_camera_images=raw)
    metadata = recorder.start(EpisodeStart(fps=50, camera_ids=["overview", "wrist"]))
    root = tmp_path / metadata["episode_id"]
    original = []
    for i in range(1, 5):
        data = observation(i).model_dump()
        stamp = i * 20_000_000
        data.update(sim_time_ns=str(stamp), gripper_type="four_bar_tendon",
                    arm_joint_position_rad=data["joint_position_rad"][:6],
                    arm_joint_velocity_rad_s=data["joint_velocity_rad_s"][:6],
                    target_arm_joint_position_rad=data["target_joint_position_rad"][:6],
                    gripper_drive_position_rad=data["joint_position_rad"][6],
                    gripper_drive_velocity_rad_s=data["joint_velocity_rad_s"][6],
                    target_gripper_drive_position_rad=data["target_joint_position_rad"][6],
                    wrist_wrench=sample(stamp).model_dump(), wrist_wrench_rate_hz=200,
                    wrist_wrench_samples=[sample(t).model_dump() for t in range(stamp-15_000_000, stamp+1, 5_000_000)],
                    scripted_task={"stage": "approach" if i < 4 else "verify", "result": None if i < 4 else "success"})
        obs = SimulationObservation.model_validate(data)
        original.append(obs)
        recorder.record_observation(obs, None)
        for camera in ("overview", "wrist"):
            meta, products = packet(i, camera)
            meta.update(sim_time_ns=str(stamp), sampling_fps=50,
                        intrinsics={"fx_px": 20.0}, extrinsics={"position_L_m": [i * 0.1, 0., 1.]})
            recorder.record_capture(meta, products)
    assert recorder.wait_for_writes()
    result = recorder.stop(EpisodeStop(outcome="success"))
    assert result["dataset_status"] == "complete", result
    assert not (root / "steps.jsonl").exists()
    assert (root / "captures.jsonl").exists() == raw
    assert not (root / "lerobot/high_frequency/wrist_wrench.jsonl").exists()
    info = json.loads((root / "lerobot/meta/info.json").read_text())
    assert "observation.platform_json" not in info["features"]
    assert not any(key.startswith("observation.camera_metadata.") for key in info["features"])
    loaded = LeRobotDataset(f"local/{root.name}", root=root / "lerobot", video_backend="pyav")
    assert len(loaded) == 4
    for i in range(4):
        frame = loaded[i]
        assert frame["observation.images.wrist"].shape == (3, 32, 32)
        np.testing.assert_allclose(frame["observation.state"][:6], original[i].joint_position_rad[:6])
        np.testing.assert_allclose(frame["action"][:6], original[i].target_joint_position_rad[:6])
    observations = list(read_observations(root))
    assert observations[-1]["scripted_task"]["result"] == "success"
    cameras = list(read_chunks(root / "lerobot/auxiliary/cameras"))
    assert len(cameras) == 8
    assert cameras[-1]["extrinsics"]["position_L_m"] == [.4, 0., 1.]
    forces = list(read_wrench(root))
    assert len(forces) == 13
    assert [int(r["sim_time_ns"]) for r in forces] == list(range(20_000_000, 80_000_001, 5_000_000))
    for row in forces:
        assert row["force_n"] == sample(int(row["sim_time_ns"])).force_n
        assert int(row["sim_time_ns"]) - int(row["sample_time_ns"]) == 2_500_000
    store = Store(tmp_path)
    assert store.listing()["episodes"][0]["readable"]
    view = json.loads(store.load(root.name))
    assert view["sync_ok"] and view["count"] == 4
    force_view = next(g for g in view["groups"] if g["id"] == "force_200hz")
    assert len(force_view["times"]) == 13
    # Persisted source digest must detect a changed training value, not merely
    # compare two reconstructions of the same corrupted row.
    file = next((root / "lerobot/data").rglob("*.parquet"))
    table = pq.read_table(file)
    rows = table.to_pylist()
    rows[0]["action"][0] += 1
    import pyarrow as pa
    pq.write_table(pa.Table.from_pylist(rows, schema=table.schema), file)
    with pytest.raises(ValueError, match="checksum"):
        list(read_observations(root))


def test_optional_task_fields_survive_chunk_boundaries(tmp_path):
    writer = ParquetChunks(tmp_path, "context", batch_size=2)
    for i in range(5):
        writer.append({"frame": i, "task": {"result": "success" if i == 4 else None}, "empty": {}})
    writer.flush()
    rows = list(read_chunks(tmp_path / "context"))
    assert [r["frame"] for r in rows] == list(range(5))
    assert rows[-1]["task"]["result"] == "success"
    assert len(list((tmp_path / "context").glob("*.parquet"))) == 3


@pytest.mark.process_writer
def test_compact_profile_reaches_spawned_writer(tmp_path):
    recorder = EpisodeRecorder(tmp_path, storage_profile="compact")
    try:
        metadata = recorder.start(EpisodeStart(fps=10, camera_ids=[]))
        for i in range(1, 4):
            recorder.record_observation(observation(i), None)
        result = recorder.stop(EpisodeStop(outcome="success"))
        assert result["dataset_status"] == "complete", result
        root = tmp_path / metadata["episode_id"]
        assert len(list(read_observations(root))) == 3
        assert not (root / "steps.jsonl").exists()
    finally:
        recorder.close()
