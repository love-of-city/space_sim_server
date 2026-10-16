"""Audit data isolation and timestamps from parallel production benchmarks."""

from __future__ import annotations
import argparse
import json
from pathlib import Path
import av
import numpy as np
import pyarrow.parquet as pq


def audit_episode(path):
    meta = json.loads((path / "metadata.json").read_text(encoding="utf-8"))
    platform = json.loads(
        (path / "lerobot/meta/platform.json").read_text(encoding="utf-8")
    )
    info = json.loads((path / "lerobot/meta/info.json").read_text(encoding="utf-8"))
    assert meta["dataset_status"] == "complete"
    assert meta["dataset_fps"] == 50
    assert info["features"]["observation.state"]["shape"] == [7]
    assert info["features"]["action"]["shape"] == [7]
    steps = [
        json.loads(x)["observation"]
        for x in (path / "steps.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    times = [int(s["sim_time_ns"]) for s in steps]
    assert len(times) > 1 and all(b - a == 20_000_000 for a, b in zip(times, times[1:]))
    sessions = {s["render_session_id"] for s in steps}
    assert len(sessions) == 1
    frames = {str(s["render_frame_id"]): s for s in steps}
    assert len(frames) == len(steps) == meta["dataset_frame_count"]
    rows = []
    for f in sorted((path / "lerobot/data").rglob("*.parquet")):
        rows.extend(pq.read_table(f).to_pylist())
    assert len(rows) == len(steps)
    for row, s in zip(rows, steps):
        np.testing.assert_allclose(
            row["observation.state"],
            s["joint_position_rad"][:6] + [s["gripper_opening_normalized"]],
            atol=1e-7,
        )
        np.testing.assert_allclose(
            row["action"],
            s["target_joint_position_rad"][:6]
            + [s["target_gripper_opening_normalized"]],
            atol=1e-7,
        )
        assert json.loads(row["observation.platform_json"]) == s
    force_meta = platform["high_frequency_wrist_wrench"]
    forces = [
        json.loads(x)
        for x in (path / "lerobot" / force_meta["path"])
        .read_text(encoding="utf-8")
        .splitlines()
    ]
    assert force_meta["rate_hz"] == 200
    assert len(forces) == force_meta["samples"] == 4 * (len(steps) - 1) + 1
    stamps = [int(f["sim_time_ns"]) for f in forces]
    assert stamps[-1] == times[-1] and stamps[0] == times[0]
    assert all(b - a == 5_000_000 for a, b in zip(stamps, stamps[1:]))
    assert all(f["valid"] for f in forces)
    expected = [
        f
        for s in steps
        for f in s["wrist_wrench_samples"]
        if int(f["interval_end_time_ns"]) >= times[0]
    ]
    assert len(forces) == len(expected)
    for actual, original in zip(forces, expected):
        assert all(actual[k] == v for k, v in original.items())
    captures = [
        json.loads(x)
        for x in (path / "captures.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    assert len(captures) == 2 * len(steps)
    pairs = set()
    for cap in captures:
        cm = cap["metadata"]
        obs = frames[str(cap["source_frame_id"])]
        assert cm["session_id"] == obs["render_session_id"]
        assert cap["sim_time_ns"] == obs["sim_time_ns"]
        assert cm["resolution"] == [1024, 1024]
        pair = (cm["camera_id"], obs["render_frame_id"])
        assert pair not in pairs
        pairs.add(pair)
    videos = list((path / "lerobot/videos").rglob("*.mp4"))
    assert len(videos) == 2
    for f in videos:
        with av.open(str(f)) as video:
            stream = video.streams.video[0]
            assert (stream.width, stream.height) == (
                1024,
                1024,
            ) and stream.average_rate == 50
            pts = [float(frame.time) for frame in video.decode(video=0)]
            assert len(pts) == len(steps)
            np.testing.assert_allclose(np.diff(pts), 0.02, atol=1e-6)
    return dict(
        episode=path.name,
        session=list(sessions)[0],
        frames=len(steps),
        images=len(captures),
        force_samples=len(forces),
        video_frames_verified=True,
        main_hz=50,
        wrench_hz=200,
        outcome=meta["outcome"],
        task_result=steps[-1]["scripted_task"]["result"],
        task_reason=steps[-1]["scripted_task"]["reason"],
        elapsed_sim_seconds=steps[-1]["scripted_task"]["elapsed_sim_seconds"],
        passed=True,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("benchmark", type=Path)
    args = parser.parse_args()
    report = json.loads((args.benchmark / "benchmark.json").read_text(encoding="utf-8"))
    assert report["complete"] and len(report["runs"]) == report["jobs"]
    results = [audit_episode(Path(r["episode_path"])) for r in report["runs"]]
    assert all(
        r["task_result"] == "success"
        or (
            r["task_reason"] == "total_simulation_timeout"
            and r["elapsed_sim_seconds"] >= report["max_sim_seconds"]
        )
        for r in results
    )
    assert len({r["episode"] for r in results}) == len(results)
    assert len({r["session"] for r in results}) == len(results)
    result = dict(passed=True, episodes=results, independent_sessions=True)
    (args.benchmark / "validation.json").write_text(
        json.dumps(result, indent=2), encoding="utf-8"
    )
    print(json.dumps(result))


if __name__ == "__main__":
    main()
