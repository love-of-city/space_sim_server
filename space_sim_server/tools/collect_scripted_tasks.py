"""Collect independent manipulation episodes using the live production recorder.

Failures and rejected initializations remain in the batch audit. Optional noise
and domain randomization are configured per episode; runtime state is never
teleported to make a task succeed.
"""

from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "backend")]
from simulation.scripted_tasks.motion_options import (
    add_motion_options,
    motion_arguments,
)
from simulation.scripted_tasks.drawer_initialization import (
    NativePoseProbe,
    randomized_scene,
)


from simulation.scripted_tasks.plug_profiles import (
    PLUG_PROFILES,
    is_plug_task,
    plug_profile,
)


from space_arm_platform.compact_storage import is_compact, read_chunks, read_observations, read_wrench, wrench_bytes
from space_arm_platform.unreal_runtime import default_unreal_root


def validate_capture_manifest(episode, meta, steps):
    """Audit frame pairing with or without the optional raw image copies."""
    from PIL import Image
    from tools.record_drawer_task import CAMERAS

    by_frame = {str(obs["render_frame_id"]): obs for obs in steps}
    if is_compact(episode) and not meta.get("raw_camera_images_saved", False):
        captures = [dict(metadata=cm, source_frame_id=cm["source_frame_id"],
                         sim_time_ns=cm["sim_time_ns"], files={})
                    for cm in read_chunks(episode / "lerobot/auxiliary/cameras")]
    else:
        captures = [json.loads(line) for line in
                    (episode / "captures.jsonl").read_text(encoding="utf-8").splitlines()]
    expected = {(camera, frame) for camera in CAMERAS for frame in by_frame}
    assert len(captures) == len(expected) == len(CAMERAS) * len(steps)
    pairs = set()
    raw_images = meta.get("raw_camera_images_saved", True)  # Historical episodes saved copies.
    for capture in captures:
        cm = capture["metadata"]
        frame = str(capture["source_frame_id"])
        obs = by_frame[frame]
        assert cm["resolution"] == [1024, 1024]
        assert cm["session_id"] == obs["render_session_id"]
        assert capture["sim_time_ns"] == obs["sim_time_ns"]
        pair = (cm["camera_id"], frame)
        assert pair in expected and pair not in pairs
        pairs.add(pair)
        if raw_images:
            with Image.open(episode / capture["files"]["rgb"]) as image:
                assert image.size == (1024, 1024)
        else:
            assert not capture["files"], "Raw image files contradict episode metadata"
    assert pairs == expected
    return len(captures)


def validate_recording(episode, task_name="drawer_cycle"):
    import av
    import numpy as np
    import pyarrow.parquet as pq
    meta = json.loads((episode / "metadata.json").read_text(encoding="utf-8"))
    info = json.loads((episode / "lerobot/meta/info.json").read_text(encoding="utf-8"))
    platform = json.loads(
        (episode / "lerobot/meta/platform.json").read_text(encoding="utf-8")
    )
    assert meta["outcome"] == "success" and meta["dataset_status"] == "complete"
    assert platform["schema"] == "space-arm-lerobot/2"
    for key in ("observation.state", "action"):
        assert info["features"][key]["shape"] == [7]
        assert info["features"][key]["names"][-1] == "gripper_opening_normalized"
    compact = is_compact(episode)
    steps = list(read_observations(episode))
    rows = []
    for path in sorted((episode / "lerobot/data").rglob("*.parquet")):
        rows.extend(pq.read_table(path).to_pylist())
    assert (
        len(rows) == len(steps) == info["total_frames"] == meta["dataset_frame_count"]
    )
    frames = [int(obs["render_frame_id"]) for obs in steps]
    assert np.all(np.diff(frames) == 1)
    from space_arm_platform.sampling import sample_tick, tick_time_ns

    first_tick = sample_tick(int(steps[0]["sim_time_ns"]), meta["dataset_fps"])
    assert first_tick is not None
    assert [int(obs["sim_time_ns"]) for obs in steps] == [
        tick_time_ns(first_tick + i, meta["dataset_fps"]) for i in range(len(steps))
    ]
    stages = []
    for row, obs in zip(rows, steps):
        stage = obs["scripted_task"]["stage"]
        if not stages or stages[-1] != stage:
            stages.append(stage)
        np.testing.assert_allclose(
            row["observation.state"],
            obs["joint_position_rad"][:6] + [obs["gripper_opening_normalized"]],
            atol=1e-7,
        )
        np.testing.assert_allclose(
            row["action"],
            obs["target_joint_position_rad"][:6]
            + [obs["target_gripper_opening_normalized"]],
            atol=1e-7,
        )
        np.testing.assert_allclose(
            row["observation.raw_joint_position"], obs["joint_position_rad"], atol=1e-7
        )
        np.testing.assert_allclose(
            row["observation.raw_target_joint_position"],
            obs["target_joint_position_rad"],
            atol=1e-7,
        )
        np.testing.assert_allclose(
            row["observation.wrist_wrench"],
            obs["wrist_wrench"]["force_n"] + obs["wrist_wrench"]["torque_nm"],
            atol=1e-5,
        )
        assert obs["wrist_wrench"]["valid"]
        assert int(obs["render_frame_id"]) == row["observation.source_frame_id"]
        if not compact:
            assert json.loads(row["observation.platform_json"]) == obs
    final = steps[-1]["scripted_task"]
    if is_plug_task(task_name):
        from simulation.scripted_tasks.plug_insert import PlugInsertTask

        profile = plug_profile(task_name)
        assert stages == list(PlugInsertTask.stages), stages
        assert final["result"] == "success" and final["ever_lifted"]
        assert (
            final["plug_body"] == profile.body
            and final["target_site"] == profile.target_site
        )
        assert final["radial_error_m"] < 0.0004 and final["axis_error_rad"] < 0.01
        assert abs(final["seat_gap_m"]) < 0.001
        assert final["insertion_depth_m"] > profile.tip_m - profile.flange_m - 0.002
        assert not profile.keyed or final["orientation_error_rad"] < 0.01
        assert np.linalg.norm(final["plug_velocity_body_m_s"]) < 0.002
        assert np.linalg.norm(final["plug_angular_velocity_body_rad_s"]) < 0.03
        assert steps[-1]["gripper_opening_normalized"] >= 0.62
    else:
        from simulation.scripted_tasks.drawer_cycle import DrawerCycleTask

        assert stages == list(DrawerCycleTask.recorded_stages(final.get("task_version"))), stages
        if final.get("task_version") in ("drawer-cycle/4", DrawerCycleTask.version):
            assert final["released_closed_seconds"] >= final["task_parameters"]["verify_hold_s"]
        assert final["result"] == "success" and final["drawer_closed"]
        assert final["open_verified"] and final["seated_before_release_verified"]
        assert final["peak_drawer_displacement_m"] >= 0.075
        assert abs(final["drawer_displacement_m"]) <= 0.002
        assert steps[-1]["gripper_opening_normalized"] >= 0.95
    high = platform.get("high_frequency_wrist_wrench")
    if meta["dataset_fps"] == 50:
        assert high and high["rate_hz"] == 200
        force_rows = list(read_wrench(episode))
        start = int(steps[0]["sim_time_ns"])
        assert len(force_rows) == high["samples"]
        stamps = [int(row["sim_time_ns"]) for row in force_rows]
        assert stamps[0] == start
        assert all(b-a == 5_000_000 for a,b in zip(stamps,stamps[1:]))
        assert stamps[-1] == int(steps[-1]["sim_time_ns"])
        if compact:
            digest = hashlib.sha256()
            for sample in force_rows:
                digest.update(wrench_bytes(sample))
            assert digest.hexdigest() == high["values_sha256"], "High-rate wrench checksum mismatch"
            assert len(force_rows) == 4 * (len(steps) - 1) + 1
            by_time = {int(row["sim_time_ns"]): row for row in force_rows}
            for obs in steps:
                sample = by_time[int(obs["sim_time_ns"])]
                assert sample["valid"] == obs["wrist_wrench"]["valid"]
                for field in ("force_n", "torque_nm"):
                    np.testing.assert_allclose(sample[field], obs["wrist_wrench"][field], rtol=1e-6, atol=1e-5)
                for field in ("sample_time_ns", "interval_end_time_ns"):
                    assert int(sample[field]) == int(obs["wrist_wrench"][field])
        else:
            expected = [sample for obs in steps for sample in obs["wrist_wrench_samples"]
                        if int(sample["interval_end_time_ns"]) >= start]
            assert len(force_rows) == len(expected)
            for actual, original in zip(force_rows, expected):
                assert all(actual[key] == value for key, value in original.items())
    capture_count = validate_capture_manifest(episode, meta, steps)
    videos = list((episode / "lerobot/videos").rglob("*.mp4"))
    assert len(videos) == 2
    for path in videos:
        with av.open(str(path)) as video:
            stream = video.streams.video[0]
            assert (stream.width, stream.height) == (1024, 1024)
            assert stream.average_rate == meta["dataset_fps"]
            times = [float(frame.time) for frame in video.decode(video=0)]
            assert len(times) == len(steps)
            np.testing.assert_allclose(
                np.diff(times), 1 / meta["dataset_fps"], atol=1e-6
            )
    return dict(
        passed=True,
        episode_id=episode.name,
        frames=len(steps),
        images=capture_count,
        raw_camera_images_saved=meta.get("raw_camera_images_saved", True),
        image_resolution=[1024, 1024],
        fps=meta["dataset_fps"],
        state_action_dimensions=7,
        stages=stages,
        final_task_state=final,
        all_source_rows_matched=True,
        source_validation="numeric_sha256_and_camera_pairing" if compact else "full_observation_comparison",
        videos_fully_decoded=True,
    )


def main(default_task="drawer_cycle"):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--task", choices=["drawer_cycle", *PLUG_PROFILES], default=default_task
    )
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--cancel-file", type=Path)
    parser.add_argument(
        "--adapter", type=Path, default=ROOT.parent / "space_sim_UE_Adapter"
    )
    parser.add_argument(
        "--unreal", type=Path, default=default_unreal_root()
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--episodes-root", type=Path, default=ROOT / "data/episodes")
    parser.add_argument("--recording-config", type=Path)
    parser.add_argument("--count", type=int, default=5)
    parser.add_argument("--max-attempts", type=int, default=15)
    parser.add_argument("--seed", type=int, default=2026101501)
    parser.add_argument("--range-deg", type=float, default=10.0)
    parser.add_argument("--control-port", type=int, default=19766)
    parser.add_argument("--capture-port", type=int, default=19767)
    parser.add_argument("--render-port", type=int, default=16558)
    parser.add_argument("--signalling-port", type=int, default=19888)
    add_motion_options(parser)
    args = parser.parse_args()
    try:
        motion_args = motion_arguments(args.task, args.linear_speed)
    except ValueError as error:
        parser.error(str(error))
    if not 1 <= args.count <= args.max_attempts <= 1000 or args.seed < 0:
        parser.error("require 1 <= count <= max-attempts <= 1000 and non-negative seed")
    from tools.record_drawer_task import recording_settings
    recording_settings(args.recording_config)
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    source = json.loads(args.scene.read_text(encoding="utf-8-sig"))
    model = ROOT / "model/SARM/platform/sarm_20261012.xml"
    started = time.monotonic()
    report = dict(
        schema="scripted-batch/2",
        task=args.task,
        linear_speed_m_s=(args.linear_speed if args.linear_speed is not None else 0.1)
        if args.task == "drawer_cycle"
        else None,
        started_at_ns=str(time.time_ns()),
        status="running",
        requested_successes=args.count,
        base_seed=args.seed,
        attempts=[],
        accepted_episodes=[],
        source_scene=str(args.scene.resolve()),
    )

    def save():
        report["wall_seconds"] = time.monotonic() - started
        temporary = args.output / "summary.tmp"
        temporary.write_text(
            json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
        )

        temporary.replace(args.output / "summary.json")

    probe = None
    try:
        save()
        probe = NativePoseProbe(model)
        for index in range(args.max_attempts):
            if args.cancel_file and args.cancel_file.exists():
                report["status"] = "cancelled"
                return 1
            if len(report["accepted_episodes"]) >= args.count:
                break
            seed = args.seed + index
            attempt_started = time.monotonic()
            scene = randomized_scene(source, seed, args.range_deg, args.task)
            directory = args.output / f"attempt-{index + 1:03d}-seed-{seed}"
            directory.mkdir()
            scene_path = directory / "scene.json"
            scene_path.write_text(
                json.dumps(scene, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            entry = dict(
                attempt=index + 1,
                seed=seed,
                initial_state=scene["collection_config"]["initialization"],
                directory=str(directory),
                status="preflight",
            )
            report["attempts"].append(entry)
            save()
            entry["preflight"] = probe.validate(model, scene, args.task)
            if not entry["preflight"]["accepted"]:
                entry["status"] = "initialization_rejected"
                entry["end_to_end_wall_seconds"] = time.monotonic() - attempt_started
                save()
                print(json.dumps(entry), flush=True)
                continue
            entry["status"] = "recording"
            save()
            command = [
                sys.executable,
                "-X",
                "utf8",
                str(ROOT / "tools/record_drawer_task.py"),
                "--task",
                args.task,
                "--scene",
                str(scene_path),
                "--output",
                str(directory / "recording"),
                "--episodes-root",
                str(args.episodes_root.resolve()),
                "--substeps",
                "1",
                "--max-sim-seconds",
                "450",
                "--timeout-seconds",
                "1800",
            ]
            command += motion_args
            command += ["--adapter", str(args.adapter), "--unreal", str(args.unreal)]
            if args.recording_config:
                command += ["--recording-config", str(args.recording_config.resolve())]
            if args.cancel_file:
                command += ["--cancel-file", str(args.cancel_file)]
            for name in (
                "control_port",
                "capture_port",
                "render_port",
                "signalling_port",
            ):
                command += ["--" + name.replace("_", "-"), str(getattr(args, name))]
            print(
                json.dumps(
                    dict(stage="attempt_start", seed=seed, directory=str(directory))
                ),
                flush=True,
            )
            with (directory / "recording.log").open("w", encoding="utf-8") as log:
                result = subprocess.run(
                    command,
                    cwd=ROOT,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
                )
            entry["returncode"] = result.returncode
            entry["end_to_end_wall_seconds"] = time.monotonic() - attempt_started
            recording_summary = directory / "recording/summary.json"
            if recording_summary.is_file():
                result_info = json.loads(recording_summary.read_text(encoding="utf-8"))
                entry["episode_id"] = result_info.get("episode_id")
                entry["task_result"] = result_info.get("task")
                entry["recording_wall_seconds"] = result_info.get(
                    "recording_wall_seconds"
                )
                entry["error"] = result_info.get("error")
                entry["recording_process_wall_seconds"] = result_info.get(
                    "end_to_end_wall_seconds"
                )
            if args.cancel_file and args.cancel_file.exists():
                entry["status"] = "cancelled"
                report["status"] = "cancelled"
                return 1
            if result.returncode != 0:
                entry["status"] = "recording_or_task_failed"
                save()
                print(
                    json.dumps(
                        dict(stage="attempt_failed", attempt=index + 1, detail=entry)
                    ),
                    flush=True,
                )
                # Infrastructure failures must be investigated, not retried forever.
                if result.returncode not in (0, 1) or not entry.get("task_result"):
                    raise RuntimeError(
                        f"Recording infrastructure failed; see {directory}"
                    )
                continue
            episode = args.episodes_root.resolve() / entry["episode_id"]
            entry["status"] = "validating"
            save()
            try:
                validation_started = time.monotonic()
                validation = validate_recording(episode, args.task)
                entry["validation_wall_seconds"] = time.monotonic() - validation_started
            except Exception as exc:
                entry["status"] = "validation_failed"
                entry["error"] = f"{type(exc).__name__}: {exc}"
                save()
                raise
            (directory / "validation.json").write_text(
                json.dumps(validation, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            entry["status"] = "accepted"
            entry["end_to_end_wall_seconds"] = time.monotonic() - attempt_started
            entry["finished_at_ns"] = str(time.time_ns())
            report["accepted_episodes"].append(str(episode))
            save()
            print(
                json.dumps(
                    dict(
                        stage="accepted",
                        successes=len(report["accepted_episodes"]),
                        episode=str(episode),
                    )
                ),
                flush=True,
            )
        report["status"] = (
            "complete"
            if len(report["accepted_episodes"]) == args.count
            else "attempt_limit"
        )
        return 0 if report["status"] == "complete" else 1
    except BaseException as exc:
        report["status"] = (
            "interrupted" if isinstance(exc, KeyboardInterrupt) else "failed"
        )
        report["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        if probe is not None:
            probe.close()
        save()


if __name__ == "__main__":
    raise SystemExit(main())
