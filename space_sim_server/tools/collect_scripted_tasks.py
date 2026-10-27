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
from simulation.scripted_tasks.threaded_tasks import THREADED_TASKS, threaded_task_class


from space_arm_platform.compact_storage import is_compact, read_chunks, read_observations, read_wrench, wrench_bytes
from space_arm_platform.unreal_runtime import default_unreal_root
from space_arm_platform.collection_runtime import PersistentRenderer, check_port_available
from space_arm_platform.collection_layouts import LayoutPool, attach_layout, validate_episode_layout
from space_arm_platform.overview_camera import validate_episode_overview_camera


def validate_threaded_recording(steps, stages, task_name):
    """Reject missing physical operations even when an episode claims success."""
    import numpy as np

    controller = threaded_task_class(task_name)
    final = steps[-1]["scripted_task"]
    assert final["task_version"] == controller.version
    assert controller.validate_recorded_stages(stages), stages
    assert final["result"] == "success"
    parameters = final["task_parameters"]
    assert not parameters["overload_stop_enabled"]
    assert final["finger_part_contacts"] in ((0, 0), [0, 0])
    # Both collection profiles must actually reach their released opening.
    # The SARM jaw range is 0..1 rad, so its normalized observation has the
    # same value. 0.40 rad is the swept clearance for these physical heads.
    release_opening = float(parameters["release_open_rad"])
    assert np.isfinite(release_opening) and .40 <= release_opening <= 1.
    assert steps[-1]["gripper_opening_normalized"] >= release_opening-.01
    rows = [obs["scripted_task"] for obs in steps]
    quality = final["reference_motion_quality"]
    assert quality["samples"] > 0
    assert np.isfinite(list(quality.values())).all()
    assert quality["max_speed_rad_s"] <= parameters["arm_speed_rad_s"] + 1e-5
    # These are actuator-reference limits, independent of contact-force peaks.
    # The old .23 rad/s one-tick restart exceeded 40 rad/s² at the task rate.
    assert quality["max_acceleration_rad_s2"] <= 1.
    assert quality["max_stage_velocity_jump_rad_s"] <= .00501
    assert quality["rotation_samples"] > 0
    assert quality["rotation_latch_samples"] >= .95*quality["rotation_samples"]
    assert quality["rotation_max_wrist_axis_error_rad"] < .05
    phases = ("unscrew", "screw_in") if task_name == "aviation_bolt_insert" else ("loosen", "tighten")
    for phase in phases:
        assert any(row["stage"] == phase and row["finger_part_contacts"] is not None
                   and all(row["finger_part_contacts"]) and row["latch_engaged"]
                   and all(row["finger_latch_contacts"]) for row in rows), phase
    # Every individual released-wrist regrasp must establish the latch again.
    for index, row in enumerate(rows):
        if row["stage"] in phases and (index == 0 or rows[index-1]["stage"] != row["stage"]):
            segment = []
            for following in rows[index:]:
                if following["stage"] != row["stage"]:
                    break
                segment.append(following)
            assert any(item["latch_engaged"] and all(item["finger_latch_contacts"])
                       for item in segment), row["stage"]

    hold_s = float(parameters["verify_hold_s"])
    end = int(steps[-1]["sim_time_ns"])
    stable_observations = [obs for obs in steps
                           if 0 <= (end - int(obs["sim_time_ns"])) * 1e-9 < max(0., hold_s - .04)]
    assert all(obs["gripper_opening_normalized"] >= release_opening-.01
               for obs in stable_observations)
    stable = [obs["scripted_task"] for obs in stable_observations]
    assert len(stable) >= max(1, int((hold_s - .06) * 50))
    assert all(row["stage"] == "verify" and row["finger_part_contacts"] in ((0, 0), [0, 0])
               for row in stable)
    if task_name == "aviation_bolt_insert":
        assert final["part_body"] == "bolt" and final["target_site"] == "bolt_assembly_hole"
        assert final["source_unscrewed"] and final["ever_lifted"]
        for row in stable:
            for phase in ("source", "target"):
                assert row[phase + "_rotation_rad"] >= (
                    row[phase + "_required_rotation_rad"] - parameters["rotation_tolerance_rad"])
            assert row["radial_error_m"] < parameters["radial_tolerance_m"]
            assert abs(row["seat_gap_m"]) < parameters["seat_tolerance_m"]
            assert row["axis_error_rad"] < parameters["axis_tolerance_rad"]
            assert np.linalg.norm(row["part_velocity_body_m_s"]) < .01
            assert np.linalg.norm(row["part_angular_velocity_body_rad_s"]) < .03
        removed = [row for row in rows if row["stage"] == "carry"]
        assert removed and removed[0]["source_displacement_m"] >= parameters["source_clearance_m"] - .0015
        assert removed[0]["source_tip_clearance_m"] >= .0001
        assert removed[0]["source_rotation_rad"] >= removed[0]["source_required_rotation_rad"] - .05
    else:
        assert final["part_body"] == "fastener"
        assert all(final[key] for key in ("loosen_verified", "folded_verified", "returned_verified", "tightened_verified"))
        assert final["released_stable_seconds"] >= hold_s
        seated_slide = final["tighten_seated_slide_m"]
        seated_spin = final["tighten_seated_spin_rad"]
        assert np.isfinite([seated_slide, seated_spin]).all()
        assert parameters["tight_slide_min_m"] <= seated_slide <= parameters["tight_slide_max_m"]
        assert final["tighten_peak_hook_force_n"] >= parameters["minimum_hook_force_n"]
        # A released latch can unload to zero reaction in orbit.  Require an
        # actual correctly aligned seating reaction during tightening instead.
        assert any(row["fastener_hook_contacts"] > 0
                   and row["fastener_hook_normal_force_n"] >= parameters["minimum_hook_force_n"]
                   and abs(row["hinge_rad"]) <= parameters["returned_hinge_tolerance_rad"]
                   and parameters["tight_slide_min_m"] <= row["fastener_slide_m"] <= parameters["tight_slide_max_m"]
                   and row["fastener_helix_error_m"] <= parameters["helix_tolerance_m"]
                   for row in rows if row["stage"] == "tighten")
        assert any(row["fastener_slide_m"] <= parameters["loose_slide_tolerance_m"]
                   and row["scalar_joints"]["fastener_hook_contacts"] == 0
                   for row in rows if row["stage"] in ("lower", "release_down"))
        assert any(abs(row["hinge_rad"] - parameters["folded_hinge_rad"]) <= parameters["hinge_tolerance_rad"]
                   and row["locking_rod_clip_contacts"] + row["fastener_clip_contacts"] > 0
                   for row in rows if row["stage"] in ("release_down", "clear_down", "reapproach"))
        for row in stable:
            assert abs(row["hinge_rad"]) <= parameters["returned_hinge_tolerance_rad"]
            assert parameters["tight_slide_min_m"] <= row["fastener_slide_m"] <= parameters["tight_slide_max_m"]
            assert row["fastener_helix_error_m"] <= parameters["helix_tolerance_m"]
            backoff = parameters["maximum_seat_backoff_m"]
            assert row["fastener_slide_m"] >= seated_slide-backoff
            # The helix check above also checks slide/spin consistency.  This
            # explicit spin gate prevents a changed turn count being accepted.
            lead = final["fastener_thread_lead_m"]
            assert np.isfinite(lead) and lead > 0
            assert row["fastener_spin_rad"] >= seated_spin-backoff*2*np.pi/lead
            assert np.linalg.norm(row["part_velocity_body_m_s"]) < .008
            assert np.linalg.norm(row["part_angular_velocity_body_rad_s"]) < .05
            assert abs(row["scalar_joints"]["locking_rod_hinge_velocity"]) < .05
            assert abs(row["scalar_joints"]["fastener_spin_velocity"]) < .05


def validate_capture_manifest(episode, meta, steps):
    """Audit frame pairing with or without the optional raw image copies."""
    from PIL import Image
    from tools.record_drawer_task import CAMERAS
    from space_arm_platform.overview_camera import validate_overview_camera, validate_overview_capture

    overview = validate_overview_camera(meta.get("scene_instance") or {})

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
        if overview is not None:
            validate_overview_capture(cm, obs, overview)
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
    if task_name in THREADED_TASKS:
        validate_threaded_recording(steps, stages, task_name)
    elif is_plug_task(task_name):
        from simulation.scripted_tasks.plug_insert import PlugInsertTask

        profile = plug_profile(task_name)
        assert stages == list(PlugInsertTask.recorded_stages(final.get("task_version"))), stages
        if final.get("task_version") in (PlugInsertTask.cylindrical_clearance_version, PlugInsertTask.latch_version, PlugInsertTask.shared_motion_version, PlugInsertTask.overlapped_release_version, PlugInsertTask.continuous_contact_version, PlugInsertTask.continuous_carry_version, PlugInsertTask.gentle_carry_version, PlugInsertTask.smooth_tracking_version, PlugInsertTask.coordinated_version, PlugInsertTask.legacy_latch_version):
            grasped = [obs["scripted_task"] for obs in steps
                       if obs["scripted_task"]["stage"] == "extract"]
            assert grasped and grasped[0]["grasp_heading_error_rad"] < 0.05
            assert all(grasped[0]["finger_plug_contacts"])
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
        "--task", choices=["drawer_cycle", *PLUG_PROFILES, *THREADED_TASKS], default=default_task
    )
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--layout-pool", type=Path,
                        help="Validated layout pool; select a reproducible layout per attempt")
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
    parser.add_argument("--reuse-renderer", action="store_true")
    parser.add_argument("--offline", action="store_true")
    parser.add_argument("--gpu-index", type=int)
    parser.add_argument("--warmup-seconds", type=float, default=5.0)
    parser.add_argument("--reuse-warmup-seconds", type=float, default=5.0)
    parser.add_argument("--count", type=int, default=5)
    parser.add_argument("--max-attempts", type=int, default=15)
    parser.add_argument("--seed", type=int, default=2026101501)
    parser.add_argument("--range-deg", type=float, default=10.0)
    parser.add_argument("--randomize-orbit-phase", action=argparse.BooleanOptionalAction,
                        default=None, help="Sample a new orbital phase per attempt; otherwise inherit scene setting")
    parser.add_argument("--randomize-overview-camera", action=argparse.BooleanOptionalAction,
                        default=None, help="Choose one of five body-fixed overview camera presets per attempt")
    parser.add_argument("--control-port", type=int, default=19766)
    parser.add_argument("--capture-port", type=int, default=19767)
    parser.add_argument("--render-port", type=int, default=16558)
    parser.add_argument("--signalling-port", type=int, default=19888)
    add_motion_options(parser)
    args = parser.parse_args()
    try:
        motion_args = motion_arguments(args.task, args.linear_speed, args.threaded_speed_profile)
    except ValueError as error:
        parser.error(str(error))
    if not 1 <= args.count <= args.max_attempts <= 1000 or args.seed < 0:
        parser.error("require 1 <= count <= max-attempts <= 1000 and non-negative seed")
    if any(not 0 <= value <= 60 for value in (args.warmup_seconds, args.reuse_warmup_seconds)):
        parser.error("warmup must be between 0 and 60 seconds")
    if args.gpu_index is not None and not 0 <= args.gpu_index <= 255:
        parser.error("gpu-index must be between 0 and 255")
    ports = (args.control_port, args.capture_port, args.render_port, args.signalling_port)
    if len(set(ports)) != 4 or any(not 1024 <= port <= 65535 for port in ports):
        parser.error("four distinct ports in 1024..65535 are required")
    for port in ports:
        check_port_available(port)
    from tools.record_drawer_task import recording_settings
    recording_settings(args.recording_config)
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    if args.cancel_file is None:
        args.cancel_file = args.output / "cancel.request"
    source = json.loads(args.scene.read_text(encoding="utf-8-sig"))
    if args.randomize_orbit_phase is not None:
        source["randomize_orbit_phase"] = args.randomize_orbit_phase
    if args.randomize_overview_camera is not None:
        source["randomize_overview_camera"] = args.randomize_overview_camera
    pool = LayoutPool(args.layout_pool) if args.layout_pool else None
    if source.get("collection_layout_randomization") and pool is None:
        raise ValueError("Layout randomization requires --layout-pool; refusing fixed-layout fallback")
    model = ROOT / "model/SARM/platform/sarm_20261012.xml"
    started = time.monotonic()
    report = dict(
        schema="scripted-batch/2",
        task=args.task,
        linear_speed_m_s=(args.linear_speed if args.linear_speed is not None else 0.1)
        if args.task == "drawer_cycle"
        else None,
        threaded_speed_profile=(args.threaded_speed_profile
                                if args.task in THREADED_TASKS else None),
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
    probe_model = None
    renderer = None
    child = None
    recorded = 0
    try:
        save()
        if pool is not None:
            (args.output / "layout-pool.snapshot.json").write_text(
                json.dumps(pool.document, indent=2), encoding="utf-8")
            report["layout_pool_sha256"] = pool.sha256
        for index in range(args.max_attempts):
            if args.cancel_file and args.cancel_file.exists():
                report["status"] = "cancelled"
                return 1
            if len(report["accepted_episodes"]) >= args.count:
                break
            seed = args.seed + index
            attempt_started = time.monotonic()
            scene = randomized_scene(source, seed, args.range_deg, args.task)
            if args.task in THREADED_TASKS:
                scene["collection_config"]["threaded_speed_profile"] = args.threaded_speed_profile
            catalog = None
            if pool is not None:
                model, catalog = attach_layout(scene, pool, seed)
            directory = args.output / f"attempt-{index + 1:03d}-seed-{seed}"
            directory.mkdir()
            preflight_model = model
            if args.task in THREADED_TASKS:
                from simulation.scripted_tasks.randomization import write_model_variant

                scene["scripted_task_name"] = args.task
                preflight_model = write_model_variant(model, directory / "preflight-model.xml", scene)
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
            if scene.get("randomize_overview_camera"):
                entry["overview_camera"] = scene["overview_camera"]
            if pool is not None:
                entry["layout_randomization"] = scene["collection_config"]["layout_randomization"]
                entry["model"], entry["catalog"] = str(model), str(catalog)
            save()
            # Native probes compile one concrete model. Never reuse the previous
            # layout's collision state when selecting a different model.
            if probe is None or probe_model != preflight_model:
                if probe is not None:
                    probe.close()
                probe = NativePoseProbe(preflight_model)
                probe_model = preflight_model
            try:
                entry["preflight"] = probe.validate(preflight_model, scene, args.task)
            except ValueError as exc:
                entry["preflight"] = dict(accepted=False, reason="planning_failed", detail=str(exc))
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
                "900" if args.task in THREADED_TASKS else "450",
                "--timeout-seconds",
                "5400" if args.task in THREADED_TASKS else "1800",
            ]
            command += motion_args
            if catalog is not None:
                command += ["--model", str(model), "--catalog", str(catalog)]
            command += ["--warmup-seconds", str(args.reuse_warmup_seconds
                        if renderer is not None and recorded else args.warmup_seconds)]
            if args.offline:
                command.append("--offline")
            if args.gpu_index is not None:
                command += ["--gpu-index", str(args.gpu_index)]
            if args.reuse_renderer:
                if renderer is None:
                    renderer = PersistentRenderer(args, args.output)
                    report["renderer_pid"] = renderer.process.pid
                renderer.check()
                command.append("--external-renderer")
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
                child = subprocess.Popen(
                    command,
                    cwd=ROOT,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
                )
                while child.poll() is None:
                    if renderer is not None:
                        renderer.check()
                    time.sleep(0.2)
            recorded += 1
            entry["returncode"] = child.returncode
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
            if child.returncode != 0:
                entry["status"] = "recording_or_task_failed"
                save()
                print(
                    json.dumps(
                        dict(stage="attempt_failed", attempt=index + 1, detail=entry)
                    ),
                    flush=True,
                )
                # Infrastructure failures must be investigated, not retried forever.
                if child.returncode not in (0, 1) or not entry.get("task_result"):
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
                if scene.get("randomize_overview_camera"):
                    validation["overview_camera"] = validate_episode_overview_camera(
                        episode, entry["overview_camera"])
                if pool is not None:
                    validation["layout_randomization"] = validate_episode_layout(
                        episode, entry["layout_randomization"])
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
        if child is not None and child.poll() is None:
            args.cancel_file.write_text("collector stopping", encoding="utf-8")
            try:
                child.wait(120)
            except subprocess.TimeoutExpired:
                import psutil
                try:
                    descendants = psutil.Process(child.pid).children(recursive=True)
                except psutil.NoSuchProcess:
                    descendants = []
                for process in reversed(descendants):
                    try:
                        process.kill()
                    except psutil.NoSuchProcess:
                        pass
                child.kill()
                child.wait(10)
        if renderer is not None:
            renderer.close()
        if probe is not None:
            probe.close()
        save()


if __name__ == "__main__":
    raise SystemExit(main())
