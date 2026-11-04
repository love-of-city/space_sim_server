"""Recording acceptance shared by batch collection and standalone tools.

Fast validation checks completion, timing and encoded stream counts. Full
validation additionally compares exported values, camera pairing and decoded
videos. Neither path loads a simulation controller or native dynamics library.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

from space_arm_platform.compact_storage import (
    is_compact, read_chunks, read_observations, read_wrench, wrench_bytes,
)
from space_arm_platform.task_catalog import DRAWER_TASKS, THREADED_TASKS
from simulation.scripted_tasks.plug_profiles import is_plug_task, plug_profile
from collection.recorded_task_contracts import (
    recorded_contract, validate_recorded_stages, validate_recorded_configuration,
)

CAMERAS = ("teleop/camera/spacecraft_overview", "teleop/camera/sarm_wrist_cam")


def validate_threaded_recording(steps, stages, task_name):
    """Reject missing physical operations even when an episode claims success."""
    import numpy as np

    final = steps[-1]["scripted_task"]
    contract = recorded_contract(task_name, final["task_version"])
    assert contract.validation_supported, final["task_version"]
    geometric_completion = contract.geometric_completion
    assert validate_recorded_stages(task_name, stages, final["task_version"]), stages
    assert final["result"] == "success"
    parameters = final["task_parameters"]
    assert not parameters["overload_stop_enabled"]
    assert final["finger_part_contacts"] in ((0, 0), [0, 0])
    # Current completion uses the swept 0.40 rad head clearance after retreat,
    # not the larger free-space opening target. Historical recordings retain
    # their configured-aperture requirement. The SARM jaw range is 0..1 rad,
    # so its normalized observation has the same value.
    release_opening = float(parameters["release_open_rad"])
    assert np.isfinite(release_opening) and .40 <= release_opening <= 1.
    minimum_opening = (contract.minimum_release_opening_rad if geometric_completion
                       else release_opening-.01)
    assert steps[-1]["gripper_opening_normalized"] >= minimum_opening, "released jaw clearance"
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
    phases = {
        "aviation_bolt_insert": ("unscrew", "screw_in"),
        "locking_rod_cycle": ("loosen", "tighten"),
        "locking_rod_fold": ("loosen",),
        "locking_rod_restore": ("tighten",),
    }[task_name]
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
    # Independent rod tasks finish opening during withdrawal/confirmation;
    # their last frame still requires the configured free-space opening.
    stable_opening = (.40 if task_name in ("locking_rod_fold", "locking_rod_restore")
                      else minimum_opening)
    assert all(obs["gripper_opening_normalized"] >= stable_opening
               for obs in stable_observations), "released jaw clearance during confirmation"
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
            if not geometric_completion:
                assert np.linalg.norm(row["part_velocity_body_m_s"]) < .01
                assert np.linalg.norm(row["part_angular_velocity_body_rad_s"]) < .03
        removed = [row for row in rows if row["stage"] == "carry"]
        assert removed and removed[0]["source_displacement_m"] >= parameters["source_clearance_m"] - .0015
        assert removed[0]["source_tip_clearance_m"] >= .0001
        assert removed[0]["source_rotation_rad"] >= removed[0]["source_required_rotation_rad"] - .05
    else:
        assert final["part_body"] == "fastener"
        assert final["released_stable_seconds"] >= hold_s
        if task_name == "locking_rod_fold":
            assert final["task_operation"] == "fold"
            assert final["loosen_verified"] and final["folded_verified"]
            assert not final["returned_verified"] and not final["tightened_verified"]
            assert abs(rows[0]["hinge_rad"]) <= parameters["returned_hinge_tolerance_rad"]
            assert any(row["stage"] == "lower" and abs(row["fastener_slide_m"]) <= parameters["loose_slide_tolerance_m"]
                       and row["fastener_hook_contacts"] == 0 for row in rows)
            for row in stable:
                assert abs(row["hinge_rad"]-parameters["folded_hinge_rad"]) <= parameters["hinge_tolerance_rad"]
                assert row["locking_rod_clip_contacts"] + row["fastener_clip_contacts"] > 0
                assert abs(row["fastener_slide_m"]) <= parameters["loose_slide_tolerance_m"]
                assert row["fastener_helix_error_m"] <= parameters["helix_tolerance_m"]
                assert row["fastener_hook_contacts"] == 0
                assert np.linalg.norm(row["part_velocity_body_m_s"]) < .008
                assert np.linalg.norm(row["part_angular_velocity_body_rad_s"]) < .05
                assert abs(row["scalar_joints"]["locking_rod_hinge_velocity"]) < .05
                assert abs(row["scalar_joints"]["fastener_spin_velocity"]) < .05
            return
        if task_name == "locking_rod_restore":
            assert final["task_operation"] == "restore"
            assert final["initial_folded_verified"]
            assert final["returned_verified"] and final["tightened_verified"]
            assert not final["loosen_verified"] and not final["folded_verified"]
            initial = rows[0]
            assert abs(initial["hinge_rad"]-parameters["folded_hinge_rad"]) <= parameters["hinge_tolerance_rad"]
            assert abs(initial["fastener_slide_m"]) <= parameters["loose_slide_tolerance_m"]
            assert any(row["stage"] in ("settle", "reapproach", "regrip")
                       and row["locking_rod_clip_contacts"] + row["fastener_clip_contacts"] > 0 for row in rows)
        else:
            assert all(final[key] for key in ("loosen_verified", "folded_verified", "returned_verified", "tightened_verified"))
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
        if task_name == "locking_rod_cycle":
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
            if not geometric_completion:
                assert np.linalg.norm(row["part_velocity_body_m_s"]) < .008
                assert np.linalg.norm(row["part_angular_velocity_body_rad_s"]) < .05
                assert abs(row["scalar_joints"]["locking_rod_hinge_velocity"]) < .05
                assert abs(row["scalar_joints"]["fastener_spin_velocity"]) < .05


def validate_drawer_recording(steps, stages, task_name):
    """Check the actual initial/final direction instead of requiring a round trip."""
    import numpy as np
    final = steps[-1]["scripted_task"]
    contract = recorded_contract(task_name, final.get("task_version"))
    assert stages == list(contract.stages), stages
    assert final["result"] == "success"
    assert steps[-1]["gripper_opening_normalized"] >= .95
    if task_name == "drawer_cycle":
        if contract.released_closed_confirmation:
            assert final["released_closed_seconds"] >= final["task_parameters"]["verify_hold_s"]
        assert final["drawer_closed"] and final["open_verified"]
        assert final["seated_before_release_verified"]
        assert final["peak_drawer_displacement_m"] >= .075
        assert abs(final["drawer_displacement_m"]) <= .002
        return
    assert contract.validation_supported
    parameters = final["task_parameters"]
    rows = [obs["scripted_task"] for obs in steps]
    assert all(row["name"] == task_name for row in rows)
    initial, target = final["drawer_initial_m"], final["drawer_target_m"]
    assert np.isfinite([initial, target]).all()
    assert abs(rows[0]["drawer_position_m"]-initial) <= .005
    phase = "lift" if task_name == "drawer_open" else "return"
    assert any(row["stage"] == phase and row["finger_drawer_contacts"] is not None
               and all(row["finger_drawer_contacts"]) for row in rows)
    if task_name == "drawer_open":
        assert abs(initial) <= .005 and abs(target-parameters["lift_m"]) <= .005
        assert final["drawer_open"] and final["open_verified"]
        assert final["released_open_seconds"] >= parameters["verify_hold_s"]
        assert final["drawer_position_m"]-initial >= parameters["lift_m"]-.005
        tolerance = .005
    else:
        assert abs(initial-parameters["lift_m"]) <= .005 and abs(target) <= .002
        assert final["drawer_closed"] and final["open_verified"]
        assert final["seated_before_release_verified"]
        assert final["released_closed_seconds"] >= parameters["verify_hold_s"]
        assert initial-final["drawer_position_m"] >= parameters["lift_m"]-.005
        tolerance = parameters["close_tolerance_m"]
    end = int(steps[-1]["sim_time_ns"])
    hold_s = parameters["verify_hold_s"]
    stable = [obs for obs in steps if 0 <= (end-int(obs["sim_time_ns"]))*1e-9 < hold_s-.04]
    assert len(stable) >= max(1, int((hold_s-.06)*50))
    for obs in stable:
        row = obs["scripted_task"]
        assert row["stage"] in ("release", "retract", "verify")
        assert abs(row["drawer_position_m"]-target) <= tolerance
        assert abs(row["drawer_velocity_m_s"]) < .003
        assert row["finger_drawer_contacts"] in ((0, 0), [0, 0])
        assert obs["gripper_opening_normalized"] >= parameters["release_clearance_opening"]
        if task_name == "drawer_close":
            assert row["drawer_closed"]


def validate_capture_manifest(episode, meta, steps):
    """Audit frame pairing with or without the optional raw image copies."""
    from PIL import Image
    from space_arm_platform.overview_camera_records import validate_overview_camera, validate_overview_capture

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


def validate_recorded_layout_model(meta, observations):
    """Bind every authoritative state to the saved layout and its exact variant."""
    scene = meta.get("scene_instance") or {}
    config = scene.get("collection_config") or {}
    layout = scene.get("task_box_layout") or {}
    selected = config.get("layout_randomization") or {}
    source = layout.get("model_sha256")
    if selected.get("enabled"):
        if not source:
            raise ValueError("Enabled layout selection has no layout model")
        if selected.get("model_sha256") != source:
            raise ValueError("Saved layout selection disagrees with layout model")
    if not source:
        return
    variant = config.get("model_randomization") or {}
    if variant:
        if variant.get("source_sha256") != source:
            raise ValueError("Randomized model source disagrees with saved layout")
        expected = variant.get("variant_sha256")
    else:
        expected = source
    if not expected or any(obs.get("model_sha256") != expected for obs in observations):
        raise ValueError("Authoritative model differs from the saved layout/variant")


def validate_plug_recording(steps, stages, task_name):
    import numpy as np

    final = steps[-1]["scripted_task"]

    profile = plug_profile(task_name)
    contract = recorded_contract(task_name, final.get("task_version"))
    assert stages == list(contract.stages), stages
    if contract.latch_grasp_required:
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
    # Historical recordings keep their original motion acceptance rules.
    if contract.released_velocity_required:
        assert np.linalg.norm(final["plug_velocity_body_m_s"]) < 0.002
        assert np.linalg.norm(final["plug_angular_velocity_body_rad_s"]) < 0.03
    assert steps[-1]["gripper_opening_normalized"] >= 0.62


def validate_task_recording(steps, stages, task_name):
    """Apply one task contract regardless of the storage audit level."""
    if task_name in THREADED_TASKS:
        validate_threaded_recording(steps, stages, task_name)
    elif is_plug_task(task_name):
        validate_plug_recording(steps, stages, task_name)
    elif task_name in DRAWER_TASKS:
        validate_drawer_recording(steps, stages, task_name)
    else:
        raise ValueError(f"Unsupported recording task: {task_name}")


def validate_recording_fast(episode, task_name):
    """Completion/count/timestamp gate; decoding and value comparisons run later."""
    import av
    episode = Path(episode)
    meta = json.loads((episode / 'metadata.json').read_text())
    info = json.loads((episode / 'lerobot/meta/info.json').read_text())
    assert meta['outcome'] == 'success' and meta['dataset_status'] == 'complete'
    steps = list(read_observations(episode))
    validate_recorded_configuration(meta.get("scene_instance") or {}, task_name,
                                    [obs["scripted_task"] for obs in steps])
    assert len(steps) == info['total_frames'] == meta['dataset_frame_count'] and steps
    from space_arm_platform.sampling import sample_tick, tick_time_ns
    first_tick = sample_tick(int(steps[0]['sim_time_ns']), meta['dataset_fps'])
    assert first_tick is not None
    stages = []
    for i, obs in enumerate(steps):
        assert int(obs['render_frame_id']) == int(steps[0]['render_frame_id']) + i
        assert int(obs['sim_time_ns']) == tick_time_ns(first_tick+i, meta['dataset_fps'])
        stage = obs['scripted_task']['stage']
        if not stages or stages[-1] != stage:
            stages.append(stage)
    final = steps[-1]['scripted_task']
    assert final['result'] == 'success'
    validate_task_recording(steps, stages, task_name)
    videos = list((episode/'lerobot/videos').rglob('*.mp4'))
    assert len(videos) == 2
    for path in videos:
        with av.open(str(path)) as video:
            stream = video.streams.video[0]
            assert stream.frames == len(steps)
            assert (stream.width, stream.height) == (1024, 1024)
            assert stream.average_rate == meta['dataset_fps']
    return dict(passed=True, level='quick', frames=len(steps), videos_fully_decoded=False,
                full_validation_pending=True)


def validate_recording(episode, task_name="drawer_cycle"):
    import av
    import numpy as np
    import pyarrow.parquet as pq
    episode = Path(episode)
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
    validate_recorded_layout_model(meta, steps)
    validate_recorded_configuration(meta.get("scene_instance") or {}, task_name,
                                    [obs["scripted_task"] for obs in steps])
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
    validate_task_recording(steps, stages, task_name)
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


