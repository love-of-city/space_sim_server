"""Recording must preserve the production schema and reject incomplete episodes."""

from dataclasses import asdict
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from simulation.scripted_tasks.capture_transport import ScriptedCaptureTransport
from simulation.scripted_tasks.drawer_lift import DrawerLiftConfig
from space_arm_platform.models import SimulationObservation
from tools.record_drawer_task import scene_copy, validate_episode


@pytest.mark.parametrize("raw", [False, True, None])
def test_capture_manifest_accepts_optional_raw_images_and_rejects_bad_pairing(tmp_path, raw):
    from PIL import Image
    from tools.collect_drawer_cycles import validate_capture_manifest
    from tools.record_drawer_task import CAMERAS

    steps = [{"render_frame_id": "42", "render_session_id": "session", "sim_time_ns": "840000000"}]
    meta = {} if raw is None else {"raw_camera_images_saved": raw}
    captures = []
    for index, camera in enumerate(CAMERAS):
        files = {}
        if raw is not False:
            path = tmp_path / f"camera-{index}.png"
            Image.new("RGB", (1024, 1024)).save(path)
            files["rgb"] = path.name
        captures.append(dict(
            source_frame_id="42", sim_time_ns="840000000", files=files,
            metadata=dict(camera_id=camera, session_id="session", resolution=[1024, 1024])))

    def save(rows):
        (tmp_path / "captures.jsonl").write_text("\n".join(map(json.dumps, rows)), encoding="utf-8")

    save(captures)
    assert validate_capture_manifest(tmp_path, meta, steps) == 2
    save([captures[0], captures[0]])
    with pytest.raises(AssertionError):
        validate_capture_manifest(tmp_path, meta, steps)
    save(captures[:1])
    with pytest.raises(AssertionError):
        validate_capture_manifest(tmp_path, meta, steps)
    captures[1]["sim_time_ns"] = "860000000"
    save(captures)
    with pytest.raises(AssertionError):
        validate_capture_manifest(tmp_path, meta, steps)
    captures[1]["sim_time_ns"] = "840000000"
    captures[1]["metadata"]["camera_id"] = "unknown-camera"
    save(captures)
    with pytest.raises(AssertionError):
        validate_capture_manifest(tmp_path, meta, steps)
    if raw is not False:
        save([captures[0], {**captures[1], "metadata": {**captures[1]["metadata"], "camera_id": CAMERAS[1]}}])
        (tmp_path / captures[0]["files"]["rgb"]).unlink()
        with pytest.raises(FileNotFoundError):
            validate_capture_manifest(tmp_path, meta, steps)


def test_scripted_recording_loads_shared_settings_and_rejects_incompatible_capture(tmp_path, monkeypatch):
    from tools import record_drawer_task
    monkeypatch.setattr(record_drawer_task, "ROOT", tmp_path)
    path = tmp_path / "config/recording.json"
    path.parent.mkdir()
    settings = {"save_raw_camera_images": True, "max_frames": 25000,
                "video": {"encoder_threads": 1}}
    path.write_text(json.dumps(settings), encoding="utf-8")
    result = record_drawer_task.recording_settings()
    assert result.save_raw_camera_images and result.max_frames == 25000
    assert result.video.encoder_threads == 1
    for incompatible in ({"capture_rate_hz": 25}, {"camera_ids": []}):
        path.write_text(json.dumps({**settings, **incompatible}), encoding="utf-8")
        with pytest.raises(ValueError, match="Scripted collection requires"):
            record_drawer_task.recording_settings()


def test_scene_copy_preserves_configuration_without_mutating_original(tmp_path):
    source = tmp_path / "scene.json"
    scene = {
        "instance_id": "interactive-scene",
        "template_id": "sarm-task-box-20261012",
        "runtime": {
            "dynamics_rate_hz": 240,
            "capture_rate_hz": 30,
            "dataset_capture": False,
        },
        "randomization": {"arm_joint_position_rad": [1.0] * 8},
        "environment": {"orbit": {"altitude_m": 500000}},
    }
    source.write_text(json.dumps(scene), encoding="utf-8")
    copied = scene_copy(source)
    assert copied["instance_id"] != scene["instance_id"]
    assert copied["source_instance_id"] == scene["instance_id"]
    assert copied["runtime"]["dataset_capture"]
    assert copied["runtime"]["dynamics_step_s"] == 0.0025
    assert copied["randomization"] == scene["randomization"]
    assert copied["environment"] == scene["environment"]
    assert json.loads(source.read_text(encoding="utf-8")) == scene
    scene["runtime"]["dynamics_rate_hz"] = 120
    source.write_text(json.dumps(scene), encoding="utf-8")
    migrated = scene_copy(source)
    assert migrated["runtime"]["dynamics_rate_hz"] == 400
    assert migrated["runtime"]["capture_rate_hz"] == 50
    assert migrated["runtime"]["ik_rate_hz"] == 100


def test_incomplete_recording_is_not_accepted_as_success():
    good = dict(
        dataset_status="complete",
        dataset_frame_count=300,
        capture_count=600,
        step_count=300,
        incomplete_sample_count=0,
        rejected_capture_count=0,
    )
    validate_episode(good)
    for changes in (
        {"dataset_status": "failed"},
        {"capture_count": 599},
        {"step_count": 299},
        {"dataset_frame_count": 0},
        {"incomplete_sample_count": 1},
        {"rejected_capture_count": 1},
    ):
        with pytest.raises(RuntimeError):
            validate_episode({**good, **changes})


@pytest.mark.parametrize("retention_enabled", [False, True])
def test_scripted_snapshot_uses_actual_held_commands_and_production_sensor_schema(retention_enabled):
    transport = ScriptedCaptureTransport.__new__(ScriptedCaptureTransport)
    from simulation.plug_retention import RetentionParameters
    retention_bodies = {"cylindrical_plug": {"status": "retained", "socket": "cylindrical_initial"}}
    transport.plug_retention = SimpleNamespace(telemetry=lambda: retention_bodies) if retention_enabled else None
    transport.retention_configuration = ({"enabled": True, "version": "plug-retention/1",
                                          "parameters": asdict(RetentionParameters())}
                                         if retention_enabled else {"enabled": False})
    stamp = 20_000_000
    transport.wrench_sampler = SimpleNamespace(drain=lambda stamp: [])
    q = np.linspace(0.0, 0.7, 10)
    velocity = np.linspace(0.01, 0.1, 10)
    stepper = SimpleNamespace(
        qpos=q,
        qvel=velocity,
        telemetry=lambda: {"orbital_environment": {"mode": "bsk"}},
        read_wrench=lambda *_: (
            np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]),
            17_500_000,
            stamp,
        ),
    )
    transport.ctx = SimpleNamespace(
        scene=object(),
        simulation=SimpleNamespace(
            local_stepper=stepper,
            attitude_control=SimpleNamespace(telemetry=lambda: {}),
        ),
    )
    transport._addresses = (list(range(8)), list(range(8)))
    transport.drawer = SimpleNamespace(getQposAdr=lambda: 8, getQvelAdr=lambda: 8)
    transport.buckle = SimpleNamespace(getQposAdr=lambda: 9)
    transport.command = q[:8] + 0.02
    transport.client = SimpleNamespace(reset_generation="generation-1")
    transport.render = SimpleNamespace(
        bridge=SimpleNamespace(
            session_id="session-1",
            last_capture_state={
                "capture_episode_id": "episode-1",
                "capture_request_id": "request-1",
            },
        )
    )
    transport.task = SimpleNamespace(
        chain=SimpleNamespace(
            forward=lambda _: (np.array([0.1, 0.2, 0.3]), np.eye(3)),
            jacobian=lambda _: np.eye(6),
        ),
        stage="lift",
        result=None,
        reason="",
        config=DrawerLiftConfig(),
    )
    transport.scene_instance = {
        "instance_id": "scene-1",
        "seed": 7,
        "template_id": "sarm-task-box-20261012",
    }
    transport.model_hash = "abc123"
    transport.sequence, transport.task_start_ns, transport.task_elapsed_s = 8, 0, 1 / 30
    packet = transport.snapshot(1, stamp)
    observation = SimulationObservation.model_validate(packet)
    assert json.loads(observation.model_dump_json())["plug_retention"] == {
        **transport.retention_configuration, "bodies": retention_bodies if retention_enabled else {}}
    np.testing.assert_array_equal(
        observation.target_joint_position_rad, transport.command
    )
    np.testing.assert_array_equal(observation.joint_position_rad, q[:8])
    assert observation.target_gripper_drive_position_rad == transport.command[6]
    assert observation.gripper_position_m is None
    assert observation.wrist_wrench.interval_end_time_ns == str(stamp)
    assert observation.wrist_wrench.force_n == [1.0, 2.0, 3.0]
    assert observation.capture_episode_id == "episode-1"
    assert observation.render_session_id == "session-1"
    assert observation.model_extra["scripted_task"]["stage"] == "lift"
    assert observation.model_extra["scripted_task"]["task_parameters"] == asdict(
        DrawerLiftConfig()
    )
    # Complete JSON serialization preserves the scripted source in platform_json.
    assert (
        json.loads(observation.model_dump_json())["scripted_task"]["command_source"]
        == "scripted_joint_reference"
    )
