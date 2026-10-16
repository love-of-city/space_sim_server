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


def test_scripted_snapshot_uses_actual_held_commands_and_production_sensor_schema():
    transport = ScriptedCaptureTransport.__new__(ScriptedCaptureTransport)
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
