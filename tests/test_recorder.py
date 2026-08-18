import json

import pytest

from space_arm_platform.models import (
    AppliedAction,
    EpisodeStart,
    EpisodeStop,
    SimulationObservation,
)
from space_arm_platform.recorder import EpisodeRecorder


def test_episode_records_actions_states_and_capture_products(tmp_path) -> None:
    recorder = EpisodeRecorder(tmp_path)
    metadata = recorder.start(EpisodeStart(instruction="抓取测试"))
    action = AppliedAction(
        episode_id=metadata["episode_id"],
        server_sequence="1",
        server_time_ns="100",
        client_sequence="1",
        client_time_ns="90",
        deadman=True,
        end_effector_linear_velocity_body_m_s=[0.01, 0.0, 0.0],
        end_effector_angular_velocity_body_rad_s=[0.0, 0.1, 0.0],
        gripper_velocity_rad_s=-0.1,
        input_source="keyboard",
    )
    observation = SimulationObservation(
        protocol="space-arm-control/1",
        type="observation",
        simulation_id="test-sim",
        step_id="7",
        render_frame_id="42",
        sim_time_ns="200",
        wall_time_ns="300",
        applied_action_sequence="1",
        joint_position_rad=[0.0] * 6,
        joint_velocity_rad_s=[0.1] * 6,
        target_joint_position_rad=[0.01] * 6,
    )
    recorder.record_action(action)
    recorder.record_observation(observation, action)
    recorder.record_authoritative_capture(
        {
            "protocol": "bsk-capture/1",
            "camera_id": "teleop/camera/overview",
            "capture_sequence": "8",
            "source_frame_id": "42",
            "sim_time_ns": "200",
            "stream_kind": "authoritative",
            "state_kind": "authoritative",
            "products": [{"name": "rgb", "file_name": "rgb.png"}],
        },
        {"rgb": b"PNG"},
    )
    closed = recorder.stop(EpisodeStop(outcome="success"))
    directory = tmp_path / metadata["episode_id"]
    assert closed["step_count"] == 1
    assert closed["capture_count"] == 1
    assert closed["capture_sync"] == "complete"
    assert json.loads((directory / "metadata.json").read_text(encoding="utf-8"))["outcome"] == "success"
    assert len((directory / "steps.jsonl").read_text(encoding="utf-8").splitlines()) == 1
    assert list((directory / "cameras").rglob("*.png"))
    capture = json.loads((directory / "captures.jsonl").read_text(encoding="utf-8"))
    assert capture["step_id"] == "7"
    assert capture["source_frame_id"] == "42"
    assert capture["authoritative_state"] is True


def test_capture_before_observation_is_paired_without_interpolated_state(tmp_path) -> None:
    recorder = EpisodeRecorder(tmp_path)
    recorder.start(EpisodeStart())
    recorder.record_authoritative_capture(
        {
            "protocol": "bsk-capture/1",
            "camera_id": "wrist",
            "capture_sequence": "1",
            "source_frame_id": "3",
            "sim_time_ns": "100",
            "stream_kind": "authoritative",
            "state_kind": "authoritative",
            "products": [{"name": "depth", "file_name": "depth.exr"}],
        },
        {"depth": b"EXR"},
    )
    assert recorder.sync_status()["pending_capture_count"] == 1
    observation = SimulationObservation(
        protocol="space-arm-control/1",
        type="observation",
        simulation_id="test-sim",
        step_id="3",
        render_frame_id="3",
        sim_time_ns="100",
        wall_time_ns="101",
        applied_action_sequence="0",
        joint_position_rad=[0.0] * 6,
        joint_velocity_rad_s=[0.0] * 6,
        target_joint_position_rad=[0.0] * 6,
    )
    recorder.record_observation(observation, None)
    assert recorder.sync_status()["pending_capture_count"] == 0
    assert recorder.sync_status()["matched_capture_count"] == 1


def test_capture_with_mismatched_authoritative_time_is_rejected(tmp_path) -> None:
    recorder = EpisodeRecorder(tmp_path)
    recorder.start(EpisodeStart())
    observation = SimulationObservation(
        protocol="space-arm-control/1",
        type="observation",
        simulation_id="test-sim",
        step_id="1",
        render_frame_id="9",
        sim_time_ns="100",
        wall_time_ns="101",
        applied_action_sequence="0",
        joint_position_rad=[0.0] * 6,
        joint_velocity_rad_s=[0.0] * 6,
        target_joint_position_rad=[0.0] * 6,
    )
    recorder.record_observation(observation, None)
    with pytest.raises(ValueError, match="sim_time_ns"):
        recorder.record_authoritative_capture(
            {
                "stream_kind": "authoritative",
                "state_kind": "authoritative",
                "source_frame_id": "9",
                "sim_time_ns": "102",
            },
            {"rgb": b"PNG"},
        )
    assert recorder.sync_status()["rejected_capture_count"] == 1
