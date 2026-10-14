import pytest
from space_arm_platform.models import SimulationObservation
from space_arm_platform.lerobot_capture import LiveLeRobotWriter
from space_arm_platform.models import EpisodeStart
from test_lerobot_recording import observation


def wrench(index=1, valid=True):
    end = index * 100_000_000
    return dict(
        sensor_name="wrist_ft",
        site_name="wrist_ft_site",
        parent_body="ee_base",
        child_body="wrist_ft_tool",
        frame="sensor_site",
        sign_convention="mujoco_force_torque",
        measurement="raw_interface_wrench",
        sample_phase="last_substep_start",
        valid=valid,
        sample_time_ns=str(end - 833333 if valid else end),
        interval_end_time_ns=str(end),
        force_n=[1.0, 2.0, 3.0] if valid else [0.0] * 3,
        torque_nm=[4.0, 5.0, 6.0] if valid else [0.0] * 3,
    )


def with_wrench(index=1, **kw):
    values = observation(index).model_dump()
    values["wrist_wrench"] = wrench(index, **kw)
    return SimulationObservation.model_validate(values)


@pytest.mark.parametrize("fault", ["nan", "future", "phase", "axes"])
def test_protocol_rejects_invalid_wrench(fault):
    values = with_wrench().model_dump()
    w = values["wrist_wrench"]
    if fault == "nan":
        w["force_n"][0] = float("nan")
    if fault == "future":
        w["interval_end_time_ns"] = "200000000"
    if fault == "phase":
        w["sample_time_ns"] = w["interval_end_time_ns"]
    if fault == "axes":
        w["force_n"] = [1, 2]
    with pytest.raises(ValueError):
        SimulationObservation.model_validate(values)


def test_lerobot_wrench_roundtrip_and_initial_invalid_sample(tmp_path):
    import numpy as np, json
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = tmp_path / "episode" / "lerobot"
    writer = LiveLeRobotWriter(root, EpisodeStart(fps=10, camera_ids=[]))
    writer.append(with_wrench(0, valid=False), {})
    writer.append(with_wrench(1), {})
    writer.finish()
    loaded = LeRobotDataset(
        f"local/{root.parent.name}", root=root, video_backend="pyav"
    )
    assert loaded[0]["observation.wrist_wrench_valid"].item() == 0
    assert loaded[1]["observation.wrist_wrench_valid"].item() == 1
    np.testing.assert_allclose(
        loaded[1]["observation.wrist_wrench"], [1, 2, 3, 4, 5, 6]
    )
    assert loaded[1]["observation.wrist_wrench_sample_time_ns"].item() == 99166667
    meta = json.loads((root / "meta/platform.json").read_text(encoding="utf-8"))
    assert meta["wrist_wrench"]["frame"] == "sensor_site"
    assert loaded.meta.info["features"]["observation.wrist_wrench"]["names"] == [
        "Fx_N",
        "Fy_N",
        "Fz_N",
        "Tx_Nm",
        "Ty_Nm",
        "Tz_Nm",
    ]


def test_dataset_rejects_sensor_disappearance(tmp_path):
    writer = LiveLeRobotWriter(
        tmp_path / "episode" / "lerobot", EpisodeStart(fps=10, camera_ids=[])
    )
    try:
        writer.append(with_wrench(), {})
        with pytest.raises(ValueError, match="availability"):
            writer.append(observation(2), {})
    finally:
        writer.close_failed()


def test_recorder_retains_wrench_in_steps_and_official_dataset(tmp_path):
    import json
    from space_arm_platform.models import EpisodeStop
    from test_lerobot_recording import make_recorder, packet

    recorder, root = make_recorder(tmp_path)
    obs = with_wrench(1)
    recorder.record_observation(obs, None)
    recorder.record_authoritative_capture(*packet(1))
    assert recorder.wait_for_writes()
    result = recorder.stop(EpisodeStop(outcome="success"))
    assert result["dataset_status"] == "complete", result
    row = json.loads((root / "steps.jsonl").read_text(encoding="utf-8").splitlines()[0])
    assert row["observation"]["wrist_wrench"] == obs.wrist_wrench.model_dump()
