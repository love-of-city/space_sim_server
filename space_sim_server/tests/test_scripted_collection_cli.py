"""Saved layout metadata and caller paths must stay consistent during collection."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from space_arm_platform.overview_camera import SCHEMA
from collection import collect_scripted_tasks as collector


@pytest.mark.parametrize("saved_layout", [
    {"collection_layout_randomization": True},
    {"task_box_layout": {"schema": "task-box-layout/3", "model_sha256": "old-model"}},
    {"collection_config": {"layout_randomization": {"enabled": True, "model_sha256": "old-model"}}},
])
def test_saved_layout_requires_explicit_pool_before_startup(tmp_path, monkeypatch, capsys, saved_layout):
    scene = tmp_path / "scene.json"
    scene.write_text(json.dumps(saved_layout), encoding="utf-8")
    output = tmp_path / "batch"
    monkeypatch.setattr("sys.argv", ["collect", "--scene", str(scene), "--output", str(output)])

    def unexpected_start(*_):
        pytest.fail("A saved layout must be rejected before checking ports or starting native preflight")

    monkeypatch.setattr(collector, "check_port_available", unexpected_start)
    monkeypatch.setattr(collector, "NativePoseProbe", unexpected_start)
    with pytest.raises(SystemExit) as error:
        collector.main()
    assert error.value.code == 2
    assert "--layout-pool is required" in capsys.readouterr().err
    assert not output.exists()


@pytest.mark.parametrize("sampling", [None, "presets"])
def test_caller_paths_and_camera_defaults_reach_recording_child(tmp_path, monkeypatch, sampling):
    from collection import record_scripted_task as recorder

    monkeypatch.chdir(tmp_path)
    (tmp_path / "scene.json").write_text(json.dumps({
        "template_id": "sarm-task-box-20261012",
        "randomization": {"arm_joint_position_rad": [0.] * 8},
        "collection_config": {"layout_randomization": {"enabled": False}},
        "overview_camera_sampling": "continuous",
    }), encoding="utf-8")
    expected = {
        "--adapter": Path.home() / "adapter",
        "--unreal": tmp_path / "engine",
        "--cancel-file": tmp_path / "cancel.request",
        "--episodes-root": tmp_path / "episodes",
        "--recording-config": Path.home() / "recording.json",
    }
    seen = []

    class Probe:
        def __init__(self, model):
            pass

        def validate(self, model, scene, task):
            return {"accepted": True}

        def close(self):
            pass

    class Renderer:
        def __init__(self, args, output):
            assert args.adapter == expected["--adapter"]
            assert args.unreal == expected["--unreal"]
            assert args.cancel_file == expected["--cancel-file"]
            assert output == tmp_path / "batch"
            self.process = SimpleNamespace(pid=123)
            seen.append("renderer")

        def check(self):
            pass

        def close(self):
            seen.append("renderer_closed")

    def popen(command, **kwargs):
        assert kwargs["cwd"] == collector.ROOT
        scene = json.loads(Path(command[command.index("--scene")+1]).read_text(encoding="utf-8"))
        assert scene["overview_camera_sampling"] == "presets"
        assert scene["overview_camera"]["schema"] == SCHEMA
        assert scene["collection_config"]["overview_camera_randomization"] == scene["overview_camera"]
        assert scene["overview_camera"]["preset_id"] in {"P0", "P1", "P2", "P3", "P4"}
        assert command[command.index("--simulation-startup-timeout-seconds")+1] == "600.0"
        for flag, path in expected.items():
            actual = Path(command[command.index(flag) + 1])
            assert actual.is_absolute() and actual == path
        # Emulate an operator cancelling the recording via the child's argument.
        # The batch must see the exact same file despite the child's different cwd.
        expected["--cancel-file"].write_text("cancel", encoding="utf-8")
        seen.append("recording")
        return SimpleNamespace(poll=lambda: 0, returncode=0)

    def settings(path):
        assert path == expected["--recording-config"]

    monkeypatch.setattr(collector, "NativePoseProbe", Probe)
    monkeypatch.setattr(collector, "PersistentRenderer", Renderer)
    monkeypatch.setattr(collector, "check_port_available", lambda _: None)
    monkeypatch.setattr(recorder, "recording_settings", settings)
    monkeypatch.setattr(collector.subprocess, "Popen", popen)
    arguments = [
        "collect", "--scene", "scene.json", "--output", "batch", "--episodes-root", "episodes",
        "--adapter", "~/adapter", "--unreal", "engine", "--cancel-file", "cancel.request",
        "--recording-config", "~/recording.json", "--reuse-renderer", "--count", "1",
        "--max-attempts", "1", "--range-deg", "30",
        "--randomize-overview-camera",
        "--simulation-startup-timeout-seconds", "600",
    ]
    if sampling is not None:
        arguments += ["--overview-camera-sampling", sampling]
    monkeypatch.setattr("sys.argv", arguments)
    assert collector.main() == 1
    report = json.loads((tmp_path / "batch/summary.json").read_text(encoding="utf-8"))
    assert report["status"] == "cancelled"
    assert seen == ["renderer", "recording", "renderer_closed"]
    assert json.loads((tmp_path / "scene.json").read_text(encoding="utf-8"))["overview_camera_sampling"] == "continuous"


@pytest.mark.parametrize("half_range", ["-1", "50.1", "nan", "inf"])
def test_invalid_initialization_range_is_rejected_before_startup(tmp_path, monkeypatch, capsys, half_range):
    output = tmp_path / "batch"
    monkeypatch.setattr("sys.argv", [
        "collect", "--scene", str(tmp_path / "missing-scene.json"), "--output", str(output),
        "--range-deg", half_range,
    ])
    with pytest.raises(SystemExit) as error:
        collector.main()
    assert error.value.code == 2
    assert "range-deg must be finite and in [0,50]" in capsys.readouterr().err
    assert not output.exists()


def layout_metadata(*, source="layout-model", variant=None):
    config = {"layout_randomization": {"enabled": True, "model_sha256": source}}
    if variant is not None:
        config["model_randomization"] = variant
    return {"scene_instance": {
        "task_box_layout": {"model_sha256": source},
        "collection_config": config,
    }}


def test_layout_model_is_checked_against_every_authoritative_observation():
    meta = layout_metadata()
    collector.validate_recorded_layout_model(meta, [{"model_sha256": "layout-model"}] * 2)
    with pytest.raises(ValueError, match="Authoritative model"):
        collector.validate_recorded_layout_model(
            meta, [{"model_sha256": "layout-model"}, {"model_sha256": "fixed-model"}])


def test_layout_acceptance_handles_dynamics_randomization_without_weakening_binding():
    meta = layout_metadata(variant={"source_sha256": "layout-model", "variant_sha256": "scaled-model"})
    collector.validate_recorded_layout_model(meta, [{"model_sha256": "scaled-model"}])
    with pytest.raises(ValueError, match="Authoritative model"):
        collector.validate_recorded_layout_model(meta, [{"model_sha256": "layout-model"}])
    meta["scene_instance"]["collection_config"]["model_randomization"]["source_sha256"] = "other-layout"
    with pytest.raises(ValueError, match="Randomized model source"):
        collector.validate_recorded_layout_model(meta, [{"model_sha256": "scaled-model"}])


def test_layout_selection_must_agree_with_saved_layout_model():
    meta = layout_metadata()
    meta["scene_instance"]["collection_config"]["layout_randomization"]["model_sha256"] = "other-layout"
    with pytest.raises(ValueError, match="layout selection"):
        collector.validate_recorded_layout_model(meta, [{"model_sha256": "layout-model"}])
    del meta["scene_instance"]["task_box_layout"]
    with pytest.raises(ValueError, match="no layout model"):
        collector.validate_recorded_layout_model(meta, [{"model_sha256": "layout-model"}])


def test_historical_episode_without_layout_metadata_keeps_existing_validation():
    collector.validate_recorded_layout_model({}, [{}])
    collector.validate_recorded_layout_model({"scene_instance": {}}, [{}])
    collector.validate_recorded_layout_model({"scene_instance": {
        "collection_config": {"layout_randomization": {"enabled": False}},
    }}, [{}])
    meta = layout_metadata()
    meta["scene_instance"]["collection_config"]["layout_randomization"] = {"enabled": False}
    collector.validate_recorded_layout_model(meta, [{"model_sha256": "layout-model"}])


@pytest.mark.parametrize("platform_model", ["layout-model", "fixed-model", None])
def test_historical_compact_layout_uses_writer_platform_identity(tmp_path, platform_model):
    from space_arm_platform.compact_storage import ParquetChunks, numeric_digest, read_observations

    row = {
        "observation.state": [0.] * 7, "action": [0.] * 7,
        "observation.joint_velocity": [0.] * 8,
        "observation.end_effector_pose_body": [0.] * 3 + [1., 0., 0., 0.],
        "observation.end_effector_twist_body": [0.] * 6,
        "observation.source_frame_id": 1, "observation.sim_time_ns": 20_000_000,
    }
    data = ParquetChunks(tmp_path, "lerobot/data/chunk-000")
    data.append(row)
    data.flush()
    context = ParquetChunks(tmp_path, "lerobot/auxiliary/context")
    context.append({"source_frame_id": 1, "sim_time_ns": 20_000_000,
                    "source_values_sha256": numeric_digest(row)})
    context.flush()
    platform = tmp_path / "lerobot/meta/platform.json"
    platform.parent.mkdir()
    platform.write_text(json.dumps({"model_sha256": platform_model}), encoding="utf-8")
    restored = list(read_observations(tmp_path))
    assert restored[0]["model_sha256"] == platform_model
    if platform_model == "layout-model":
        collector.validate_recorded_layout_model(layout_metadata(), restored)
    else:
        with pytest.raises(ValueError, match="Authoritative model"):
            collector.validate_recorded_layout_model(layout_metadata(), restored)


def test_recording_validator_rejects_stale_layout_metadata_before_accepting_dataset(tmp_path, monkeypatch):
    meta = layout_metadata(source="stale-layout-model")
    meta.update(outcome="success", dataset_status="complete")
    (tmp_path / "metadata.json").write_text(json.dumps(meta), encoding="utf-8")
    directory = tmp_path / "lerobot/meta"
    directory.mkdir(parents=True)
    feature = {"shape": [7], "names": ["gripper_opening_normalized"]}
    (directory / "info.json").write_text(json.dumps({
        "features": {"observation.state": feature, "action": feature},
    }), encoding="utf-8")
    (directory / "platform.json").write_text(json.dumps({"schema": "space-arm-lerobot/2"}), encoding="utf-8")
    monkeypatch.setattr(collector, "is_compact", lambda _: True)
    monkeypatch.setattr(collector, "read_observations", lambda _: iter([{"model_sha256": "actual-fixed-model"}]))
    with pytest.raises(ValueError, match="Authoritative model"):
        collector.validate_recording(tmp_path)
