"""Batch reuse must preserve per-episode identity and full-cycle timing."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from tools import collect_scripted_tasks as collector


@pytest.mark.parametrize("retire_after_first", [False, True])
def test_batch_routes_two_validated_episodes_and_recycles_only_between_them(
        tmp_path, monkeypatch, retire_after_first):
    from simulation.scripted_tasks import randomization
    from tools import record_drawer_task as recorder

    source = tmp_path / "source.json"
    source.write_text("{}")
    model, catalog = tmp_path / "layout.xml", tmp_path / "catalog.json"
    episodes = tmp_path / "episodes"
    workers, commands = [], []
    finished = []

    class Worker:
        def __init__(self, server, output, **limits):
            assert limits == {"max_episodes": 4, "max_rss_mb": 8192}
            self.directory = output / f"dynamics-{len(workers)}"
            self.pid = 100 + len(workers)
            self.closed = False
            workers.append(self)

        def reusable(self):
            return not retire_after_first

        def close(self):
            self.closed = True

    class Probe:
        def __init__(self, path):
            assert path == model

        def validate(self, path, scene, task):
            assert scene["task_box_layout"]["model_sha256"] == "layout-hash"
            return {"accepted": True}

        def close(self):
            pass

    def randomized(scene, seed, half_range, task):
        assert half_range == 30.0
        return {"collection_config": {"initialization": {"seed": seed}}}

    def popen(command, **_):
        commands.append(command)
        assert Path(command[command.index("--model") + 1]) == model
        assert Path(command[command.index("--catalog") + 1]) == catalog
        assert Path(command[command.index("--dynamics-worker") + 1]) == workers[-1].directory
        output = Path(command[command.index("--output") + 1])
        output.mkdir()
        episode_id = f"episode-{len(commands)}"
        (output / "summary.json").write_text(json.dumps({
            "episode_id": episode_id, "task": {"result": "success"},
            "recording_wall_seconds": 1.0, "end_to_end_wall_seconds": 2.0,
            "dynamics_worker": {"pid": workers[-1].pid},
        }))
        return SimpleNamespace(poll=lambda: 0, returncode=0)

    def validate(path, task):
        finished.append(path)
        return {"passed": True}

    monkeypatch.setattr(collector, "PersistentDynamics", Worker)
    monkeypatch.setattr(collector, "NativePoseProbe", Probe)
    monkeypatch.setattr(collector, "randomized_scene", randomized)
    monkeypatch.setattr(collector, "check_port_available", lambda _: None)
    monkeypatch.setattr(collector, "validate_recording", validate)
    monkeypatch.setattr(collector.subprocess, "Popen", popen)
    monkeypatch.setattr(recorder, "recording_settings", lambda _: None)
    monkeypatch.setattr(randomization, "prepare_episode", lambda scene: scene)
    monkeypatch.setattr(randomization, "write_model_variant", lambda model, *_: model)
    monkeypatch.setattr("simulation.task_box_layout.validated_capture_layout",
                        lambda m, c: {"model_sha256": "layout-hash"})
    monkeypatch.setattr("sys.argv", [
        "collect", "--scene", str(source), "--output", str(tmp_path / "batch"),
        "--episodes-root", str(episodes), "--model", str(model), "--catalog", str(catalog),
        "--reuse-dynamics", "--count", "2", "--max-attempts", "2", "--range-deg", "30",
    ])
    assert collector.main() == 0
    assert len(workers) == (2 if retire_after_first else 1)
    assert all(worker.closed for worker in workers)
    assert finished == [episodes / "episode-1", episodes / "episode-2"]
    report = json.loads((tmp_path / "batch/summary.json").read_text())
    assert report["status"] == "complete" and report["reuse_dynamics"]
    for attempt in report["attempts"]:
        assert attempt["status"] == "accepted"
        assert int(attempt["finished_at_ns"]) >= int(attempt["started_at_ns"])
        assert attempt["end_to_end_wall_seconds"] >= attempt["validation_wall_seconds"]
        assert attempt["preflight_wall_seconds"] >= 0


def test_layout_acceptance_binds_each_observation_to_its_variant():
    meta = {"scene_instance": {
        "task_box_layout": {"model_sha256": "source"},
        "collection_config": {"model_randomization": {
            "source_sha256": "source", "variant_sha256": "coloured"}},
    }}
    collector.validate_recorded_layout_model(meta, [{"model_sha256": "coloured"}])
    with pytest.raises(ValueError, match="Authoritative model"):
        collector.validate_recorded_layout_model(meta, [{"model_sha256": "source"}])


def test_fixed_layout_and_pool_are_mutually_exclusive(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr("sys.argv", ["collect", "--scene", str(tmp_path / "scene.json"),
        "--output", str(tmp_path / "output"), "--model", "layout.xml", "--catalog", "catalog.json",
        "--layout-pool", "pool.json"])
    with pytest.raises(SystemExit) as error:
        collector.main()
    assert error.value.code == 2
    assert "provide --model and --catalog together, or --layout-pool" in capsys.readouterr().err
