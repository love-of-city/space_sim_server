"""CLI defaults preserve complete snapshots; explicit conflicts remain errors."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import pytest

from collection import run_scripted_task as runner
from collection.record_scripted_task import freeze_plug_configuration, task_target_site
from simulation.scripted_tasks.plug_motion_config import config_snapshot, plug_motion_config


def frozen_scene():
    task = "cylindrical_plug_insert"
    config = plug_motion_config(task, force_limit_n=20., target_site="cylindrical_insert_40p1")
    snapshot = config_snapshot(config, task)
    return task, {"collection_config": {"plug_config_snapshot": snapshot}}, snapshot


@pytest.mark.parametrize("extra,conflicts", [([], False), (["--force-limit", "20"], False),
                                           (["--force-limit", "30"], True),
                                           (["--arm-speed", "0.2"], True)])
def test_actual_runner_cli_distinguishes_defaults_from_explicit_options(tmp_path, monkeypatch, extra, conflicts):
    task_name, scene, snapshot = frozen_scene()

    def task_factory(model, config):
        return SimpleNamespace(config=config, configuration_snapshot=config_snapshot(config, task_name))

    monkeypatch.setattr("simulation.scripted_tasks.plug_insert.PlugInsertTask", task_factory)
    monkeypatch.setattr(runner, "run", lambda args, **kwargs: runner.create_task(args, Path("unused.xml"), deepcopy(scene)))
    argv = ["--task", task_name, "--output", str(tmp_path / "run"), *extra]
    if conflicts:
        with pytest.raises(ValueError, match="conflicts with frozen"):
            runner.main(argv=argv)
    else:
        task = runner.main(argv=argv)
        assert task.configuration_snapshot == snapshot


def test_recording_freezes_actual_snapshot_before_resolving_target_metadata():
    task_name, scene, snapshot = frozen_scene()
    frozen = freeze_plug_configuration(scene, task_name, None)
    assert frozen == snapshot
    assert task_target_site(task_name, frozen["values"]["target_site"]) == "cylindrical_insert_40p1"
    with pytest.raises(ValueError, match="conflicts with frozen"):
        freeze_plug_configuration(scene, task_name, "cylindrical_assembly_hole")
