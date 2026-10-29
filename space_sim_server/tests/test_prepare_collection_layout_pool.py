"""Pool preparation admits geometry-passed layouts without task execution."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from tools.assets import prepare_collection_layout_pool as prepare


@pytest.mark.parametrize("shipped_assets", [False, True])
@pytest.mark.parametrize("failed_geometry", [None, "check_independent_task_box_layout.py", "check_inset_target_geometry.py"])
def test_preparation_checks_geometry_then_imports_without_running_tasks(tmp_path, monkeypatch, shipped_assets, failed_geometry):
    calls = []
    monkeypatch.setattr(prepare, "pool_entry", lambda identifier, *_: {"id": identifier})
    monkeypatch.setattr(prepare, "ManagerLease", lambda *_: SimpleNamespace(close=lambda: None))

    def load_pool(path):
        document = json.loads(Path(path).read_text())
        assert len(document["layouts"]) == 2
        return SimpleNamespace(document=document)

    monkeypatch.setattr(prepare, "LayoutPool", load_pool)

    def run(command, **kwargs):
        calls.append(Path(command[1]).name)
        assert kwargs["env"]["OMP_NUM_THREADS"] == "1"
        assert kwargs["env"]["PYTHONPATH"].split(prepare.os.pathsep)[0] == str(tmp_path / "adapter/Adapters")
        assert calls[-1] != "validate_task_trajectory_contacts.py"
        if calls[-1] == failed_geometry:
            raise prepare.subprocess.CalledProcessError(1, command)
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(prepare.subprocess, "run", run)
    prepared = []

    def base_catalog(path, **kwargs):
        prepared.append((path, kwargs["adapter"]))
        path.write_text('{}')
        return path

    monkeypatch.setattr("simulation.collection_assets.prepare_base_catalog", base_catalog)
    output = tmp_path / "pool"
    args = SimpleNamespace(count=1, existing=[("old-model", "old-catalog", "old-native")], seed=7,
                           yaw_range_deg=0., geometry_python=Path("/test/python"), output=output,
                           source=tmp_path / "source.xml", adapter=tmp_path / "adapter",
                           unreal=tmp_path / "ue", base_catalog=None if shipped_assets else tmp_path / "base.json")
    if failed_geometry:
        with pytest.raises(prepare.subprocess.CalledProcessError):
            prepare.prepare(args)
        assert not (output / "pool.json").exists()
        assert "prepare_independent_layout_catalog.py" not in calls
        rejection = json.loads((output / "rejection.json").read_text())
        assert rejection["status"] == "rejected" and rejection["pool_published"] is False
    else:
        prepare.prepare(args)
        assert (output / "pool.json").is_file()
        assert calls == ["build_independent_task_box_layout.py", "check_independent_task_box_layout.py",
                         "check_inset_target_geometry.py", "prepare_independent_layout_catalog.py"]
    assert prepared == ([(output / "base.catalog.json", args.adapter)] if shipped_assets else [])
