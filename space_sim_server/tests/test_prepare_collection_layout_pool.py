"""Pool publication must follow the actual native trajectory gate."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from tools import prepare_collection_layout_pool as prepare
from space_arm_platform.layout_trajectories import TASKS, validate_scene


def test_default_validation_scene_uses_official_fixed_template(tmp_path):
    scene = prepare.validation_scene(None, tmp_path, 7)
    record = validate_scene(scene)
    assert record["randomization_profile"] == "teleop-zero-prepare-v1"
    assert record["initialization_enabled"] is False
    assert record["collection_layout_randomization"] is False
    assert record["randomize_orbit_phase"] is False
    assert not any(item["enabled"] for item in record["collection_randomization"].values())
    copied = tmp_path / "copy"
    copied.mkdir()
    assert prepare.validation_scene(scene, copied, 7).read_bytes() == scene.read_bytes()


@pytest.mark.parametrize("shipped_assets", [False, True])
def test_rejected_native_run_never_imports_ue_assets_or_publishes_pool(tmp_path, monkeypatch, shipped_assets):
    calls = []
    monkeypatch.setattr(prepare, "pool_entry", lambda *_: {})
    monkeypatch.setattr(prepare, "ManagerLease", lambda *_: SimpleNamespace(close=lambda: None))

    def run(command, **kwargs):
        calls.append(command)
        assert kwargs["env"]["OMP_NUM_THREADS"] == "1"
        assert kwargs["env"]["PYTHONPATH"].split(prepare.os.pathsep)[0] == str(tmp_path / "adapter/Adapters")
        assert Path(command[1]).name != "prepare_independent_layout_catalog.py"
        if Path(command[1]).name == "validate_task_trajectory_contacts.py":
            assert "--ignore-free-bolt" in command
            output = Path(command[command.index("--output")+1])
            output.mkdir()
            (output / "acceptance.json").write_text(json.dumps(dict(
                task_success=False, trajectory_clear=True, coverage_complete=True,
                operational_path_complete=False, observed_stages=["settle", "approach"],
                missing_operation_stages=["retract"])))
            # Target failure is not a subprocess crash; inspect the reports.
            return SimpleNamespace(returncode=1)
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(prepare.subprocess, "run", run)

    def reject(*_):
        assert len([c for c in calls if Path(c[1]).name == "validate_task_trajectory_contacts.py"]) == len(TASKS)
        raise ValueError("incomplete native operation path")

    monkeypatch.setattr(prepare, "bind_trajectory_acceptance", reject)
    output = tmp_path / "pool"
    prepared = []
    def base_catalog(path, **kwargs):
        prepared.append((path, kwargs["adapter"]))
        path.write_text('{}')
        return path
    monkeypatch.setattr("simulation.collection_assets.prepare_base_catalog", base_catalog)
    args = SimpleNamespace(count=1, existing=[("old-model", "old-catalog", "old-native")], seed=7,
                           yaw_range_deg=0., geometry_python=Path("/test/python"), output=output,
                           source=tmp_path / "source.xml", adapter=tmp_path / "adapter",
                           unreal=tmp_path / "ue", base_catalog=None if shipped_assets else tmp_path / "base.json", validation_scene=None)
    with pytest.raises(ValueError, match="rejected before UE import"):
        prepare.prepare(args)
    assert not (output / "pool.json").exists()
    assert prepared == ([(output / "base.catalog.json", args.adapter)] if shipped_assets else [])
    rejection = json.loads((output / "rejection.json").read_text())
    assert rejection["status"] == "rejected" and rejection["pool_published"] is False
    assert set(rejection["tasks"]) == set(TASKS)
    assert all(case["operational_path_complete"] is False for case in rejection["tasks"].values())
