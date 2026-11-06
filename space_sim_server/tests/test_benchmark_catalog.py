"""Guard source traceability and prevent requirement counts becoming fake results."""
from copy import deepcopy
import json

import pytest
from pydantic import ValidationError

from space_arm_platform.benchmark_catalog import AutonomyCatalog, load_catalog
from tools.autonomy_benchmark import main


@pytest.fixture
def payload():
    return load_catalog().model_dump(mode="json", by_alias=True)


def test_latest_ids_do_not_silently_select_older_teacher_tasks():
    catalog = load_catalog()
    assert catalog.task("S24").id == "ssb.recovery.safe-mode"
    assert catalog.task("D26").id == "ssb.vision.unknown-geometry-pose"
    assert catalog.task("D55").id == "ssb.grasp.stable-target"
    assert catalog.task("ssb.approach.waypoint-dwell").source_id == "D32"
    with pytest.raises(KeyError):
        catalog.task("D63")


def test_source_cannot_be_shrunk_by_editing_declared_counts(payload):
    payload["tasks"] = [task for task in payload["tasks"] if task["source_id"] != "D61"]
    payload["source"]["expected_counts"]["D"] = 61
    with pytest.raises(ValidationError, match="final source"):
        AutonomyCatalog.model_validate(payload)


@pytest.mark.parametrize("mutation", ["missing", "duplicate_source", "duplicate_stable", "out_of_range"])
def test_missing_and_aliased_requirements_are_rejected(payload, mutation):
    if mutation == "missing":
        payload["tasks"].pop()
    elif mutation == "duplicate_source":
        payload["tasks"][1]["source_id"] = payload["tasks"][0]["source_id"]
    elif mutation == "duplicate_stable":
        payload["tasks"][1]["id"] = payload["tasks"][0]["id"]
    else:
        payload["tasks"][0]["source_id"] = "S27"
    with pytest.raises(ValidationError):
        AutonomyCatalog.model_validate(payload)


@pytest.mark.parametrize("change", [
    {"target_cooperation": ["none", "C"]},
    {"requested_vision_modes": ["V0", "V0"]},
    {"requested_vision_modes": []},
    {"requested_vision_modes": ["V4"]},
    {"title": "  "},
    {"success": True},
])
def test_invalid_tags_and_fake_success_cannot_enter_catalogue(payload, change):
    payload["tasks"][0].update(change)
    with pytest.raises(ValidationError):
        AutonomyCatalog.model_validate(payload)


def test_postcapture_scope_cannot_be_removed(payload):
    payload["scope"]["excluded"].remove("refueling")
    with pytest.raises(ValidationError, match="every exclusion"):
        AutonomyCatalog.model_validate(payload)


def test_mission_tags_disclose_they_are_engineering_classification(payload):
    payload["tasks"][-1]["classification_origin"] = "source"
    with pytest.raises(ValidationError, match="disclose"):
        AutonomyCatalog.model_validate(payload)


def test_revision_namespace_is_required(payload):
    payload["source"]["id"] = "teacher-first-list"
    with pytest.raises(ValidationError):
        AutonomyCatalog.model_validate(payload)


def test_filter_does_not_change_full_catalogue_or_treat_modes_as_joint_requirement():
    catalog = load_catalog()
    selected = catalog.select(priority="P0", source_group="D", vision="V2")
    assert "D32" in {task.source_id for task in selected}
    assert "D05" not in {task.source_id for task in selected}
    assert all(task.priority == "P0" and task.source_id.startswith("D") for task in selected)
    assert len(catalog.tasks) == 122
    assert catalog.coverage()["scope"] == "requirement_catalogue_only"


def test_cli_reports_catalogue_scope_and_rejects_invalid_inputs(tmp_path, capsys, payload):
    assert main(["validate"]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["requirements"] == 122
    assert result["scope"] == "requirement_catalogue_only"
    assert "success_rate" not in result
    invalid = deepcopy(payload)
    invalid["tasks"].pop()
    path = tmp_path / "broken.json"
    path.write_text(json.dumps(invalid), encoding="utf-8")
    assert main(["--catalog", str(path), "validate"]) == 2
    assert "invalid autonomy catalogue" in capsys.readouterr().err


def test_cli_distinguishes_registered_configurations_from_requirement_ids(capsys, tmp_path):
    assert main(["tasks"]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["scope"] == "registered_execution_configurations"
    assert [item["id"] for item in result["configurations"]] == ["d32-cooperative-gnss-v1"]
    output = tmp_path / "must-not-exist"
    assert main(["run", "D26", "--seed", "201", "--output", str(output)]) == 2
    assert "not registered" in capsys.readouterr().err
    assert not output.exists()
