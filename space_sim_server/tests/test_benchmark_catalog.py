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
    assert [item["id"] for item in result["configurations"]] == ["d01-cooperative-gnss-estimation-v1", "d05-cooperative-far-rendezvous-v1", "d12-continuous-gnss-hold-v1", "d14-gnss-formation-establish-v1", "d15-gnss-formation-hold-v1", "d16-gnss-formation-reconfigure-v1", "d23-marked-mono-pose-v1", "d24-marked-stereo-pose-v1", "d27-stereo-gyro-spin-v1", "d29-vbar-gnss-approach-v1", "d30-rbar-gnss-approach-v1", "d32-cooperative-gnss-v1", "d33-gnss-flyaround-v1", "d37-stereo-body-approach-v1", "d40-gnss-proximity-abort-v1", "m01-gnss-formation-establish-v1", "m02-gnss-formation-hold-v1", "m03-gnss-formation-reconfigure-v1", "s01-wheel-detumble-v1", "s02-wheel-attitude-v1", "s03-large-angle-wheel-v1", "s04-sequential-wheel-slew-v1", "s05-ground-pointing-v1", "s06-earth-pointing-v1", "s06-inertial-pointing-v1", "s06-sun-pointing-v1", "s09-finite-burn-v1", "s11-orbit-lower-v1", "s11-orbit-raise-v1", "s13-gnss-orbit-maintenance-v1", "s14-gnss-state-estimation-v1", "s16-catalog-debris-avoidance-v1", "s22-wheel-effectiveness-recovery-v1"]
    output = tmp_path / "must-not-exist"
    assert main(["run", "D26", "--seed", "201", "--output", str(output)]) == 2
    assert "not registered" in capsys.readouterr().err
    assert not output.exists()


def test_matrix_keeps_unregistered_requirements_and_does_not_claim_acceptance(capsys):
    assert main(["matrix"]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["scope"] == "execution_registration_not_runtime_acceptance"
    assert result["requirements"] == len(result["tasks"]) == 122
    assert result["registered_requirements"] == 30
    assert result["unregistered_requirements"] == 92
    assert result["configuration_count"] == 33
    rows = {row["source_id"]: row for row in result["tasks"]}
    assert len(rows["S11"]["configurations"]) == 2
    assert rows["T06"]["execution_registration"] == "not_registered"
    assert rows["T06"]["configurations"] == []
    assert "success_rate" not in result and "accepted" not in result


@pytest.mark.parametrize('kind', ['vision-approach', 'vision-approach-observation', 'vision-approach-action',
                                 'wheel-recovery', 'wheel-recovery-observation', 'wheel-recovery-action', 'debris-avoidance', 'debris-observation'])
def test_new_family_schemas_are_reachable_through_cli_argument_parser(capsys, kind):
    assert main(['schema', '--kind', kind]) == 0
    schema = json.loads(capsys.readouterr().out)
    assert schema['type'] == 'object' and 'schema_version' in schema['properties']
