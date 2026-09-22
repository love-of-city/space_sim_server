"""Custom J1-J6 scene initialization, API validation and saved-state compatibility."""
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient
from pydantic import ValidationError

from space_arm_platform.app import PlatformConfig, create_app
from space_arm_platform.models import SceneInstanceCreate
from space_arm_platform.scene_runtime import SceneRuntimeManager, SCENE_TEMPLATES

ANGLES = [-45.0, -20.0, 25.0, -90.0, -50.0, 275.0]


@pytest.mark.parametrize("profile", ["none", "training-v1", "teleop-balanced-v1"])
@pytest.mark.parametrize("template", [item["id"] for item in SCENE_TEMPLATES])
def test_only_six_arm_axes_overridden_and_persisted(tmp_path, profile, template):
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    common = dict(seed=123, randomization_profile=profile, template_id=template, randomize_orbit_phase=True)
    baseline = manager.create_instance(SceneInstanceCreate(**common))
    instance = manager.create_instance(SceneInstanceCreate(**common, initial_arm_joint_position_deg=ANGLES))
    expected = baseline["randomization"].copy()
    expected["arm_joint_position_rad"] = [*map(math.radians, ANGLES), *expected["arm_joint_position_rad"][6:]]
    assert instance["randomization"] == expected
    assert instance["environment"] == baseline["environment"]
    assert instance["initial_arm_joint_position_deg"] == ANGLES
    stored = json.loads(Path(instance["config_path"]).read_text(encoding="utf-8"))
    assert stored["randomization"] == expected
    assert stored["initial_arm_joint_position_deg"] == ANGLES
    # The existing loader/reset path consumes the saved eight-DOF SI vector.
    pytest.importorskip("Basilisk")
    from simulation.teleop_grasp_unreal import _load_scene_instance
    loaded = _load_scene_instance(Path(instance["config_path"]))
    assert loaded["randomization"]["arm_joint_position_rad"] == expected["arm_joint_position_rad"]


@pytest.mark.parametrize("bad", [[], [0]*5, [0]*7, "0,0,0,0,0,0",
    [True, 0, 0, 0, 0, 0], ["1", 0, 0, 0, 0, 0], [None, 0, 0, 0, 0, 0],
    [float("nan"), 0, 0, 0, 0, 0], [float("inf"), 0, 0, 0, 0, 0]])
def test_strict_six_finite_degree_values(bad):
    with pytest.raises(ValidationError):
        SceneInstanceCreate(initial_arm_joint_position_deg=bad)


def test_default_and_explicit_null_keep_original_randomization(tmp_path):
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    default = manager.create_instance(SceneInstanceCreate(seed=42))
    explicit = manager.create_instance(SceneInstanceCreate(seed=42, initial_arm_joint_position_deg=None))
    assert default["randomization"] == explicit["randomization"]
    assert default["initial_arm_joint_position_deg"] is None


def test_catalog_bounds_are_enforced_before_writing(tmp_path):
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    for template in manager.catalog()["templates"]:
        limits = template["arm_joint_limits_deg"]
        assert len(limits) == 6
        for i, bounds in enumerate(limits):
            if bounds == [None, None]:
                continue
            for bound, sign in zip(bounds, [-1, 1]):
                angles = ANGLES.copy()
                angles[i] = bound
                instance = manager.create_instance(SceneInstanceCreate(randomization_profile="teleop-balanced-v1", template_id=template["id"], initial_arm_joint_position_deg=angles))
                assert instance["initial_arm_joint_position_deg"][i] == bound
                from simulation.teleop_grasp_unreal import _load_scene_instance
                _load_scene_instance(Path(instance["config_path"]))
                before = set(manager.scene_root.iterdir())
                angles[i] += sign * 0.001
                with pytest.raises(ValueError, match=f"J{i+1}"):
                    manager.create_instance(SceneInstanceCreate(randomization_profile="teleop-balanced-v1", template_id=template["id"], initial_arm_joint_position_deg=angles))
                assert set(manager.scene_root.iterdir()) == before


def test_selected_deployment_model_limits_not_hardcoded(tmp_path):
    model_root = tmp_path / "models"
    model_root.mkdir()
    (model_root / "sarm_platform.xml").write_text(
        '<mujoco><compiler angle="degree"/><worldbody><body>'
        + ''.join(f'<joint name="joint{i}" range="-10 10"/>' for i in range(1, 7))
        + '</body></worldbody></mujoco>', encoding="utf-8")
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    manager.launch = SimpleNamespace(model_root=model_root)
    with pytest.raises(ValueError, match="J1"):
        manager.create_instance(SceneInstanceCreate(randomization_profile="teleop-balanced-v1", template_id="spacecraft-arm-teleop", initial_arm_joint_position_deg=ANGLES))


def test_api_accepts_custom_angles_and_reports_invalid_input(tmp_path):
    project = tmp_path / "project"
    (project / "frontend").mkdir(parents=True)
    (project / "frontend/index.html").write_text("test", encoding="utf-8")
    app = create_app(PlatformConfig(project_root=project, data_root=tmp_path / "episodes", simulation_port=0, capture_port=0))
    with TestClient(app) as client:
        assert client.post("/api/auth/login", json={"username": "admin", "password": "ChangeMe123!"}).status_code == 200
        catalog = client.get("/api/scenes/catalog").json()
        assert len(catalog["initial_arm_presets_deg"]["teleop-balanced-v1"]) == 6
        response = client.post("/api/scenes/instances", json={"seed": 123, "randomization_profile": "teleop-balanced-v1", "initial_arm_joint_position_deg": ANGLES})
        assert response.status_code == 200
        assert response.json()["initial_arm_joint_position_deg"] == ANGLES
        for bad in ([0]*5, [True]*6, [10000]*6):
            response = client.post("/api/scenes/instances", json={"randomization_profile": "teleop-balanced-v1", "initial_arm_joint_position_deg": bad})
            assert response.status_code == 422
        assert "J1" in response.json()["detail"]
