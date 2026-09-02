from __future__ import annotations

from pathlib import Path

from fastapi.testclient import TestClient

from space_arm_platform.app import PlatformConfig, create_app
from space_arm_platform.models import SceneInstanceCreate
from space_arm_platform.scene_runtime import (
    SceneRuntimeManager,
    _NATIVE_COMMON_VELOCITY,
    _NATIVE_PREGRASP,
    _NATIVE_TARGET_POSITION,
    _NATIVE_TARGET_QUATERNION,
    _NATIVE_TARGET_SPIN,
    _DEFAULT_EPHEMERIS_CENTER,
    _DEFAULT_EPHEMERIS_EPOCH_UTC,
    _DEFAULT_EPHEMERIS_FRAME,
)


def request(seed: int | None, profile: str = "training-v1") -> SceneInstanceCreate:
    return SceneInstanceCreate(seed=seed, randomization_profile=profile)


def test_scene_randomization_is_reproducible(tmp_path: Path) -> None:
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    first = manager.create_instance(request(12345))
    second = manager.create_instance(request(12345))
    third = manager.create_instance(request(54321))

    assert first["randomization"] == second["randomization"]
    assert first["randomization"] != third["randomization"]
    assert first["instance_id"] != second["instance_id"]
    assert Path(first["config_path"]).is_file()
    assert first["environment"] == {
        "ephemeris_epoch_utc": _DEFAULT_EPHEMERIS_EPOCH_UTC,
        "ephemeris_center": _DEFAULT_EPHEMERIS_CENTER,
        "ephemeris_frame": _DEFAULT_EPHEMERIS_FRAME,
    }


def test_fixed_profile_matches_native_initial_state(tmp_path: Path) -> None:
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    instance = manager.create_instance(request(7, "none"))

    assert instance["randomization"] == {
        "target_position_m": list(_NATIVE_TARGET_POSITION),
        "target_orientation_wxyz": list(_NATIVE_TARGET_QUATERNION),
        "target_linear_velocity_m_s": list(_NATIVE_COMMON_VELOCITY),
        "target_angular_velocity_rad_s": list(_NATIVE_TARGET_SPIN),
        "arm_joint_position_rad": list(_NATIVE_PREGRASP),
    }


def test_scene_api_catalog_and_instance_generation_without_launcher(tmp_path: Path) -> None:
    project_root = tmp_path / "project"
    frontend = project_root / "frontend"
    frontend.mkdir(parents=True)
    (frontend / "index.html").write_text("<!doctype html><title>test</title>", encoding="utf-8")
    app = create_app(
        PlatformConfig(
            project_root=project_root,
            data_root=tmp_path / "episodes",
            simulation_port=0,
            capture_port=0,
        )
    )

    with TestClient(app) as client:
        login = client.post(
            "/api/auth/login", json={"username": "admin", "password": "ChangeMe123!"}
        )
        assert login.status_code == 200
        catalog = client.get("/api/scenes/catalog")
        assert catalog.status_code == 200
        assert catalog.json()["defaults"]["randomization_profile"] == "training-v1"

        created = client.post("/api/scenes/instances", json={"seed": 99})
        assert created.status_code == 200
        assert created.json()["seed"] == 99
        assert Path(created.json()["config_path"]).is_file()

        state = client.get("/api/state").json()
        assert state["scene_runtime"]["enabled"] is False
        start = client.post("/api/scenes/start", json={"seed": 99})
        assert start.status_code == 409
