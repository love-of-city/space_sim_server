from __future__ import annotations

from pathlib import Path
import inspect
import json
from unittest.mock import Mock

import pytest

from fastapi.testclient import TestClient

from space_arm_platform.app import PlatformConfig, create_app
from space_arm_platform.models import SceneInstanceCreate
from space_arm_platform.control_defaults import (
    BALANCED_TELEOP_HOME,
    BALANCED_TELEOP_JOINT_SPANS,
    BALANCED_TELEOP_PROFILE,
    DEFAULT_RANDOMIZATION_PROFILE,
    AUTO_PREPARE_TELEOP_PROFILE,
)
from space_arm_platform.scene_runtime import (
    SceneRuntimeManager,
    SceneLaunchConfig,
    _NATIVE_COMMON_VELOCITY,
    _NATIVE_PREGRASP,
    _NATIVE_TARGET_POSITION,
    _NATIVE_TARGET_QUATERNION,
    _NATIVE_TARGET_SPIN,
    _DEFAULT_EPHEMERIS_CENTER,
    _DEFAULT_EPHEMERIS_EPOCH_UTC,
    _DEFAULT_EPHEMERIS_FRAME,
    _DEFAULT_ORBIT,
)


def request(seed: int | None, profile: str = "training-v1") -> SceneInstanceCreate:
    return SceneInstanceCreate(seed=seed, randomization_profile=profile, template_id="spacecraft-arm-teleop")


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
        "orbit": _DEFAULT_ORBIT,
        "lighting": {"sunlight_intensity_scale": 1.0},
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
        assert catalog.json()["defaults"]["randomization_profile"] == AUTO_PREPARE_TELEOP_PROFILE

        created = client.post("/api/scenes/instances", json={"seed": 99})
        assert created.status_code == 200
        assert created.json()["seed"] == 99
        assert Path(created.json()["config_path"]).is_file()

        state = client.get("/api/state").json()
        assert state["scene_runtime"]["enabled"] is False
        start = client.post("/api/scenes/start", json={"seed": 99})
        assert start.status_code == 409


def test_balanced_teleop_profile_stays_near_validated_home(tmp_path: Path) -> None:
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    instance = manager.create_instance(request(12345, BALANCED_TELEOP_PROFILE))
    joints = instance["randomization"]["arm_joint_position_rad"]
    for actual, home, span in zip(joints, BALANCED_TELEOP_HOME, BALANCED_TELEOP_JOINT_SPANS, strict=True):
        assert home - span <= actual <= home + span


def test_start_keeps_previous_identity_until_launcher_cleanup():
    assert 'self.state_path.unlink' not in inspect.getsource(SceneRuntimeManager.start)


def test_stop_requires_success_and_supplies_adapter_for_orphan_cleanup():
    source = inspect.getsource(SceneRuntimeManager.stop)
    assert 'check=True' in source
    assert '"-AdapterRoot"' in source
    assert 'timeout=90' in source


@pytest.mark.parametrize("old_phase", ["stopped", "completed", "failed"])
def test_start_reports_new_instance_without_overwriting_cleanup_state(tmp_path, monkeypatch, old_phase):
    script = tmp_path / "scripts" / "start_scene_instance.ps1"
    script.parent.mkdir()
    script.touch()
    launch = SceneLaunchConfig(
        project_root=tmp_path, adapter_root=tmp_path,
        model_root=Path(__file__).resolve().parents[1] / "model/SARM/platform",
        unreal_root=tmp_path, powershell_exe=script, control_port=1, capture_port=2,
        render_port=3, pixel_streamer_port=4, pixel_streaming_id="test",
        pixel_streaming_camera_ids=(), pixel_streaming_camera_width=64,
        pixel_streaming_camera_height=64, preview_rate=30, renderer_ready_timeout=30,
        ik_rate=120, simulation_rate=1, capture_rate=30, default_dataset_capture=False,
    )
    manager = SceneRuntimeManager(launch)
    old_state = {"phase": old_phase, "instance": {"instance_id": "old"},
                 "renderer_pid": 123, "error": "old error"}
    manager.state_path.write_text(json.dumps(old_state), encoding="utf-8")
    process = Mock(pid=456, returncode=None)
    process.poll.return_value = None
    monkeypatch.setattr("space_arm_platform.scene_runtime.subprocess.Popen", lambda *args, **kwargs: process)
    try:
        instance = manager.start(request(7, "none"))
        status = manager.status()
        assert status["phase"] == "launching"
        assert status["active"] is True
        assert status["instance"]["instance_id"] == instance["instance_id"]
        assert "error" not in status
        assert "renderer_pid" not in status
        assert json.loads(manager.state_path.read_text(encoding="utf-8")) == old_state
        manager.create_instance(request(8, "none"))
        assert manager.status()["instance"]["instance_id"] == instance["instance_id"]
        current_state = {"phase": "starting_renderer", "instance": instance, "renderer_pid": 789}
        manager.state_path.write_text(json.dumps(current_state), encoding="utf-8")
        assert manager.status()["phase"] == "starting_renderer"
        assert manager.status()["renderer_pid"] == 789
    finally:
        manager._close_logs()


def test_launcher_failure_before_state_handoff_keeps_new_identity(tmp_path):
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    manager.state_path.write_text(json.dumps({"phase": "stopped", "instance": {"instance_id": "old"}}))
    manager._launched_instance = {"instance_id": "new"}
    manager._process = Mock(pid=456, returncode=1)
    manager._process.poll.return_value = 1
    status = manager.status()
    assert status["phase"] == "failed"
    assert status["active"] is False
    assert status["launcher_exit_code"] == 1
    assert status["instance"]["instance_id"] == "new"
