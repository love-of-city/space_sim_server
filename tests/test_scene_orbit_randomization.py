"""Seeded orbital phase generation, API boundaries and recording compatibility."""
import json
import random
from pathlib import Path

import pytest
from fastapi.testclient import TestClient
from pydantic import ValidationError

from space_arm_platform.app import PlatformConfig, create_app
from space_arm_platform.models import EpisodeStart, EpisodeStop, SceneInstanceCreate
from space_arm_platform.recorder import EpisodeRecorder
from space_arm_platform.scene_runtime import SceneRuntimeManager, _DEFAULT_ORBIT


def test_orbital_randomization_is_opt_in_and_keeps_the_original_start(tmp_path):
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    assert manager.catalog()["defaults"]["randomize_orbit_phase"] is False
    for profile in ("none", "training-v1"):
        for seed in (0, 42, 2**31 - 1):
            instance = manager.create_instance(SceneInstanceCreate(seed=seed, randomization_profile=profile))
            assert instance["randomize_orbit_phase"] is False
            assert instance["environment"]["orbit"] == _DEFAULT_ORBIT
            assert instance["environment"]["orbit"]["true_anomaly_deg"] == 180.0


@pytest.mark.parametrize("profile", ["none", "training-v1"])
def test_same_seed_only_changes_the_orbital_phase_when_enabled(tmp_path, profile):
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    global_rng_state = random.getstate()
    request = SceneInstanceCreate(seed=12345, randomization_profile=profile)
    fixed = manager.create_instance(request)
    enabled = request.model_copy(update={"randomize_orbit_phase": True})
    first = manager.create_instance(enabled)
    second = manager.create_instance(enabled)
    assert random.getstate() == global_rng_state
    assert first["randomize_orbit_phase"] is True
    assert first["randomization"] == second["randomization"] == fixed["randomization"]
    assert first["environment"] == second["environment"]
    assert first["runtime"] == fixed["runtime"]
    angle = first["environment"]["orbit"]["true_anomaly_deg"]
    assert 0.0 <= angle < 360.0
    assert angle != 180.0
    expected = {**fixed["environment"], "orbit": {**_DEFAULT_ORBIT, "true_anomaly_deg": angle}}
    assert first["environment"] == expected
    assert _DEFAULT_ORBIT["true_anomaly_deg"] == 180.0


def test_orbital_phase_is_independent_of_local_profile_and_lighting(tmp_path):
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    angles = []
    for profile in ("none", "training-v1"):
        for light in (0, 1, 12500):
            instance = manager.create_instance(SceneInstanceCreate(
                seed=42, randomization_profile=profile,
                randomize_orbit_phase=True, sunlight_intensity_scale=light,
            ))
            angles.append(instance["environment"]["orbit"]["true_anomaly_deg"])
    assert len(set(angles)) == 1


def test_different_seeds_cover_the_whole_circle(tmp_path):
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    angles = [manager.create_instance(SceneInstanceCreate(
        seed=seed, randomize_orbit_phase=True,
    ))["environment"]["orbit"]["true_anomaly_deg"] for seed in range(64)]
    assert len(set(angles)) == len(angles)
    assert all(0 <= angle < 360 for angle in angles)
    assert {int(angle // 90) for angle in angles} == {0, 1, 2, 3}


def test_empty_seed_generates_and_persists_a_reproducible_start(tmp_path, monkeypatch):
    seeds = iter([12345, 54321])
    monkeypatch.setattr("space_arm_platform.scene_runtime.secrets.randbelow", lambda _: next(seeds))
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    first, second = [manager.create_instance(SceneInstanceCreate(randomize_orbit_phase=True)) for _ in range(2)]
    assert first["seed"] == 12345
    assert second["seed"] == 54321
    assert first["environment"]["orbit"] != second["environment"]["orbit"]
    replay = manager.create_instance(SceneInstanceCreate(seed=first["seed"], randomize_orbit_phase=True))
    assert first["environment"] == replay["environment"]
    assert first["randomization"] == replay["randomization"]
    stored = json.loads(Path(first["config_path"]).read_text(encoding="utf-8"))
    assert stored["randomize_orbit_phase"] is True
    assert stored["environment"]["orbit"] == first["environment"]["orbit"]
    recorder = EpisodeRecorder(tmp_path / "episodes")
    episode = recorder.start(EpisodeStart(scene_instance=first))
    recorder.stop(EpisodeStop())
    metadata = json.loads((tmp_path / "episodes" / episode["episode_id"] / "metadata.json").read_text(encoding="utf-8"))
    assert metadata["scene_instance"]["randomize_orbit_phase"] is True
    assert metadata["scene_instance"]["environment"]["orbit"] == stored["environment"]["orbit"]


@pytest.mark.parametrize("bad", [None, 0, 1, "true", "false", [], {}])
def test_orbit_randomization_flag_requires_a_boolean(bad):
    with pytest.raises(ValidationError):
        SceneInstanceCreate(randomize_orbit_phase=bad)


def test_api_catalog_create_and_start_accept_the_new_option(tmp_path, monkeypatch):
    (tmp_path / "frontend").mkdir()
    (tmp_path / "frontend/index.html").write_text("<!doctype html><title>test</title>", encoding="utf-8")
    # Exercise the start request boundary without launching user processes.
    starts = []
    def start_without_processes(manager, request, creator):
        starts.append(request)
        return manager.create_instance(request, creator)
    monkeypatch.setattr(SceneRuntimeManager, "start", start_without_processes)
    app = create_app(PlatformConfig(project_root=tmp_path, data_root=tmp_path / "episodes", simulation_port=0, capture_port=0))
    with TestClient(app) as client:
        assert client.post("/api/auth/login", json={"username": "admin", "password": "ChangeMe123!"}).status_code == 200
        assert client.get("/api/scenes/catalog").json()["defaults"]["randomize_orbit_phase"] is False
        for endpoint in ("/api/scenes/instances", "/api/scenes/start"):
            for enabled in (False, True):
                response = client.post(endpoint, json={"seed": 42, "randomize_orbit_phase": enabled})
                assert response.status_code == 200
                assert response.json()["randomize_orbit_phase"] is enabled
                angle = response.json()["environment"]["orbit"]["true_anomaly_deg"]
                assert (angle != 180.0) is enabled
            for bad in (None, 0, 1, "false", [], {}):
                assert client.post(endpoint, json={"randomize_orbit_phase": bad}).status_code == 422
        legacy = client.post("/api/scenes/instances", json={"seed": 42}).json()
        assert legacy["randomize_orbit_phase"] is False
        assert legacy["environment"]["orbit"] == _DEFAULT_ORBIT
    assert [request.randomize_orbit_phase for request in starts] == [False, True]
