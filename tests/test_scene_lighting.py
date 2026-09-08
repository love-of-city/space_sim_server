"""Creation, persistence, compatibility and render-only solar light settings."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient
from pydantic import ValidationError

from space_arm_platform.app import PlatformConfig, create_app
from space_arm_platform.models import SceneInstanceCreate, EpisodeStart, EpisodeStop
from space_arm_platform.recorder import EpisodeRecorder
from space_arm_platform.scene_runtime import SceneRuntimeManager
from simulation.teleop_grasp_unreal import _load_scene_instance, _register_celestial_bodies


@pytest.mark.parametrize('scale', [0, .5, 1, 2.5, 12_500, 20_000])
def test_scene_lighting_persists_without_changing_physics(tmp_path, scale):
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    baseline = manager.create_instance(SceneInstanceCreate(seed=123))
    instance = manager.create_instance(SceneInstanceCreate(seed=123, sunlight_intensity_scale=scale))
    assert instance['randomization'] == baseline['randomization']
    baseline_environment = baseline['environment'].copy()
    actual_environment = instance['environment'].copy()
    baseline_environment.pop('lighting')
    assert actual_environment.pop('lighting') == {'sunlight_intensity_scale': scale}
    assert actual_environment == baseline_environment
    stored = json.loads(Path(instance['config_path']).read_text(encoding='utf-8'))
    loaded = _load_scene_instance(Path(instance['config_path']))
    assert stored['environment']['lighting'] == loaded['environment']['lighting'] == {'sunlight_intensity_scale': scale}
    recorder = EpisodeRecorder(tmp_path / 'episodes')
    episode = recorder.start(EpisodeStart(scene_instance=instance))
    recorder.stop(EpisodeStop())
    metadata = json.loads((tmp_path/'episodes'/episode['episode_id']/'metadata.json').read_text(encoding='utf-8'))
    assert metadata['scene_instance']['environment']['lighting']['sunlight_intensity_scale'] == scale


@pytest.mark.parametrize('bad', [-.1, 20_000.1, float('nan'), float('inf'), True, '2', None])
def test_invalid_api_and_hand_edited_scene_values_are_rejected(tmp_path, bad):
    with pytest.raises(ValidationError):
        SceneInstanceCreate(sunlight_intensity_scale=bad)
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    instance = manager.create_instance(SceneInstanceCreate(seed=7))
    instance['environment']['lighting']['sunlight_intensity_scale'] = bad
    path = tmp_path/'invalid.json'
    path.write_text(json.dumps(instance), encoding='utf-8')
    with pytest.raises(ValueError, match='sunlight_intensity_scale'):
        _load_scene_instance(path)


def test_old_scene_defaults_to_one_and_malformed_lighting_is_rejected(tmp_path):
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    instance = manager.create_instance(SceneInstanceCreate())
    del instance['environment']['lighting']
    path = tmp_path/'legacy.json'
    path.write_text(json.dumps(instance), encoding='utf-8')
    assert _load_scene_instance(path)['environment']['lighting']['sunlight_intensity_scale'] == 1
    del instance['environment']
    path.write_text(json.dumps(instance), encoding='utf-8')
    assert _load_scene_instance(path)['environment']['lighting']['sunlight_intensity_scale'] == 1
    for malformed in (None, [], 1, True):
        instance['environment'] = {'lighting': malformed}
        path.write_text(json.dumps(instance), encoding='utf-8')
        with pytest.raises(ValueError, match='lighting must be an object'):
            _load_scene_instance(path)


def test_scene_api_exposes_default_and_accepts_zero(tmp_path):
    (tmp_path/"frontend").mkdir()
    (tmp_path/"frontend/index.html").write_text("<!doctype html><title>Test</title>", encoding="utf-8")
    app = create_app(PlatformConfig(project_root=tmp_path, data_root=tmp_path/'episodes', simulation_port=0, capture_port=0))
    with TestClient(app) as client:
        assert client.post('/api/auth/login', json={'username':'admin','password':'ChangeMe123!'}).status_code == 200
        assert client.get('/api/scenes/catalog').json()['defaults']['sunlight_intensity_scale'] == 1
        for scale in (0, .5, 2, 12_500, 20_000):
            result = client.post('/api/scenes/instances', json={'seed':5,'sunlight_intensity_scale':scale})
            assert result.status_code == 200
            assert result.json()['environment']['lighting']['sunlight_intensity_scale'] == scale
        for bad in (-1, 20_001, '2', True, None):
            assert client.post('/api/scenes/instances', json={'sunlight_intensity_scale':bad}).status_code == 422
            assert client.post('/api/scenes/start', json={'sunlight_intensity_scale':bad}).status_code == 422


def test_reference_solar_illumination_and_body_readers_are_unchanged():
    calls = []
    earth, sun = object(), object()
    bridge = SimpleNamespace(add_celestial_bodies=lambda bodies, **kwargs: calls.append((bodies,kwargs)))
    _register_celestial_bodies(bridge, earth, sun)
    assert calls[0][0] == [earth, sun]
    assert calls[0][1]['visual_overrides']['sun']['light_illuminance_lux_at_reference_distance'] == 8
