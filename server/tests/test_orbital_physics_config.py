"""Persisted environment settings and safe replay of pre-J2 scenes."""
import json

import pytest

from space_arm_platform.models import SceneInstanceCreate
from space_arm_platform.orbital_physics import OrbitalPhysics, ExponentialDragSettings
from space_arm_platform.scene_runtime import SceneRuntimeManager


def drag_parameters():
    return dict(reference_altitude_m=500000.0, reference_density_kg_m3=1e-12,
                scale_height_m=60000.0, density_source="synthetic test atmosphere",
                bodies=[dict(body="cubesat_bus", area_m2=0.06, drag_coefficient=2.2,
                             parameter_source="synthetic test body")])


def test_new_scene_persists_j2_and_old_scene_replays_point_mass(tmp_path):
    pytest.importorskip("Basilisk")
    from simulation.teleop_grasp_unreal import _load_scene_instance
    manager = SceneRuntimeManager(None, project_root=tmp_path)
    instance = manager.create_instance(SceneInstanceCreate(seed=123))
    path = tmp_path / "scene.json"
    path.write_text(json.dumps(instance), encoding="utf-8")
    loaded = _load_scene_instance(path)
    assert loaded["environment"]["physics"] == OrbitalPhysics().model_dump()
    del instance["environment"]["physics"]
    path.write_text(json.dumps(instance), encoding="utf-8")
    assert _load_scene_instance(path)["environment"]["physics"]["earth_gravity"] == "point_mass"


def test_explicit_point_mass_and_drag_settings_survive_scene_creation(tmp_path):
    settings = OrbitalPhysics(earth_gravity="point_mass", drag=drag_parameters())
    instance = SceneRuntimeManager(None, project_root=tmp_path).create_instance(
        SceneInstanceCreate(seed=123, orbital_physics=settings))
    assert instance["environment"]["physics"] == settings.model_dump()


@pytest.mark.parametrize("patch", [
    {"reference_density_kg_m3": -1.0}, {"reference_density_kg_m3": float("nan")},
    {"scale_height_m": 0.0}, {"scale_height_m": 1.0}, {"density_source": " "},
    {"bodies": []}, {"density_model": "unknown"}, {"unexpected": 1},
])
def test_invalid_or_untraceable_atmosphere_is_rejected(patch):
    with pytest.raises(ValueError):
        ExponentialDragSettings.model_validate(drag_parameters() | patch)


def test_duplicate_body_area_is_rejected():
    data = drag_parameters()
    data["bodies"] *= 2
    with pytest.raises(ValueError, match="unique"):
        ExponentialDragSettings.model_validate(data)


def test_unknown_gravity_model_and_nonfinite_coefficient_are_rejected():
    for data in ({"earth_gravity": "j6"}, {"j2_coefficient": float("inf")},
                 {"j2_coefficient": True}, {"coefficient_source": " "}):
        with pytest.raises(ValueError):
            OrbitalPhysics.model_validate(data)


def test_override_is_explicit_and_incompatible_backends_fail(tmp_path):
    from simulation.orbital_configuration import resolve_orbital_physics
    old_scene = {"environment": {}}
    assert resolve_orbital_physics(old_scene).earth_gravity == "point_mass"
    path = tmp_path / "physics.json"
    path.write_text(OrbitalPhysics().model_dump_json(), encoding="utf-8")
    assert resolve_orbital_physics(old_scene, override=path).earth_gravity == "j2"
    for kwargs in ({"backend": "basilisk"}, {"mode": "linear_tidal"}):
        with pytest.raises(ValueError, match="requires local"):
            resolve_orbital_physics(old_scene, override=path, **kwargs)
    with pytest.raises(ValueError, match="explicit density"):
        resolve_orbital_physics(old_scene, require_drag=True)
