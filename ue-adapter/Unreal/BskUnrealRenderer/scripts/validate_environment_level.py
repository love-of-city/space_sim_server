"""Prove editor save/reload persistence in a disposable copy of the source map.

The source map and shared materials are never saved. Uses the UE editor Python
runtime, with the same -run=pythonscript invocation as the generation script.
"""

from __future__ import annotations

import hashlib
from pathlib import Path

import unreal


SOURCE_MAP = "/Game/BSK/Maps/BskEnvironment"
TEST_MAP = "/Game/Developers/__BskEnvironmentValidation"


def rig_in_current_level():
    actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem).get_all_level_actors()
    rigs = [actor for actor in actors if isinstance(actor, unreal.BskEnvironment)]
    if len(rigs) != 1:
        raise AssertionError(f"Expected exactly one environment rig, found {len(rigs)}")
    return rigs[0]


def vector_values(value):
    return (value.x, value.y, value.z)


def rotation_values(value):
    return (value.pitch, value.yaw, value.roll)


def close(actual, expected, label):
    if len(actual) != len(expected) or any(abs(a - b) > 0.001 for a, b in zip(actual, expected)):
        raise AssertionError(f"{label} did not persist: {actual} != {expected}")


def snapshot(rig):
    earth = rig.get_editor_property("earth")
    sun = rig.get_editor_property("sun_visual")
    stars = rig.get_editor_property("star_sphere")
    camera = rig.get_editor_property("preview_camera")
    light = rig.get_editor_property("sun_light").get_component_by_class(unreal.DirectionalLightComponent)
    return {
        "earth_location": vector_values(earth.get_actor_location()),
        "earth_rotation": rotation_values(earth.get_actor_rotation()),
        "earth_scale": vector_values(earth.get_actor_scale3d()),
        "earth_surface_material": earth.get_editor_property("surface").get_material(0).get_path_name(),
        "sun_location": vector_values(sun.get_actor_location()),
        "stars_rotation": rotation_values(stars.get_actor_rotation()),
        "camera_location": vector_values(camera.get_actor_location()),
        "camera_rotation": rotation_values(camera.get_actor_rotation()),
        "camera_fov": camera.camera_component.get_editor_property("field_of_view"),
        "exposure_bias": rig.get_editor_property("exposure_volume").get_editor_property("settings").get_editor_property("auto_exposure_bias"),
        "sun_intensity": light.get_editor_property("intensity"),
    }


def validate_persistence():
    # Duplicating a UWorld in an interactive editor can leave a standalone
    # package alive across LoadLevel and trigger UE's world-leak assertion.
    # This destructive lifecycle test belongs in its own commandlet process.
    if '-run=pythonscript' not in unreal.SystemLibrary.get_command_line().lower():
        raise RuntimeError('Run this disposable-map test in an isolated -run=pythonscript commandlet, not a live editor')
    if not unreal.EditorAssetLibrary.does_asset_exist(SOURCE_MAP):
        raise RuntimeError(f"Generate the source environment first: {SOURCE_MAP}")
    if unreal.EditorAssetLibrary.does_asset_exist(TEST_MAP):
        raise RuntimeError(f"Validation refuses to overwrite an existing asset: {TEST_MAP}")
    source_file = Path(unreal.Paths.project_content_dir()) / "BSK/Maps/BskEnvironment.umap"
    before_hash = hashlib.sha256(source_file.read_bytes()).hexdigest()
    levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    created = False
    try:
        # Nothing is ever changed or saved in the source world. Duplicate first;
        # all test edits below take place in a distinct package.
        created = True
        if not unreal.EditorAssetLibrary.duplicate_asset(SOURCE_MAP, TEST_MAP):
            raise RuntimeError("Could not duplicate the environment map")
        if not levels.load_level(TEST_MAP):
            raise RuntimeError("Could not load the disposable environment map")
        rig = rig_in_current_level()
        earth = rig.get_editor_property("earth")
        current = earth.get_actor_location()
        earth.set_actor_location(unreal.Vector(current.x + 12345.0, current.y - 23456.0, current.z + 34567.0),
                                 sweep=False, teleport=True)
        earth.set_actor_rotation(unreal.Rotator(pitch=7.0, yaw=21.0, roll=13.0), teleport_physics=True)
        earth.set_actor_scale3d(unreal.Vector(1.1, 1.2, 1.3))
        # Swap a component override in this copy, without editing the shared
        # material asset. This also verifies that reconstruction does not reset it.
        clouds_material = earth.get_editor_property("clouds").get_material(0)
        earth.get_editor_property("surface").set_material(0, clouds_material)
        sun = rig.get_editor_property("sun_visual")
        current = sun.get_actor_location()
        sun.set_actor_location(unreal.Vector(current.x + 54321.0, current.y + 12345.0, current.z - 3210.0),
                               sweep=False, teleport=True)
        rig.get_editor_property("star_sphere").set_actor_rotation(
            unreal.Rotator(pitch=3.0, yaw=22.0, roll=7.0), teleport_physics=True)
        camera = rig.get_editor_property("preview_camera")
        camera.set_actor_location(unreal.Vector(321.0, -456.0, 789.0), sweep=False, teleport=True)
        camera.set_actor_rotation(unreal.Rotator(pitch=-11.0, yaw=27.0, roll=0.0), teleport_physics=True)
        camera.camera_component.set_field_of_view(73.0)
        volume = rig.get_editor_property("exposure_volume")
        settings = volume.get_editor_property("settings")
        settings.set_editor_property("override_auto_exposure_bias", True)
        settings.set_editor_property("auto_exposure_bias", 1.25)
        volume.set_editor_property("settings", settings)
        rig.get_editor_property("sun_light").get_component_by_class(unreal.DirectionalLightComponent).set_intensity(123456.0)
        expected = snapshot(rig)
        if not levels.save_current_level():
            raise RuntimeError("Could not save the disposable environment map")
        if not levels.load_level("/Engine/Maps/Entry") or not levels.load_level(TEST_MAP):
            raise RuntimeError("Could not unload and reload the disposable environment map")
        actual = snapshot(rig_in_current_level())
        for label, expected_value in expected.items():
            actual_value = actual[label]
            if isinstance(expected_value, tuple):
                close(actual_value, expected_value, label)
            elif isinstance(expected_value, str):
                if actual_value != expected_value:
                    raise AssertionError(f"{label} did not persist: {actual_value} != {expected_value}")
            elif abs(actual_value - expected_value) > 0.001:
                raise AssertionError(f"{label} did not persist: {actual_value} != {expected_value}")
    finally:
        if created and unreal.EditorAssetLibrary.does_asset_exist(TEST_MAP):
            levels.load_level("/Engine/Maps/Entry")
            if not unreal.EditorAssetLibrary.delete_asset(TEST_MAP):
                raise RuntimeError(f"Could not delete this validation run's temporary map: {TEST_MAP}")
        after_hash = hashlib.sha256(source_file.read_bytes()).hexdigest()
        if before_hash != after_hash:
            raise AssertionError("Source environment package changed during disposable-copy validation")
    unreal.log("BSK_ENVIRONMENT_PERSISTENCE_OK: actor transforms, layer material, lighting, exposure and camera survive save/reload; source unchanged")


if __name__ == "__main__":
    validate_persistence()
