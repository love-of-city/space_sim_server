"""Create the editable source environment once, using the UE editor Python runtime.

An existing map is never overwritten. Run after building BskUnrealRuntime:
UnrealEditor-Cmd <project> -run=pythonscript -script=<this file> -unattended -NullRHI
"""

from __future__ import annotations

import math

import unreal


MAP_PATH = "/Game/BSK/Maps/BskEnvironment"
EARTH_RADIUS_M = 6_371_000.0
CM_PER_M = 100.0
ASSETS = {
    "sphere": "/Game/_GENERATED/Hyperlovimia/Sphere_732702C4.Sphere_732702C4",
    "earth": "/Game/Planets/Earth/M_Earth.M_Earth",
    "clouds": "/Game/Planets/Earth/M_Clouds.M_Clouds",
    "atmosphere": "/Game/Planets/Earth/M_Atmosphere.M_Atmosphere",
    "stars": "/Game/Planets/Stars/M_Stars.M_Stars",
    "sun_mesh": "/Game/SpaceFX/Meshes/SM_PlanetSphere.SM_PlanetSphere",
    "sun_material": "/Game/SpaceFX/Materials/M_Star.M_Star",
    "sun_bursts": "/Game/SpaceFX/Particles/P_Sun_Bursts.P_Sun_Bursts",
    "sun_halo": "/Game/SpaceFX/Particles/P_Sun_Halo.P_Sun_Halo",
    "sun_lines": "/Game/SpaceFX/Particles/P_Sun_Lines.P_Sun_Lines",
}


def spawn(actor_class, label: str, location=None, rotation=None):
    actor = unreal.get_editor_subsystem(unreal.EditorActorSubsystem).spawn_actor_from_class(
        actor_class, location or unreal.Vector(), rotation or unreal.Rotator(), transient=False
    )
    if not actor:
        raise RuntimeError(f"Could not create environment actor: {label}")
    actor.set_actor_label(label)
    actor.set_folder_path("BSK Environment")
    root = actor.get_editor_property("root_component")
    if root:
        root.set_mobility(unreal.ComponentMobility.MOVABLE)
    return actor


def layer(component, mesh, material, radius_cm: float, celestial: bool = False):
    """Preserve the imported south-pole pivot while putting actor origin at body centre."""
    bounds = mesh.get_bounds()
    source_radius_cm = max(bounds.box_extent.x, bounds.box_extent.y, bounds.box_extent.z)
    if source_radius_cm <= 0.0:
        raise RuntimeError(f"Invalid mesh bounds: {mesh.get_path_name()}")
    scale = radius_cm / source_radius_cm
    component.set_static_mesh(mesh)
    component.set_material(0, material)
    component.set_mobility(unreal.ComponentMobility.MOVABLE)
    component.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
    component.set_cast_shadow(False)
    component.set_relative_scale3d(unreal.Vector(scale, scale, scale))
    component.set_relative_location(
        unreal.Vector(-bounds.origin.x * scale, -bounds.origin.y * scale, -bounds.origin.z * scale),
        sweep=False, teleport=True,
    )
    if celestial:
        component.set_editor_property(
            "lighting_channels", unreal.LightingChannels(channel0=False, channel1=True, channel2=False)
        )
    return scale


def sunlight(label: str, rotation, intensity: float, celestial: bool = False):
    actor = spawn(unreal.DirectionalLight, label, rotation=rotation)
    light = actor.get_component_by_class(unreal.DirectionalLightComponent)
    light.set_intensity(intensity)
    light.set_light_color(unreal.LinearColor(1.0, 1.0, 1.0, 1.0))
    light.set_cast_shadows(not celestial)
    light.set_editor_property("atmosphere_sun_light", not celestial)
    light.set_editor_property("light_source_angle", 0.5357)
    if celestial:
        light.set_editor_property(
            "lighting_channels", unreal.LightingChannels(channel0=False, channel1=True, channel2=False)
        )
        light.set_editor_property("indirect_lighting_intensity", 0.0)
    else:
        light.set_editor_property("atmosphere_sun_light_index", 0)
        light.set_editor_property("per_pixel_atmosphere_transmittance", True)
        light.set_editor_property("forward_shading_priority", 1)
    return actor


def sun_preview_material(parent):
    path = "/Game/BSK/Environment/MI_SunPreview"
    material = unreal.load_asset(path)
    if material:
        return material
    material = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
        "MI_SunPreview", "/Game/BSK/Environment", unreal.MaterialInstanceConstant,
        unreal.MaterialInstanceConstantFactoryNew(),
    )
    if not material:
        raise RuntimeError(f"Could not create {path}")
    unreal.MaterialEditingLibrary.set_material_instance_parent(material, parent)
    unreal.MaterialEditingLibrary.set_material_instance_scalar_parameter_value(material, "EmissiveStrength", 1.5)
    if not unreal.EditorAssetLibrary.save_loaded_asset(material, only_if_is_dirty=False):
        raise RuntimeError(f"Could not save {path}")
    return material


def validate_environment_level():
    actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem).get_all_level_actors()
    rigs = [actor for actor in actors if isinstance(actor, unreal.BskEnvironment)]
    if len(rigs) != 1:
        raise RuntimeError(f"{MAP_PATH} must contain exactly one BskEnvironment, found {len(rigs)}")
    rig = rigs[0]
    for field in (
        "earth", "star_sphere", "sun_visual", "sun_light", "celestial_sun_light", "fill_light",
        "headlight", "exposure_volume", "earth_atmosphere", "preview_camera",
    ):
        if not rig.get_editor_property(field):
            raise RuntimeError(f"Saved environment has a missing binding: {field}")
    earth = rig.get_editor_property("earth")
    sun = rig.get_editor_property("sun_visual")
    stars = rig.get_editor_property("star_sphere")
    for name, component in (
        ("Earth surface", earth.get_editor_property("surface")),
        ("Earth clouds", earth.get_editor_property("clouds")),
        ("Sun surface", sun.get_editor_property("surface")),
        ("Stars", stars.static_mesh_component),
    ):
        if not component.get_editor_property("static_mesh") or not component.get_material(0):
            raise RuntimeError(f"Saved environment has missing mesh or material: {name}")
    if rig.get_editor_property("earth_radius_meters") <= 0.0:
        raise RuntimeError("Saved environment Earth Radius Meters must be positive")
    unreal.log(f"BSK_ENVIRONMENT_VALIDATED: {MAP_PATH}; one rig, bindings, meshes and materials")


def create_environment_level():
    levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    rig_class = unreal.load_class(None, "/Script/BskUnrealRuntime.BskEnvironment")
    body_class = unreal.load_class(None, "/Script/BskUnrealRuntime.BskEnvironmentBody")
    if not rig_class or not body_class:
        raise RuntimeError("Build BskUnrealRuntime before creating the environment level")
    if unreal.EditorAssetLibrary.does_asset_exist(MAP_PATH):
        if not levels.load_level(MAP_PATH):
            raise RuntimeError(f"Could not load existing environment map: {MAP_PATH}")
        validate_environment_level()
        unreal.log(f"BSK_ENVIRONMENT_UNCHANGED: {MAP_PATH}; saved artist edits preserved")
        return

    assets = {name: unreal.load_asset(path) for name, path in ASSETS.items()}
    missing = [ASSETS[name] for name, asset in assets.items() if asset is None]
    if missing:
        raise RuntimeError(f"Environment assets missing; run git lfs pull: {missing}")
    sun_material = sun_preview_material(assets["sun_material"])
    # Unlike LevelEditorSubsystem.new_level, NewBlankMap does not save an empty
    # asset first. A failed Python call therefore cannot leave a valid-looking
    # but incomplete BskEnvironment.umap for later startup checks to accept.
    world = unreal.EditorLoadingAndSavingUtils.new_blank_map(False)
    if not world:
        raise RuntimeError("Could not create unsaved environment world")

    rig = spawn(rig_class, "BSK Environment — ownership and bindings")
    earth_location = unreal.Vector(0.0, 0.0, -(EARTH_RADIUS_M + 500_000.0) * CM_PER_M)
    earth = spawn(body_class, "Earth — saved preview / backend ephemeris", earth_location)
    for name, material, ratio in (
        ("surface", "earth", 1.0), ("clouds", "clouds", 1.001), ("atmosphere", "atmosphere", 1.001)
    ):
        layer(earth.get_editor_property(name), assets["sphere"], assets[material],
              EARTH_RADIUS_M * CM_PER_M * ratio, celestial=True)
    # Use the same physical SkyAtmosphere path as the runtime. The optional
    # material shell is saved but disabled so the atmosphere is not drawn twice.
    earth.get_editor_property("atmosphere").set_visibility(False)
    earth.get_editor_property("atmosphere").set_hidden_in_game(True)
    earth.set_editor_property("tags", ["BSK.Celestial.earth", "BSK.Semantic.earth", "BSK.Environment.Persistent"])

    stars = spawn(unreal.StaticMeshActor, "Stars — saved material and orientation")
    layer(stars.static_mesh_component, assets["sphere"], assets["stars"], 500_000.0 * 100_000.0)
    stars.set_editor_property("tags", ["BSK.Environment.Persistent"])

    sun_rotation = unreal.Rotator(pitch=-25.0, yaw=-35.0, roll=15.0)
    key = sunlight("Sun — local objects and atmosphere", sun_rotation, 132_821.5)
    celestial_key = sunlight("Sun — celestial lighting channel", sun_rotation, 132_821.5, celestial=True)
    fill = sunlight("Fill — artist light (off by default)",
                    unreal.Rotator(pitch=12.5, yaw=130.0, roll=0.0), 0.0)
    fill_light = fill.get_component_by_class(unreal.DirectionalLightComponent)
    fill_light.set_editor_property("atmosphere_sun_light", False)
    fill_light.set_cast_shadows(False)
    fill_light.set_light_color(unreal.LinearColor(0.55, 0.68, 1.0, 1.0))

    ray = key.get_actor_forward_vector()
    sun_position = unreal.Vector(-ray.x * 1_000_000.0, -ray.y * 1_000_000.0, -ray.z * 1_000_000.0)
    sun = spawn(body_class, "Sun visual — saved preview / runtime proxy", sun_position)
    sun_scale = layer(sun.get_editor_property("surface"), assets["sun_mesh"], sun_material,
                      1_000_000.0 * math.tan(math.radians(1.0)))
    for field, asset_key in (("sun_bursts", "sun_bursts"), ("sun_halo", "sun_halo"), ("sun_lines", "sun_lines")):
        particle = sun.get_editor_property(field)
        particle.set_template(assets[asset_key])
        particle.set_editor_property("auto_activate", True)
        particle.set_relative_scale3d(unreal.Vector(sun_scale, sun_scale, sun_scale))
        particle.activate(True)
    sun.set_editor_property("tags", ["BSK.Environment.SunVisual", "BSK.Semantic.sun", "BSK.Environment.Persistent"])

    atmosphere = spawn(unreal.SkyAtmosphere, "Earth atmosphere — planet centre", earth_location)
    atmosphere_component = atmosphere.get_component_by_class(unreal.SkyAtmosphereComponent)
    atmosphere_component.set_editor_property("transform_mode", unreal.SkyAtmosphereTransformMode.PLANET_CENTER_AT_COMPONENT_TRANSFORM)
    atmosphere_component.set_bottom_radius(EARTH_RADIUS_M / 1000.0)
    atmosphere_component.set_atmosphere_height(100.0)
    atmosphere_component.set_multi_scattering_factor(1.0)

    exposure = spawn(unreal.PostProcessVolume, "Exposure — saved artist settings")
    exposure.set_editor_property("unbound", True)
    exposure.set_editor_property("priority", 1000.0)
    settings = exposure.get_editor_property("settings")
    for name, value in {
        "override_auto_exposure_method": True,
        "auto_exposure_method": unreal.AutoExposureMethod.AEM_MANUAL,
        "override_auto_exposure_apply_physical_camera_exposure": True,
        "auto_exposure_apply_physical_camera_exposure": True,
        "override_camera_iso": True, "camera_iso": 100.0,
        "override_camera_shutter_speed": True, "camera_shutter_speed": 125.0,
        "override_depth_of_field_fstop": True, "depth_of_field_fstop": 16.0,
        "override_auto_exposure_bias": True, "auto_exposure_bias": 0.0,
    }.items():
        settings.set_editor_property(name, value)
    exposure.set_editor_property("settings", settings)

    headlight = spawn(unreal.SpotLight, "Headlight — reserved for backend model lighting")
    headlight.get_component_by_class(unreal.SpotLightComponent).set_intensity(0.0)
    headlight.set_actor_hidden_in_game(True)

    view_rotation = unreal.Rotator(pitch=0.0, yaw=145.0, roll=0.0)
    camera = spawn(unreal.CameraActor, "Preview camera — edit / pilot and save", rotation=view_rotation)
    camera.camera_component.set_field_of_view(90.0)
    camera.camera_component.set_editor_property("constrain_aspect_ratio", False)

    for name, actor in {
        "earth": earth, "star_sphere": stars, "sun_visual": sun, "sun_light": key,
        "celestial_sun_light": celestial_key, "fill_light": fill, "headlight": headlight,
        "exposure_volume": exposure, "earth_atmosphere": atmosphere, "preview_camera": camera,
    }.items():
        rig.set_editor_property(name, actor)
    rig.set_editor_property("earth_radius_meters", EARTH_RADIUS_M)
    unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).set_level_viewport_camera_info(unreal.Vector(), view_rotation)
    # Commandlets have no active viewport. The native authoring action also
    # stores World.EditorViews, which is not exposed through editor Python.
    rig.store_editor_preview_view()
    validate_environment_level()
    if not unreal.EditorLoadingAndSavingUtils.save_map(world, MAP_PATH):
        raise RuntimeError(f"Could not save {MAP_PATH}")
    unreal.log(f"BSK_ENVIRONMENT_CREATED: {MAP_PATH}; editable actors and preview camera saved")


if __name__ == "__main__":
    create_environment_level()
