"""Build the editable Earth surface/cloud shaders in the UE 5.6 editor.

Running this file builds/reconnects graphs without saving. After visual review,
call install_instances() to save only these assets and bind the clean source map.
Existing day/night maps, astronomical transforms and backend ownership are kept.
"""

from __future__ import annotations

import json

import unreal


EARTH = "/Game/Planets/Earth/M_Earth"
CLOUDS = "/Game/Planets/Earth/M_Clouds"
DENSITY = "/Game/Planets/Earth/8k_earth_clouds"
MAP = "/Game/BSK/Maps/BskEnvironment"
SURFACE_INSTANCE = "/Game/BSK/Environment/MI_EarthPreview"
CLOUD_INSTANCE = "/Game/BSK/Environment/MI_EarthCloudsPreview"
TAG = "BSK Earth Realism: "
ML = unreal.MaterialEditingLibrary

SURFACE_DEFAULTS = {
    "LandNormalStrength": 0.25,
    "OceanNormalStrength": 0.0,
    "OceanAlbedoScale": 0.65,
    "CloudHeightRatio": 0.001,
    "CloudOpticalDepth": 1.6,
    "CloudDensityPower": 1.35,
    "CloudShadowStrength": 0.65,
}
CLOUD_DEFAULTS = {
    "CloudOpticalDepth": 1.6,
    "CloudDensityPower": 1.35,
    # Tuned toward the denser, higher-contrast cloud appearance of the
    # reference NASA imagery while keeping the surface anti-plastic fixes.
    "CloudOpacityScale": 0.88,
    "CloudSunLuminance": 28000.0,
    "CloudNightLuminance": 1.0,
}

NORMAL_CODE = """
float strength = lerp(saturate(LandStrength), saturate(OceanStrength), saturate(OceanMask));
return normalize(lerp(float3(0,0,1), SourceNormal, strength));
"""

# Intersect a ray from the unit ground sphere with the cloud sphere. Geometric
# tangents follow the mesh's UVs, so no absolute position, actor pivot, fixed
# world longitude or backend material parameter update is needed.
SHADOW_UV_CODE = """
float3 S = normalize(SunTS);
float c = S.z;
if (c <= 0.0) return UV;
float k = max(HeightRatio, 0.000001);
float q = 2.0*k + k*k;
float t = q / (sqrt(c*c + q) + c);
float a = (1.0 + t*c) / (1.0 + k);
float b = t*S.x / (1.0 + k);
float d = t*S.y / (1.0 + k);
float lat = 3.141592653589793 * (0.5 - UV.y);
float z = a*sin(lat) - d*cos(lat);
float horizontal = a*cos(lat) + d*sin(lat);
float lonOffset = atan2(b, horizontal);
return float2(UV.x + lonOffset / 6.283185307179586,
              atan2(length(float2(horizontal, b)), z) / 3.141592653589793);
"""

# This is a low-cost direct-light approximation, not a shadow-map replacement:
# attenuating Specular does not remove the grazing Fresnel term completely.
# The night-emission chain must NOT use this solar-projected transmittance.
SURFACE_CODE = """
float density = pow(saturate(CloudDensity), max(DensityPower, 0.05));
float optical = max(OpticalDepth, 0.0) * density;
float sunlightTransmission = exp(-optical / max(SunCos, 0.15));
ShadowTransmission = (SunCos > 0.0)
    ? lerp(1.0, sunlightTransmission, saturate(ShadowStrength)) : 1.0;
float albedoScale = lerp(1.0, max(OceanAlbedo, 0.0), saturate(OceanMask));
return DayColor * albedoScale * ShadowTransmission;
"""

# Physical luminance units keep cloud brightness consistent with the saved
# manual exposure. Unlit avoids the two directional lights adding transparent
# lighting. Density/view path length control opacity even on the night side.
CLOUD_CODE = """
float density = pow(saturate(Density), max(DensityPower, 0.05));
float optical = max(OpticalDepth, 0.0) * density;
float viewPath = max(abs(ViewCos), 0.15);
CloudOpacity = saturate((1.0 - exp(-optical / viewPath)) * OpacityScale);
float thicknessLight = lerp(0.75, 1.0, 1.0 - exp(-optical));
float luminance = max(NightLuminance, 0.0)
    + max(SunLuminance, 0.0) * saturate(SunCos) * thicknessLight;
return max(Tint, 0.0) * luminance;
"""


def nodes(material):
    return [o for o in unreal.ObjectIterator(unreal.MaterialExpression)
            if o.get_outer() == material]


def expression(material, cls, label, x, y):
    for node in nodes(material):
        if str(node.get_editor_property("desc")) == TAG + label:
            return node
        if isinstance(node, unreal.MaterialExpressionCustom):
            if str(node.get_editor_property("description")) == TAG + label:
                return node
    node = ML.create_material_expression(material, cls, x, y)
    node.set_editor_property("desc", TAG + label)
    return node


def scalar(material, name, default, group, x, y, maximum=1.0):
    for node in nodes(material):
        if isinstance(node, unreal.MaterialExpressionScalarParameter):
            if str(node.get_editor_property("parameter_name")) == name:
                return node
    node = expression(material, unreal.MaterialExpressionScalarParameter, name, x, y)
    node.set_editor_property("parameter_name", name)
    node.set_editor_property("default_value", default)
    node.set_editor_property("group", group)
    node.set_editor_property("slider_min", 0.0)
    node.set_editor_property("slider_max", maximum)
    return node


def custom(material, label, code, inputs, output_type, x, y, outputs=()):
    node = expression(material, unreal.MaterialExpressionCustom, label, x, y)
    node.set_editor_property("description", TAG + label)
    custom_inputs = []
    for name in inputs:
        value = unreal.CustomInput()
        value.set_editor_property("input_name", name)
        custom_inputs.append(value)
    custom_outputs = []
    for name, kind in outputs:
        value = unreal.CustomOutput()
        value.set_editor_property("output_name", name)
        value.set_editor_property("output_type", kind)
        custom_outputs.append(value)
    # Replacing an existing input array disconnects its wires and can trigger
    # intermediate missing-input compiles. Preserve it when the contract matches.
    old_inputs = [str(v.get_editor_property("input_name"))
                  for v in node.get_editor_property("inputs")]
    if old_inputs != list(inputs):
        node.set_editor_property("inputs", custom_inputs)
    old_outputs = [(str(v.get_editor_property("output_name")), v.get_editor_property("output_type"))
                   for v in node.get_editor_property("additional_outputs")]
    if old_outputs != list(outputs):
        node.set_editor_property("additional_outputs", custom_outputs)
    node.set_editor_property("output_type", output_type)
    node.set_editor_property("code", code)
    return node


def wire(source, output, target, pin):
    if not ML.connect_material_expressions(source, output, target, pin):
        raise RuntimeError(f"Cannot connect {source.get_name()}.{output} -> {target.get_name()}.{pin}")


def main_input(node, output, prop):
    if not ML.connect_material_property(node, output, prop):
        raise RuntimeError(f"Cannot connect material property {prop}")


def apply_material_graphs():
    earth = unreal.load_asset(EARTH)
    clouds = unreal.load_asset(CLOUDS)
    density = unreal.load_asset(DENSITY)
    if not all((earth, clouds, density)):
        raise RuntimeError("Missing Earth assets; run git lfs pull first")
    night = ML.get_material_default_scalar_parameter_value(earth, "NightLightIntensity")
    if night <= 0:
        raise RuntimeError("Expected the existing night-light parameter")

    density.set_editor_property("srgb", False)
    density.set_editor_property("compression_settings", unreal.TextureCompressionSettings.TC_MASKS)
    params = {name: scalar(earth, name, value, "Earth Surface" if name in
              ("LandNormalStrength", "OceanNormalStrength", "OceanAlbedoScale") else "Cloud Shadows",
              -1200, 1400 + i*140, 4.0 if name in ("CloudOpticalDepth", "CloudDensityPower") else 1.0)
              for i, (name, value) in enumerate(SURFACE_DEFAULTS.items())}

    normal = custom(earth, "Normal", NORMAL_CODE,
                    ["SourceNormal", "OceanMask", "LandStrength", "OceanStrength"],
                    unreal.CustomMaterialOutputType.CMOT_FLOAT3, -64, 512)
    wire(unreal.find_object(earth, "MaterialExpressionTextureSample_2"), "RGB", normal, "SourceNormal")
    mask = unreal.find_object(earth, "MaterialExpressionTextureSample_3")
    wire(mask, "R", normal, "OceanMask")
    wire(params["LandNormalStrength"], "", normal, "LandStrength")
    wire(params["OceanNormalStrength"], "", normal, "OceanStrength")
    main_input(normal, "", unreal.MaterialProperty.MP_NORMAL)

    sun = unreal.find_object(earth, "MaterialExpressionSkyAtmosphereLightDirection_0")
    tangent_sun = expression(earth, unreal.MaterialExpressionTransform, "Sun in geometric tangent frame", -1600, -600)
    tangent_sun.set_editor_property("transform_source_type", unreal.MaterialVectorCoordTransformSource.TRANSFORMSOURCE_WORLD)
    tangent_sun.set_editor_property("transform_type", unreal.MaterialVectorCoordTransform.TRANSFORM_TANGENT)
    wire(sun, "", tangent_sun, "")
    uv = expression(earth, unreal.MaterialExpressionTextureCoordinate, "Cloud UV", -1600, -840)
    shadow_uv = custom(earth, "Spherical cloud projection", SHADOW_UV_CODE,
                       ["UV", "SunTS", "HeightRatio"], unreal.CustomMaterialOutputType.CMOT_FLOAT2, -1000, -600)
    wire(uv, "", shadow_uv, "UV")
    wire(tangent_sun, "", shadow_uv, "SunTS")
    wire(params["CloudHeightRatio"], "", shadow_uv, "HeightRatio")
    sample = expression(earth, unreal.MaterialExpressionTextureSample, "Projected cloud density", -640, -600)
    sample.set_editor_property("texture", density)
    sample.set_editor_property("sampler_type", unreal.MaterialSamplerType.SAMPLERTYPE_MASKS)
    wire(shadow_uv, "", sample, "UVs")

    surface = custom(earth, "Surface albedo and solar cloud attenuation", SURFACE_CODE,
                     ["DayColor", "OceanMask", "OceanAlbedo", "CloudDensity", "SunCos",
                      "OpticalDepth", "DensityPower", "ShadowStrength"],
                     unreal.CustomMaterialOutputType.CMOT_FLOAT3, 800, -320,
                     [("ShadowTransmission", unreal.CustomMaterialOutputType.CMOT_FLOAT1)])
    for source, output, pin in [
        (unreal.find_object(earth, "MaterialExpressionTextureSample_0"), "RGB", "DayColor"),
        (mask, "R", "OceanMask"), (params["OceanAlbedoScale"], "", "OceanAlbedo"),
        (sample, "R", "CloudDensity"),
        (unreal.find_object(earth, "MaterialExpressionDotProduct_0"), "", "SunCos"),
        (params["CloudOpticalDepth"], "", "OpticalDepth"),
        (params["CloudDensityPower"], "", "DensityPower"),
        (params["CloudShadowStrength"], "", "ShadowStrength"),
    ]:
        wire(source, output, surface, pin)
    main_input(surface, "", unreal.MaterialProperty.MP_BASE_COLOR)
    specular = expression(earth, unreal.MaterialExpressionMultiply, "Cloud attenuation of specular", 1150, -480)
    wire(unreal.find_object(earth, "MaterialExpressionLinearInterpolate_1"), "", specular, "A")
    wire(surface, "ShadowTransmission", specular, "B")
    main_input(specular, "", unreal.MaterialProperty.MP_SPECULAR)

    clouds.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_UNLIT)
    cloud_sample = unreal.find_object(clouds, "MaterialExpressionTextureSample_0")
    cloud_sample.set_editor_property("sampler_type", unreal.MaterialSamplerType.SAMPLERTYPE_MASKS)
    cloud_params = {name: scalar(clouds, name, value, "Cloud Appearance", -800, 1000+i*140,
                    60000.0 if "Luminance" in name else 4.0)
                    for i, (name, value) in enumerate(CLOUD_DEFAULTS.items())}
    tint = next((n for n in nodes(clouds) if isinstance(n, unreal.MaterialExpressionVectorParameter)
                 and str(n.get_editor_property("parameter_name")) == "CloudTint"), None)
    if tint is None:
        tint = expression(clouds, unreal.MaterialExpressionVectorParameter, "CloudTint", -800, 800)
        tint.set_editor_property("parameter_name", "CloudTint")
        tint.set_editor_property("default_value", unreal.LinearColor(1, 1, 1, 1))
        tint.set_editor_property("group", "Cloud Appearance")
    view = expression(clouds, unreal.MaterialExpressionCameraVectorWS, "View direction", -1200, 480)
    view_cos = expression(clouds, unreal.MaterialExpressionDotProduct, "Cloud view path", -800, 480)
    wire(view, "", view_cos, "A")
    wire(unreal.find_object(clouds, "MaterialExpressionVertexNormalWS_0"), "", view_cos, "B")
    lighting = custom(clouds, "Density and explicit sunlight", CLOUD_CODE,
                      ["Density", "SunCos", "ViewCos", "OpticalDepth", "DensityPower", "OpacityScale",
                       "SunLuminance", "NightLuminance", "Tint"],
                      unreal.CustomMaterialOutputType.CMOT_FLOAT3, 320, 320,
                      [("CloudOpacity", unreal.CustomMaterialOutputType.CMOT_FLOAT1)])
    for source, output, pin in [
        (cloud_sample, "R", "Density"),
        (unreal.find_object(clouds, "MaterialExpressionDotProduct_0"), "", "SunCos"),
        (view_cos, "", "ViewCos"), (cloud_params["CloudOpticalDepth"], "", "OpticalDepth"),
        (cloud_params["CloudDensityPower"], "", "DensityPower"),
        (cloud_params["CloudOpacityScale"], "", "OpacityScale"),
        (cloud_params["CloudSunLuminance"], "", "SunLuminance"),
        (cloud_params["CloudNightLuminance"], "", "NightLuminance"), (tint, "", "Tint"),
    ]:
        wire(source, output, lighting, pin)
    main_input(lighting, "", unreal.MaterialProperty.MP_EMISSIVE_COLOR)
    main_input(lighting, "CloudOpacity", unreal.MaterialProperty.MP_OPACITY)

    ML.recompile_material(earth)
    ML.recompile_material(clouds)
    assert ML.get_material_default_scalar_parameter_value(earth, "NightLightIntensity") == night
    return {"saved": False, "surface": EARTH, "clouds": CLOUDS, "nightLightIntensity": night,
            "surfaceParameters": {n: ML.get_material_default_scalar_parameter_value(earth, n)
                                  for n in SURFACE_DEFAULTS},
            "cloudParameters": {n: ML.get_material_default_scalar_parameter_value(clouds, n)
                                for n in CLOUD_DEFAULTS}}


def install_instances():
    """Save reviewed graphs and bind instances only when the source map is clean."""
    if unreal.EditorLoadingAndSavingUtils.get_dirty_map_packages():
        raise RuntimeError("Save/review existing map edits before installing Earth instances")
    world = unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
    if world.get_outermost().get_path_name() != MAP:
        raise RuntimeError("Open BskEnvironment before installing Earth instances")
    actors = [a for a in unreal.get_editor_subsystem(unreal.EditorActorSubsystem).get_all_level_actors()
              if "BSK.Celestial.earth" in [str(t) for t in a.tags]]
    if len(actors) != 1:
        raise RuntimeError("Expected exactly one saved Earth actor")
    earth_actor = actors[0]
    assets = [unreal.load_asset(p) for p in (EARTH, CLOUDS, DENSITY)]
    if not all(assets):
        raise RuntimeError("Missing Earth assets")
    # Validate existing instances before rebinding any component. Artist
    # overrides are preserved; an unrelated asset at these paths is an error.
    for path, parent in [(SURFACE_INSTANCE, assets[0]), (CLOUD_INSTANCE, assets[1])]:
        existing = unreal.load_asset(path) if unreal.EditorAssetLibrary.does_asset_exist(path) else None
        if existing and (not isinstance(existing, unreal.MaterialInstanceConstant)
                         or existing.get_editor_property("parent") != parent):
            raise RuntimeError(f"Unrelated existing material instance: {path}")
    for instance_path, parent, component in [(SURFACE_INSTANCE, assets[0], "surface"),
                                              (CLOUD_INSTANCE, assets[1], "clouds")]:
        instance = (unreal.load_asset(instance_path)
                    if unreal.EditorAssetLibrary.does_asset_exist(instance_path) else None)
        if not instance:
            folder, name = instance_path.rsplit("/", 1)
            instance = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
                name, folder, unreal.MaterialInstanceConstant, unreal.MaterialInstanceConstantFactoryNew())
        ML.set_material_instance_parent(instance, parent)
        assets.append(instance)
    for asset in assets:
        if not unreal.EditorAssetLibrary.save_loaded_asset(asset, only_if_is_dirty=False):
            raise RuntimeError(f"Could not save {asset.get_path_name()}")
    earth_actor.modify()
    for component, instance in zip(("surface", "clouds"), assets[3:]):
        mesh = earth_actor.get_editor_property(component)
        mesh.modify()
        mesh.set_material(0, instance)
    if not unreal.EditorLoadingAndSavingUtils.save_map(world, MAP):
        raise RuntimeError("Could not save BskEnvironment")
    return {"savedAssets": [a.get_path_name() for a in assets], "savedMap": MAP}


if __name__ == "__main__":
    print(json.dumps(apply_material_graphs()))
