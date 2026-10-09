"""Build an isolated UV-free Foil002 material; run with UE's Python editor.

Only /Game/BSK/Materials/Foil002 is written. No meshes, physics, generic PBR
materials, levels or simulation XML are changed. Source maps are in ContentSource.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import unreal

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "ContentSource" / "Foil002"
DEST = "/Game/BSK/Materials/Foil002"
LIB = unreal.MaterialEditingLibrary


def node(material, cls, x=0, y=0):
    result = LIB.create_material_expression(material, cls, x, y)
    if result is None:
        raise RuntimeError(f"Could not create {cls}")
    return result


def connect(source, target, pin, output=""):
    if not LIB.connect_material_expressions(source, output, target, pin):
        raise RuntimeError(f"Could not connect {source} -> {target}.{pin}")


def scalar(material, name, value, x=0, y=0):
    result = node(material, unreal.MaterialExpressionScalarParameter, x, y)
    result.set_editor_property("parameter_name", name)
    result.set_editor_property("default_value", value)
    return result


def custom(material, name, code, inputs, output_type, x=0, y=0):
    result = node(material, unreal.MaterialExpressionCustom, x, y)
    result.set_editor_property("description", name)
    result.set_editor_property("code", code)
    result.set_editor_property("output_type", output_type)
    pins = []
    for key in inputs:
        pin = unreal.CustomInput()
        pin.set_editor_property("input_name", key)
        pins.append(pin)
    result.set_editor_property("inputs", pins)
    for key, expr in inputs.items():
        connect(expr, result, key)
    return result


def save(asset):
    if not unreal.EditorAssetLibrary.save_loaded_asset(asset, only_if_is_dirty=False):
        raise RuntimeError(f"Could not save {asset.get_path_name()}")


def load_textures():
    textures = {}
    manifest = json.loads((SOURCE / "source.json").read_text(encoding="utf-8"))
    for kind in ("Color", "NormalDX", "Roughness", "Metalness", "AmbientOcclusion"):
        path = SOURCE / f"Foil002_2K-PNG_{kind}.png"
        if hashlib.sha256(path.read_bytes()).hexdigest() != manifest["sha256"][path.name]:
            raise RuntimeError(f"Source texture checksum mismatch: {path}")
        task = unreal.AssetImportTask()
        task.set_editor_property("filename", str(path))
        task.set_editor_property("destination_path", DEST)
        task.set_editor_property("destination_name", f"T_Foil002_{kind}")
        task.set_editor_property("automated", True)
        task.set_editor_property("replace_existing", True)
        task.set_editor_property("save", True)
        unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
        texture = unreal.load_asset(f"{DEST}/T_Foil002_{kind}")
        if not isinstance(texture, unreal.Texture2D):
            raise RuntimeError(f"Could not import {path}")
        normal = kind == "NormalDX"
        texture.set_editor_property("srgb", kind == "Color")
        texture.set_editor_property("compression_settings", (
            unreal.TextureCompressionSettings.TC_NORMALMAP if normal else
            unreal.TextureCompressionSettings.TC_DEFAULT if kind == "Color" else
            unreal.TextureCompressionSettings.TC_MASKS
        ))
        texture.set_editor_property("flip_green_channel", False)  # Already DirectX.
        # UV-free custom projection has no mesh UV density for the streamer.
        # Keep this small, fixed 2K trial resident rather than sampling blurry mips.
        texture.set_editor_property("never_stream", True)
        texture.set_editor_property("address_x", unreal.TextureAddress.TA_WRAP)
        texture.set_editor_property("address_y", unreal.TextureAddress.TA_WRAP)
        save(texture)
        textures[kind] = texture
    return textures


def build():
    textures = load_textures()
    tools = unreal.AssetToolsHelpers.get_asset_tools()
    material = unreal.load_asset(f"{DEST}/M_BskFoil002")
    if material is None:
        material = tools.create_asset("M_BskFoil002", DEST, unreal.Material, unreal.MaterialFactoryNew())
    if not isinstance(material, unreal.Material):
        raise RuntimeError("Foil master path is not a material")
    LIB.delete_all_material_expressions(material)
    material.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_DEFAULT_LIT)
    material.set_editor_property("blend_mode", unreal.BlendMode.BLEND_OPAQUE)
    material.set_editor_property("two_sided", False)
    material.set_editor_property("tangent_space_normal", False)
    material.set_editor_property("used_with_instanced_static_meshes", True)

    world_position = node(material, unreal.MaterialExpressionWorldPosition, -2400, -800)
    local_position = node(material, unreal.MaterialExpressionTransformPosition, -2200, -800)
    local_position.set_editor_property("transform_source_type", unreal.MaterialPositionTransformSource.TRANSFORMPOSSOURCE_WORLD)
    local_position.set_editor_property("transform_type", unreal.MaterialPositionTransformSource.TRANSFORMPOSSOURCE_LOCAL)
    connect(world_position, local_position, "")
    world_normal = node(material, unreal.MaterialExpressionVertexNormalWS, -2400, -500)
    local_normal = node(material, unreal.MaterialExpressionTransform, -2200, -500)
    local_normal.set_editor_property("transform_source_type", unreal.MaterialVectorCoordTransformSource.TRANSFORMSOURCE_WORLD)
    local_normal.set_editor_property("transform_type", unreal.MaterialVectorCoordTransform.TRANSFORM_LOCAL)
    connect(world_normal, local_normal, "")
    tile = scalar(material, "TileSizeCm", 25.0, -2200, -1000)
    strength = scalar(material, "NormalStrength", 0.55, -700, 300)
    roughness_bias = scalar(material, "RoughnessBias", 0.16, -700, 600)
    weights = custom(material, "Local triplanar weights", "float3 w=pow(abs(normalize(N)),4); return w/max(dot(w,1.0),0.0001);", {"N": local_normal}, unreal.CustomMaterialOutputType.CMOT_FLOAT3, -1900, -400)
    uvs = []
    # Right-handed projection bases for all six face orientations.
    for i, (a, b, axis) in enumerate((("y", "z", "x"), ("z", "x", "y"), ("x", "y", "z"))):
        uvs.append(custom(material, f"Local {axis.upper()} projection", f"return float2(P.{a},P.{b}*(N.{axis}<0?-1:1))/max(Tile,0.01);", {"P": local_position, "N": local_normal, "Tile": tile}, unreal.CustomMaterialOutputType.CMOT_FLOAT2, -1800, -1000+i*240))

    sampled = {}
    for row, (kind, texture) in enumerate(textures.items()):
        samples = []
        for i, uv in enumerate(uvs):
            sample = node(material, unreal.MaterialExpressionTextureSampleParameter2D, -1400+i*260, row*330)
            sample.set_editor_property("parameter_name", f"Foil{kind}")
            sample.set_editor_property("texture", texture)
            sample.set_editor_property("sampler_type", (
                unreal.MaterialSamplerType.SAMPLERTYPE_NORMAL if kind == "NormalDX" else
                unreal.MaterialSamplerType.SAMPLERTYPE_COLOR if kind == "Color" else
                unreal.MaterialSamplerType.SAMPLERTYPE_MASKS
            ))
            connect(uv, sample, "UVs")
            samples.append(sample)
        sampled[kind] = samples

    def blend(kind, scalar_output=False):
        code = "return (X*W.x+Y*W.y+Z*W.z)" + (".r;" if scalar_output else ";")
        return custom(material, f"Blend {kind}", code, dict(zip(("X", "Y", "Z", "W"), (*sampled[kind], weights))), unreal.CustomMaterialOutputType.CMOT_FLOAT1 if scalar_output else unreal.CustomMaterialOutputType.CMOT_FLOAT3, -300, list(textures).index(kind)*300)

    base = blend("Color")
    rough = blend("Roughness", True)
    rough = custom(material, "Controlled foil roughness", "return clamp(R+B,0.08,0.95);", {"R": rough, "B": roughness_bias}, unreal.CustomMaterialOutputType.CMOT_FLOAT1, 0, 600)
    metal = blend("Metalness", True)
    ao = blend("AmbientOcclusion", True)
    normals = custom(material, "Object-space normal blend (neutral map preserves mesh normals)", """
float3 n=normalize(N); float3 s=float3(n.x<0?-1:1,n.y<0?-1:1,n.z<0?-1:1);
float3 x=normalize(float3(X.xy*Strength,max(X.z,0.001)));
float3 y=normalize(float3(Y.xy*Strength,max(Y.z,0.001)));
float3 z=normalize(float3(Z.xy*Strength,max(Z.z,0.001)));
float3 nx=normalize(float3(n.x*x.z,n.y+x.x,n.z+x.y*s.x));
float3 ny=normalize(float3(n.x+y.y*s.y,n.y*y.z,n.z+y.x));
float3 nz=normalize(float3(n.x+z.x,n.y+z.y*s.z,n.z*z.z));
return normalize(nx*W.x+ny*W.y+nz*W.z);
""", dict(zip(("X", "Y", "Z", "W", "N", "Strength"), (*sampled["NormalDX"], weights, local_normal, strength))), unreal.CustomMaterialOutputType.CMOT_FLOAT3, 0, 300)
    transformed = node(material, unreal.MaterialExpressionTransform, 350, 300)
    transformed.set_editor_property("transform_source_type", unreal.MaterialVectorCoordTransformSource.TRANSFORMSOURCE_LOCAL)
    transformed.set_editor_property("transform_type", unreal.MaterialVectorCoordTransform.TRANSFORM_WORLD)
    connect(normals, transformed, "")
    for expr, prop in ((base, "MP_BASE_COLOR"), (rough, "MP_ROUGHNESS"), (metal, "MP_METALLIC"), (ao, "MP_AMBIENT_OCCLUSION"), (transformed, "MP_NORMAL")):
        if not LIB.connect_material_property(expr, "", getattr(unreal.MaterialProperty, prop)):
            raise RuntimeError(f"Could not connect {prop}")
    LIB.recompile_material(material)
    save(material)
    instance = unreal.load_asset(f"{DEST}/MI_BskFoil002")
    if instance is None:
        instance = tools.create_asset("MI_BskFoil002", DEST, unreal.MaterialInstanceConstant, unreal.MaterialInstanceConstantFactoryNew())
    LIB.set_material_instance_parent(instance, material)
    for name, value in (("TileSizeCm", 25.0), ("NormalStrength", 0.55), ("RoughnessBias", 0.16)):
        # UE 5.6 setter always returns false; verify the actual stored value.
        LIB.set_material_instance_scalar_parameter_value(instance, name, value)
        if abs(LIB.get_material_instance_scalar_parameter_value(instance, name) - value) > 1e-5:
            raise RuntimeError(f"Could not set {name}")
    LIB.update_material_instance(instance)
    save(instance)
    unreal.log("FOIL002_BUILD_OK: isolated local-space material and 5 textures saved")


if __name__ == "__main__":
    build()
