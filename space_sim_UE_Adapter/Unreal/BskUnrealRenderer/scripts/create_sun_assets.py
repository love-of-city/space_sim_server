"""Import the Solar System Scope Sun texture and create an unlit UE material."""

from __future__ import annotations

from pathlib import Path

import unreal


TEXTURE_ASSET = "/Game/Planets/Sun/8k_sun"
MATERIAL_ASSET = "/Game/Planets/Sun/M_Sun"
SOURCE_FILE = Path(unreal.Paths.project_saved_dir()) / "AssetSources" / "SolarSystemScope" / "8k_sun.jpg"


def import_texture() -> unreal.Texture2D:
    if not SOURCE_FILE.is_file():
        raise RuntimeError(f"Sun texture source is missing: {SOURCE_FILE}")

    task = unreal.AssetImportTask()
    task.set_editor_property("filename", str(SOURCE_FILE))
    task.set_editor_property("destination_path", "/Game/Planets/Sun")
    task.set_editor_property("destination_name", "8k_sun")
    task.set_editor_property("automated", True)
    task.set_editor_property("replace_existing", True)
    task.set_editor_property("replace_existing_settings", True)
    task.set_editor_property("save", True)
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])

    texture = unreal.EditorAssetLibrary.load_asset(TEXTURE_ASSET)
    if not isinstance(texture, unreal.Texture2D):
        raise RuntimeError(f"failed to import {TEXTURE_ASSET}")
    texture.set_editor_property("srgb", True)
    texture.set_editor_property("compression_settings", unreal.TextureCompressionSettings.TC_DEFAULT)
    texture.set_editor_property("lod_group", unreal.TextureGroup.TEXTUREGROUP_SKYBOX)
    texture.set_editor_property("address_x", unreal.TextureAddress.TA_WRAP)
    texture.set_editor_property("address_y", unreal.TextureAddress.TA_CLAMP)
    if not unreal.EditorAssetLibrary.save_loaded_asset(texture, only_if_is_dirty=False):
        raise RuntimeError(f"failed to save {TEXTURE_ASSET}")
    return texture


def expression(material: unreal.Material, expression_class, x: int, y: int):
    return unreal.MaterialEditingLibrary.create_material_expression(material, expression_class, x, y)


def create_material(texture: unreal.Texture2D) -> None:
    if unreal.EditorAssetLibrary.does_asset_exist(MATERIAL_ASSET):
        if not unreal.EditorAssetLibrary.delete_asset(MATERIAL_ASSET):
            raise RuntimeError(f"failed to replace {MATERIAL_ASSET}")

    material = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
        "M_Sun",
        "/Game/Planets/Sun",
        unreal.Material,
        unreal.MaterialFactoryNew(),
    )
    if not material:
        raise RuntimeError(f"failed to create {MATERIAL_ASSET}")

    material.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_UNLIT)
    material.set_editor_property("blend_mode", unreal.BlendMode.BLEND_OPAQUE)
    material.set_editor_property("two_sided", False)

    texcoord = expression(material, unreal.MaterialExpressionTextureCoordinate, -900, -120)
    sample = expression(material, unreal.MaterialExpressionTextureSampleParameter2D, -650, -120)
    sample.set_editor_property("parameter_name", "SunTexture")
    sample.set_editor_property("texture", texture)
    unreal.MaterialEditingLibrary.connect_material_expressions(texcoord, "", sample, "Coordinates")

    tint = expression(material, unreal.MaterialExpressionVectorParameter, -650, 80)
    tint.set_editor_property("parameter_name", "SunTint")
    tint.set_editor_property("default_value", unreal.LinearColor(1.0, 1.0, 1.0, 1.0))
    tinted = expression(material, unreal.MaterialExpressionMultiply, -380, -40)
    unreal.MaterialEditingLibrary.connect_material_expressions(sample, "RGB", tinted, "A")
    unreal.MaterialEditingLibrary.connect_material_expressions(tint, "", tinted, "B")

    strength = expression(material, unreal.MaterialExpressionScalarParameter, -380, 130)
    strength.set_editor_property("parameter_name", "EmissiveStrength")
    strength.set_editor_property("default_value", 1.5)
    emissive = expression(material, unreal.MaterialExpressionMultiply, -120, 0)
    unreal.MaterialEditingLibrary.connect_material_expressions(tinted, "", emissive, "A")
    unreal.MaterialEditingLibrary.connect_material_expressions(strength, "", emissive, "B")

    if not unreal.MaterialEditingLibrary.connect_material_property(
        emissive, "", unreal.MaterialProperty.MP_EMISSIVE_COLOR
    ):
        raise RuntimeError("failed to connect Sun emissive output")

    unreal.MaterialEditingLibrary.recompile_material(material)
    if not unreal.EditorAssetLibrary.save_loaded_asset(material, only_if_is_dirty=False):
        raise RuntimeError(f"failed to save {MATERIAL_ASSET}")


texture_asset = import_texture()
create_material(texture_asset)
unreal.log(f"Solar System Scope Sun assets ready: {TEXTURE_ASSET}, {MATERIAL_ASSET}")
