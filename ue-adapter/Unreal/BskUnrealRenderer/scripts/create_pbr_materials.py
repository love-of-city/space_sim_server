"""Create the cooked runtime PBR master materials used by MJCF geometry."""

from __future__ import annotations

import unreal


def expression(material, expression_class, x, y):
    return unreal.MaterialEditingLibrary.create_material_expression(material, expression_class, x, y)


def scalar(material, name: str, default: float, x: int, y: int):
    node = expression(material, unreal.MaterialExpressionScalarParameter, x, y)
    node.set_editor_property("parameter_name", name)
    node.set_editor_property("default_value", default)
    return node


def vector(material, name: str, default: unreal.LinearColor, x: int, y: int):
    node = expression(material, unreal.MaterialExpressionVectorParameter, x, y)
    node.set_editor_property("parameter_name", name)
    node.set_editor_property("default_value", default)
    return node


def create_material(name: str, translucent: bool) -> None:
    asset_path = f"/Game/BSK/{name}"
    if unreal.EditorAssetLibrary.does_asset_exist(asset_path):
        if not unreal.EditorAssetLibrary.delete_asset(asset_path):
            raise RuntimeError(f"failed to replace {asset_path}")
    material = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
        name,
        "/Game/BSK",
        unreal.Material,
        unreal.MaterialFactoryNew(),
    )
    if not material:
        raise RuntimeError(f"failed to create {asset_path}")

    material.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_DEFAULT_LIT)
    material.set_editor_property("blend_mode", unreal.BlendMode.BLEND_TRANSLUCENT if translucent else unreal.BlendMode.BLEND_OPAQUE)
    material.set_editor_property("two_sided", False)
    material.set_editor_property("used_with_instanced_static_meshes", True)

    base_color = vector(material, "BaseColor", unreal.LinearColor(0.7, 0.7, 0.7, 1.0), -900, -250)
    texture_repeat = vector(material, "TextureRepeat", unreal.LinearColor(1.0, 1.0, 0.0, 0.0), -1100, -550)
    texture_coordinates = expression(material, unreal.MaterialExpressionTextureCoordinate, -1100, -680)
    uv_multiply = expression(material, unreal.MaterialExpressionMultiply, -850, -600)
    unreal.MaterialEditingLibrary.connect_material_expressions(texture_coordinates, "", uv_multiply, "A")
    unreal.MaterialEditingLibrary.connect_material_expressions(texture_repeat, "", uv_multiply, "B")
    texture_sample = expression(material, unreal.MaterialExpressionTextureSampleParameter2D, -620, -560)
    texture_sample.set_editor_property("parameter_name", "BaseColorTexture")
    texture_sample.set_editor_property("texture", unreal.load_asset("/Engine/EngineResources/WhiteSquareTexture"))
    unreal.MaterialEditingLibrary.connect_material_expressions(uv_multiply, "", texture_sample, "Coordinates")
    textured_color = expression(material, unreal.MaterialExpressionMultiply, -390, -420)
    unreal.MaterialEditingLibrary.connect_material_expressions(base_color, "", textured_color, "A")
    unreal.MaterialEditingLibrary.connect_material_expressions(texture_sample, "RGB", textured_color, "B")
    use_texture = scalar(material, "UseTexture", 0.0, -390, -590)
    final_color = expression(material, unreal.MaterialExpressionLinearInterpolate, -150, -300)
    unreal.MaterialEditingLibrary.connect_material_expressions(base_color, "", final_color, "A")
    unreal.MaterialEditingLibrary.connect_material_expressions(textured_color, "", final_color, "B")
    unreal.MaterialEditingLibrary.connect_material_expressions(use_texture, "", final_color, "Alpha")
    roughness = scalar(material, "Roughness", 0.45, -600, -100)
    specular = scalar(material, "Specular", 0.5, -600, 25)
    metallic = scalar(material, "Metallic", 0.0, -600, 150)
    emission = scalar(material, "Emission", 0.0, -600, 300)
    ambient = scalar(material, "Ambient", 0.18, -600, 400)
    light_sum = expression(material, unreal.MaterialExpressionAdd, -430, 300)
    unreal.MaterialEditingLibrary.connect_material_expressions(emission, "", light_sum, "A")
    unreal.MaterialEditingLibrary.connect_material_expressions(ambient, "", light_sum, "B")
    emissive_multiply = expression(material, unreal.MaterialExpressionMultiply, -300, 260)
    unreal.MaterialEditingLibrary.connect_material_expressions(final_color, "", emissive_multiply, "A")
    unreal.MaterialEditingLibrary.connect_material_expressions(light_sum, "", emissive_multiply, "B")

    connections = (
        (final_color, unreal.MaterialProperty.MP_BASE_COLOR),
        (roughness, unreal.MaterialProperty.MP_ROUGHNESS),
        (specular, unreal.MaterialProperty.MP_SPECULAR),
        (metallic, unreal.MaterialProperty.MP_METALLIC),
        (emissive_multiply, unreal.MaterialProperty.MP_EMISSIVE_COLOR),
    )
    for node, material_property in connections:
        if not unreal.MaterialEditingLibrary.connect_material_property(node, "", material_property):
            raise RuntimeError(f"failed to connect {material_property} for {asset_path}")
    if translucent:
        opacity = scalar(material, "Opacity", 1.0, -600, 525)
        if not unreal.MaterialEditingLibrary.connect_material_property(opacity, "", unreal.MaterialProperty.MP_OPACITY):
            raise RuntimeError(f"failed to connect opacity for {asset_path}")

    unreal.MaterialEditingLibrary.recompile_material(material)
    if not unreal.EditorAssetLibrary.save_loaded_asset(material, only_if_is_dirty=False):
        raise RuntimeError(f"failed to save {asset_path}")
    unreal.log(f"Created BSK runtime PBR material: {asset_path}")


create_material("M_BskPbrOpaque", False)
create_material("M_BskPbrTranslucent", True)
