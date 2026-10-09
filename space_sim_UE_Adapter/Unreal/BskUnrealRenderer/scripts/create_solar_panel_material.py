"""Create a UV-free procedural solar-panel material."""

import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import unreal
import create_foil_material as helper

DEST = "/Game/BSK/Materials/SolarPanel"
LIB = unreal.MaterialEditingLibrary


def build():
    tools = unreal.AssetToolsHelpers.get_asset_tools()

    material = unreal.load_asset(f"{DEST}/M_BskSolarPanel")
    if material is None:
        material = tools.create_asset(
            "M_BskSolarPanel",
            DEST,
            unreal.Material,
            unreal.MaterialFactoryNew(),
        )

    LIB.delete_all_material_expressions(material)
    material.set_editor_property(
        "shading_model",
        unreal.MaterialShadingModel.MSM_DEFAULT_LIT,
    )
    material.set_editor_property("blend_mode", unreal.BlendMode.BLEND_OPAQUE)
    material.set_editor_property("two_sided", True)
    material.set_editor_property("used_with_instanced_static_meshes", True)

    world_position = helper.node(
        material, unreal.MaterialExpressionWorldPosition, -1200, -300
    )
    local_position = helper.node(
        material, unreal.MaterialExpressionTransformPosition, -1000, -300
    )
    local_position.set_editor_property(
        "transform_source_type",
        unreal.MaterialPositionTransformSource.TRANSFORMPOSSOURCE_WORLD,
    )
    local_position.set_editor_property(
        "transform_type",
        unreal.MaterialPositionTransformSource.TRANSFORMPOSSOURCE_LOCAL,
    )
    helper.connect(world_position, local_position, "")

    world_normal = helper.node(
        material, unreal.MaterialExpressionVertexNormalWS, -1200, 0
    )
    local_normal = helper.node(
        material, unreal.MaterialExpressionTransform, -1000, 0
    )
    local_normal.set_editor_property(
        "transform_source_type",
        unreal.MaterialVectorCoordTransformSource.TRANSFORMSOURCE_WORLD,
    )
    local_normal.set_editor_property(
        "transform_type",
        unreal.MaterialVectorCoordTransform.TRANSFORM_LOCAL,
    )
    helper.connect(world_normal, local_normal, "")

    cell_size = helper.scalar(material, "CellSizeCm", 5.2, -900, 300)

    base = helper.custom(
        material,
        "Blue photovoltaic cells with silver grid",
        """
float3 an = abs(normalize(N));
float2 uv = an.x > an.y
    ? (an.x > an.z ? P.yz : P.xy)
    : (an.y > an.z ? P.xz : P.xy);

float2 f = abs(frac(uv / max(Cell, 0.1)) - 0.5);
float grid = smoothstep(0.40, 0.47, max(f.x, f.y));

float variation = 0.025 * sin(uv.x * 0.55) * sin(uv.y * 0.47);
float3 cellBlue = float3(0.025, 0.14, 0.42) + variation;
float3 conductor = float3(0.65, 0.72, 0.78);

return lerp(cellBlue, conductor, grid);
""",
        {"P": local_position, "N": local_normal, "Cell": cell_size},
        unreal.CustomMaterialOutputType.CMOT_FLOAT3,
        -500,
        -200,
    )

    roughness = helper.scalar(material, "Roughness", 0.28, -400, 100)
    metallic = helper.scalar(material, "Metallic", 0.0, -400, 220)
    specular = helper.scalar(material, "Specular", 0.35, -400, 340)

    connections = (
        (base, "MP_BASE_COLOR"),
        (roughness, "MP_ROUGHNESS"),
        (metallic, "MP_METALLIC"),
        (specular, "MP_SPECULAR"),
    )

    for expression, property_name in connections:
        if not LIB.connect_material_property(
            expression,
            "",
            getattr(unreal.MaterialProperty, property_name),
        ):
            raise RuntimeError(f"Could not connect {property_name}")

    LIB.recompile_material(material)
    helper.save(material)

    instance = unreal.load_asset(f"{DEST}/MI_BskSolarPanel")
    if instance is None:
        instance = tools.create_asset(
            "MI_BskSolarPanel",
            DEST,
            unreal.MaterialInstanceConstant,
            unreal.MaterialInstanceConstantFactoryNew(),
        )

    LIB.set_material_instance_parent(instance, material)

    for name, value in (
        ("CellSizeCm", 5.2),
        ("Roughness", 0.28),
        ("Metallic", 0.0),
        ("Specular", 0.35),
    ):
        LIB.set_material_instance_scalar_parameter_value(instance, name, value)

    LIB.update_material_instance(instance)
    helper.save(instance)
    unreal.log("SOLAR_PANEL_BUILD_OK")


if __name__ == "__main__":
    build()
