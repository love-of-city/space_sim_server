"""Offline regression contracts for the isolated Foil002 visual trial.

The generated shader is additionally compiled and exercised in UE; these checks
cover source integrity, binding scope, build plumbing, and safe fallback ordering.
"""
import ast
import configparser
import hashlib
import json
from pathlib import Path
import struct

ROOT = Path(__file__).resolve().parents[1]


def test_source_maps_are_exact_2k_downloads():
    source = ROOT / "ContentSource/Foil002"
    manifest = json.loads((source / "source.json").read_text(encoding="utf-8"))
    assert manifest["license"] == "CC0-1.0"
    assert len(manifest["sha256"]) == 5
    for name, expected in manifest["sha256"].items():
        data = (source / name).read_bytes()
        assert data[:8] == b"\x89PNG\r\n\x1a\n"
        assert struct.unpack(">II", data[16:24]) == (2048, 2048)
        assert hashlib.sha256(data).hexdigest() == expected


def test_override_targets_only_one_fixed_panel_by_exact_asset_path():
    config = configparser.ConfigParser(strict=False)
    config.optionxform = str
    config.read(ROOT / "Config/DefaultGame.ini", encoding="utf-8")
    entries = dict(config["Bsk.MaterialOverrides"])
    assert entries.pop("Enabled") == "True"
    assert entries.pop("/Game/BSK/Generated/SARM/part_001_color_00.part_001_color_00") == (
        "/Game/BSK/Materials/Foil002/MI_BskFoil002.MI_BskFoil002")
    assert entries == {
        f"/Game/BSK/Generated/SARM/part_{part}_color_00.part_{part}_color_00":
        "/Game/BSK/Materials/SolarPanel/MI_BskSolarPanel.MI_BskSolarPanel"
        for part in ("055", "056", "066", "067")
    }


def test_material_override_precedes_generic_and_native_material_paths():
    source = (ROOT / "Plugins/BskUnrealRuntime/Source/BskUnrealRuntime/Private/BskSceneController.cpp").read_text(encoding="utf-8")
    block = source.split("void ApplyGeometryMaterial(", 1)[1].split("\n}", 1)[0]
    assert block.index("ApplyConfiguredMaterialOverride") < block.index("bUseAssetMaterials")
    assert block.index("ApplyConfiguredMaterialOverride") < block.index('TEXT("Metallic"), 0.0f')
    helper = source.split("bool ApplyConfiguredMaterialOverride(", 1)[1].split("\n}", 1)[0]
    assert 'Geometry.RenderRole != TEXT("visual")' in helper
    assert 'TEXT("BskDisableMaterialOverrides")' in helper
    assert '*Geometry.AssetPath, MaterialPath, GGameIni' in helper
    assert "if (!Material)" in helper and "using original material" in helper
    assert "Contains(" not in helper  # Never broadly match a mesh name.
    assert "SetAllMassScale" not in helper and "SetCollision" not in helper


def test_shader_uses_local_projection_without_uvs_or_displacement():
    source = (ROOT / "scripts/create_foil_material.py").read_text(encoding="utf-8-sig")
    ast.parse(source)
    assert "TRANSFORMPOSSOURCE_WORLD" in source and "TRANSFORMPOSSOURCE_LOCAL" in source
    assert "MaterialExpressionVertexNormalWS" in source
    assert '"tangent_space_normal", False' in source
    assert "TRANSFORMSOURCE_LOCAL" in source and "TRANSFORM_WORLD" in source
    assert "MaterialExpressionTextureCoordinate" not in source
    assert "MP_WORLD_POSITION_OFFSET" not in source and "MP_DISPLACEMENT" not in source
    for prop in ("MP_BASE_COLOR", "MP_NORMAL", "MP_ROUGHNESS", "MP_METALLIC", "MP_AMBIENT_OCCLUSION"):
        assert prop in source
    assert '"srgb", kind == "Color"' in source
    assert '"flip_green_channel", False' in source
    assert '"never_stream", True' in source
    assert '"TileSizeCm", 25.0' in source
    assert '"NormalStrength", 0.55' in source
    assert '"RoughnessBias", 0.16' in source


def test_foil_build_is_fingerprinted_and_runs_before_general_material_cache():
    entry = (ROOT / "scripts/prepare_runtime_materials.ps1").read_text(encoding="utf-8-sig")
    assert entry.index("prepare_foil_material.ps1") < entry.index("if (!$Force")
    prepare = (ROOT / "scripts/prepare_foil_material.ps1").read_text(encoding="utf-8-sig")
    for required in ("Get-FileHash", "SHA256", "FOIL002_BUILD_OK", "Failed to compile Material", "-WindowStyle Hidden"):
        assert required in prepare
    assert prepare.index("FOIL002_BUILD_OK") < prepare.index("Set-Content -LiteralPath $marker")


def test_foil_assets_are_inside_existing_cook_directory():
    config = (ROOT / "Config/DefaultGame.ini").read_text(encoding="utf-8")
    assert '+DirectoriesToAlwaysCook=(Path="/Game/BSK")' in config
    assert '/Game/BSK/Materials/Foil002/' in config
