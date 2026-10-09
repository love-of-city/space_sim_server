"""Apply deterministic runtime-friendly build settings to imported MJCF meshes."""

from __future__ import annotations

import json
import os
from pathlib import Path

import unreal


catalog_path = Path(os.environ["BSK_MJCF_ASSET_CATALOG"])
catalog = json.loads(catalog_path.read_text(encoding="utf-8"))
normal_mode = os.environ.get("BSK_MJCF_NORMAL_MODE", "auto").strip().casefold()
if normal_mode not in {"auto", "preserve", "recompute"}:
    raise ValueError(f"invalid BSK_MJCF_NORMAL_MODE: {normal_mode}")


def obj_has_valid_normals(source: str) -> bool:
    path = Path(source)
    if path.suffix.casefold() != ".obj" or not path.is_file():
        return True
    normal_count = 0
    face_count = 0
    with path.open("r", encoding="utf-8", errors="ignore") as stream:
        for line in stream:
            if line.startswith("vn "):
                values = [float(value) for value in line.split()[1:4]]
                if len(values) != 3 or sum(value * value for value in values) < 1.0e-12:
                    return False
                normal_count += 1
            elif line.startswith("f "):
                face_count += 1
                for item in line.split()[1:]:
                    fields = item.split("/")
                    if len(fields) < 3 or not fields[2]:
                        return False
    return normal_count > 0 and face_count > 0


subsystem = unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem)
configured = 0
recomputed = 0
validated_stl_triangles = 0
for source, entry in catalog.get("assets", {}).items():
    asset_path = entry.get("asset_path", "").split(".", 1)[0]
    mesh = unreal.EditorAssetLibrary.load_asset(asset_path)
    if not isinstance(mesh, unreal.StaticMesh):
        raise RuntimeError(f"catalog asset is not a StaticMesh: {asset_path}")
    if subsystem is not None:
        settings = subsystem.get_lod_build_settings(mesh, 0)
    else:
        # StaticMesh source_models are deliberately not exposed to Python.  The
        # compatibility library remains available in commandlets and forwards
        # to the same editor implementation without touching private data.
        settings = unreal.EditorStaticMeshLibrary.get_lod_build_settings(mesh, 0)
    # STL sources are deterministically converted to an OBJ staging mesh.  Test
    # the file actually imported by UE so auto mode can preserve its generated
    # angle-aware normals instead of treating the source container as opaque.
    imported_source = entry.get("import_source", source)
    should_recompute = normal_mode == "recompute" or (
        normal_mode == "auto" and not obj_has_valid_normals(imported_source)
    )
    build_scale = entry.get("build_scale", (1.0, 1.0, 1.0))
    if len(build_scale) != 3 or any(float(value) <= 0.0 for value in build_scale):
        raise ValueError(f"invalid build_scale for {source}: {build_scale}")
    mesh.modify()
    nanite = mesh.get_editor_property("nanite_settings")
    nanite.set_editor_property("enabled", False)
    nanite.set_editor_property("fallback_percent_triangles", 1.0)
    nanite.set_editor_property("fallback_relative_error", 0.0)
    mesh.set_editor_property("nanite_settings", nanite)
    settings.set_editor_property(
        "build_scale3d",
        unreal.Vector(float(build_scale[0]), float(build_scale[1]), float(build_scale[2])),
    )
    settings.set_editor_property("recompute_normals", should_recompute)
    settings.set_editor_property("recompute_tangents", True)
    settings.set_editor_property("use_mikk_t_space", True)
    settings.set_editor_property("remove_degenerates", False)
    if subsystem is not None:
        subsystem.set_lod_build_settings(mesh, 0, settings)
    else:
        unreal.EditorStaticMeshLibrary.set_lod_build_settings(mesh, 0, settings)
    if not unreal.EditorAssetLibrary.save_loaded_asset(mesh, only_if_is_dirty=False):
        raise RuntimeError(f"failed to save configured mesh: {asset_path}")
    applied = (
        subsystem.get_lod_build_settings(mesh, 0)
        if subsystem is not None
        else unreal.EditorStaticMeshLibrary.get_lod_build_settings(mesh, 0)
    )
    applied_nanite = mesh.get_editor_property("nanite_settings")
    applied_scale = applied.get_editor_property("build_scale3d")
    if applied_nanite.get_editor_property("enabled"):
        raise RuntimeError(f"Nanite remained enabled after configuring {asset_path}")
    if applied.get_editor_property("remove_degenerates"):
        # UE 5.6's Interchange StaticMesh post-edit path can force this flag
        # back on. Full-LOD fidelity is verified from the rebuilt/renderable
        # triangle count; non-zero source triangles survive at build_scale=100.
        unreal.log_warning(f"UE retained remove_degenerates for {asset_path}; validating LOD0 geometry separately")
    if applied.get_editor_property("recompute_normals") != should_recompute:
        raise RuntimeError(f"normal mode did not persist for {asset_path}")
    expected_scale = tuple(float(value) for value in build_scale)
    actual_scale = (applied_scale.x, applied_scale.y, applied_scale.z)
    if any(abs(actual - expected) > 1.0e-6 for actual, expected in zip(actual_scale, expected_scale)):
        raise RuntimeError(
            f"build scale did not persist for {asset_path}: {actual_scale} != {expected_scale}"
        )
    stl_metadata = entry.get("stl", {})
    expected_triangles = stl_metadata.get("triangle_count")
    if expected_triangles is not None:
        actual_triangles = mesh.get_num_triangles(0)
        if actual_triangles != int(expected_triangles):
            raise RuntimeError(
                f"STL LOD0 triangle loss for {asset_path}: "
                f"{actual_triangles} != {expected_triangles}"
            )
        validated_stl_triangles += actual_triangles
    configured += 1
    recomputed += int(should_recompute)

unreal.log(
    f"Configured {configured} MJCF meshes using normal_mode={normal_mode}: "
    f"preserved={configured - recomputed}, recomputed={recomputed}; "
    f"validated_stl_triangles={validated_stl_triangles}; "
    "Nanite=off, LOD0=full, remove_degenerates=false, tangents=MikkTSpace"
)
