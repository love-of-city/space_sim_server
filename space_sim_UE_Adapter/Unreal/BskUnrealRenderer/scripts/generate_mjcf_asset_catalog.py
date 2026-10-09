"""Generate UE import settings and a runtime MJCF asset catalog."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from bsk_render_adapter.mjcf_assets import parse_mjcf_geometry_metadata
from bsk_render_adapter.stl_conversion import prepare_stl_for_import


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mjcf", type=Path, required=True)
    parser.add_argument("--destination", required=True, help="UE content path, for example /Game/BSK/Generated/UR5e")
    parser.add_argument("--catalog", type=Path, required=True)
    parser.add_argument("--import-settings", type=Path, required=True)
    parser.add_argument("--build-scale", type=float, default=100.0)
    parser.add_argument("--component-scale", type=float, default=1.0)
    parser.add_argument("--mesh-cache", type=Path)
    parser.add_argument("--normal-mode", choices=("auto", "preserve", "recompute"), default="auto")
    parser.add_argument("--stl-smoothing-angle", type=float, default=60.0)
    args = parser.parse_args()

    metadata = parse_mjcf_geometry_metadata(args.mjcf)
    sources = sorted(
        {Path(item.source_path).resolve() for item in metadata if item.source_path},
        key=lambda item: item.as_posix().casefold(),
    )
    if not sources:
        raise RuntimeError(f"no mesh assets were found in {args.mjcf}")
    missing = [source for source in sources if not source.is_file()]
    if missing:
        raise FileNotFoundError(f"mesh source does not exist: {missing[0]}")
    stems = [source.stem.casefold() for source in sources]
    if len(stems) != len(set(stems)):
        raise ValueError("mesh file stems must be unique within one UE destination")

    cache_directory = (
        args.mesh_cache.resolve()
        if args.mesh_cache
        else (args.import_settings.resolve().parent / f"{args.catalog.stem}_mesh_cache")
    )
    prepared_sources: dict[Path, Path] = {}
    stl_results = {}
    for source in sources:
        if source.suffix.casefold() == ".stl":
            result = prepare_stl_for_import(
                source,
                cache_directory,
                normal_mode=args.normal_mode,
                smoothing_angle_degrees=args.stl_smoothing_angle,
            )
            prepared_sources[source] = Path(result.output_path)
            stl_results[source] = result
            if result.degenerate_triangle_count:
                print(
                    f"WARNING: {source.name} contains {result.degenerate_triangle_count} "
                    "degenerate STL triangles; they were retained"
                )
        else:
            prepared_sources[source] = source

    destination = args.destination.rstrip("/")
    texture_sources = sorted(
        {Path(item.material_texture_source).resolve() for item in metadata if item.material_texture_source},
        key=lambda item: item.as_posix().casefold(),
    )
    missing_textures = [source for source in texture_sources if not source.is_file()]
    if missing_textures:
        raise FileNotFoundError(f"texture source does not exist: {missing_textures[0]}")
    all_stems = stems + [source.stem.casefold() for source in texture_sources]
    if len(all_stems) != len(set(all_stems)):
        raise ValueError("mesh and texture file stems must be unique within one UE destination")
    catalog = {
        "schema": "bsk-render-asset-catalog/1",
        "source_mjcf": str(args.mjcf.resolve()),
        "assets": {
            str(source): {
                "asset_type": "static_mesh",
                "asset_path": f"{destination}/{source.stem}.{source.stem}",
                "source_format": source.suffix.lstrip(".").casefold(),
                "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                "import_source": str(prepared_sources[source]),
                # Mesh source formats have no reliable unit metadata. Apply
                # metres-to-centimetres while UE builds LOD0 so small geometry
                # is not simplified or classified as degenerate first.
                "build_scale": [args.build_scale] * 3,
                "component_scale": [args.component_scale] * 3,
                **(
                    {
                        "stl": {
                            "encoding": stl_results[source].source_format,
                            "triangle_count": stl_results[source].triangle_count,
                            "unique_vertex_count": stl_results[source].unique_vertex_count,
                            "degenerate_triangle_count": stl_results[source].degenerate_triangle_count,
                            "normal_mode": stl_results[source].normal_mode,
                            "smoothing_angle_degrees": stl_results[source].smoothing_angle_degrees,
                        }
                    }
                    if source in stl_results
                    else {}
                ),
            }
            for source in sources
        },
        "textures": {
            str(source): f"{destination}/{source.stem}.{source.stem}"
            for source in texture_sources
        },
    }
    import_settings = {
        "ImportGroups": [
            {
                "GroupName": "BSK MJCF meshes",
                "Filenames": [
                    *(str(prepared_sources[source]) for source in sources),
                    *(str(source) for source in texture_sources),
                ],
                "DestinationPath": destination,
                "bReplaceExisting": True,
                "bSkipReadOnly": False,
            }
        ]
    }
    args.catalog.parent.mkdir(parents=True, exist_ok=True)
    args.import_settings.parent.mkdir(parents=True, exist_ok=True)
    args.catalog.write_text(json.dumps(catalog, indent=2), encoding="utf-8")
    args.import_settings.write_text(json.dumps(import_settings, indent=2), encoding="utf-8")
    cached_count = sum(int(result.cached) for result in stl_results.values())
    print(
        f"Prepared catalog for {len(sources)} meshes ({len(stl_results)} STL, "
        f"{cached_count} cached) and {len(texture_sources)} texture assets"
    )


if __name__ == "__main__":
    main()
