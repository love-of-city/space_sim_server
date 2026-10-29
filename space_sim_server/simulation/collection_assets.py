"""Resolve the shipped common assets for newly generated random layouts.

Only reusable source meshes and UE packages live in this catalog. The layout
builder and importer still produce each layout's independent socket surfaces,
matching deck apertures and collision geometry before a pool can be published.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _checked_path(root, relative):
    path = (root / relative).resolve()
    if Path(relative).is_absolute() or not path.is_relative_to(root.resolve()):
        raise ValueError(f"Asset path must stay inside its repository root: {relative}")
    return path


def _verify(path, expected):
    if not path.is_file():
        raise ValueError(f"Missing collection asset: {path}. Run git lfs pull in the unified repository.")
    content = path.read_bytes()
    if content.startswith(b"version https://git-lfs.github.com/spec/v1"):
        raise ValueError(f"Collection asset is a Git LFS pointer: {path}. Run git lfs pull.")
    if hashlib.sha256(content).hexdigest() != expected:
        raise ValueError(f"Collection asset differs from the shipped catalog: {path}. "
                         "Restore matching assets or pass an explicitly regenerated --base-catalog.")


def prepare_base_catalog(output, *, server_root=ROOT, adapter=None):
    """Validate shipped common assets and write a machine-local absolute catalog.

    This does not create or choose a layout, change physics, or certify a pool.
    The same result can be merged with any newly generated layout's 12 meshes.
    """
    server_root = Path(server_root).resolve()
    adapter = Path(adapter or server_root.parent / "space_sim_UE_Adapter").resolve()
    source = server_root / "config/collection-base-assets.json"
    document = json.loads(source.read_text(encoding="utf-8"))
    if document.get("schema") != "collection-base-assets/1":
        raise ValueError("Unsupported collection base-asset schema")
    content = adapter / "Unreal/BskUnrealRenderer/Content"
    assets = {}
    for relative, entry in document["assets"].items():
        mesh = _checked_path(server_root, relative)
        _verify(mesh, entry["source_sha256"])
        package = entry["asset_path"].split(".")[0]
        if not package.startswith("/Game/"):
            raise ValueError(f"Expected a project-local UE package: {package}")
        ue_relative = package.removeprefix("/Game/") + ".uasset"
        _verify(_checked_path(content, ue_relative), document["ue_assets"][ue_relative])
        assets[str(mesh)] = entry
    model = _checked_path(server_root, document["source_mjcf"])
    if not model.is_file():
        raise ValueError(f"Missing collection source model: {model}")
    catalog = dict(schema="bsk-render-asset-catalog/1", source_mjcf=str(model),
                   assets=assets, textures={})
    output = Path(output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(catalog, indent=2) + "\n", encoding="utf-8")
    return output
