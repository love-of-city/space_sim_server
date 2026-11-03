"""Validate model/catalog/asset bindings for supported saved capture layouts.

Reading historical layout schemas never invokes an old layout generator.
"""
from __future__ import annotations
import hashlib
import json
from pathlib import Path

SUPPORTED_LAYOUT_SCHEMAS = ("task-box-layout/1", "task-box-layout/2", "task-box-layout/3")

def validated_capture_layout(model: Path, catalog: Path) -> dict:
    """Bind a generated model and its adapted visual to one capture catalog."""
    model, catalog = Path(model).resolve(), Path(catalog).resolve()
    manifest_path = model.with_suffix(".layout.json")
    manifest = json.loads(manifest_path.read_text())
    if "package_root" in manifest:
        from bsk_render_adapter.portable_assets import package_path, package_root
        root = package_root(manifest_path, manifest["package_root"])
        manifest["adapted_base"]["visual"] = str(package_path(root, manifest["adapted_base"]["visual"]))
        for item in manifest.get("generated_visuals", []):
            item["file"] = str(package_path(root, item["file"]))
        for key in ("source", "model"):
            if key in manifest:
                manifest[key] = str(package_path(root, manifest[key]))
        manifest["source_asset_sha256"] = {str(package_path(root, key)): value
                                           for key, value in manifest.get("source_asset_sha256", {}).items()}
    independent = manifest.get("schema") in ("task-box-layout/2", "task-box-layout/3")
    if manifest.get("schema") not in SUPPORTED_LAYOUT_SCHEMAS or not manifest.get("adapted_base"):
        raise ValueError("Capture requires an adapted task-box layout manifest")
    if hashlib.sha256(model.read_bytes()).hexdigest() != manifest["model_sha256"]:
        raise ValueError("Layout manifest does not match the selected model")
    from bsk_render_adapter.portable_assets import catalog_document
    assets = catalog_document(catalog)
    if Path(assets["source_mjcf"]).resolve() != model:
        raise ValueError("Capture catalog belongs to a different model")
    visual = Path(manifest["adapted_base"]["visual"]).resolve()
    entry = assets["assets"].get(str(visual), {})
    if entry.get("source_sha256") != hashlib.sha256(visual.read_bytes()).hexdigest():
        raise ValueError("Capture catalog does not contain the adapted base visual")
    if independent:
        from .independent_task_box_layout import PICKUPS, TARGETS, validate_layout_record
        validate_layout_record(manifest)
        visuals = manifest.get("generated_visuals", [])
        if len(visuals) != 12 or {v["name"] for v in visuals} != {*PICKUPS, *TARGETS, "task_box_base"}:
            raise ValueError("Independent layout requires all 12 generated visual assets")
        for generated in visuals:
            file = Path(generated["file"]).resolve()
            sha = hashlib.sha256(file.read_bytes()).hexdigest()
            if sha != generated["sha256"] or assets["assets"].get(str(file), {}).get("source_sha256") != sha:
                raise ValueError(f"Capture catalog has a missing or stale independent visual: {file.name}")
    return manifest
