"""Prepare SARM catalogs and UE assets without a PowerShell dependency.

--catalog-only generates local paths and mesh staging files without launching UE.
It does not certify that Unreal assets have been imported or rendered.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "backend"), str(ROOT / "tools")]

from select_sarm_scene import selected_scene
from space_arm_platform.scene_targets import DEFAULT_TEMPLATE, SARM_TEMPLATES
from space_arm_platform.unreal_runtime import default_unreal_root, unreal_editor


def asset_files(project: Path, catalog: dict) -> list[Path]:
    paths = [entry["asset_path"] for entry in catalog["assets"].values()]
    paths.extend(catalog.get("textures", {}).values())
    result = []
    content = (project / "Content").resolve()
    for object_path in paths:
        package = object_path.split(".", 1)[0]
        if not package.startswith("/Game/"):
            raise ValueError(f"Asset must be under /Game/: {object_path}")
        path = (content / (package[len("/Game/"):] + ".uasset")).resolve()
        if not path.is_relative_to(content):
            raise ValueError(f"Asset escapes Content: {object_path}")
        result.append(path)
    return result


def prepare(args):
    project = args.adapter.expanduser().resolve() / "Unreal/BskUnrealRenderer"
    project_file = project / "BskUnrealRenderer.uproject"
    if not project_file.is_file():
        raise FileNotFoundError(project_file)
    editor = None if args.catalog_only else unreal_editor(args.unreal)
    scripts = project / "scripts"
    plugin_args = []
    catalog_name, destination = "sarm_platform", "/Game/BSK/Generated/SARM"
    if args.template_id in SARM_TEMPLATES:
        if not getattr(args, "use_existing_model", False):
            subprocess.run([sys.executable, str(ROOT / "tools/build_sarm_20261012.py"),
                            "--fastener-model", "sdf" if args.template_id.endswith("-sdf") else "captive"], check=True)
        subprocess.run([sys.executable, str(ROOT / "tools/build_sarm_plugin.py")], check=True)
        manifest = json.loads((ROOT / "run/native_acceleration/sarm_thread.json").read_text())
        plugin_args = ["--mujoco-plugin", str(ROOT / manifest["library"])]
        catalog_name, destination = "sarm_20261012", "/Game/BSK/Generated/Sarm20261012"
    elif args.template_id == "sarm-task-box-contacts":
        catalog_name, destination = "sarm_task_box", "/Game/BSK/Generated/TaskBox"
    elif args.template_id == "sarm-task-box-free-plugs":
        catalog_name, destination = "sarm_task_box_plugs", "/Game/BSK/Generated/TaskBoxPlugs"
    scene = selected_scene(args.model_root, args.template_id, check=True)
    output = project / "Saved/AssetImport"
    output.mkdir(parents=True, exist_ok=True)
    catalog_path = output / f"{catalog_name}.catalog.json"
    settings = output / f"{catalog_name}.catalog.import.json"
    subprocess.run([
        sys.executable, str(scripts / "generate_mjcf_asset_catalog.py"),
        "--mjcf", str(scene), "--destination", destination,
        "--catalog", str(catalog_path), "--import-settings", str(settings),
        "--mesh-cache", str(output / f"{catalog_name}.catalog_mesh_cache"),
        "--normal-mode", "preserve", "--stl-smoothing-angle", "60",
        "--build-scale", "100", "--component-scale", "1", *plugin_args,
    ], check=True)
    catalog = json.loads(catalog_path.read_text(encoding="utf-8"))
    expected = asset_files(project, catalog)
    if args.catalog_only:
        print(json.dumps({"stage": "catalog_only", "catalog": str(catalog_path),
                          "expected_assets": len(expected), "ue_import_verified": False}))
        return
    signature = hashlib.sha256(catalog_path.read_bytes()).hexdigest()
    marker = output / f"{catalog_name}.linux-import.json"
    previous = {}
    if marker.is_file():
        try:
            previous = json.loads(marker.read_text())
        except ValueError:
            pass
    # Include importer policy so later settings changes invalidate the cache.
    policy = "preserve|stlAngle=60|build=100|component=1|nanite=off|lod0=full|v=3"
    config_hash = hashlib.sha256((scripts / "configure_imported_meshes.py").read_bytes()).hexdigest()
    target = {"catalog_sha256": signature, "policy": policy, "config_sha256": config_hash}
    if not args.force and previous == target and all(path.is_file() for path in expected):
        print(f"Verified cached asset import: {catalog_path}")
        return
    # Never leave a successful marker across an interrupted or failed reimport.
    marker.unlink(missing_ok=True)
    subprocess.run([
        str(editor), str(project_file), "-run=ImportAssets",
        f"-importsettings={settings}", f"-dest={destination}",
        "-replaceexisting", "-nosourcecontrol", "-unattended", "-nop4", "-nosplash",
        "-RenderOffscreen",
    ], check=True)
    missing = [path for path in expected if not path.is_file()]
    if missing:
        raise RuntimeError(f"UE import missing {len(missing)} assets; first: {missing[0]}")
    result_path = output / f"mesh-result-{uuid.uuid4().hex}.json"
    env = {**os.environ, "BSK_MJCF_ASSET_CATALOG": str(catalog_path),
           "BSK_MJCF_NORMAL_MODE": "preserve", "BSK_MJCF_RESULT_PATH": str(result_path)}
    try:
        subprocess.run([
            str(editor), str(project_file),
            f"-ExecutePythonScript={scripts / 'configure_imported_meshes.py'}",
            "-nullrhi", "-unattended", "-nop4", "-nosplash",
        ], env=env, check=True)
        # UE may exit zero despite a Python exception. Require explicit success.
        result = json.loads(result_path.read_text())
        if result != {"configured": len(catalog["assets"]), "catalog_sha256": signature}:
            raise RuntimeError(f"UE mesh configuration acknowledgement mismatch: {result}")
    finally:
        result_path.unlink(missing_ok=True)
    marker.write_text(json.dumps(target, indent=2) + "\n", encoding="utf-8")
    print(f"Imported and configured {len(expected)} assets: {catalog_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--adapter", type=Path, default=ROOT.parent / "space_sim_UE_Adapter")
    parser.add_argument("--model-root", type=Path, default=ROOT / "model/SARM/platform")
    parser.add_argument("--unreal", type=Path, default=default_unreal_root())
    parser.add_argument("--template-id", default=DEFAULT_TEMPLATE)
    parser.add_argument("--catalog-only", action="store_true")
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--use-existing-model", action="store_true",
                        help="Validate the selected model provenance without regenerating tracked XML at startup")
    prepare(parser.parse_args())


if __name__ == "__main__":
    main()
