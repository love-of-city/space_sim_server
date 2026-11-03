"""Import generated layout visuals into an isolated UE namespace and catalog.

Run with the simulation Python. Geometry generation and native validation are
separate. This never replaces the public scene's asset catalog or source mesh.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'backend'))
from space_arm_platform.unreal_runtime import unreal_editor


def unreal_command(unreal_root, project):
    return [str(unreal_editor(unreal_root)),str(project/'BskUnrealRenderer.uproject'),
            '-unattended','-nosplash','-nosourcecontrol','-NullRHI','-DDC=InstalledDerivedDataBackendGraph']
from simulation.independent_task_box_layout import PICKUPS, POSE_SCHEMA, SCHEMA, TARGETS, validate_layout_record, NATIVE_CANDIDATE_SCHEMA
from simulation.task_box_layout import validated_capture_layout


def prepare(args):
    model = args.layout.resolve()
    manifest = json.loads(model.with_suffix(".layout.json").read_text())
    expected = {*PICKUPS, *TARGETS, "task_box_base"}
    preview_candidate = getattr(args, 'preview_candidate', False)
    schemas = (SCHEMA, POSE_SCHEMA, NATIVE_CANDIDATE_SCHEMA) if preview_candidate else (SCHEMA, POSE_SCHEMA)
    if (manifest["schema"] not in schemas or {x["name"] for x in manifest["generated_visuals"]} != expected
            or hashlib.sha256(model.read_bytes()).hexdigest() != manifest["model_sha256"]):
        raise ValueError("Invalid independent layout manifest")
    validate_layout_record(manifest, allow_native_candidate=preview_candidate)
    # Never replace an existing import namespace, including public assets.
    destination = args.destination.rstrip("/")
    if not destination.startswith("/Game/BSK/Generated/") or ".." in destination or len(destination.split("/")) != 5:
        raise ValueError("Use a new /Game/BSK/Generated/<layout-name> namespace")
    project = args.adapter.resolve()/"Unreal/BskUnrealRenderer"
    if (project/"Content"/destination.removeprefix("/Game/")).exists():
        raise ValueError("Destination already exists; select a fresh layout namespace")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    merged = json.loads(args.base_catalog.read_text())
    reusable = {}
    if getattr(args, 'reuse_identical_assets', False):
        for item in merged['assets'].values():
            asset_path = item['asset_path'].split('.')[0]
            if asset_path.startswith('/Game/') and (project/'Content'/(asset_path[6:]+'.uasset')).is_file():
                reusable[item['source_sha256']] = item
    pending, reused = [], []
    root = ET.Element("mujoco", model="independent_layout_visuals")
    asset, body = ET.SubElement(root, "asset"), ET.SubElement(ET.SubElement(root, "worldbody"), "body")
    for visual in manifest["generated_visuals"]:
        path = Path(visual["file"])
        if hashlib.sha256(path.read_bytes()).hexdigest() != visual["sha256"]:
            raise ValueError(f"Generated visual changed: {path}")
        if visual['sha256'] in reusable:
            merged['assets'][str(path)] = reusable[visual['sha256']]
            reused.append(dict(source=str(path), sha256=visual['sha256'], asset=reusable[visual['sha256']]['asset_path']))
            continue
        pending.append(visual)
        ET.SubElement(asset, "mesh", name=visual["name"], file=str(path))
        ET.SubElement(body, "geom", type="mesh", mesh=visual["name"], group="2")
    minimal = output/"visuals.xml"
    ET.ElementTree(root).write(minimal, encoding="utf-8", xml_declaration=True)
    (output/'reused-assets.json').write_text(json.dumps(reused, indent=2)+'\n')
    if not pending:
        merged['source_mjcf'] = str(model)
        final = output/'layout.catalog.json'
        final.write_text(json.dumps(merged, indent=2)+'\n')
        if not preview_candidate:
            validated_capture_layout(model, final)
        print(json.dumps(dict(stage='complete', catalog=str(final), reused_visuals=len(reused))), flush=True)
        return
    catalog, settings = output/"new-assets.catalog.json", output/"new-assets.import.json"
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join([str(args.adapter.resolve()/"Adapters"), environment.get("PYTHONPATH", "")])

    def run(command, name, env=environment):
        with (output/(name+".log")).open("w") as log:
            subprocess.run(command, check=True, stdout=log, stderr=subprocess.STDOUT, env=env)
        print(json.dumps(dict(stage=name, output=str(output))), flush=True)

    run([sys.executable, str(project/"scripts/generate_mjcf_asset_catalog.py"), "--mjcf", str(minimal),
         "--destination", destination, "--catalog", str(catalog), "--import-settings", str(settings),
         "--mesh-cache", str(output/"mesh-cache"), "--normal-mode", "preserve", "--stl-smoothing-angle", "60",
         "--build-scale", "100", "--component-scale", "1"], "generate")
    common = unreal_command(args.unreal.resolve(),project)
    run([*common, "-run=ImportAssets", "-importsettings="+str(settings), "-dest="+destination,
         "-AllowCommandletRendering"], "import")
    # Explicit acknowledgement: UE sometimes exits 0 after a Python exception.
    acknowledgement = output/"configured.json"
    wrapper = output/"configure.py"
    generated = json.loads(catalog.read_text())
    expected_triangles = {generated["assets"][str(Path(visual["file"]))]["asset_path"]: visual["triangle_count"]
                          for visual in pending}
    wrapper.write_text("import runpy, json\nfrom pathlib import Path\n"
                       f"runpy.run_path({str(project/'scripts/configure_imported_meshes.py')!r}, run_name='__main__')\n"
                       "import unreal\n"
                       f"expected = {expected_triangles!r}\n"
                       "checks = []\n"
                       "for path, count in expected.items():\n"
                       "    mesh = unreal.load_asset(path)\n"
                       "    actual = mesh.get_num_triangles(0)\n"
                       "    if actual != count:\n"
                       "        raise RuntimeError(f'Generated LOD0 triangle loss: {path}: {actual} != {count}')\n"
                       "    checks.append(dict(asset=path, expected=count, actual=actual))\n"
                       f"Path({str(acknowledgement)!r}).write_text(json.dumps(dict(configured={len(expected_triangles)}, geometry=checks)))\n")
    configured_env = environment.copy()
    configured_env.update(BSK_MJCF_ASSET_CATALOG=str(catalog), BSK_MJCF_NORMAL_MODE="preserve")
    run([*common, "-ExecutePythonScript="+str(wrapper)], "configure", configured_env)
    if not acknowledgement.is_file() or json.loads(acknowledgement.read_text())["configured"] != len(pending):
        raise RuntimeError(f"UE configuration did not acknowledge all new assets; see {output/'configure.log'}")
    if len(generated["assets"]) != len(pending) or len(pending)+len(reused) != 12:
        raise ValueError("Generated and reused catalogs do not contain all 12 visual meshes")
    merged["assets"].update(generated["assets"])
    merged["source_mjcf"] = str(model)
    final = output/"layout.catalog.json"
    final.write_text(json.dumps(merged, indent=2)+"\n")
    if not preview_candidate:
        validated_capture_layout(model, final)
    print(json.dumps(dict(stage="complete", catalog=str(final), generated_visuals=12)), flush=True)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--layout", type=Path, required=True)
    p.add_argument("--base-catalog", type=Path, required=True)
    p.add_argument("--adapter", type=Path, required=True)
    p.add_argument("--unreal", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--destination", required=True)
    p.add_argument('--preview-candidate', action='store_true')
    p.add_argument('--reuse-identical-assets', action='store_true', help='Reuse installed base-catalog assets only on an exact source SHA-256 match')
    prepare(p.parse_args())
