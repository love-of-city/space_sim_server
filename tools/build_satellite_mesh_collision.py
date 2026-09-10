"""Build an isolated, original-triangle collision candidate (NOT the default scene).

MuJoCo 3.7 rigid flexes retain non-convex OBJ triangle surfaces. No convex hull,
box fitting, decimation or source-file rewriting. Rendering stays on the original
mesh geoms. Each surface gets an explicit contact skin; this is not CAD-exact
volumetric contact and does not invent the missing hinge internals.

NumPy is required for the numerical-degeneracy audit. Validate using the runtime-matched
MuJoCo environment AND the actual Basilisk MJScene before considering adoption.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "model/ground_validation_satellite/ground_validation_satellite_articulated.xml"
BASE = ROOT / "model/SARM/platform/sarm_ground_target.xml"
DEFAULT_OUTPUT = SOURCE.parent / "mesh_collision_trial"
SCHEMA = "original-triangle-collision-trial/1"
DEFAULT_RADIUS_M = 1e-5  # 10 micrometres on EACH surface, explicitly not zero.
TARGET_BODIES = ("capture_target", "satellite_inner_panel", "satellite_outer_panel")


def digest(path):
    path = Path(path)
    data = path.read_bytes()
    if path.suffix in (".xml", ".json"):
        data = data.replace(b"\r\n", b"\n")
    return hashlib.sha256(data).hexdigest()


def relative(path, directory):
    return Path(os.path.relpath(path, directory)).as_posix()


def load_obj(path):
    import numpy as np
    vertices, faces = [], []
    with Path(path).open(encoding="utf-8") as stream:
        for line in stream:
            if line.startswith("v "):
                vertices.append([float(x) for x in line.split()[1:4]])
            elif line.startswith("f "):
                fields = line.split()[1:]
                if len(fields) != 3:
                    raise ValueError(f"Non-triangle face in {path}; refusing silent triangulation")
                ids = [int(x.split("/")[0]) for x in fields]
                faces.append([i-1 if i > 0 else len(vertices)+i for i in ids])
    if not vertices or not faces:
        raise ValueError(f"Missing real triangle data in {path}; check Git LFS")
    v, f = np.asarray(vertices, dtype=np.float64), np.asarray(faces, dtype=np.int32)
    if not np.isfinite(v).all() or f.min() < 0 or f.max() >= len(v):
        raise ValueError(f"Invalid mesh data in {path}")
    return v, f


def audit_obj(path):
    """Only remove faces collapsing to zero area at MuJoCo's float32 OBJ precision.

    Source cut surfaces contain repeated vertices. They are legal visual mesh
    input but illegal flex elements. Record every removed index and its original
    area; never close holes or drop an ordinary non-degenerate surface.
    """
    import numpy as np
    v, f = load_obj(path)
    quantized = v.astype(np.float32).astype(np.float64)
    error = float(np.max(np.linalg.norm(v - quantized, axis=1)))
    if error > 1e-7:
        raise ValueError(f"OBJ precision loss exceeds 0.1 micrometre: {path}: {error}")
    tri = quantized[f]
    collapsed = np.linalg.norm(np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0]), axis=1) == 0
    original = v[f[collapsed]]
    lost_area = float(np.linalg.norm(np.cross(original[:, 1]-original[:, 0], original[:, 2]-original[:, 0]), axis=1).sum()/2)
    total = v[f]
    source_area = float(np.linalg.norm(np.cross(total[:, 1]-total[:, 0], total[:, 2]-total[:, 0]), axis=1).sum()/2)
    if lost_area > max(1e-12, source_area * 1e-9):
        raise ValueError(f"Degenerate-face cleanup loses measurable surface area: {path}: {lost_area}")
    cleaned = None
    if collapsed.any():
        # Retain every original vertex coordinate. MuJoCo's importer itself
        # performs float32 storage and duplicate-vertex merging.
        lines = ["# Original coordinates; only float32-zero-area faces removed; see manifest."]
        lines.extend("v " + " ".join(format(float(x), ".17g") for x in row) for row in v)
        lines.extend("f " + " ".join(str(int(x)+1) for x in row) for row in f[~collapsed])
        cleaned = ("\n".join(lines) + "\n").encode()
    info = {"vertices": len(v), "triangles": len(f), "retained_triangles": int((~collapsed).sum()),
            "removed_float32_zero_area_face_indices": np.flatnonzero(collapsed).tolist(),
            "removed_original_area_m2": lost_area, "source_area_m2": source_area,
            "max_import_coordinate_error_m": error}
    return info, cleaned


def build_candidates(output=DEFAULT_OUTPUT, radius_m=DEFAULT_RADIUS_M):
    output = Path(output).resolve()
    if not math.isfinite(radius_m) or not 0 < radius_m <= 0.00025:
        raise ValueError("radius_m must be positive and <= 0.25 mm; do not hide a coarse collision skin")
    if output in (SOURCE.parent.resolve(), BASE.parent.resolve()):
        raise ValueError("Use a dedicated trial directory, not a source/runtime directory")
    base_manifest = BASE.with_suffix(".manifest.json")
    if base_manifest.is_file():
        prior = json.loads(base_manifest.read_text(encoding="utf-8"))
        if prior["sha256"] != digest(BASE) or prior["sources"].get(relative(SOURCE, ROOT)) != digest(SOURCE):
            raise ValueError("Base scene is stale relative to source XML; regenerate build_sarm_ground_target.py first")
    root = ET.parse(BASE).getroot()
    compiler = root.find("compiler")
    compiler.set("meshdir", ".")
    meshes = {}
    for asset in root.findall("./asset/mesh[@file]"):
        file = (BASE.parent / asset.get("file")).resolve()
        meshes[asset.get("name")] = (file, asset.get("scale", "1 1 1"))
        asset.set("file", relative(file, output))
    target = root.find('./worldbody/body[@name="capture_target"]')
    if target is None:
        raise ValueError("Combined base has no capture_target")
    root.set("model", "SARM_original_triangle_collision_EXPERIMENT")
    root.insert(0, ET.Comment(" EXPERIMENTAL: original OBJ triangles, with audited numerical-degeneracy cleanup; 10 kg inertia still synthetic. No production scene is switched. See manifest and validation report. "))
    records = []
    file_records = {}
    cleaned_assets = {}
    collision_paths = {}
    for body in target.iter("body"):
        name = body.get("name")
        if name not in TARGET_BODIES:
            raise ValueError(f"Unexpected target body: {name}")
        for geom in list(body.findall("geom")):
            if geom.get("name") == name + "_collision":
                body.remove(geom)  # Remove the three old boxes, not source surfaces.
        for geom in body.findall("geom"):
            if not geom.get("mesh") or geom.get("group") != "2":
                raise ValueError(f"Unexpected target geometry: {geom.attrib}")
            file, scale = meshes[geom.get("mesh")]
            if file not in file_records:
                info, cleaned = audit_obj(file)
                collision_paths[file] = file
                if cleaned is not None:
                    name_in_output = f"cleaned_meshes/{file.stem}_{digest(file)[:10]}.obj"
                    cleaned_assets[name_in_output] = cleaned
                    collision_paths[file] = output / name_in_output
                file_records[file] = {"sha256": digest(file), **info}
            # Fixed body + fixed inner plate share a welded body. Do not collide
            # their rigidly overlapping assembly surfaces with one another.
            # Moving-vs-fixed is ENABLED, including every split hinge surface.
            moving = name == "satellite_outer_panel"
            flex_name = "contact_" + geom.get("name")
            flex = ET.Element("flexcomp", name=flex_name, type="mesh", dim="2", rigid="true",
                              file=relative(collision_paths[file], output), scale=scale,
                              pos=geom.get("pos", "0 0 0"), quat=geom.get("quat", "1 0 0 0"),
                              radius=format(radius_m, ".12g"), group="3", rgba="0.15 0.85 0.4 0")
            ET.SubElement(flex, "contact", contype="4" if moving else "2", conaffinity="3" if moving else "5",
                          selfcollide="none", internal="false", condim="3", margin="0", gap="0",
                          friction="1 0.005 0.0005", solref="0.01 1")
            # No child rigid bodies/DOFs are generated because rigid=true.
            body.append(flex)
            records.append({"flex": flex_name, "body": name, "visual_geom": geom.get("name"),
                            "source_obj": relative(file, ROOT), "collision_file": relative(collision_paths[file], output), "scale": scale,
                            "pos": geom.get("pos", "0 0 0"), "quat": geom.get("quat", "1 0 0 0"),
                            **file_records[file]})
    contact = root.find("contact")
    if contact is not None:
        for entry in list(contact):
            if entry.tag == "exclude" and entry.get("body1") in TARGET_BODIES and entry.get("body2") in TARGET_BODIES:
                contact.remove(entry)
    # Original visual mesh geoms remain disabled for physics. All moving/fixed
    # target contact is evaluated on the rigid triangle flexes, not mesh hulls.
    standalone = copy.deepcopy(root)
    standalone.set("model", "satellite_original_triangle_collision_EXPERIMENT")
    world = standalone.find("worldbody")
    for body in list(world):
        if body.tag == "body" and body.get("name") != "capture_target":
            world.remove(body)
    standalone.find('./worldbody/body[@name="capture_target"]').set("pos", "0 0 0")
    for tag in ("actuator", "custom", "contact", "keyframe"):
        elem = standalone.find(tag)
        if elem is not None:
            standalone.remove(elem)
    for asset in list(standalone.find("asset")):
        if asset.tag == "mesh" and not asset.get("name", "").startswith("target_"):
            standalone.find("asset").remove(asset)
    outputs = dict(cleaned_assets)
    settings = BASE.with_name("attitude_control.json")
    source_paths = [SOURCE, BASE]
    if settings.is_file():
        # Native SARM loads this sibling file. Copy the unchanged configuration,
        # not a new controller tune, so the isolated combined candidate can run.
        outputs["attitude_control.json"] = settings.read_bytes().replace(b"\r\n", b"\n")
        source_paths.append(settings)
    for name, tree in (("sarm_mesh_collision.xml", root), ("satellite_mesh_collision.xml", standalone)):
        ET.indent(tree, space=" ")
        outputs[name] = ET.tostring(tree, encoding="utf-8", xml_declaration=True) + b"\n"
    manifest = {
        "schema": SCHEMA, "experimental_only": True, "default_scene_changed": False,
        "method": "MuJoCo rigid flexcomp type=mesh, dim=2; original OBJ triangle surfaces, no hull or decimation",
        "validation_runtime_mujoco_version": "3.7.0",
        "contact_radius_per_surface_m": radius_m,
        "two_surface_contact_envelope_m": 2 * radius_m,
        "synthetic_total_mass_kg": sum(float(i.get("mass")) for i in target.findall(".//inertial")), "engineering_inertia_known": False,
        "source_cad_mesh_linear_deflection_m": 0.00025,
        "hinge_internals_reconstructed": False,
        "hinge_split_surfaces_included_in_collision": True,
        "source_hashes": {relative(p, ROOT): digest(p) for p in source_paths},
        "surface_count": len(records), "unique_source_meshes": len(file_records),
        "triangle_instances": sum(r["triangles"] for r in records),
        "vertex_instances": sum(r["vertices"] for r in records),
        "retained_triangle_instances": sum(r["retained_triangles"] for r in records),
        "max_float32_import_coordinate_error_m": max(r["max_import_coordinate_error_m"] for r in records),
        "total_removed_original_area_m2": sum(r["removed_original_area_m2"] for r in records),
        "cleaned_mesh_files": list(cleaned_assets),
        "cleanup_policy": "Only triangles with zero area after MuJoCo float32 OBJ import; original vertex coordinates retained; per-face audit recorded",
        "collision_filter": {"robot_contype": 1, "fixed_target_contype": 2, "moving_target_contype": 4,
                             "fixed_target_conaffinity": 5, "moving_target_conaffinity": 3,
                             "body_pair_exclusions": [], "same_rigid_surface_selfcollision": False},
        "surfaces": records,
        "output_sha256": {name: hashlib.sha256(value).hexdigest() for name, value in outputs.items()},
    }
    outputs["manifest.json"] = (json.dumps(manifest, indent=2, ensure_ascii=False) + "\n").encode()
    return outputs, manifest


def write_candidates(output=DEFAULT_OUTPUT, radius_m=DEFAULT_RADIUS_M, check=False):
    output = Path(output).resolve()
    outputs, manifest = build_candidates(output, radius_m)
    if output.exists():
        marker = output / "manifest.json"
        if any(output.iterdir()) and (not marker.is_file() or json.loads(marker.read_text()).get("schema") != SCHEMA):
            raise ValueError("Refusing to overwrite a non-trial directory")
    if not check:
        output.mkdir(parents=True, exist_ok=True)
    for name, data in outputs.items():
        path = output / name
        if check:
            if not path.is_file() or path.read_bytes().replace(b"\r\n", b"\n") != data:
                raise ValueError(f"Stale mesh-collision candidate: {path}")
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--radius-m", type=float, default=DEFAULT_RADIUS_M)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    report = write_candidates(args.output, args.radius_m, args.check)
    print(json.dumps({"output": str(args.output), "surfaces": report["surface_count"],
                      "triangle_instances": report["triangle_instances"], "radius_m": args.radius_m}))


if __name__ == "__main__":
    main()
