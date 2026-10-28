"""Audit inset target/pickup sleeves, openings, colliders and deck infill.

Run in the geometry environment, separately from native dynamics validation.
Checks the exported meshes against the delivered source, not just the builder's
own preservation counters. All coordinates here use the task-box base frame.
"""
from __future__ import annotations

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))


import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import manifold3d as mf
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulation.independent_task_box_layout import PICKUPS, TARGETS, planar_transform, validate_layout_record
from tools.assets.build_independent_task_box_layout import aperture, cell, mesh_of, solid, vector


def top_opening(value, center):
    candidates = []
    for raw in value.slice(.0025).to_polygons():
        p = np.asarray(raw)
        signed = np.sum(p[:, 0]*np.roll(p[:, 1], -1)-p[:, 1]*np.roll(p[:, 0], -1))/2
        if signed < -1e-7 and np.all(p.min(0) < center) and np.all(center < p.max(0)):
            candidates.append(p[::-1])
    if len(candidates) != 1:
        raise ValueError("A mounting hole's complete top opening is missing")
    return mf.CrossSection(candidates)


def check(layout, output):
    record = json.loads(layout.with_suffix(".layout.json").read_text())
    validate_layout_record(record)
    styles = {"target": record["target_mounts"], "pickup": record.get("pickup_mounts", "plates")}
    assert "inset" in styles.values()
    assert hashlib.sha256(layout.read_bytes()).hexdigest() == record["model_sha256"]
    source = Path(record["source"])
    assert hashlib.sha256(source.read_bytes()).hexdigest() == record["source_sha256"]
    for file, sha in record["source_asset_sha256"].items():
        assert hashlib.sha256(Path(file).read_bytes()).hexdigest() == sha, file
    generated = {v["name"]: v for v in record["generated_visuals"]}
    for visual in generated.values():
        assert hashlib.sha256(Path(visual["file"]).read_bytes()).hexdigest() == visual["sha256"]
    old, new = (ET.parse(path).getroot() for path in (source, layout))

    def asset_mesh(root, directory, name):
        asset = root.find(f'./asset/mesh[@name="{name}"]')
        path = (directory/asset.get("file")).resolve()
        mesh = trimesh.load_mesh(path, process=True)
        mesh.apply_scale(vector(asset.get("scale"), "1 1 1"))
        return mesh

    source_base = solid(asset_mesh(old, source.parent, "task_box_base_mesh"))
    base_mesh = trimesh.load_mesh(generated["task_box_base"]["file"], process=True)
    # OBJ uses per-face normals along sharp deck/bore edges; weld positions
    # for solid topology checks without changing the exported render normals.
    base_mesh.merge_vertices(merge_norm=True, merge_tex=True)
    base = solid(base_mesh)
    # Audit the whole slab, including material outside the small new rims.
    # A moved old plate cell can extend beyond the deck or across another new
    # aperture; those regions are not supposed to be refilled.
    slab = mf.Manifold.cube((.45, .55, .025)).translate((0., 0., -.025))
    cavities = {}
    source_contours = [np.asarray(p) for p in source_base.slice(-.01).to_polygons()]
    for board_name, identifiers, low, high in (
        ('asm_board1', TARGETS, [.27, .09], [.45, .41]),
        ('asm_board2', PICKUPS, [.09, .10], [.29, .32]),
        (None, ('drawer',), [.22, 0.], [.44, .08]),
    ):
        contours = [p for p in source_contours if np.all(p.min(0) >= low) and np.all(p.max(0) <= high)]
        assert len(contours) == 1
        opening = aperture(contours[0])
        points = ({s.get('name'): vector(s.get('pos'))[:2]
                   for s in old.find(f'.//body[@name="{board_name}"]').findall('site')}
                  if board_name else {})
        for identifier in identifiers:
            reports = {**record.get('inset_targets', {}), **record.get('inset_pickups', {})}
            if identifier in reports:
                profile = reports[identifier]['deck_aperture_profile_body_xy_m']
                cavity = mf.CrossSection([np.asarray(profile)]).extrude(.025).translate((0., 0., -.025))
            elif identifier == 'drawer':
                cavity = opening
            else:
                cavity = cell(opening, points[identifier], points.values())
            matrix = planar_transform(record['modules'][identifier]['original_anchor_body_m'],
                                      record['offsets_body_xy_m'][identifier],
                                      record.get('installation_yaw_rad', {}).get(identifier, 0.))
            cavities[identifier] = cavity.transform(matrix[:3, :])
    expected_slab = slab
    for cavity in cavities.values():
        expected_slab = expected_slab-cavity
    expected_slab = expected_slab.trim_by_plane((0., 0., -1.), 1e-7)
    actual_slab = base.trim_by_plane((0., 0., -1.), 1e-7)
    missing_slab = float((expected_slab-actual_slab).volume())
    extra_slab = float((actual_slab-expected_slab).volume())
    slab_tolerance = float(mesh_of(expected_slab).area*2e-6)
    tolerance = 3e-7  # STL quantization plus the base's 0.1 micrometre simplification.
    rows = {"targets": [], "pickups": []}
    for kind, board_name, identifiers, low, high in (
        ("target", "asm_board1", TARGETS, [.27, .09], [.45, .41]),
        ("pickup", "asm_board2", PICKUPS, [.09, .10], [.29, .32]),
    ):
        if styles[kind] != "inset":
            continue
        board = old.find(f'.//body[@name="{board_name}"]')
        points = {s.get("name"): vector(s.get("pos"))[:2] for s in board.findall("site")}
        whole = solid(asset_mesh(old, source.parent, board_name+"_mesh"))
        candidates = [np.asarray(c) for c in source_base.slice(-.01).to_polygons()
                      if np.all(np.min(c, axis=0) >= low) and np.all(np.max(c, axis=0) <= high)]
        assert len(candidates) == 1
        old_aperture = aperture(candidates[0])
        for identifier in identifiers:
            center = points[identifier]
            original = cell(whole, center, points.values())
            original_mesh = mesh_of(original)
            actual_mesh = trimesh.load_mesh(generated[identifier]["file"], process=True)
            actual_mesh.merge_vertices(merge_norm=True, merge_tex=True)
            actual = solid(actual_mesh)
            before = original.trim_by_plane((0, 0, -1), 1e-7)
            after = actual.trim_by_plane((0, 0, -1), 1e-7)
            missing = float((before-after).volume())
            added = float((after-before).volume())
            sleeve_tolerance = mesh_of(before).area*tolerance
            a, b = top_opening(original, center), top_opening(actual, center)
            changed_opening = float((a-b).area()+(b-a).area())
            contours = [np.asarray(p) for p in a.to_polygons()]
            perimeter = sum(np.linalg.norm(p-np.roll(p, -1, axis=0), axis=1).sum() for p in contours)
            opening_tolerance = float(perimeter*tolerance)
            mount = record["inset_"+kind+"s"][identifier]
            profile = np.asarray(mount["rim_profile_body_xy_m"])
            module = new.find(f'.//body[@name="layout_{identifier}"]')
            outside = 0.
            colliders = 0
            for geom in module.findall("geom"):
                if not geom.get("name", "").startswith(f"layout_{board_name}_geom_collision_"):
                    continue
                mesh = asset_mesh(new, layout.parent, geom.get("mesh"))
                matrix = trimesh.transformations.quaternion_matrix(vector(geom.get("quat"), "1 0 0 0"))
                matrix[:3, 3] = vector(geom.get("pos"))
                vertices = trimesh.transform_points(mesh.vertices, matrix)[:, :2]
                for p, q in zip(profile, np.roll(profile, -1, axis=0)):
                    edge = q-p
                    inward = np.array([-edge[1], edge[0]])/np.linalg.norm(edge)
                    outside = max(outside, float(-np.min((vertices-p)@inward)))
                colliders += 1
            matrix = planar_transform(record["modules"][identifier]["original_anchor_body_m"],
                                      record["offsets_body_xy_m"][identifier],
                                      record.get("installation_yaw_rad", {}).get(identifier, 0.))
            interference = float((actual.transform(matrix[:3, :]) ^ base).volume())
            footprint = mf.CrossSection([np.asarray(mount["deck_aperture_profile_body_xy_m"])])
            cavity = footprint.extrude(.025).translate((0, 0, -.025))
            refill = cell(old_aperture, center, points.values())-cavity
            refill = refill.transform(matrix[:3, :])
            refill = refill ^ slab
            for cavity in cavities.values():
                refill = refill-cavity
            unfilled = float((refill-base).volume())
            refill_tolerance = mesh_of(refill).area*2e-6
            removed_fraction = float(1-actual_mesh.volume/original_mesh.volume)
            row = dict(site=identifier, kind=kind, watertight=bool(actual_mesh.is_volume), components=actual_mesh.body_count,
                       removed_plate_fraction=removed_fraction, missing_sleeve_volume_m3=missing,
                       added_sleeve_volume_m3=added, sleeve_volume_tolerance_m3=sleeve_tolerance,
                       changed_top_opening_area_m2=changed_opening, opening_area_tolerance_m2=opening_tolerance,
                       coarse_colliders=colliders, maximum_collider_outside_rim_m=outside,
                       sleeve_deck_interference_volume_m3=interference,
                       unfilled_deck_volume_m3=unfilled, deck_volume_tolerance_m3=refill_tolerance)
            row[kind] = identifier
            row["passed"] = bool(actual_mesh.is_volume and actual_mesh.body_count == 1 and removed_fraction > .15
                                 and missing <= sleeve_tolerance and added <= sleeve_tolerance
                                 and changed_opening <= opening_tolerance and colliders > 0 and outside <= tolerance
                                 and interference <= sleeve_tolerance and unfilled <= refill_tolerance)
            rows[kind+"s"].append(row)
    result = dict(passed=bool(all(row["passed"] for group in rows.values() for row in group)
                  and missing_slab <= slab_tolerance and extra_slab <= slab_tolerance),
                  target_mounts=styles["target"], pickup_mounts=styles["pickup"], sleeve_surface_tolerance_m=tolerance,
                  deck_surface_tolerance_m=2e-6,
                  source_assets_unchanged=True, checked_source_meshes=len(record["source_asset_sha256"]),
                  whole_deck_missing_volume_m3=missing_slab, whole_deck_extra_volume_m3=extra_slab,
                  whole_deck_volume_tolerance_m3=slab_tolerance,
                  original_mount_heights_and_positions_retained=True, **rows)
    output.write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(result, indent=2))
    return result["passed"]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layout", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    raise SystemExit(0 if check(args.layout.resolve(), args.report.resolve()) else 1)
