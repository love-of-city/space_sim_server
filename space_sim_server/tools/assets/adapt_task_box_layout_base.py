"""Relocate the two existing base apertures with their boards, offline only.

Requires trimesh, manifold3d and coacd in a separate geometry environment.
Original assets remain immutable. The two aperture outlines are sampled from
the delivered solid's mid-deck section and extruded through the deck. Old
apertures are filled, then the translated apertures are subtracted. Upper
latch features and the drawer opening are retained.
"""
from __future__ import annotations

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))


import argparse
import copy
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import coacd
import manifold3d
import numpy as np
import trimesh


def adapt(layout: Path):
    layout = layout.resolve()
    manifest_path = layout.with_suffix(".layout.json")
    record = json.loads(manifest_path.read_text())
    if record.get("adapted_base"):
        raise ValueError("This layout already has an adapted base")
    tree = ET.parse(layout)
    root = tree.getroot()
    asset = root.find('./asset/mesh[@name="task_box_base_mesh"]')
    base = root.find('.//body[@name="task_box_base"]')
    original_path = Path(asset.get("file"))
    mesh = trimesh.load_mesh(original_path, process=True)
    mesh.apply_scale(np.fromstring(asset.get("scale", "1 1 1"), sep=" "))
    if not mesh.is_volume:
        raise ValueError("Base must be a closed, consistently wound solid")
    original_volume = float(mesh.volume)
    # Read the actual through-opening contours at the middle of the deck. A
    # volume-box complement would also collect unrelated underside recesses.
    section = mesh.section(plane_origin=[0, 0, -.01], plane_normal=[0, 0, 1])
    if section is None:
        raise ValueError("Delivered base has no mid-deck section")
    contours = [section.vertices[e.points, :2] for e in section.entities]
    windows = {"pickup_board": ([.09, .10], [.29, .32]),
               "assembly_board": ([.27, .09], [.45, .41])}
    def aperture_solid(contour):
        area2 = np.sum(contour[:, 0] * np.roll(contour[:, 1], -1)
                       - contour[:, 1] * np.roll(contour[:, 0], -1))
        if area2 < 0:
            contour = contour[::-1]
        solid = manifold3d.CrossSection([contour]).extrude(.025).translate((0, 0, -.025)).to_mesh()
        return trimesh.Trimesh(vertices=np.asarray(solid.vert_properties)[:, :3],
                               faces=np.asarray(solid.tri_verts), process=True)

    drawer = [c for c in contours if np.all(c.min(0) >= [.22, 0])
              and np.all(c.max(0) <= [.44, .08])]
    if len(drawer) != 1:
        raise ValueError("Cannot identify the protected drawer opening")
    protected_drawer = aperture_solid(drawer[0])
    cavities, moved = {}, {}
    for name, (lower, upper) in windows.items():
        matching = [c for c in contours if np.all(c.min(0) >= lower)
                    and np.all(c.max(0) <= upper)]
        if len(matching) != 1:
            raise ValueError(f"Cannot identify the delivered opening for {name}")
        cavity = aperture_solid(matching[0])
        if not cavity.is_volume or not 1e-5 < cavity.volume < .002:
            raise ValueError(f"Unexpected original cavity for {name}")
        cavities[name] = cavity
        moved[name] = cavity.copy()
        moved[name].apply_translation([*record["offsets_body_xy_m"][name], 0])
        bounds = moved[name].bounds
        if np.any(bounds[0, :2] < [.003, .003]) or np.any(bounds[1, :2] > [.447, .547]):
            raise ValueError(f"{name}: moved aperture leaves less than 3 mm of outer frame")
        interference = trimesh.boolean.intersection([moved[name], protected_drawer], engine="manifold")
        if len(interference.faces) and interference.volume > 1e-10:
            raise ValueError(f"{name}: moved aperture overlaps the protected drawer opening")
    overlap = trimesh.boolean.intersection(list(moved.values()), engine="manifold")
    if len(overlap.faces) and overlap.volume > 1e-10:
        raise ValueError("Moved base apertures overlap")
    filled = trimesh.boolean.union([mesh, *cavities.values()], engine="manifold")
    result = trimesh.boolean.difference([filled, *moved.values()], engine="manifold")
    if not result.is_volume:
        raise ValueError("Adapted base is not a watertight solid")
    # Underside recesses make the deck nonuniform in thickness. Repositioning
    # an aperture can therefore slightly change material volume; report it,
    # and reject a large change rather than claiming identical inertia.
    if abs(result.volume-original_volume) > original_volume * .01:
        raise ValueError("Moved opening changes more than 1% of base volume")
    folder = layout.parent / "geometry"
    folder.mkdir(exist_ok=True)
    visual = folder / "task_box_base_layout.stl"
    result.export(visual)
    asset.set("file", str(visual))
    asset.set("scale", "1 1 1")
    original_geoms = [g for g in base.findall("geom")
                      if g.get("name", "").startswith("task_box_base_geom_collision_")]
    if not original_geoms:
        raise ValueError("Original base collision decomposition missing")
    template = copy.deepcopy(original_geoms[0])
    old_assets = {g.get("mesh") for g in original_geoms}
    for geom in original_geoms:
        base.remove(geom)
    assets = root.find("asset")
    for old in list(assets):
        if old.get("name") in old_assets:
            assets.remove(old)
    coacd.set_log_level("warn")
    started = time.monotonic()
    # The boolean result is already a validated watertight solid. Voxel
    # preprocessing can fragment these thin decks into thousands of pieces.
    # Keep the original surface and require explicit hole-clearance validation
    # of the bounded convex approximation below.
    convex = coacd.run_coacd(coacd.Mesh(np.asarray(result.vertices, dtype=np.float64),
                                       np.asarray(result.faces, dtype=np.int32)),
                             threshold=.02, max_convex_hull=128, preprocess_mode="off",
                             resolution=2000, mcts_iterations=100,
                             mcts_nodes=20, mcts_max_depth=3, seed=0)
    for index, (vertices, faces) in enumerate(convex):
        file = folder / f"base_collision_{index:03d}.obj"
        trimesh.Trimesh(vertices=vertices, faces=faces, process=False).export(file)
        name = f"task_box_layout_base_collision_{index:03d}"
        ET.SubElement(assets, "mesh", name=name, file=str(file))
        geom = copy.deepcopy(template)
        geom.set("name", name)
        geom.set("mesh", name)
        base.insert(index+1, geom)
    tree.write(layout, encoding="utf-8", xml_declaration=True)
    record["model_sha256"] = hashlib.sha256(layout.read_bytes()).hexdigest()
    record["adapted_base"] = dict(
        original_visual=str(original_path), visual=str(visual),
        source_visual_sha256=hashlib.sha256(original_path.read_bytes()).hexdigest(),
        original_volume_m3=original_volume, adapted_volume_m3=float(result.volume),
        material_volume_change_fraction=float(result.volume/original_volume-1),
        method="extrude delivered mid-deck aperture outlines, fill old apertures, subtract translated apertures; manifold solid booleans",
        collision_method="CoACD convex approximation; requires native insertion validation",
        collision_parts=len(convex), collision_threshold=.02,
        collision_preprocess="off", collision_max_parts=128,
        decomposition_seconds=time.monotonic()-started,
        cavity_bounds={k:v.bounds.tolist() for k,v in moved.items()})
    manifest_path.write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps(record["adapted_base"], indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("layout", type=Path)
    adapt(parser.parse_args().layout)
