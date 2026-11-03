"""Resize the authored target housing while keeping its original capture face.

This is an XML construction step, not a dynamics or mounting-clearance test.
The original four tags are separate render attachments and must use
``fixed_body_geometry``; this function never transforms those attachments.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np

from .scaling import scale_satellite


DEFAULT_PIVOT = (.28865, .04975, .04207870013)
INTERFACE_GEOMS = ("target_instance_0030_part_015_color_00",) + tuple(
    f"capture_x_piece_{i}" for i in range(898)
)
PROXY_GEOMS = tuple("capture_bus_" + n for n in ("core", "left", "right", "lower", "upper"))
REMOVED_INSTANCES = (45, 46, 48, 49, 50)
INVENTORY = Path(__file__).resolve().parents[2] / "model/ground_validation_satellite/articulation_review/assembly_inventory.json"


def _numbers(values):
    return " ".join(format(float(x), ".17g") for x in values)


def _digest(node):
    return hashlib.sha256(ET.tostring(node, encoding="utf-8")).hexdigest()


def _box(node, low, high):
    low, high = np.asarray(low), np.asarray(high)
    if np.any(high <= low):
        raise ValueError(f"Degenerate box {node.get('name')}")
    node.set("pos", _numbers((low + high) / 2))
    node.set("size", _numbers((high - low) / 2))
    return dict(name=node.get("name"), bounds_m=[low.tolist(), high.tolist()])


def resize_housing(root, factor=5.6, pivot=DEFAULT_PIVOT, *, clear_back_mount=False, back_center_xz=None):
    """Return a new XML tree and an explicit geometry-change manifest.

    Input must be the bare, original target with the original WF10 names.
    Only target-local coordinates change; the root pose, service spacecraft,
    arms and cameras are untouched. Source inertial masses are retained and
    inertia is multiplied by factor squared as an engineering approximation.
    The pivot must be the original +X/+Y wall intersection; arbitrary pivots
    would invalidate the fixed wall/interface contract and are rejected.
    """
    pivot = np.asarray(pivot, dtype=float)
    if pivot.shape != (3,) or not np.allclose(pivot, DEFAULT_PIVOT, atol=1e-10, rtol=0):
        raise ValueError("Preserved capture face requires the authored +X/+Y wall pivot")
    original = root.find('.//body[@name="capture_target"]')
    if original is None:
        raise ValueError("Missing capture_target")
    original_geoms = {g.get("name"): g for g in original.iter("geom")}
    if any(g.get("mesh", "").startswith("capture_target_scale_") for g in original_geoms.values()):
        raise ValueError("Resize the authored source, not an already scaled target")
    missing = set(INTERFACE_GEOMS).difference(original_geoms)
    if missing:
        raise ValueError(f"Missing original interface geometry: {sorted(missing)[:3]}")
    records = json.loads(INVENTORY.read_text(encoding="utf-8"))
    leaves = {r["id"]: r for r in records if not r.get("assembly") and r.get("bounds_m")}
    # Whole original parts wholly outside the +Y housing wall remain metric.
    # This includes both front plates and both original connector assemblies.
    front_parts = [r for r in leaves.values() if r["bounds_m"][0][1] >= pivot[1] - 1e-9]
    protected = set(INTERFACE_GEOMS)
    for record in front_parts:
        protected.update("target_" + name for name in record["geoms"])
    missing = protected.difference(original_geoms)
    if missing:
        raise ValueError(f"Missing original protected face parts: {sorted(missing)}")
    if any(original_geoms[n] not in list(original) for n in protected):
        raise ValueError("Protected face parts must remain direct children of capture_target")

    result, scaling = scale_satellite(root, "capture_target", factor, mass_rule="fixed_mass")
    target = result.find('.//body[@name="capture_target"]')
    shift = (1 - factor) * pivot
    # The ordinary scaler works about zero. Translate direct children once;
    # descendants already inherit the translation through their parent body.
    for node in target:
        if node.tag in ("body", "geom", "site", "inertial", "camera", "light"):
            position = np.fromstring(node.get("pos", "0 0 0"), sep=" ")
            node.set("pos", _numbers(position + shift))

    for node in list(target):
        if node.tag == "geom" and node.get("name") in protected:
            index = list(target).index(node)
            target.remove(node)
            target.insert(index, deepcopy(original_geoms[node.get("name")]))
    removed = []
    for node in list(target):
        match = re.match(r"target_instance_(\d+)_", node.get("name", ""))
        if node.tag == "geom" and match and int(match.group(1)) in REMOVED_INSTANCES:
            removed.append(dict(name=node.get("name"), reason="original +X protruding attachment at task-box mounting wall",
                                original_attributes=dict(original_geoms[node.get("name")].attrib)))
            target.remove(node)

    # The actual side-wall envelope excludes rear-cover ears and equipment.
    # Keep the fixed +X mounting wall; rear cover/wing meshes retain their own
    # explicit geometry and must be checked by the caller for task-box contact.
    left = leaves["instance_0027"]["bounds_m"]
    right = leaves["instance_0024"]["bounds_m"]
    rear = leaves["instance_0018"]["bounds_m"]
    original_low = np.array([left[0][0], rear[0][1], left[0][2]])
    original_high = np.array([right[1][0], pivot[1], right[1][2]])
    low, high = [pivot + factor * (bound - pivot) for bound in (original_low, original_high)]
    high[0], high[1] = pivot[0], pivot[1]
    center = (low + high) / 2
    if back_center_xz is not None:
        offset = np.asarray(back_center_xz, dtype=float)
        if offset.shape != (2,) or not np.all(np.isfinite(offset)):
            raise ValueError("Back mounting center must be two finite X/Z coordinates")
        if np.any(offset - [.225, .275] < low[[0, 2]] - 1e-9) or np.any(offset + [.225, .275] > high[[0, 2]] + 1e-9):
            raise ValueError("Back task box footprint exceeds the housing wall")
        center[[0, 2]] = offset
    if clear_back_mount:
        # Clear only obstructing rear-surface parts. Preserve the outer panel,
        # its hinge, all body frames, and the complete protected front face.
        footprint_low = center[[0, 2]] - [.225, .275]
        footprint_high = center[[0, 2]] + [.225, .275]
        inventory_by_geom = {'target_' + name: record for record in leaves.values() for name in record['geoms']}
        inner = target.find('body[@name="satellite_inner_panel"]')
        for parent in (target, inner):
            for node in list(parent):
                name = node.get('name', '')
                if node.tag != 'geom' or name in protected:
                    continue
                record = inventory_by_geom.get(name)
                # 0077 is an authored fixed rear-edge protrusion, encountered
                # when the box shifts toward the low-Z end of the wide wall.
                eligible = parent is inner or any(name.startswith(f'target_instance_{i:04d}_') for i in (77, *range(222, 226)))
                overlap = False
                if eligible and record:
                    bounds = pivot + factor * (np.asarray(record['bounds_m']) - pivot)
                    overlap = bool(np.all(bounds[1, [0, 2]] > footprint_low) and np.all(bounds[0, [0, 2]] < footprint_high))
                if overlap or name == 'satellite_inner_panel_collision':
                    removed.append(dict(name=name, reason='rear mounting footprint obstruction; associated fixed-panel proxy removed with panel', original_attributes=dict(original_geoms[name].attrib)))
                    parent.remove(node)

    # Five active collision proxies retain the original interface's X/Z
    # aperture. The front band ends at original Y=.05475, behind WF10 and tags.
    iface = leaves["instance_0030"]["bounds_m"]
    x0, x1, z0, z1 = iface[0][0], iface[1][0], iface[0][2], iface[1][2]
    front_y = pivot[1] + .005
    sections = {
        "capture_bus_core": (low, high),
        "capture_bus_left": ([low[0], high[1], low[2]], [x0, front_y, high[2]]),
        "capture_bus_right": ([x1, high[1], low[2]], [high[0], front_y, high[2]]),
        "capture_bus_lower": ([x0, high[1], low[2]], [x1, front_y, z0]),
        "capture_bus_upper": ([x0, high[1], z1], [x1, front_y, high[2]]),
    }
    rebuilt = []
    for name, (a, b) in sections.items():
        node = target.find(f'geom[@name="{name}"]')
        if node is None:
            raise ValueError(f"Missing original collision proxy {name}")
        # Restore original contact parameters instead of inheriting any scale
        # mutation of margin/gap; these are authored metre/contact contracts.
        node.attrib.clear()
        node.attrib.update(original_geoms[name].attrib)
        if node.get("contype") == "0" and node.get("conaffinity") == "0":
            raise ValueError(f"Original active collision proxy is disabled: {name}")
        rebuilt.append(_box(node, a, b))
    legacy = target.find('geom[@name="capture_target_collision"]')
    if legacy is not None:
        debug_high = high.copy()
        debug_high[1] = front_y
        rebuilt.append(_box(legacy, low, debug_high))

    # Extend the original front plate at Y<=.04975. Its right edge already
    # meets the fixed +X wall, so split the left strip into two: four valid
    # non-overlapping rectangles without zero-size or duplicate right strips.
    patch = np.array(leaves["instance_0006"]["bounds_m"], dtype=float)
    added = []
    if factor > 1:
        px0, px1, pz0, pz1 = patch[0, 0], patch[1, 0], patch[0, 2], patch[1, 2]
        middle = (low[2] + high[2]) / 2
        rectangles = {
            "left_lower": (low[0], px0, low[2], middle),
            "left_upper": (low[0], px0, middle, high[2]),
            "lower": (px0, px1, low[2], pz0),
            "upper": (px0, px1, pz1, high[2]),
        }
        for label, (ax0, ax1, az0, az1) in rectangles.items():
            node = ET.SubElement(target, "geom", name="housing_front_extension_" + label,
                                 type="box", mass="0", contype="0", conaffinity="0", group="2",
                                 rgba="0.64 0.68 0.73 1")
            added.append(_box(node, [ax0, pivot[1] - .003, az0], [ax1, pivot[1], az1]))

    # Protected mesh assets are the unchanged originals; remove unreferenced
    # scaled duplicates so render/provenance code cannot select the wrong one.
    used_meshes = {n.get("mesh") for n in result.iter() if n.get("mesh")}
    assets = result.find("asset")
    dropped_clones = []
    for mesh in list(assets):
        if mesh.tag == "mesh" and mesh.get("name") in scaling["cloned_meshes"].values() and mesh.get("name") not in used_meshes:
            dropped_clones.append(mesh.get("name"))
            assets.remove(mesh)
    scaling["cloned_meshes"] = {old: new for old, new in scaling["cloned_meshes"].items() if new in used_meshes}
    removed_names = {r["name"] for r in removed}
    rebuilt_names = {r["name"] for r in rebuilt}
    scaled = sorted(set(original_geoms) - protected - removed_names - rebuilt_names)
    target_geoms = {g.get("name"): g for g in target.iter("geom")}
    protected_digests = {name: _digest(target_geoms[name]) for name in sorted(protected)}
    if any(protected_digests[n] != _digest(original_geoms[n]) for n in protected):
        raise AssertionError("A protected original geom was modified")
    masses = [float(n.get("mass")) for n in original.iter("inertial")]
    report = dict(
        schema="preserved-target-housing/1", body="capture_target", uniform_housing_scale=factor,
        pivot_m=pivot.tolist(), scaling=scaling, protected_geometry=sorted(protected),
        rear_mount_cleared=clear_back_mount,
        back_mount_center_xz_m=center[[0, 2]].tolist(),
        protected_geometry_sha256=protected_digests, protected_interface_geometry=list(INTERFACE_GEOMS),
        protected_face_wholeparts=[dict(instance=r["id"], bounds_m=r["bounds_m"], geoms=r["geoms"]) for r in front_parts],
        protected_wholepart_selection="authored inventory leaf minY >= original front wall Y=.04975",
        removed_geometry=removed, rebuilt_geometry=rebuilt, scaled_geometry=scaled, added_geometry=added,
        unused_scaled_meshes_removed=dropped_clones, original_housing_bounds_m=[original_low.tolist(), original_high.tolist()],
        housing_bounds_m=[low.tolist(), high.tolist()], original_front_patch_bounds_m=patch.tolist(),
        interface_aperture_xz_m=[[x0, z0], [x1, z1]], collision_front_y_m=front_y,
        annulus_collision="visible strips covered by the five active housing collision proxies",
        source_declared_inertial_mass_kg=sum(masses), target_declared_inertial_mass_kg=sum(float(n.get("mass")) for n in target.iter("inertial")),
        inertia_assumption="source declared masses fixed; all source inertial tensors multiplied by scale squared; not an exact redistributed CAD inertia",
        inventory=str(INVENTORY), inventory_sha256=hashlib.sha256(INVENTORY.read_bytes()).hexdigest(),
        render_tag_contract="original target_aruco_5cm geometry with fixed_body_geometry; no offsets/scaling",
        validation="XML construction only; caller must verify actual meshes, clearance, native dynamics and UE views",
    )
    return result, report
