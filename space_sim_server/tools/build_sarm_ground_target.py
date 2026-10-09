"""Compose SARM and the articulated satellite without modifying either source.

Only standard library dependencies. Checked-in output is used by both BSK and
UE asset import. Run --check in tests/startup to reject stale generated output.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "backend"))
from space_arm_platform.scene_targets import GROUND_TARGET_TEMPLATE, SELF_COLLISION_TEMPLATE, capture_target

BASE = ROOT / "model/SARM/platform/sarm_platform.xml"
SPEC = capture_target(GROUND_TARGET_TEMPLATE)
SOURCE = ROOT / SPEC.source_model
OUTPUT = BASE.with_name(SPEC.model_file)
MANIFEST = BASE.with_name("sarm_ground_target.manifest.json")
SELF_OUTPUT = BASE.with_name(capture_target(SELF_COLLISION_TEMPLATE).model_file)
HINGE_ENVELOPE_CLEARANCE_M = 0.001
# Conservative component AABBs from the source model at q=0, in each body's
# local frame. These collision/inertia proxies are NOT engineering geometry.
PROXIES = {
    "capture_target": (8.0, (0.20359075, 0.00475, 0.0), (0.16109075, 0.0785, 0.30007871)),
    "satellite_inner_panel": (1.0, (0.185, -0.07275, 0.0470787), (0.19950001, 0.01050001, 0.20725001)),
    "satellite_outer_panel": (1.0, (-0.18375, 0.0046, 0.0), (0.18625001, 0.01035001, 0.20725001)),
}


def values(items):
    return " ".join(format(float(v), ".14g") for v in items)


def internal_collision_geometry(body):
    """Two panel-only internal boxes; NEVER change visuals/external grasp boxes/inertia.

    The preview hinge is not an engineering contact assembly. Clear its envelope
    with a documented approximation instead of using overlapping AABBs as stops.
    This construction assumes this model's parallel panels and local Z hinge.
    """
    inner = body.find('.//body[@name="satellite_inner_panel"]')
    outer = body.find('.//body[@name="satellite_outer_panel"]')
    hinge = tuple(float(v) for v in outer.get("pos").split())
    joint = outer.find('joint[@name="outer_panel_hinge"]')
    axis = tuple(float(v) for v in joint.get("axis").split())
    if max(abs(axis[0]), abs(axis[1]), abs(abs(axis[2]) - 1)) > 1e-10:
        raise ValueError("Internal collision proxy construction requires the validated local Z hinge")
    if any(abs(float(v)) > 1e-10 for v in joint.get("pos", "0 0 0").split()):
        raise ValueError("Internal proxy construction expects the hinge at the outer body origin")
    for panel in (inner, outer):
        quat = tuple(float(v) for v in panel.get("quat", "1 0 0 0").split())
        if quat != (1., 0., 0., 0.) or any(key in panel.attrib for key in ("euler", "axisangle", "xyaxes", "zaxis")):
            raise ValueError("Internal proxy construction requires the validated parallel panel frames")
    # In the hinge frame, clear the larger original panel's transverse envelope
    # plus 1 mm. This is NOT a measured physical hinge gap or an angle limit.
    transverse = []
    for name, offset in (("satellite_inner_panel", hinge[1]), ("satellite_outer_panel", 0.)):
        _, center, half = PROXIES[name]
        transverse += [abs(center[1] - offset - half[1]), abs(center[1] - offset + half[1])]
    keepout = max(transverse) + HINGE_ENVELOPE_CLEARANCE_M
    boxes = {}
    for name in ("satellite_inner_panel", "satellite_outer_panel"):
        _, center, half = PROXIES[name]
        low, high = center[0] - half[0], center[0] + half[0]
        original_bounds = (low, high)
        if name == "satellite_inner_panel":
            low = hinge[0] + keepout
        else:
            high = -keepout
        if not original_bounds[0] <= low < high <= original_bounds[1]:
            raise ValueError("Internal collision hinge envelope is outside the source proxy")
        boxes[name] = {
            "center_m": [(low + high) / 2, center[1], center[2]],
            "half_size_m": [(high - low) / 2, half[1], half[2]],
            "original_x_bounds_m": list(original_bounds), "internal_x_bounds_m": [low, high],
        }
    return boxes, {
        "hinge_envelope_half_width_m": max(transverse),
        "extra_clearance_m": HINGE_ENVELOPE_CLEARANCE_M,
        "hinge_keepout_each_side_m": keepout,
        "zero_pose_internal_panel_gap_m": 2 * keepout,
        "geometry_is_approximate": True, "engineering_hinge_clearance_known": False,
        "external_collision_shapes_changed": False,
        "boxes": boxes,
        "pairs": [
            ["capture_target_collision", "satellite_outer_panel_internal_collision"],
            ["satellite_inner_panel_internal_collision", "satellite_outer_panel_internal_collision"],
        ],
    }


def composed_bytes(*, target_self_collision=False):
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    root = ET.parse(BASE, parser=parser).getroot()
    target_source = ET.parse(SOURCE).getroot()
    selected_output = SELF_OUTPUT if target_self_collision else OUTPUT
    selected_template = SELF_COLLISION_TEMPLATE if target_self_collision else GROUND_TARGET_TEMPLATE
    root.set("model", "SARM_ground_validation_capture_self_collision" if target_self_collision else "SARM_ground_validation_capture")
    root.insert(0, ET.Comment(" Generated by tools/build_sarm_ground_target.py. Target total mass 10 kg and AABB inertia/collision are SYNTHETIC, not measured hardware. Sources are unmodified. "))
    compiler = root.find("compiler")
    base_meshdir = compiler.get("meshdir", "")
    compiler.set("meshdir", ".")
    compiler.set("fusestatic", "false")
    assets = root.find("asset")
    for mesh in assets.findall("mesh[@file]"):
        resolved = (BASE.parent / base_meshdir / mesh.get("file")).resolve()
        mesh.set("file", Path(os.path.relpath(resolved, OUTPUT.parent)).as_posix())
    used_meshes = {g.get("mesh") for g in target_source.findall(".//geom[@mesh]")}
    source_meshdir = target_source.find("compiler").get("meshdir", "")
    for original in target_source.findall("./asset/mesh"):
        if original.get("name") not in used_meshes:
            continue
        mesh = copy.deepcopy(original)
        mesh.set("name", "target_" + mesh.get("name"))
        mesh.set("file", Path(os.path.relpath((SOURCE.parent / source_meshdir / mesh.get("file")).resolve(), OUTPUT.parent)).as_posix())
        assets.append(mesh)
    body = copy.deepcopy(target_source.find("./worldbody/body"))
    body.set("name", "capture_target")
    body.set("pos", values(SPEC.position_m))
    body.set("quat", values(SPEC.orientation_wxyz))
    body.insert(0, ET.Element("joint", name="capture_target_free", type="free", damping="0", frictionloss="0", stiffness="0", armature="0"))
    for geom in body.iter("geom"):
        geom.set("name", "target_" + geom.get("name"))
        geom.set("mesh", "target_" + geom.get("mesh"))
        geom.set("contype", "0")
        geom.set("conaffinity", "0")
        geom.set("mass", "0")
        geom.set("group", "2")
    internal_boxes, internal_metadata = internal_collision_geometry(body) if target_self_collision else ({}, {})
    for part in body.iter("body"):
        name = part.get("name")
        mass, center, half_size = PROXIES[name]
        inertia = [mass / 3 * sum(half_size[j] ** 2 for j in range(3) if j != i) for i in range(3)]
        for old in part.findall("inertial"):
            part.remove(old)
        part.insert(0, ET.Element("inertial", mass=str(mass), pos=values(center), diaginertia=values(inertia)))
        # The render adapter matches source geoms to MJScene's compiled order.
        # Keep all local geoms before child bodies (MuJoCo groups by body),
        # otherwise parent proxies shift the child's mesh/material metadata.
        # Robot geoms use bit 1. Target external boxes retain their shapes but
        # use bit 2, affinity 1: grasp contact on, target automatic self-contact off.
        part.append(ET.Element("geom", name=name + "_collision", type="box", pos=values(center), size=values(half_size), mass="0", contype="2" if target_self_collision else "1", conaffinity="1", condim="6", priority="1", friction="1 0.005 0.0005", solref="0.01 1", group="3", rgba="0.2 0.8 0.2 0"))
        if name in internal_boxes:
            box = internal_boxes[name]
            # Pair-only proxies: never add duplicate robot contacts or extra mass.
            part.append(ET.Element("geom", name=name + "_internal_collision", type="box", pos=values(box["center_m"]), size=values(box["half_size_m"]), mass="0", contype="0", conaffinity="0", group="3", rgba="0.8 0.5 0.2 0"))
        for child in list(part):
            if child.tag in ("body", "frame"):
                part.remove(child)
                part.append(child)
        for joint in part.findall("joint"):
            for field in ("damping", "frictionloss", "stiffness", "armature"):
                joint.set(field, "0")
    outer = body.find('.//body[@name="satellite_outer_panel"]')
    outer.append(ET.Element("site", name="capture_target_grasp", pos="-0.36 0.0046 0", size="0.004", group="4", rgba="1 0.8 0.1 0.5"))
    world = root.find("worldbody")
    world.remove(world.find('./body[@name="capture_target"]'))
    world.append(body)
    contact = root.find("contact")
    if contact is None:
        contact = ET.SubElement(root, "contact")
    if target_self_collision:
        # Explicit geom pairs bypass parent/weld filtering and bitmask filtering.
        # Do not globally disable filterparent: other SARM pairs remain unchanged.
        for index, (geom1, geom2) in enumerate(internal_metadata["pairs"]):
            ET.SubElement(contact, "pair", name=f"target_internal_contact_{index}",
                          geom1=geom1, geom2=geom2, condim="6",
                          friction="1 1 0.005 0.0005 0.0005", solref="0.01 1",
                          solimp="0.9 0.95 0.001", margin="0", gap="0")
    else:
        # Preserve the historical external-only artifact byte-for-byte. The
        # original conservative boxes overlap at the hinge.
        for parent in ("capture_target", "satellite_inner_panel"):
            ET.SubElement(contact, "exclude", body1=parent, body2="satellite_outer_panel")
    ET.indent(root, space=" ")
    output = ET.tostring(root, encoding="utf-8", xml_declaration=True) + b"\n"
    manifest = {
        "schema": "sarm-ground-capture/1", "template_id": selected_template,
        "sources": {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_text(encoding="utf-8").encode("utf-8")).hexdigest() for p in (BASE, SOURCE)},
        "output": selected_output.relative_to(ROOT).as_posix(), "sha256": hashlib.sha256(output).hexdigest(),
        "source_visual_geoms": len(target_source.findall(".//geom")), "synthetic_physics": True,
        "target_mass_kg": sum(x[0] for x in PROXIES.values()),
        "proxy_method": "component AABBs; synthetic mass and uniform-box inertia; external contacts only",
        "proxies": {name: {"mass_kg": p[0], "center_m": p[1], "half_size_m": p[2]} for name, p in PROXIES.items()},
        "initial_position_m": SPEC.position_m, "initial_orientation_wxyz": SPEC.orientation_wxyz,
        "passive_joint": "outer_panel_hinge", "initial_joint_position_rad": 0.0,
        "real_joint_limits_known": False, "automated_grasp_validated": False,
    }
    if target_self_collision:
        manifest["proxy_method"] = "unchanged external AABBs; two pair-only panel AABBs with approximate hinge clearance; inertia unchanged"
        manifest["target_internal_collision_enabled"] = True
        manifest["collision_model"] = capture_target(SELF_COLLISION_TEMPLATE).collision_model
        manifest["internal_contact"] = internal_metadata
        manifest["contact_filter"] = {"external_target_contype": 2, "external_target_conaffinity": 1,
                                      "internal_proxy_contype": 0, "internal_proxy_conaffinity": 0,
                                      "target_body_exclusions": [], "global_parent_filter_changed": False}
    return output, (json.dumps(manifest, indent=2, ensure_ascii=False) + "\n").encode("utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    # Maintain both coarse variants without rewriting identical old artifacts.
    for internal in (False, True):
        xml, manifest = composed_bytes(target_self_collision=internal)
        output_path = SELF_OUTPUT if internal else OUTPUT
        for path, content in ((output_path, xml), (output_path.with_suffix(".manifest.json"), manifest)):
            current = path.read_bytes().replace(b"\r\n", b"\n") if path.is_file() else None
            if args.check:
                if current != content:
                    raise SystemExit(f"Stale combined scene: {path}. Run tools/build_sarm_ground_target.py.")
            elif current != content:
                path.write_bytes(content)
            print(path)


if __name__ == "__main__":
    main()
