"""Derive a stowed flight configuration from the team's authoritative MJCF.

Source meshes, body transforms and inertias are preserved. Arm/finger/panel
coordinates are held at their authored zero pose for distant flight; their
original definitions are retained in the provenance manifest for handoff.
"""

from __future__ import annotations

import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

DEFAULT_MODEL = Path("model/SARM/platform/sarm_ground_target_self_collision.xml")


def rotation(element):
    if any(key in element.attrib for key in ("euler", "axisangle", "xyaxes", "zaxis")):
        raise ValueError("Flight model requires explicit quaternion body transforms")
    q = np.fromstring(element.get("quat", "1 0 0 0"), sep=" ")
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) == 0:
        raise ValueError("Invalid model quaternion")
    w, x, y, z = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def assembly_properties(root):
    parts = []

    def walk(body, position, orientation, top=False):
        if not top:
            position = position + orientation @ np.fromstring(
                body.get("pos", "0 0 0"), sep=" "
            )
            orientation = orientation @ rotation(body)
        inertial = body.find("inertial")
        if inertial is None:
            raise ValueError(f"Explicit inertia required for {body.get('name')}")
        mass = float(inertial.get("mass"))
        center = position + orientation @ np.fromstring(
            inertial.get("pos", "0 0 0"), sep=" "
        )
        if "fullinertia" in inertial.attrib:
            xx, yy, zz, xy, xz, yz = np.fromstring(inertial.get("fullinertia"), sep=" ")
            tensor = np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
        else:
            ri = rotation(inertial)
            tensor = (
                ri @ np.diag(np.fromstring(inertial.get("diaginertia"), sep=" ")) @ ri.T
            )
        if (
            mass <= 0
            or not np.isfinite(tensor).all()
            or np.min(np.linalg.eigvalsh(tensor)) <= 0
        ):
            raise ValueError("Invalid authored mass/inertia")
        parts.append((mass, center, orientation @ tensor @ orientation.T))
        for child in body.findall("body"):
            walk(child, position, orientation)

    walk(root, np.zeros(3), np.eye(3), True)
    mass = sum(item[0] for item in parts)
    center = sum(m * p for m, p, _ in parts) / mass
    tensor = sum(
        i
        + m
        * (
            np.dot(p - center, p - center) * np.eye(3)
            - np.outer(p - center, p - center)
        )
        for m, p, i in parts
    )
    return {
        "mass_kg": mass,
        "center_body_m": center.tolist(),
        "inertia_com_body_kg_m2": tensor.tolist(),
    }


def prepare_model(repository: Path, destination: Path, *, force_limit_n=25.0):
    source = repository / DEFAULT_MODEL
    root = ET.parse(source).getroot()
    compiler = root.find("compiler")
    meshdir = source.parent / compiler.get("meshdir", "")
    files = []
    mesh_hashes = {}
    for mesh in root.findall("./asset/mesh[@file]"):
        path = (meshdir / mesh.get("file")).resolve()
        content = path.read_bytes()
        if content.startswith(b"version https://git-lfs.github.com/spec/v1"):
            raise ValueError(f"Missing LFS mesh contents: {path}")
        mesh.set("file", path.as_posix())
        files.append(str(path))
        mesh_hashes[path.relative_to(repository).as_posix()] = hashlib.sha256(
            content
        ).hexdigest()
    compiler.set("meshdir", ".")
    compiler.set("fusestatic", "false")
    option = root.find("option")
    option.set("gravity", "0 0 0")
    root.set("model", "SpaceSim_team_100m_handoff")
    properties = {
        body.get("name"): assembly_properties(body)
        for body in root.findall("./worldbody/body")
    }
    if set(properties) != {"cubesat_bus", "capture_target"}:
        raise ValueError("Expected the default SARM and ground-target two-body asset")
    stowed = []
    for body in root.findall(".//body"):
        for joint in list(body.findall("joint")):
            if joint.get("type") == "free" or joint.get("name", "").startswith("rw_"):
                continue
            if float(joint.get("ref", "0")) != 0:
                raise ValueError(
                    "Nonzero authored joint reference requires a pose conversion"
                )
            stowed.append(
                {"body": body.get("name"), "joint": dict(joint.attrib), "position": 0.0}
            )
            body.remove(joint)
    fixed_names = {item["joint"]["name"] for item in stowed}
    # Source keyframes encode the articulated qpos layout. They cannot be
    # loaded into a stowed scene with fewer coordinates.
    keyframes = root.find("keyframe")
    if keyframes is not None:
        root.remove(keyframes)
    for section in ("actuator", "sensor", "equality"):
        parent = root.find(section)
        if parent is not None:
            for item in list(parent):
                if any(
                    item.get(key) in fixed_names
                    for key in ("joint", "joint1", "joint2")
                ):
                    parent.remove(item)
    # Eliminate source-local initial separation; physical initial states are
    # assigned once by the orbital scenario after native initialization.
    for body in root.findall("./worldbody/body"):
        body.set("pos", "0 0 0" if body.get("name") == "cubesat_bus" else "1000 0 0")
        body.set("quat", "1 0 0 0")
    bus = root.find('./worldbody/body[@name="cubesat_bus"]')
    ET.SubElement(
        bus,
        "site",
        name="orbital_thrust_center",
        pos=" ".join(map(str, properties["cubesat_bus"]["center_body_m"])),
    )
    target = root.find('./worldbody/body[@name="capture_target"]')
    ET.SubElement(
        target,
        "site",
        name="target_flight_center",
        pos=" ".join(map(str, properties["capture_target"]["center_body_m"])),
    )
    actuators = root.find("actuator")
    for axis in range(3):
        gear = [0.0] * 6
        gear[axis] = 1.0
        ET.SubElement(
            actuators,
            "motor",
            name=f"orbital_force_{axis}",
            site="orbital_thrust_center",
            gear=" ".join(map(str, gear)),
            ctrllimited="true",
            forcelimited="true",
            ctrlrange=f"{-force_limit_n} {force_limit_n}",
            forcerange=f"{-force_limit_n} {force_limit_n}",
        )
    destination.mkdir(parents=True, exist_ok=True)
    output = destination / "team_flight.xml"
    ET.indent(root)
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)
    settings = json.loads(source.with_name("attitude_control.json").read_text())
    # Flight controller and wheel protection still act on native rotors.
    settings["control_rate_hz"] = 20.0
    (destination / "attitude_control.json").write_text(json.dumps(settings, indent=2))
    manifest = {
        "source_model": DEFAULT_MODEL.as_posix(),
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "generated_sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
        "meshes": mesh_hashes,
        "bodies": properties,
        "stowed_joints": stowed,
        "geometry_count": len(root.findall(".//geom")),
        "mesh_count": len(mesh_hashes),
        "body_count": len(root.findall(".//body")),
        "propulsion": {
            "model": "bounded three-axis force at assembled center of mass",
            "limit_n": force_limit_n,
            "hardware_status": "provisional simulation actuator, not flight hardware",
            "mass_model": "constant mass; propellant consumption not modeled",
        },
        "scope": "distant flight with joints stowed at authored zero; stop at 100 m; no docking claim",
    }
    (destination / "model_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return output, files, manifest
