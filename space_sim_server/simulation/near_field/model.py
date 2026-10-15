"""Runtime instrumentation of the original articulated model, without asset edits."""
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from simulation.orbital_rendezvous.model import DEFAULT_MODEL, assembly_properties
from simulation.serial_chain_kinematics import SerialChainKinematics

JOINTS = tuple(f"joint{i}" for i in range(1, 7)) + ("joint_finger1", "joint_finger2")
ACTUATORS = tuple(f"sarm_{name}_motor" for name in JOINTS)
GRASP_TARGET = np.array([-0.3745, -0.07242, 0.04707870013])
NOMINAL_TCP = np.array([0.60, 0.10, 0.30])


def prepare(repository: Path, output: Path):
    repository = repository.resolve()
    source = repository / DEFAULT_MODEL
    root = ET.parse(source).getroot()
    before = {tag: [ET.tostring(x) for x in root.iter(tag)] for tag in ("joint", "inertial", "geom")}
    compiler = root.find("compiler")
    meshdir = source.parent / compiler.get("meshdir", "")
    mesh_files, hashes = [], {}
    for mesh in root.findall("./asset/mesh[@file]"):
        path = (meshdir / mesh.get("file")).resolve()
        content = path.read_bytes()
        if content.startswith(b"version https://git-lfs"):
            raise ValueError(f"LFS mesh missing: {path}")
        mesh.set("file", path.as_posix())
        mesh_files.append(str(path))
        hashes[path.relative_to(repository).as_posix()] = hashlib.sha256(content).hexdigest()
    compiler.set("meshdir", ".")
    compiler.set("fusestatic", "false")
    properties = {b.get("name"): assembly_properties(b) for b in root.findall("./worldbody/body")}
    bus = root.find('./worldbody/body[@name="cubesat_bus"]')
    target = root.find('./worldbody/body[@name="capture_target"]')
    for body, name in ((bus, "orbital_thrust_center"), (target, "target_flight_center")):
        ET.SubElement(body, "site", name=name, pos=" ".join(map(str, properties[body.get("name")]["center_body_m"])))
    for side, z in (("left", 0.34), ("right", 0.46)):
        # ONE rigid stereo pair above the bus envelope, looking along bus +X.
        # Existing overview/wrist cameras remain operator previews only.
        ET.SubElement(bus, "camera", name=f"near_stereo_{side}", pos=f"-0.10 -0.10 {z}",
                      quat="0.707106781187 0 -0.707106781187 0", fovy="38.06", resolution="1920 1200")
    for axis in range(3):
        gear = [0.] * 6
        gear[axis] = 1.
        ET.SubElement(root.find("actuator"), "motor", name=f"orbital_force_{axis}",
                      site="orbital_thrust_center", gear=" ".join(map(str, gear)),
                      ctrllimited="true", ctrlrange="-25 25", forcelimited="true", forcerange="-25 25")
    assert all(before[tag] == [ET.tostring(x) for x in root.iter(tag)] for tag in before)
    output.mkdir(parents=True, exist_ok=True)
    path = output / "near_field.xml"
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    settings = json.loads(source.with_name("attitude_control.json").read_text())
    settings["control_rate_hz"] = 20.0
    (output / "attitude_control.json").write_text(json.dumps(settings, indent=2))
    chain = SerialChainKinematics.from_mjcf(path, base_body="cubesat_bus", joint_names=JOINTS[:6], tool_site="sarm_ee")
    manifest = {"source_model": DEFAULT_MODEL.as_posix(), "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                "bodies": properties, "meshes": hashes, "joints_inertias_geometries_unchanged": True,
                "proxy": "original two-finger gripper; virtual contact window; logical latch, no weld or load transfer",
                "nominal_tcp_bus_m": NOMINAL_TCP.tolist(), "grasp_target_origin_m": GRASP_TARGET.tolist()}
    (output / "model_manifest.json").write_text(json.dumps(manifest, indent=2))
    return path, mesh_files, manifest, chain
