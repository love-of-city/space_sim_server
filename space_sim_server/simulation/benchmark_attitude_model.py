"""Single SARM derivative of the team's stowed model; no surrogate geometry."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from simulation.orbital_rendezvous.model import prepare_model


def prepare_attitude_model(repository: Path, destination: Path):
    model, _, manifest = prepare_model(repository, destination)
    root = ET.parse(model).getroot()
    target = root.find('./worldbody/body[@name="capture_target"]')
    removed_names = {element.get("name") for element in target.iter() if element.get("name")}
    root.find("worldbody").remove(target)
    for section in ("contact", "sensor", "equality"):
        parent = root.find(section)
        if parent is not None:
            for item in list(parent):
                if any(value in removed_names for key, value in item.attrib.items() if key != "name"):
                    parent.remove(item)
    actuators = root.find("actuator")
    for motor in list(actuators):
        if motor.get("name", "").startswith("orbital_force_"):
            actuators.remove(motor)
    if {item.get("name") for item in actuators} != {f"rw_{axis}_motor" for axis in "xyz"}:
        raise ValueError("single attitude model must contain exactly three original wheel motors")
    meshes = root.find("asset")
    used = {geom.get("mesh") for geom in root.findall(".//geom[@mesh]")}
    files = []
    for mesh in list(meshes.findall("mesh")):
        if mesh.get("name") not in used:
            meshes.remove(mesh)
        else:
            files.append(mesh.get("file"))
    root.set("model", "SpaceSim_single_SARM_attitude")
    ET.indent(root)
    ET.ElementTree(root).write(model, encoding="utf-8", xml_declaration=True)
    selected = {Path(path).relative_to(repository).as_posix() for path in files}
    manifest.update({
        "generated_sha256": hashlib.sha256(model.read_bytes()).hexdigest(),
        "meshes": {name: digest for name, digest in manifest["meshes"].items() if name in selected},
        "bodies": {"cubesat_bus": manifest["bodies"]["cubesat_bus"]},
        "stowed_joints": [item for item in manifest["stowed_joints"] if item["body"] not in removed_names],
        "geometry_count": len(root.findall(".//geom")), "mesh_count": len(selected),
        "body_count": len(root.findall(".//body")), "spacecraft_count": 1,
        "propulsion": {"model": "none; original reaction-wheel joint motors only", "mass_model": "constant mass"},
        "derivation": "remove the independent target, its contact pairs, unused meshes and translation actuators; preserve SARM geometry, mass, stowed transforms and wheel hardware",
        "scope": "single-spacecraft stowed attitude maneuver; no target, rendezvous or contact task",
    })
    (destination / "model_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return model, files, manifest
