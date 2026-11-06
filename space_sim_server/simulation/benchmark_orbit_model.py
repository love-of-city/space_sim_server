"""Original stowed SARM with one spacecraft and bounded COM translation motors."""
import hashlib
import json
import xml.etree.ElementTree as ET

from simulation.benchmark_attitude_model import prepare_attitude_model


def prepare_orbit_model(repository, destination, *, force_limit_n):
    model, files, manifest = prepare_attitude_model(repository, destination)
    root = ET.parse(model).getroot()
    actuators = root.find("actuator")
    for axis in range(3):
        gear = [0.] * 6
        gear[axis] = 1.
        ET.SubElement(actuators, "motor", name=f"orbital_force_{axis}", site="orbital_thrust_center",
                      gear=" ".join(map(str, gear)), ctrllimited="true", forcelimited="true",
                      ctrlrange=f"{-force_limit_n} {force_limit_n}", forcerange=f"{-force_limit_n} {force_limit_n}")
    root.set("model", "SpaceSim_single_SARM_orbit")
    ET.indent(root)
    ET.ElementTree(root).write(model, encoding="utf-8", xml_declaration=True)
    manifest.update(generated_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
        propulsion={"model": "bounded three-axis force at assembled center of mass", "limit_n": force_limit_n,
                    "hardware_status": "provisional simulation actuator, not flight hardware",
                    "mass_model": "constant mass; propellant consumption not modeled"},
        derivation="single stowed SARM, retaining original geometry, inertias and passive reaction wheels; add bounded COM translation motors",
        scope="single-spacecraft orbital maneuver; no independent target or contact task")
    (destination / "model_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return model, files, manifest
