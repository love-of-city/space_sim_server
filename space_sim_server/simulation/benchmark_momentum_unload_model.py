"""Original stowed SARM plus three declared pure body torque site motors."""
from __future__ import annotations

import hashlib
import json
import xml.etree.ElementTree as ET

from simulation.benchmark_attitude_model import prepare_attitude_model


def prepare_momentum_unload_model(repository, destination):
    model, meshes, manifest = prepare_attitude_model(repository, destination)
    root = ET.parse(model).getroot()
    actuators = root.find('actuator')
    for axis in range(3):
        gear = [0]*6
        gear[3+axis] = 1
        ET.SubElement(actuators, 'motor', name=f'momentum_dump_{axis}', site='orbital_thrust_center',
            gear=' '.join(map(str, gear)), ctrllimited='true', forcelimited='true',
            ctrlrange='-0.02 0.02', forcerange='-0.02 0.02')
    root.set('model', 'SpaceSim_single_SARM_momentum_unload')
    ET.indent(root)
    ET.ElementTree(root).write(model, encoding='utf-8', xml_declaration=True)
    manifest.update(generated_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
        propulsion={'model': 'three ideal pure body torque site motors; vector norm limited to 0.02 N m', 'mass_model': 'constant mass'},
        derivation=manifest['derivation']+'; add three pure rotational site motors, preserving all body and rotor inertias',
        scope='single-spacecraft wheel momentum unloading; ideal external torque model, no translation command or thruster geometry')
    (destination/'model_manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    return model, meshes, manifest
