"""Keep both team spacecraft, remove translation actuators, preserve native wheels."""
import hashlib
import json
import xml.etree.ElementTree as ET

from simulation.orbital_rendezvous.model import prepare_model


def prepare_target_pointing_model(repository, destination):
    model, meshes, manifest = prepare_model(repository, destination)
    root = ET.parse(model).getroot()
    actuators = root.find('actuator')
    for motor in list(actuators):
        if motor.get('name','').startswith('orbital_force_'):
            actuators.remove(motor)
    if {motor.get('name') for motor in actuators} != {f'rw_{axis}_motor' for axis in 'xyz'}:
        raise ValueError('target-pointing model requires only original chaser reaction-wheel motors')
    if set(manifest['bodies']) != {'cubesat_bus','capture_target'}:
        raise ValueError('two distinct native spacecraft are required')
    root.set('model','SpaceSim_two_spacecraft_wheel_pointing')
    ET.indent(root)
    ET.ElementTree(root).write(model,encoding='utf-8',xml_declaration=True)
    manifest.update(generated_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),spacecraft_count=2,
        controlled_spacecraft=1,propulsion={'model':'none; original chaser wheel motors only','mass_model':'constant mass'},
        target_control='passive free flight; no target actuator exists',
        derivation='preserve both team models and native wheel hardware; remove ideal translation actuators',
        scope='cooperative satellite COM pointing; no translation control, optical terminal, image or contact claim')
    (destination/'model_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    return model,meshes,manifest
