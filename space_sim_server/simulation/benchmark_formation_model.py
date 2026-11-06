"""Individually actuated copies of the original stowed SARM, sharing mesh assets."""
from copy import deepcopy
import hashlib
import json
import xml.etree.ElementTree as ET

from simulation.benchmark_orbit_model import prepare_orbit_model


def prepare_formation_model(repository, destination, *, spacecraft_count, force_limit_n):
    if type(spacecraft_count) is not int or spacecraft_count not in (2, 3):
        raise ValueError('this formation model supports two or three original SARM instances')
    model, files, manifest = prepare_orbit_model(repository, destination, force_limit_n=force_limit_n)
    root = ET.parse(model).getroot()
    world = root.find('worldbody')
    source_body = world.find('./body[@name="cubesat_bus"]')
    if source_body is None or len(world.findall('body')) != 1:
        raise ValueError('formation derivation requires the verified single SARM model')
    sections = {tag: deepcopy(root.find(tag)) for tag in ('actuator', 'sensor', 'equality', 'contact') if root.find(tag) is not None}
    named = {element.get('name') for element in source_body.iter() if element.get('name')}
    named.update(element.get('name') for section in sections.values() for element in section.iter() if element.get('name'))
    world.remove(source_body)
    for tag in sections:
        parent = root.find(tag)
        for child in list(parent):
            parent.remove(child)
    crafts = []
    for index in range(spacecraft_count):
        prefix = f'craft{index}_'
        names = {name: prefix+name for name in named}

        def renamed(element):
            value = deepcopy(element)
            for item in value.iter():
                for key, attribute in list(item.attrib.items()):
                    if key not in {'mesh', 'material', 'class', 'childclass', 'texture', 'file'} and attribute in names:
                        item.set(key, names[attribute])
            return value

        body = renamed(source_body)
        body.set('pos', f'{index*1000} 0 0')
        world.append(body)
        for tag, section in sections.items():
            for item in section:
                root.find(tag).append(renamed(item))
        crafts.append(dict(index=index, body=names['cubesat_bus'], center_site=names['orbital_thrust_center'],
                           force_actuators=[names[f'orbital_force_{axis}'] for axis in range(3)],
                           wheel_actuators=[names[f'rw_{axis}_motor'] for axis in 'xyz']))
    root.set('model', f'SpaceSim_{spacecraft_count}_original_SARM_formation')
    ET.indent(root)
    ET.ElementTree(root).write(model, encoding='utf-8', xml_declaration=True)
    source_properties = deepcopy(manifest['bodies']['cubesat_bus'])
    source_stowed = deepcopy(manifest['stowed_joints'])
    manifest.update(generated_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
                    bodies={craft['body']: deepcopy(source_properties) for craft in crafts},
                    spacecraft_count=spacecraft_count, controlled_spacecraft=spacecraft_count, crafts=crafts,
                    geometry_count=len(root.findall('.//geom')), body_count=len(root.findall('.//body')),
                    stowed_joints=[dict(item, body=f'craft{index}_'+item['body'],
                                        joint=dict(item['joint'], name=f'craft{index}_'+item['joint']['name']))
                                   for index in range(spacecraft_count) for item in source_stowed],
                    derivation='duplicate the original single stowed SARM body and its internal references; preserve geometry, mass, inertias and wheel hardware; share original mesh assets',
                    scope='two or three separately free-flying original SARM spacecraft with individual native COM motors; no contact task')
    (destination/'model_manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    return model, files, manifest
