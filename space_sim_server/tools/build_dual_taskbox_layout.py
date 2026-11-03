"""Mount generated 13-unit layouts into an existing closed-hull dual scene.

This creates an initial-geometry candidate, never a task-accepted layout.
The spacecraft, interfaces, camera mounts and operation-arm initial pose stay
unchanged. Free parts remain in their target-star pickup slots.
"""
from copy import deepcopy
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from simulation.scene_library.assembly import _absolutize
from simulation.scene_library.manifest import digest, write_json
from simulation.scene_library.taskbox_mount import (
    ROOT_BODIES, LOOSE_BODIES, SECTIONS, _names, _references, _pose,
    extract_taskbox, mount_taskbox,
)
from simulation.independent_task_box_layout import validate_layout_record


def remove_box(root, prefix, loose):
    nodes = [root.find(f'.//body[@name="{prefix+n}"]')
             for n in ROOT_BODIES + (LOOSE_BODIES if loose else ())]
    nodes = [n for n in nodes if n is not None]
    names = _names(nodes)
    parents = {child: parent for parent in root.iter() for child in parent}
    for node in nodes:
        parents[node].remove(node)
    for name in SECTIONS:
        section = root.find(name)
        if section is not None:
            for item in list(section):
                if _references(item) & names:
                    section.remove(item)


def unique_asset_filenames(root, output):
    """Basilisk's MuJoCo VFS keys files by basename, unlike from_xml_path."""
    groups = {}
    for node in root.findall('./asset/*'):
        if node.get('file'):
            path = Path(node.get('file')).resolve()
            groups.setdefault(path.name.casefold(), {}).setdefault(path, []).append(node)
    aliases = []
    for paths in groups.values():
        if len(paths) < 2:
            continue
        directory = output/'vfs-assets'
        directory.mkdir(exist_ok=True)
        for source, nodes in paths.items():
            name = hashlib.sha256(str(source).encode()).hexdigest()[:16]+'_'+source.name
            target = directory/name
            shutil.copyfile(source, target)
            checksum = digest(source)
            if digest(target) != checksum:
                raise ValueError('Asset copy changed contents')
            for node in nodes:
                node.set('file', str(target.resolve()))
            aliases.append(dict(source=str(source), path=str(target.resolve()), sha256=checksum))
    return aliases


def cross_box_contacts(root):
    """Keep each plug compatible with the other box's precision recipients.

    Populating the service box must not remove the target plug's existing
    service-side bore pairs/convex exclusions. Only free-part versus fixed-box
    rules are cloned; owner exclusions and finger contacts are not broadened.
    """
    section = root.find('contact')
    if section is None:
        return
    for prefix, other in (('', 'service_'), ('service_', '')):
        loose = _names([root.find(f'.//body[@name="{prefix+n}"]') for n in LOOSE_BODIES])
        fixed = _names([root.find(f'.//body[@name="{prefix+n}"]') for n in ROOT_BODIES])
        for node in list(section):
            if node.get('name', '').startswith('cross_box_'):
                continue
            refs = _references(node)
            if not refs & loose or not refs & fixed or refs - loose - fixed:
                continue
            copy = deepcopy(node)
            copy.set('name', 'cross_box_'+node.get('name'))
            for key, value in list(copy.attrib.items()):
                if value in loose and key != 'name':
                    copy.set(key, other+value.removeprefix(prefix))
            section.append(copy)


def compose(base, target_model, service_model, output):
    output.mkdir(parents=True, exist_ok=False)
    model = base/'preheld-dual.xml'
    root = ET.parse(model).getroot()
    _absolutize(root, model)
    original = deepcopy(root)
    records = {}
    for role, donor, owner, prefix, loose in (
            ('target', target_model, 'capture_target', '', True),
            ('service', service_model, 'cubesat_bus', 'service_', True)):
        record = json.loads(donor.with_suffix('.layout.json').read_text(encoding='utf-8'))
        if digest(donor) != record['model_sha256']:
            raise ValueError('Donor model changed')
        validate_layout_record(record, allow_native_candidate=True)
        source = ET.parse(donor).getroot()
        _absolutize(source, donor)
        # Namespace donor meshes: the two boxes may have different deck holes.
        template = extract_taskbox(source)
        used = {g.get('mesh') for part in template.roots+template.loose for g in part.iter('geom') if g.get('mesh')}
        rename = {name: 'random_'+role+'_'+name for name in used}
        for name in used:
            node = deepcopy(source.find(f'./asset/mesh[@name="{name}"]'))
            node.set('name', rename[name])
            root.find('asset').append(node)
        for part in template.roots+template.loose:
            for node in part.iter('geom'):
                if node.get('mesh') in rename:
                    node.set('mesh', rename[node.get('mesh')])
        destination = root.find(f'.//body[@name="{prefix}task_box_base"]')
        dp, dr = _pose(destination)
        sp, sr = _pose(template.roots[0])
        rotation = dr @ sr.T
        position = dp - rotation @ sp
        remove_box(root, prefix, loose)
        mount_taskbox(root, template, owner, position=position, rotation=rotation,
                      prefix=prefix, move_loose_parts=loose)
        installed = root.find(f'.//body[@name="{prefix}task_box_base"]')
        np.testing.assert_allclose(_pose(installed)[0], dp, atol=1e-12)
        np.testing.assert_allclose(_pose(installed)[1], dr, atol=1e-12)
        records[role] = dict(layout_seed=record['seed'], model_sha256=record['model_sha256'],
                             manifest_sha256=digest(donor.with_suffix('.layout.json')),
                             offsets_body_xy_m=record['offsets_body_xy_m'],
                             installation_yaw_rad=record.get('installation_yaw_rad', {}))
    cross_box_contacts(root)
    # Remove only task bodies from copies, then compare the entire remainder.
    remainder = deepcopy(root)
    for tree in (original, remainder):
        remove_box(tree, '', True)
        remove_box(tree, 'service_', True)
    def structure(node):
        return node.tag, node.attrib, [structure(c) for c in node]
    if structure(original.find('worldbody')) != structure(remainder.find('worldbody')):
        raise ValueError('Non-task spacecraft geometry changed')
    used = {n.get('mesh') for n in root.iter('geom')}
    for node in list(root.findall('./asset/mesh')):
        if node.get('name') not in used:
            root.find('asset').remove(node)
    aliases = unique_asset_filenames(root, output)
    ET.indent(root)
    final = output/'preheld-dual.xml'
    ET.ElementTree(root).write(final, encoding='utf-8', xml_declaration=True)
    for name in ('initial-scene.json', 'attitude_control.json', 'hull-closure.json'):
        shutil.copy2(base/name, output/name)
    report = dict(schema='dual-taskbox-layout/1', model_sha256=digest(final),
                  base_model_sha256=digest(model), target=records['target'], service=records['service'],
                  mounting_poses_preserved=True, spacecraft_and_cameras_preserved=True,
                  initial_states_randomized=False, full_task_accepted=False,
                  complete_parts_on_both_spacecraft=True,
                  vfs_asset_aliases=aliases,
                  acceptance_scope='pending_initial_geometry')
    write_json(output/'dual-layout.json', report)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('base', 'target', 'service', 'output'):
        parser.add_argument('--'+key, type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(compose(args.base, args.target, args.service, args.output)))
