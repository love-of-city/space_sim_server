"""Copy complete passive task boxes and remap their spacecraft mount exclusions.

Mount transforms map the source spacecraft frame into the destination owner
frame: ``p_owner = position + rotation @ p_source``. Meshes/materials are shared;
bodies, joints, sites, constraints and plugin instances get independent names.
"""
from copy import deepcopy
from dataclasses import dataclass
import xml.etree.ElementTree as ET

import numpy as np

from simulation.serial_chain_kinematics import (
    matrix_to_quaternion_wxyz,
    quaternion_wxyz_to_matrix,
)

ROOT_BODIES = ('task_box_base', 'drawer_base')
LOOSE_BODIES = ('aviation_plug', 'guide_plug', 'cylindrical_plug', 'bolt')
REFERENCE_KEYS = frozenset(('body', 'body1', 'body2', 'joint', 'joint1', 'joint2',
    'geom', 'geom1', 'geom2', 'site', 'site1', 'site2', 'tendon', 'tendon1',
    'tendon2', 'objname', 'refname', 'instance'))
SECTIONS = ('equality', 'contact', 'tendon', 'actuator', 'sensor')


def _numbers(values):
    return ' '.join(format(float(v), '.16g') for v in values)


def _pose(node):
    if any(k in node.attrib for k in ('euler', 'axisangle', 'xyaxes', 'zaxis')):
        raise ValueError('Task mounting requires explicit quaternion transforms')
    p = np.fromstring(node.get('pos', '0 0 0'), sep=' ')
    q = np.fromstring(node.get('quat', '1 0 0 0'), sep=' ')
    if p.shape != (3,) or q.shape != (4,) or not np.isfinite(np.r_[p, q]).all() or np.linalg.norm(q) == 0:
        raise ValueError('Invalid authored body pose')
    return p, quaternion_wxyz_to_matrix(q / np.linalg.norm(q))


def _set_pose(node, p, r):
    node.set('pos', _numbers(p))
    node.set('quat', _numbers(matrix_to_quaternion_wxyz(r)))


def _world_pose(root, body):
    parents = {child: parent for parent in root.iter() for child in parent}
    p, r = np.zeros(3), np.eye(3)
    node = body
    while node.tag != 'worldbody':
        if node.tag != 'body':
            raise ValueError('Body is not in worldbody')
        np_, nr = _pose(node)
        p, r = np_ + nr @ p, nr @ r
        node = parents[node]
    return p, r


def _names(nodes):
    return {n.get('name') for root in nodes for n in root.iter() if n.get('name')}


def _references(node):
    return {v for n in node.iter() for k, v in n.attrib.items() if k in REFERENCE_KEYS}


@dataclass
class TaskBoxTemplate:
    roots: tuple
    loose: tuple
    sections: dict
    plugin_instances: dict
    source_owner: str


def extract_taskbox(source_root, source_owner='cubesat_bus'):
    """Snapshot source geometry/constraints before removing the old installation."""
    owner = source_root.find(f'.//body[@name="{source_owner}"]')
    if owner is None:
        raise ValueError(f'Missing source owner {source_owner}')
    op, rotation = _world_pose(source_root, owner)
    groups = []
    for names in (ROOT_BODIES, LOOSE_BODIES):
        group = []
        for name in names:
            original = source_root.find(f'.//body[@name="{name}"]')
            if original is None:
                raise ValueError(f'Missing task body {name}')
            p, r = _world_pose(source_root, original)
            node = deepcopy(original)
            _set_pose(node, rotation.T @ (p-op), rotation.T @ r)
            group.append(node)
        groups.append(tuple(group))
    instances = {}
    for plugin in source_root.findall('./extension/plugin'):
        for instance in plugin.findall('instance'):
            instances[instance.get('name')] = (dict(plugin.attrib), deepcopy(instance))
    return TaskBoxTemplate(*groups,
        {s: tuple(deepcopy(list(source_root.find(s)))) if source_root.find(s) is not None else () for s in SECTIONS},
        instances, source_owner)


def detach_taskbox(root, *, include_loose_parts=True):
    """Remove original task bodies and references touching them, keeping assets.

    Capture the template first. Mounting restores the captured hull exclusions
    against the destination owner rather than leaving references to the donor.
    """
    names = ROOT_BODIES + (LOOSE_BODIES if include_loose_parts else ())
    nodes = [root.find(f'.//body[@name="{n}"]') for n in names]
    nodes = [n for n in nodes if n is not None]
    removed = _names(nodes)
    parents = {child: parent for parent in root.iter() for child in parent}
    for node in nodes:
        parents[node].remove(node)
    for section_name in SECTIONS:
        section = root.find(section_name)
        if section is not None:
            for node in list(section):
                if _references(node) & removed:
                    section.remove(node)
    return sorted(removed)


def restore_owner_exclusions(root, template, owner_name, *, prefix=''):
    """Restore source rules when a builder reuses an already-mounted box."""
    owned = _names(template.roots + template.loose)
    bodies = {n.get('name') for n in root.iter('body')}
    if owner_name not in bodies:
        raise ValueError('Missing destination owner '+owner_name)
    section = root.find('contact')
    if section is None:
        section = ET.SubElement(root, 'contact')
    restored = []
    for node in template.sections['contact']:
        refs = _references(node)
        if node.tag != 'exclude' or template.source_owner not in refs or not refs & owned:
            continue
        copied = deepcopy(node)
        copied.set('name', prefix+node.get('name'))
        for key in ('body1', 'body2'):
            original = node.get(key)
            copied.set(key, owner_name if original == template.source_owner else prefix+original)
        if any(copied.get(k) not in bodies for k in ('body1', 'body2')):
            continue
        existing = section.find(f'exclude[@name="{copied.get("name")}"]')
        if existing is None:
            section.append(copied)
        elif existing.attrib != copied.attrib:
            raise ValueError('Conflicting owner exclusion '+copied.get('name'))
        restored.append(dict(copied.attrib))
    return restored


def mount_taskbox(root, template, owner_name, *, position, rotation,
                  prefix='', move_loose_parts=False):
    """Install one box; optionally place its free donor parts in worldbody.

    The destination must already contain the template's shared assets/defaults.
    ``move_loose_parts=False`` leaves the source pickup slots empty. Contact
    pairs between that box and the existing donor/fingers are preserved, but
    source spacecraft-body contact exclusions are mapped to the new owner.
    """
    p, r = np.asarray(position, dtype=float), np.asarray(rotation, dtype=float)
    if (p.shape != (3,) or r.shape != (3, 3) or not np.isfinite(p).all()
            or not np.isfinite(r).all() or not np.allclose(r.T @ r, np.eye(3), atol=1e-8)
            or not np.isclose(np.linalg.det(r), 1., atol=1e-8)):
        raise ValueError('Mount requires a finite rigid transform')
    owner = root.find(f'.//body[@name="{owner_name}"]')
    if owner is None:
        raise ValueError(f'Missing destination owner {owner_name}')
    parts = list(template.roots) + (list(template.loose) if move_loose_parts else [])
    owned = _names(parts)
    names = {name: prefix+name for name in owned}
    existing = {n.get('name') for n in root.findall('./worldbody//*') if n.get('name')}
    collision = existing & set(names.values())
    if collision:
        raise ValueError(f'Task names already installed: {sorted(collision)[:3]}')
    needed_instances = {n.get('instance') for part in parts for n in part.iter('plugin') if n.get('instance')}
    instance_names = {name: prefix+name for name in needed_instances}
    reference_names = names | instance_names

    def clone(node):
        result = deepcopy(node)
        for item in result.iter():
            if item.get('name'):
                item.set('name', names.get(item.get('name'), prefix+item.get('name')))
            for key, value in list(item.attrib.items()):
                if key in REFERENCE_KEYS and value in reference_names:
                    item.set(key, reference_names[value])
        return result

    # Check all references before mutating the destination.
    constraints = {s: [] for s in SECTIONS}
    available = existing | set(names.values())
    for section, nodes in template.sections.items():
        for node in nodes:
            refs = _references(node)
            if not refs & owned:
                continue
            # A copied passive mechanism must not acquire a constraint to an
            # unrelated source satellite or to a joint that was not copied.
            if section != 'contact' and refs - owned:
                raise ValueError(f'External reference in task {section}: {sorted(refs-owned)}')
            copied = clone(node)
            if section == 'contact' and node.tag == 'exclude':
                for key in ('body1', 'body2'):
                    if node.get(key) == template.source_owner:
                        copied.set(key, owner_name)
            missing = _references(copied) - available
            if missing:
                # Missing donor bodies are legitimate only when omitted from
                # the entire model. Do not leave dangling contacts behind.
                if section == 'contact':
                    continue
                raise ValueError(f'Missing task references: {sorted(missing)}')
            constraints[section].append(copied)

    extension = root.find('extension')
    for old in sorted(needed_instances):
        if old not in template.plugin_instances:
            raise ValueError(f'Missing plugin instance {old}')
        attrs, definition = template.plugin_instances[old]
        new = instance_names[old]
        current = root.find(f'./extension/plugin/instance[@name="{new}"]')
        definition = deepcopy(definition)
        definition.set('name', new)
        if current is not None:
            def structure(node):
                return node.tag, node.attrib, (node.text or '').strip(), [structure(c) for c in node]
            if structure(current) != structure(definition):
                raise ValueError(f'Conflicting plugin instance {new}')
            continue
        if extension is None:
            extension = ET.SubElement(root, 'extension')
        plugin = next((n for n in extension if n.attrib == attrs), None)
        if plugin is None:
            plugin = ET.SubElement(extension, 'plugin', attrs)
        plugin.append(definition)

    for part in template.roots:
        node = clone(part)
        old_p, old_r = _pose(part)
        _set_pose(node, p+r @ old_p, r @ old_r)
        owner.append(node)
    if move_loose_parts:
        op, owner_r = _world_pose(root, owner)
        world = root.find('worldbody')
        for part in template.loose:
            node = clone(part)
            old_p, old_r = _pose(part)
            _set_pose(node, op+owner_r @ (p+r @ old_p), owner_r @ r @ old_r)
            world.append(node)
    for section_name, nodes in constraints.items():
        if not nodes:
            continue
        section = root.find(section_name)
        if section is None:
            section = ET.SubElement(root, section_name)
        existing_names = {n.get('name') for n in section if n.get('name')}
        for node in nodes:
            if node.get('name') in existing_names:
                raise ValueError(f'Duplicate task constraint {node.get("name")}')
            section.append(node)
    return dict(owner=owner_name, names=names, plugin_instances=instance_names,
        roots=[prefix+n for n in ROOT_BODIES],
        loose_parts=[prefix+n for n in LOOSE_BODIES] if move_loose_parts else [],
        copied_constraints={s: [n.get('name') for n in nodes] for s, nodes in constraints.items()},
        transform_position=p.tolist(), transform_rotation=r.tolist())
