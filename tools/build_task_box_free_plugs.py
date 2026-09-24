"""Derive an unlatched two-free-body scene from the validated fixed module.

Offline geometry only. Preserves CAD surfaces and upstream robot controls.
Density is an explicit provisional assumption, NOT calibrated CAD mass.
"""
import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

from build_task_box_module import digest, fmt, save_json
from select_sarm_scene import verified_source_tree

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'model/SARM/platform/sarm_task_box_module.xml'
OUTPUT = SOURCE.with_name('sarm_task_box_plugs.xml')
FOLDER = ROOT / 'model/task_box/free_plugs'


def build():
    sources = verified_source_tree(ROOT, SOURCE)
    config_path = FOLDER / 'configuration.json'
    config = json.loads(config_path.read_text(encoding='utf-8'))
    sources[config_path.relative_to(ROOT).as_posix()] = digest(config_path)
    tree = ET.parse(SOURCE)
    root = tree.getroot()
    root.set('model', 'SARM_local_task_box_free_plugs')
    world = root.find('worldbody')
    bus = world.find('body[@name="cubesat_bus"]')
    q = np.fromstring(bus.get('quat', '1 0 0 0'), sep=' ')
    rotation = Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix()
    position = np.fromstring(bus.get('pos', '0 0 0'), sep=' ')
    # Fixed geometry accepts new plug bit 4 while retaining arm bit 1/target bit 2.
    for element in list(bus.findall('geom')) + list(bus.findall('flexcomp/contact')):
        if element.get('contype') == '0' and element.get('conaffinity') == '3':
            element.set('conaffinity', '7')
    metadata = []
    # A coarse inherited convex solid at the guide storage bore closes its void.
    # Recover only its original surface triangles, rather than ignoring contact.
    for hull in list(bus.findall('geom')):
        if not hull.get('name', '').startswith('task_box_interface_hull_'):
            continue
        coarse = trimesh.load_mesh(SOURCE.parent / root.find(f'./asset/mesh[@name="{hull.get("mesh")}"]').get('file'))
        regions = [np.array([[.25, .245, -.081], [.31, .305, .001]]),
                   np.array([[.50, .16, -.081], [.57, .31, .001]])]
        if not any(np.all(coarse.bounds[1] >= r[0]) and np.all(coarse.bounds[0] <= r[1]) for r in regions):
            continue
        # These small legacy socket-wall fragments sit inside the owned CAD
        # interface cells. The local task_box_001 surface is authoritative here;
        # retaining another (different tessellation/diameter) wall blocks the bore.
        socket_cell = regions[1]
        if np.all(coarse.bounds[0] >= socket_cell[0]) and np.all(coarse.bounds[1] <= socket_cell[1]):
            bus.remove(hull)
            continue
        original = trimesh.load_mesh(ROOT / 'model/task_box/module/upstream_base_without_task_box.obj')
        bounds = coarse.bounds
        for axis in range(3):
            for side in (0, 1):
                normal = np.zeros(3)
                normal[axis] = 1 if side == 0 else -1
                origin = np.zeros(3)
                origin[axis] = bounds[side, axis]
                original = trimesh.intersections.slice_mesh_plane(original, normal, origin, cap=False)
        original.vertices = original.vertices.astype(np.float32).astype(float)
        original.merge_vertices(digits_vertex=7)
        original.update_faces(original.nondegenerate_faces(height=1e-7))
        original.remove_unreferenced_vertices()
        surface_name = hull.get('name') + '_surface'
        path = FOLDER / (surface_name + '.obj')
        original.export(path)
        sources[path.relative_to(ROOT).as_posix()] = digest(path)
        bus.remove(hull)
        surface = ET.SubElement(bus, 'flexcomp', name=surface_name, type='mesh',
            file='../../task_box/free_plugs/' + path.name, dim='2', rigid='true', radius='.00001', group='3', rgba='0 .7 .3 0')
        ET.SubElement(surface, 'contact', contype='0', conaffinity='7', selfcollide='none',
                      internal='false', condim='3', friction='.6 .005 .0001', solref='.01 1')
    for row in config['plugs']:
        number, name = row['id'], row['name']
        key = f'task_box_{number:03}'
        visual = bus.find(f'geom[@name="{key}_visual"]')
        surface = bus.find(f'flexcomp[@name="{key}_contact"]')
        mesh = trimesh.load_mesh(SOURCE.parent / surface.get('file'))
        assert mesh.is_watertight and mesh.volume > 0, name
        center = mesh.center_mass.copy()
        mesh.density = row['provisional_density_kg_m3']
        mass = float(mesh.mass)
        inertia = mesh.moment_inertia.copy()
        mesh.vertices -= center
        path = FOLDER / (name + '.obj')
        mesh.export(path)
        sources[path.relative_to(ROOT).as_posix()] = digest(path)
        relative = '../../task_box/free_plugs/' + path.name
        root.find(f'./asset/mesh[@name="{key}"]').set('file', relative)
        initial_center = center + np.array([0., 0., row['initial_lift_m']])
        body = ET.SubElement(world, 'body', name=name,
                             pos=fmt(position + rotation @ initial_center), quat=fmt(q))
        ET.SubElement(body, 'freejoint', name=name + '_free')
        ET.SubElement(body, 'inertial', pos='0 0 0', mass=fmt([mass]),
                      fullinertia=fmt([inertia[0, 0], inertia[1, 1], inertia[2, 2],
                                       inertia[0, 1], inertia[0, 2], inertia[1, 2]]))
        body.append(copy.deepcopy(visual))
        flex = copy.deepcopy(surface)
        flex.set('file', relative)
        contact = flex.find('contact')
        contact.set('contype', '4')
        contact.set('conaffinity', '7')
        contact.set('friction', fmt(row['friction']))
        body.append(flex)
        bus.remove(visual)
        bus.remove(surface)
        metadata.append(dict(name=name, bus_center_m=center.tolist(), initial_center_m=initial_center.tolist(), mass_kg=mass,
                             mass_is_provisional=True, original_bounds_m=(mesh.bounds + center).tolist()))
    # New world bodies are appended, so existing joint state indices stay stable.
    for keyframe in root.findall('./keyframe/key'):
        if keyframe.get('qpos') is not None:
            values = np.fromstring(keyframe.get('qpos'), sep=' ')
            # Upstream home omits the final passive panel hinge (default zero).
            if len(values) == 25:
                values = np.r_[values, 0.]
            if len(values) != 26:
                raise ValueError('Upstream joint layout changed; review keyframe expansion')
            bus_rotation = Rotation.from_quat(values[3:7][[1, 2, 3, 0]]).as_matrix()
            added = []
            for item in metadata:
                added.extend(values[:3] + bus_rotation @ item['initial_center_m'])
                added.extend(values[3:7])
            keyframe.set('qpos', fmt(np.r_[values, added]))
        if keyframe.get('qvel') is not None:
            raise ValueError('Explicitly map upstream keyframe velocities to free plugs')
    tree.write(OUTPUT, encoding='utf-8', xml_declaration=True)
    OUTPUT.write_text('\n'.join(s.rstrip() for s in OUTPUT.read_text(encoding='utf-8').splitlines()) + '\n', encoding='utf-8')
    save_json(OUTPUT.with_suffix('.manifest.json'), dict(schema='task-box-free-plugs/1',
        sha256=digest(OUTPUT), sources=sources, plugs=metadata,
        limitations=['Uncalibrated plug density; bus inertia retained from upstream, mass redistribution pending.',
                     'No latch or weld; motion depends on actual contact.',
                     'Full gripper insertion acceptance is reported separately.']))
    print(json.dumps(metadata, indent=2))


if __name__ == '__main__':
    build()
