"""Derive the captive joint from delivered geometry, without editing the delivery.

The geometry defines travel. Friction and stop compliance are simulation tuning,
not measured material properties. All torques/reactions are solved by MuJoCo.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as E

import numpy as np

LEAD = 0.016
FRICTION_TORQUE = 0.03  # N m, symmetric dry friction
VISCOUS_DAMPING = 0.04  # N m s/rad


def quaternion_product(a, b):
    w, v = a[0], np.asarray(a[1:])
    x, u = b[0], np.asarray(b[1:])
    return np.r_[w*x - v@u, w*u + x*v + np.cross(v, u)]


def rotation(q):
    w, x, y, z = np.asarray(q)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def body_frames(root):
    frames = {}
    def visit(node, p, q):
        for body in node.findall('body'):
            pos = p + rotation(q) @ np.fromstring(body.get('pos', '0 0 0'), sep=' ')
            quat = quaternion_product(q, np.fromstring(body.get('quat', '1 0 0 0'), sep=' '))
            quat /= np.linalg.norm(quat)
            frames[body.get('name')] = (pos, quat)
            visit(body, pos, quat)
    visit(root.find('worldbody'), np.zeros(3), np.array([1., 0, 0, 0]))
    return frames


def mesh_vertices(root, body_name, output_dir):
    geom = root.find(f'.//body[@name="{body_name}"]/geom')
    mesh = root.find(f'./asset/mesh[@name="{geom.get("mesh")}"]')
    path = Path(output_dir) / mesh.get('file')
    dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
    with path.open('rb') as stream:
        stream.seek(80)
        count = int.from_bytes(stream.read(4), 'little')
    if path.stat().st_size != 84 + 50*count:
        raise ValueError(f'Expected binary STL: {path}')
    vertices = np.fromfile(path, dtype=dtype, offset=84)['vertices'].reshape(-1, 3).astype(float)
    vertices *= np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
    return (rotation(np.fromstring(geom.get('quat', '1 0 0 0'), sep=' ')) @ vertices.T).T + np.fromstring(geom.get('pos', '0 0 0'), sep=' ')


def apply_captive_fastener(root, output_dir):
    frames = body_frames(root)
    rod_p, rod_q = frames['locking_rod']
    rod_r = rotation(rod_q)
    vertices = {}
    for name in ('locking_rod', 'fastener', 'hook'):
        p, q = frames[name]
        world = (rotation(q) @ mesh_vertices(root, name, output_dir).T).T + p
        vertices[name] = (rod_r.T @ (world-rod_p).T).T
    rod = vertices['locking_rod']
    # This delivery has a 24-mm thread diameter and a 35-mm terminal cap.
    cap = rod[(rod[:, 1] > .12) & (np.hypot(rod[:, 0], rod[:, 2]) > .014)]
    if not len(cap):
        raise ValueError('No terminal retaining cap found in locking_rod geometry')
    nut = vertices['fastener']
    hook = vertices['hook']
    annulus = np.hypot(hook[:, 0], hook[:, 2])
    seat = hook[(annulus > .0105) & (annulus < .0225)]
    if not len(seat):
        raise ValueError('No hook seating envelope found')
    envelope_gap = float(nut[:, 1].min() - seat[:, 1].max())
    # The source rod SDF spans Y=.010..120 m. Stop only at the inner
    # thread runout; the hook's actual collision surface supplies seating.
    # Independent projected extrema do NOT locate mating contact on a helix.
    stroke = float(nut[:, 1].min() - .010)
    cap_gap = float(cap[:, 1].min() - nut[:, 1].max())
    if not .05 < stroke < .06 or not 0 < envelope_gap < .008 or not -1e-5 < cap_gap < 1e-5:
        raise ValueError(f'Unexpected captive geometry: stroke={stroke}, cap gap={cap_gap}')
    body = root.find('./worldbody/body[@name="fastener"]')
    rod_body = root.find('.//body[@name="locking_rod"]')
    pos, quat = frames['fastener']
    local_pos = rod_r.T @ (pos-rod_p)
    local_quat = quaternion_product(rod_q * np.array([1, -1, -1, -1]), quat)
    body.set('pos', ' '.join(format(v, '.17g') for v in local_pos))
    body.set('quat', ' '.join(format(v, '.17g') for v in local_quat))
    body.remove(body.find('freejoint'))
    # Axes are in fastener coordinates: +Z goes toward the hook, opposite rod +Y.
    # A finite slide supplies the outer captive and inner thread-runout stops.
    # Seating reaction goes through the retained hook/fastener contact geometry.
    body.insert(0, E.Element('joint', name='fastener_slide', type='slide', axis='0 0 1',
        limited='true', range=f'0 {stroke:.17g}', damping='0',
        solreflimit='0.01 1', solimplimit='0.9999 0.9999 0.001', margin='0'))
    body.insert(1, E.Element('joint', name='fastener_spin', type='hinge', axis='0 0 1',
        limited='false', damping=str(VISCOUS_DAMPING), frictionloss=str(FRICTION_TORQUE),
        solreffriction='0.01 1', solimpfriction='0.9999 0.9999 0.001'))
    E.SubElement(root.find('equality'), 'joint', name='fastener_helix',
        # Normalize the residual in radians. The inverse (metre residual)
        # becomes excessively compliant under torque because of mixed units.
        joint1='fastener_spin', joint2='fastener_slide',
        polycoef=f'0 {2*math.pi/LEAD:.17g} 0 0 0',
        solref='0.005 1', solimp='0.9999 0.9999 0.001')
    root.find('worldbody').remove(body)
    rod_body.append(body)
    contact = root.find('contact')
    for pair in list(contact):
        if pair.get('name') in ('physical_locking_thread', 'locking_rod_fastener_radial_guide'):
            contact.remove(pair)
    # Unused SDF assets stay registered for identical visual/mass provenance.
    # Both geoms already have contype=conaffinity=0; no search is performed.
    return {'model': 'captive', 'lead_m': LEAD, 'stroke_m': stroke,
            'max_turns_to_runout': stroke/LEAD, 'friction_torque_nm': FRICTION_TORQUE,
            'viscous_damping_nm_s': VISCOUS_DAMPING,
            'geometry': {'retainer_radius_m': float(np.hypot(cap[:, 0], cap[:, 2]).max()),
                         'retainer_start_y_m': float(cap[:, 1].min()),
                         'loose_end_y_m': float(nut[:, 1].max()),
                         'hook_envelope_y_m': float(seat[:, 1].max()),
                         'envelope_gap_m': envelope_gap,
                         'thread_start_y_m': .010},
            'calibration': 'runout from source SDF extent; seating by actual contact; friction and compliance uncalibrated'}
