"""Attach body-local display geometry without touching the physics scene."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from simulation.serial_chain_kinematics import matrix_to_quaternion_wxyz, quaternion_wxyz_to_matrix


def attachment_transform(spec, model_path=None):
    """Resolve an explicitly fixed body-local or opt-in mesh-anchored attachment.

    fixed_body_geometry preserves the original pixel/plate records verbatim,
    independent of any hull scaling or pivot. Optional mesh anchoring is a
    separate contract; neither mode changes physics or camera definitions.
    """
    anchor = spec.get('mjcf_anchor')
    mode = spec.get('transform_mode')
    if mode not in (None, 'fixed_body_geometry', 'mjcf_anchor'):
        raise ValueError('Unknown visual attachment transform mode')
    if mode == 'fixed_body_geometry':
        if anchor is not None:
            raise ValueError('fixed_body_geometry forbids hull scale or pose anchoring')
        return np.zeros(3), np.eye(3), 1.
    if mode == 'mjcf_anchor' and anchor is None:
        raise ValueError('MJCF anchor mode requires its calibration')
    if anchor is None:
        return np.zeros(3), np.eye(3), 1.
    if model_path is None:
        raise ValueError('MJCF-anchored visual attachment requires the runtime model')
    model_path = Path(model_path).resolve()
    root = ET.parse(model_path).getroot()
    body_name = spec['parent_id'].rsplit('/', 1)[-1]
    matches = root.findall(f'.//body[@name="{body_name}"]/geom[@name="{anchor["geom"]}"]')
    if len(matches) != 1:
        raise ValueError('Visual anchor must be a unique direct geometry of its parent body')
    geom = matches[0]
    mesh = root.find(f'./asset/mesh[@name="{geom.get("mesh")}"]')
    if mesh is None or not mesh.get('file') or mesh.get('refpos') or mesh.get('refquat'):
        raise ValueError('Visual anchor requires a file mesh without reference transforms')
    if any(key in geom.attrib for key in ('euler', 'axisangle', 'xyaxes', 'zaxis', 'fromto')):
        raise ValueError('Visual anchor requires an explicit quaternion pose')
    compiler = root.find('compiler')
    meshdir = compiler.get('meshdir', '') if compiler is not None else ''
    mesh_path = (model_path.parent / meshdir / mesh.get('file')).resolve()
    if hashlib.sha256(mesh_path.read_bytes()).hexdigest() != anchor['mesh_sha256']:
        raise ValueError('Visual anchor mesh differs from the calibrated protected face')
    scale = np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
    reference_scale = np.asarray(anchor['reference_mesh_scale'], dtype=float)
    p = np.fromstring(geom.get('pos', '0 0 0'), sep=' ')
    q = np.fromstring(geom.get('quat', '1 0 0 0'), sep=' ')
    p0 = np.asarray(anchor['reference_position_body_m'], dtype=float)
    q0 = np.asarray(anchor['reference_orientation_wxyz'], dtype=float)
    vectors = [(scale, (3,)), (reference_scale, (3,)), (p, (3,)), (q, (4,)), (p0, (3,)), (q0, (4,))]
    if (any(v.shape != shape or not np.isfinite(v).all() for v, shape in vectors)
            or np.any(scale <= 0) or np.any(reference_scale <= 0)
            or np.linalg.norm(q) < 1e-12 or np.linalg.norm(q0) < 1e-12):
        raise ValueError('Invalid visual anchor calibration')
    ratio = scale/reference_scale
    if not np.allclose(ratio, ratio[0], atol=1e-10, rtol=1e-10):
        raise ValueError('Mesh-anchored visual geometry requires uniform scaling')
    r = quaternion_wxyz_to_matrix(q/np.linalg.norm(q)) @ quaternion_wxyz_to_matrix(q0/np.linalg.norm(q0)).T
    factor = float(ratio[0])
    return p-factor*(r@p0), r, factor


def attach_visual_geometries(objects, attachments, root: Path, *, model_path=None, contracts=None) -> int:
    from bsk_render_adapter import GeometryVisual

    root = Path(root).resolve()
    bodies = {body.object_id: body for body in objects}
    ids = {g.geometry_id for body in bodies.values() for g in body.geometries}
    pending = []
    resolved = []
    for spec in attachments:
        parent = bodies.get(spec['parent_id'])
        if parent is None:
            raise ValueError(f"visual attachment parent is not registered: {spec['parent_id']}")
        path = (root / spec['geometry_file']).resolve()
        if not path.is_relative_to(root):
            raise ValueError('visual attachment file must be inside the server directory')
        source = path.read_bytes()
        if spec.get('sha256') and hashlib.sha256(source).hexdigest() != spec['sha256']:
            raise ValueError(f'visual attachment hash mismatch: {path}')
        offset, rotation, factor = attachment_transform(spec, model_path)
        for row in json.loads(source):
            if spec.get('mjcf_anchor'):
                row['position_body_m'] = (offset+factor*(rotation@np.asarray(row.get('position_body_m', [0., 0., 0.])))).tolist()
                row['orientation_body_from_geometry_wxyz'] = matrix_to_quaternion_wxyz(
                    rotation@quaternion_wxyz_to_matrix(np.asarray(row.get('orientation_body_from_geometry_wxyz', [1., 0., 0., 0.])))).tolist()
                row['dimensions_m'] = (factor*np.asarray(row['dimensions_m'])).tolist()
                row['scale'] = (factor*np.asarray(row.get('scale', [1., 1., 1.]))).tolist()
            geometry = GeometryVisual(**row)
            if geometry.render_role != 'visual':
                raise ValueError('attachments must contain visual geometry only')
            if geometry.geometry_id in ids:
                raise ValueError(f'duplicate visual attachment geometry: {geometry.geometry_id}')
            ids.add(geometry.geometry_id)
            pending.append((parent, geometry))
        resolved.append(dict(parent_id=spec['parent_id'], geometry_file=spec['geometry_file'],
                             geometry_sha256=hashlib.sha256(source).hexdigest(),
                             transform_mode=spec.get('transform_mode', 'mjcf_anchor' if spec.get('mjcf_anchor') else 'body_local'),
                             anchor_geom=(spec.get('mjcf_anchor') or {}).get('geom'),
                             uniform_scale=factor, position_offset_body_m=offset.tolist(),
                             rotation_body_from_reference=rotation.tolist(),
                             model_sha256=(hashlib.sha256(Path(model_path).read_bytes()).hexdigest()
                                           if spec.get('mjcf_anchor') else None)))
    for parent, geometry in pending:
        parent.geometries.append(geometry)
    if contracts is not None:
        contracts.extend(resolved)
    return len(pending)
