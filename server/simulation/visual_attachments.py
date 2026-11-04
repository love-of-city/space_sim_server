"""Attach body-local display geometry without touching the physics scene."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path


def attach_visual_geometries(objects, attachments, root: Path) -> int:
    from bsk_render_adapter import GeometryVisual

    root = Path(root).resolve()
    bodies = {body.object_id: body for body in objects}
    ids = {g.geometry_id for body in bodies.values() for g in body.geometries}
    pending = []
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
        for row in json.loads(source):
            geometry = GeometryVisual(**row)
            if geometry.render_role != 'visual':
                raise ValueError('attachments must contain visual geometry only')
            if geometry.geometry_id in ids:
                raise ValueError(f'duplicate visual attachment geometry: {geometry.geometry_id}')
            ids.add(geometry.geometry_id)
            pending.append((parent, geometry))
    for parent, geometry in pending:
        parent.geometries.append(geometry)
    return len(pending)
