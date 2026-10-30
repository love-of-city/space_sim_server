"""Regenerate the locking sweep's precise base collision hulls from original CAD.

Run in the separate layout-geometry environment (trimesh/manifold3d installed).
The coarse source hull fills a CAD recess and falsely blocks the loose nut.
We intersect that hull with actual base material, then split by existing face
planes. No robot trajectory is subtracted and no CAD material is removed.
"""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))

import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from tools.assets.build_independent_task_box_layout import mesh_of, solid
from simulation.model_preparation.captive_fastener import mesh_vertices

def convex_parts(value):
    queue, output = [value], []
    cuts = 0
    while queue:
        part = queue.pop()
        mesh = mesh_of(part)
        mesh.merge_vertices()
        if np.linalg.svd(mesh.vertices-mesh.vertices.mean(0), compute_uv=False)[-1] < 1e-7:
            continue  # numerical zero-thickness Boolean seam, below 0.1 micrometre
        volume = part.volume()
        hull = mesh.convex_hull
        error = hull.volume-volume
        if error < max(1e-14, volume*.005):
            output.append(hull)
            continue
        planes, seen = [], set()
        for face, normal in zip(mesh.faces, mesh.face_normals, strict=True):
            offset = float(mesh.vertices[face[0]]@normal)
            key = tuple(np.round(np.r_[normal, offset], 6))
            if key not in seen:
                seen.add(key)
                planes.append((normal, offset))
        best = None
        for normal, offset in planes:
            pieces = part.split_by_plane(normal, offset)
            if min(piece.volume() for piece in pieces) < 1e-13:
                continue
            pieces = [piece for half in pieces for piece in half.decompose()]
            with np.errstate(invalid='ignore', divide='ignore'):
                gain = error-sum(mesh_of(piece).convex_hull.volume-piece.volume() for piece in pieces)
            if best is None or gain > best[0]:
                best = gain, pieces
        if best is None or best[0] < 1e-14:
            if error < 2e-10:
                output.append(hull)
                continue
            raise ValueError(f'No accurate supporting-plane split: volume={volume}, error={error}')
        queue.extend(best[1])
        cuts += 1
        if cuts > 200:
            raise ValueError('Collision partition exceeded its supporting-plane budget')
    return output


def build(source, output):
    source, output = Path(source).resolve(), Path(output).resolve()
    root = ET.parse(source).getroot()
    geom = root.find('.//geom[@name="task_box_base_geom_collision_016"]')
    if geom is None:
        raise ValueError('Expected the original coarse base collision hull')
    asset = root.find(f'./asset/mesh[@name="{geom.get("mesh")}"]')
    compiler = root.find('compiler')
    directory = compiler.get('meshdir', '')
    coarse_path = (source.parent/directory/asset.get('file')).resolve()
    coarse = trimesh.load(coarse_path, process=True)
    coarse.vertices *= np.fromstring(asset.get('scale', '1 1 1'), sep=' ')
    vertices = mesh_vertices(root, 'task_box_base', source.parent/directory)
    base = trimesh.Trimesh(vertices=vertices, faces=np.arange(len(vertices)).reshape(-1, 3), process=True)
    exact = solid(base)^solid(coarse)
    pieces = convex_parts(exact)
    volume = sum(piece.volume for piece in pieces)
    relative_error = volume/exact.volume()-1
    if not 0 <= relative_error < 2e-5:
        raise ValueError(f'Collision partition volume error exceeds acceptance: {relative_error}')
    output.mkdir(parents=True, exist_ok=True)
    files = []
    for index, piece in enumerate(pieces):
        path = output/f'locking_sweep_base_{index:03d}.obj'
        path.write_text(trimesh.exchange.obj.export_obj(piece, digits=17))
        files.append(path.name)
    record = dict(schema='locking-sweep-collision/1', source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  coarse_mesh_sha256=hashlib.sha256(coarse_path.read_bytes()).hexdigest(),
                  source_geom=geom.get('name'), files=files,
                  method='Original CAD intersection partitioned by supporting face planes',
                  volume_m3=volume, source_volume_m3=exact.volume(), relative_volume_error=relative_error,
                  discarded_seam_thickness_m=1e-7, visual_geometry_changed=False,
                  sha256={name: hashlib.sha256((output/name).read_bytes()).hexdigest() for name in files})
    (output/'locking_sweep_base.json').write_text(json.dumps(record, indent=2)+'\n')
    return record


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=ROOT/'model/SARM/platform/sarm_20261012.xml')
    parser.add_argument('--output', type=Path, default=ROOT/'model/SARM/platform/mechanism_collision')
    args = parser.parse_args()
    print(json.dumps(build(args.source, args.output), indent=2))
