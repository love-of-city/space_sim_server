"""Compose local task-box surfaces with upstream satellite/robot (offline only).

Requires numpy/scipy/trimesh/rtree. Never imports a physics DLL. See docs/TASK_BOX.md.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
import trimesh
from select_sarm_scene import verified_source_tree

ROOT = Path(__file__).resolve().parents[1]
FOLDER = ROOT / 'model/task_box/module'
SOURCE = ROOT / 'model/SARM/platform/sarm_ground_target_self_collision.xml'
OUTPUT = SOURCE.with_name('sarm_task_box_module.xml')
OWNED = (1, 8, 9, 10, 11, 12, 13, 14, 15)
ROTATION = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
TRANSLATION = np.array([-.2, .265, -.245])


def digest(path):
    data = path.read_bytes()
    if path.suffix in ('.xml', '.json'):
        data = data.replace(b'\r\n', b'\n')
    return hashlib.sha256(data).hexdigest()


def save_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def prepare(geometry):
    """Import ONLY owned source meshes, retaining transforms and source hashes."""
    package = json.loads(geometry.read_text(encoding='utf-8'))
    if package['units'] != 'm':
        raise ValueError('Only metre exports are supported')
    FOLDER.mkdir(parents=True, exist_ok=True)
    parts = []
    rows = {r['id']: r for r in package['instances']}
    for number in OWNED:
        row = rows[f'component_{number:03}']
        path = geometry.parent / row['mesh']
        name = f'component_{number:03}.stl'
        shutil.copyfile(path, FOLDER / name)
        cad = Path(row['source'])
        parts.append(dict(id=number, name=row['name'], mesh=name,
                          mesh_sha256=digest(path), cad_file=cad.name.split('_', 2)[-1],
                          cad_sha256=digest(cad), matrix=row['matrix'],
                          friction=[.6, .005, .0001], solref=[.01, 1.0]))
    save_json(FOLDER / 'assembly.json', dict(schema='task-box-module/1', units='m',
        source_geometry_sha256=digest(geometry), rotation=ROTATION.tolist(),
        translation_m=TRANSLATION.tolist(), parts=parts,
        attachment='cubesat_bus', motion='fixed_in_this_contact_validation_stage'))


def fmt(values):
    return ' '.join(f'{v:.12g}' for v in np.asarray(values).flat)


def clip_hull(mesh, axis, value, keep_less):
    """Exact half-space intersection of a convex polyhedron."""
    vertices = mesh.vertices
    d = vertices[:, axis] - value
    keep = d <= 1e-12 if keep_less else d >= -1e-12
    edges = mesh.edges_unique
    edges = edges[d[edges[:, 0]] * d[edges[:, 1]] < -1e-24]
    points = [vertices[keep]]
    if len(edges):
        a, b = edges.T
        t = d[a] / (d[a] - d[b])
        points.append(vertices[a] + t[:, None] * (vertices[b] - vertices[a]))
    points = np.unique(np.vstack(points), axis=0)
    if len(points) < 4 or np.linalg.matrix_rank(points - points.mean(0), tol=1e-10) < 3:
        return None
    hull = ConvexHull(points)
    return trimesh.Trimesh(points, hull.simplices, process=False)


def split_outside(mesh, bounds):
    result = []
    for axis in range(3):
        for side in (0, 1):
            piece = clip_hull(mesh, axis, bounds[side, axis], side == 0)
            if piece is not None:
                result.append(piece)
            mesh = clip_hull(mesh, axis, bounds[side, axis], side == 1)
            if mesh is None:
                return result
    return result


def connected_meshes(mesh):
    mesh = mesh.copy()
    mesh.merge_vertices()
    edges = mesh.edges_unique
    graph = coo_matrix((np.ones(len(edges)), edges.T), shape=(len(mesh.vertices),) * 2)
    _, labels = connected_components(graph, directed=False)
    face_labels = labels[mesh.faces[:, 0]]
    for label in np.unique(face_labels):
        part = mesh.copy()
        part.update_faces(face_labels == label)
        part.remove_unreferenced_vertices()
        yield part


def remove_coincident_deck(mesh, box_bounds):
    """Trim upstream z=0 deck patches only inside the local box footprint.

    The upstream fused CAD triangulates across the shared deck/box seam. Keep
    every outside triangle fragment, avoiding duplicate coplanar display/contact.
    This does not replace the upstream satellite cover with the local cover.
    """
    tri = mesh.triangles
    mask = np.all(np.abs(tri[:, :, 2]) < 5e-7, axis=1)
    mask &= np.all(tri.max(1)[:, :2] >= box_bounds[0, :2], axis=1)
    mask &= np.all(tri.min(1)[:, :2] <= box_bounds[1, :2], axis=1)
    retained = mesh.copy()
    retained.update_faces(~mask)
    deck = mesh.copy()
    deck.update_faces(mask)
    pieces = [retained]
    for axis in (0, 1):
        for side in (0, 1):
            if not len(deck.faces):
                break
            normal = np.zeros(3)
            normal[axis] = -1 if side == 0 else 1
            origin = np.zeros(3)
            origin[axis] = box_bounds[side, axis]
            pieces.append(trimesh.intersections.slice_mesh_plane(deck, normal, origin, cap=False))
            deck = trimesh.intersections.slice_mesh_plane(deck, -normal, origin, cap=False)
    return trimesh.util.concatenate(pieces), int(mask.sum())


def matching_faces(base, part, tolerance=5e-5):
    """Match CAD surface vertices within the owned part's bounds.

    Independent curved-surface tessellations have different chord midpoints even
    when their vertices lie on the same CAD surface. Testing chord midpoints
    leaves duplicate old hole walls behind. Require all three vertices within
    50 um instead; never remove a face merely because its bounding box overlaps.
    """
    tri = base.triangles
    bounds = part.bounds
    candidates = np.flatnonzero(np.all(tri.min(1) >= bounds[0] - tolerance, axis=1)
                                & np.all(tri.max(1) <= bounds[1] + tolerance, axis=1))
    selected = []
    for batch in np.array_split(candidates, max(1, (len(candidates) + 15) // 16)):
        if not len(batch):
            continue
        t = tri[batch]
        points = t
        _, distance, _ = trimesh.proximity.closest_point(part, points.reshape(-1, 3))
        selected.extend(batch[(distance.reshape(-1, 3) <= tolerance).all(1)])
    return np.array(selected, dtype=int)


def build():
    upstream_sources = verified_source_tree(ROOT, SOURCE)
    config_path = FOLDER / 'assembly.json'
    config = json.loads(config_path.read_text(encoding='utf-8'))
    if tuple(p['id'] for p in config['parts']) != OWNED:
        raise ValueError('Ownership changed; review module boundary before building')
    rotation, translation = np.array(config['rotation']), np.array(config['translation_m'])
    tree = ET.parse(SOURCE)
    root = tree.getroot()
    root.set('model', 'SARM_local_task_box_contacts')
    option = root.find('option')
    flag = option.find('flag')
    if flag is None:
        flag = ET.SubElement(option, 'flag')
    flag.attrib.update(contact='enable', constraint='enable', override='disable')
    asset = root.find('asset')
    bus = root.find('.//body[@name="cubesat_bus"]')
    original = bus.find('geom[@name="sarm_base_high_precision"]')
    base_asset = asset.find('mesh[@name="base_link_high_precision"]')
    base_path = (SOURCE.parent / base_asset.get('file')).resolve()
    base = trimesh.load_mesh(base_path)
    sources = {SOURCE.relative_to(ROOT).as_posix(): digest(SOURCE),
               base_path.relative_to(ROOT).as_posix(): digest(base_path),
               config_path.relative_to(ROOT).as_posix(): digest(config_path)}
    sources.update(upstream_sources)
    for element in asset.findall('mesh'):
        if element.get('file'):
            path = (SOURCE.parent / element.get('file')).resolve()
            sources[path.relative_to(ROOT).as_posix()] = digest(path)
    parts = []
    for row in config['parts']:
        path = FOLDER / row['mesh']
        if digest(path) != row['mesh_sha256']:
            raise ValueError(f'Changed source mesh: {path.name}; re-import CAD package')
        sources[path.relative_to(ROOT).as_posix()] = digest(path)
        part = trimesh.load_mesh(path)
        matrix = np.array(row['matrix']).reshape(4, 4)
        part.vertices = (part.vertices @ matrix[:3, :3].T + matrix[:3, 3]) @ rotation.T + translation
        part.update_faces(part.area_faces > 1e-15)
        parts.append((row, part))
    bounds = np.array([np.vstack([p.vertices for _, p in parts]).min(0),
                       np.vstack([p.vertices for _, p in parts]).max(0)])
    bounds += np.array([[-.001], [.001]])
    removed = np.zeros(len(base.faces), dtype=bool)
    replacements = []
    for row, part in parts:
        print(f'Matching component {row["id"]}: {len(part.faces)} CAD triangles', flush=True)
        ids = matching_faces(base, part)
        removed[ids] = True
        replacements.append(dict(id=row['id'], replaced_upstream_triangles=len(ids)))
    if replacements[0]['replaced_upstream_triangles'] < 100:
        raise ValueError('Task box does not align with upstream base; review registration')
    remainder = base.copy()
    remainder.update_faces(~removed)
    remainder, deck_seam_triangles = remove_coincident_deck(remainder, parts[0][1].bounds)

    def export(mesh, name):
        mesh.remove_unreferenced_vertices()
        path = FOLDER / (name + '.obj')
        mesh.export(path)
        sources[path.relative_to(ROOT).as_posix()] = digest(path)
        return '../../task_box/module/' + path.name

    def add_geom(geom):
        index = next((i for i, child in enumerate(bus) if child.tag == 'body'), len(bus))
        bus.insert(index, geom)

    def shell(mesh, name, row):
        # Plane clipping can leave numerical slivers; OBJ rounds coordinates.
        # Weld at export precision and remove resulting zero-width triangles.
        mesh.vertices = mesh.vertices.astype(np.float32).astype(float)
        mesh.merge_vertices(digits_vertex=7)
        mesh.update_faces(mesh.nondegenerate_faces(height=1e-7))
        flex = ET.Element('flexcomp', name=name, type='mesh', file=export(mesh, name),
                          dim='2', rigid='true', radius='0.00001', group='3', rgba='0 .7 .3 0')
        ET.SubElement(flex, 'contact', contype='0', conaffinity='3', selfcollide='none',
                      internal='false', condim='3', friction=fmt(row['friction']),
                      solref=fmt(row['solref']), margin='0')
        bus.append(flex)

    # Preserve the original visual triangles everywhere except matched owned surfaces.
    base_asset.set('file', export(remainder, 'upstream_base_without_task_box'))
    original.set('contype', '0')
    original.set('conaffinity', '0')
    # Keep the upstream convex collision volume OUTSIDE the task-box workspace.
    # Within it, use original triangles + local task-box triangles so holes stay open.
    for i, piece in enumerate(split_outside(base.convex_hull, bounds)):
        name = f'task_box_base_partition_{i}'
        ET.SubElement(asset, 'mesh', name=name, file=export(piece, name))
        add_geom(ET.Element('geom', name=name, type='mesh', mesh=name, mass='0',
                            group='3', rgba='0 .7 .3 0', contype='0', conaffinity='3'))
    tri = remainder.triangles
    overlap = np.all(tri.max(1) >= bounds[0], axis=1) & np.all(tri.min(1) <= bounds[1], axis=1)
    local_remainder = remainder.copy()
    local_remainder.update_faces(overlap & (remainder.area_faces > 1e-15))
    # Clip crossing triangles too: merely selecting overlapping triangles can
    # retain a large cover face extending all the way to the robot mounting base.
    for axis in range(3):
        for side in (0, 1):
            normal = np.zeros(3)
            normal[axis] = 1 if side == 0 else -1
            origin = np.zeros(3)
            origin[axis] = bounds[side, axis]
            local_remainder = trimesh.intersections.slice_mesh_plane(
                local_remainder, normal, origin, cap=False)
    # Original small satellite solids had a global convex collider already.
    # Per-solid hulls keep those coarse contacts cheap; large cover surfaces stay
    # triangles, preventing a new hull across the task-box openings.
    exact = []
    interface_hulls = 0
    for i, part in enumerate(connected_meshes(local_remainder)):
        if max(part.extents) > .3 or np.linalg.matrix_rank(part.vertices - part.vertices.mean(0), tol=1e-9) < 3:
            exact.append(part)
            continue
        name = f'task_box_interface_hull_{i}'
        ET.SubElement(asset, 'mesh', name=name, file=export(part.convex_hull, name))
        add_geom(ET.Element('geom', name=name, type='mesh', mesh=name, mass='0',
                            group='3', rgba='0 .7 .3 0', contype='0', conaffinity='3'))
        interface_hulls += 1
    exact_mesh = trimesh.util.concatenate(exact)
    shell(exact_mesh, 'task_box_upstream_interface', dict(friction=[1, .005, .0001], solref=[.02, 1]))
    for row, part in parts:
        name = f'task_box_{row["id"]:03}'
        ET.SubElement(asset, 'mesh', name=name, file=export(part, name))
        add_geom(ET.Element('geom', name=name + '_visual', type='mesh', mesh=name, mass='0',
                            contype='0', conaffinity='0', group='2', rgba='.55 .62 .7 1'))
        shell(part, name + '_contact', row)
    # Do not re-indent upstream subtrees; retain robot/control XML exactly.
    tree.write(OUTPUT, encoding='utf-8', xml_declaration=True)
    OUTPUT.write_text('\n'.join(line.rstrip() for line in OUTPUT.read_text(encoding='utf-8').splitlines()) + '\n', encoding='utf-8')
    save_json(OUTPUT.with_suffix('.manifest.json'), dict(schema='task-box-module/1',
        sha256=digest(OUTPUT), sources=sources, replacements=replacements,
        region_bounds_m=bounds.tolist(), source_base_triangles=len(base.faces),
        kept_upstream_triangles=int((~removed).sum()),
        coincident_deck_triangles_clipped=deck_seam_triangles,
        local_upstream_contact_triangles=len(local_remainder.faces),
        exact_upstream_contact_triangles=len(exact_mesh.faces),
        upstream_interface_convex_solids=interface_hulls,
        motion='fixed_in_this_contact_validation_stage',
        mass_policy='Preserve upstream bus inertia; not calibrated task-box mass.',
        limitations=['No connector release or insertion controller.',
                    'Physical connector insertion/release is not validated by clearance probes.',
                    'Outside task-box region upstream coarse convex collision is retained.']))
    print(OUTPUT)
    print(json.dumps(replacements))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry', type=Path, help='Import a new SolidWorks geometry.json first')
    args = parser.parse_args()
    if args.geometry:
        prepare(args.geometry)
    build()
