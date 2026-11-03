"""Cut an actual rear equipment pocket, with matching collision solids.

Mesh booleans run only in the geometry Python process. The portable replay
contract uses standard XML/JSON so native verification need not load MuJoCo.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def subtract_box(low, high, cut_low, cut_high):
    """Disjoint boxes covering exactly the original AABB minus the cut AABB."""
    lo, hi = list(low), list(high)
    cl = [max(a, b) for a, b in zip(lo, cut_low)]
    ch = [min(a, b) for a, b in zip(hi, cut_high)]
    if any(b <= a for a, b in zip(cl, ch)):
        return [(lo, hi)]
    parts = []
    for axis in range(3):
        if lo[axis] < cl[axis]:
            end = hi[:]; end[axis] = cl[axis]
            parts.append((lo[:], end)); lo[axis] = cl[axis]
        if hi[axis] > ch[axis]:
            start = lo[:]; start[axis] = ch[axis]
            parts.append((start, hi[:])); hi[axis] = ch[axis]
    return parts


def clip_surface_pocket(mesh, low, high):
    """Exact polygon clipping for authored open CAD surfaces, without caps.

    Emit disjoint outside polygons at each of the six planes, then discard
    only the remaining interior polygon. Keep the original winding.
    """
    import numpy as np
    import trimesh
    triangles = []
    def half(poly, axis, bound, sign):
        out = []
        for a, b in zip(poly, poly[1:] + poly[:1]):
            da, db = sign*(a[axis]-bound), sign*(b[axis]-bound)
            if da >= 0:
                out.append(a)
            if (da >= 0) != (db >= 0):
                out.append(a + (b-a)*(da/(da-db)))
        return out
    def emit(poly):
        for i in range(1, len(poly)-1):
            tri = np.array([poly[0], poly[i], poly[i+1]])
            if np.linalg.norm(np.cross(tri[1]-tri[0], tri[2]-tri[0])) > 1e-16:
                triangles.append(tri)
    for tri in mesh.triangles:
        if np.any(tri.max(0) <= low) or np.any(tri.min(0) >= high):
            triangles.append(tri)
            continue
        poly = list(tri)
        for axis in range(3):
            for bound, sign in ((low[axis], 1), (high[axis], -1)):
                emit(half(poly, axis, bound, -sign))
                poly = half(poly, axis, bound, sign)
                if not poly:
                    break
            if not poly:
                break
    vertices = np.asarray(triangles).reshape(-1, 3)
    return trimesh.Trimesh(vertices=vertices, faces=np.arange(len(vertices)).reshape(-1, 3), process=True)


def apply_recess_contract(root, contract_path):
    """Replay declared scoped edits and verify original/new mesh file hashes."""
    report = json.loads(Path(contract_path).read_text(encoding='utf-8'))
    if report['schema'] != 'taskbox-recess/1':
        raise ValueError('Unsupported recess contract')
    result = deepcopy(root)
    for asset in report['assets']:
        if sha(asset['attributes']['file']) != asset['sha256']:
            raise ValueError('Recess mesh bytes changed')
        ET.SubElement(result.find('asset'), 'mesh', asset['attributes'])
    for record in report['edits']:
        parent = result.find(f'.//body[@name="{record["parent"]}"]')
        old = parent.find(f'geom[@name="{record["original"]["name"]}"]')
        if old is None or old.attrib != record['original']:
            raise ValueError('Recess source geometry mismatch: '+record['original']['name'])
        if record.get('source_mesh') and sha(record['source_mesh']['file']) != record['source_mesh']['sha256']:
            raise ValueError('Original recess mesh bytes changed')
        index = list(parent).index(old); parent.remove(old)
        for offset, attributes in enumerate(record['replacement']):
            parent.insert(index+offset, ET.Element('geom', attributes))
    return result, report


def carve_rear_pocket(root, housing, output):
    import numpy as np
    center = np.asarray(housing['back_mount_center_xz_m'])
    hull = np.asarray(housing['housing_bounds_m'])
    # 5 mm around the complete 450 x 550 mm panel and 5 mm behind its
    # 102 mm internal liner. The cutter starts outside the rear surface.
    low = np.array([center[0]-.230, hull[0, 1]-.005, center[1]-.280])
    high = np.array([center[0]+.230, hull[0, 1]+.107, center[1]+.280])
    if np.any(low[[0, 2]] < hull[0, [0, 2]]) or np.any(high[[0, 2]] > hull[1, [0, 2]]) or high[1] >= hull[1, 1]:
        raise ValueError('Pocket must fit inside the housing and preserve its front face')
    return carve_pocket(root, housing, output, low, high, surface_axis=1, surface=float(hull[0, 1]))


def carve_pocket(root, housing, output, low, high, *, surface_axis, surface):
    """Carve a negative-axis hull face while retaining declared protected parts."""
    import numpy as np
    import trimesh
    from .taskbox_mount import _world_pose, _pose
    low, high = np.asarray(low), np.asarray(high)
    hull = np.asarray(housing['housing_bounds_m'])
    tangents = [i for i in range(3) if i != surface_axis]
    if (surface_axis not in (0, 1, 2) or np.any(high <= low)
            or np.any(low[tangents] < hull[0, tangents])
            or np.any(high[tangents] > hull[1, tangents])
            or high[surface_axis] >= hull[1, surface_axis]
            or not low[surface_axis] < surface < high[surface_axis]):
        raise ValueError('Pocket must enter only its selected hull face')
    target = root.find('.//body[@name="capture_target"]')
    tp, tr = _world_pose(root, target)
    cutter = trimesh.creation.box(extents=high-low, transform=trimesh.transformations.translation_matrix((high+low)/2))
    output = Path(output); output.mkdir(parents=True, exist_ok=False)
    report = dict(schema='taskbox-recess/1', pocket_bounds_target_m=[low.tolist(), high.tolist()],
                  work_surface_axis=surface_axis, work_surface_coordinate_m=surface,
                  depth_inside_hull_m=float(high[surface_axis]-surface),
                  panel_clearance_m=.005, protected_front_unchanged=True, assets=[], edits=[])
    if surface_axis == 1:
        report['work_surface_y_target_m'] = surface
    assets = {n.get('name'): n for n in root.find('asset') if n.tag == 'mesh'}
    protected = set(housing['protected_geometry'])
    fmt = lambda a: ' '.join(format(float(v), '.17g') for v in a)
    for parent in target.iter('body'):
        bp, br = _world_pose(root, parent)
        for geom in list(parent):
            if geom.tag != 'geom':
                continue
            if geom.get('name') in protected:
                continue
            gp, gr = _pose(geom); rotation = tr.T@br@gr; position = tr.T@(bp+br@gp-tp)
            source_mesh = None
            if geom.get('mesh'):
                asset = assets[geom.get('mesh')]
                if any(k in asset.attrib for k in ('refpos', 'refquat')):
                    raise ValueError('Explicit mesh reference transform needs handling')
                mesh = trimesh.load(asset.get('file'), force='mesh', process=True)
                vertices = np.asarray(mesh.vertices)*np.fromstring(asset.get('scale', '1 1 1'), sep=' ')
                vertices = vertices@rotation.T+position
                bound = np.array([vertices.min(0), vertices.max(0)])
                source_mesh = dict(file=asset.get('file'), sha256=sha(asset.get('file')), attributes=dict(asset.attrib))
            elif geom.get('type') == 'box':
                if not np.allclose(rotation, np.eye(3), atol=1e-10):
                    raise ValueError('Rotated box requires explicit clipping')
                half = np.fromstring(geom.get('size'), sep=' '); bound = np.array([position-half, position+half])
            else:
                continue
            if np.any(bound[1] <= low) or np.any(bound[0] >= high):
                continue
            name = geom.get('name')
            if name in protected:
                raise ValueError('Pocket would cut protected capture face: '+name)
            replacements = []
            if source_mesh:
                if geom.get('contype', '0') != '0' or geom.get('conaffinity', '0') != '0':
                    raise ValueError('Active collision mesh needs convex decomposition: '+name)
                mesh.vertices = vertices
                mesh.fix_normals(multibody=True)
                source_mesh['closed_volume'] = bool(mesh.is_volume)
                source_mesh['cut_method'] = 'manifold_closed_solid' if mesh.is_volume else 'exact_open_surface_polygon_clipping'
                cut = (trimesh.boolean.difference([mesh, cutter], engine='manifold', check_volume=True)
                       if mesh.is_volume else clip_surface_pocket(mesh, low, high))
                if not len(cut.faces) and not (np.all(bound[0] >= low) and np.all(bound[1] <= high)):
                    raise ValueError('Boolean unexpectedly erased geometry outside the pocket: '+name)
                if len(cut.faces):
                    cut.vertices = (cut.vertices-position)@rotation
                    path = output/(name+'.stl'); cut.export(path)
                    asset_name = 'recess_'+name
                    attributes = dict(name=asset_name, file=path.resolve().as_posix(), scale='1 1 1', inertia='shell')
                    report['assets'].append(dict(attributes=attributes, sha256=sha(path)))
                    replacement = dict(geom.attrib); replacement['mesh'] = asset_name; replacements.append(replacement)
            else:
                for index, (a, b) in enumerate(subtract_box(*bound, low, high)):
                    a, b = np.asarray(a), np.asarray(b)
                    replacement = dict(geom.attrib)
                    replacement['name'] = name if index == 0 else name+'_recess_'+str(index)
                    replacement['pos'] = fmt(br.T@(tp+tr@((a+b)/2)-bp))
                    replacement['size'] = fmt((b-a)/2)
                    replacements.append(replacement)
            report['edits'].append(dict(parent=parent.get('name'), original=dict(geom.attrib),
                                        replacement=replacements, source_mesh=source_mesh,
                                        original_bounds_target_m=bound.tolist()))
    contract = output.parent/'recess-contract.json'
    contract.write_text(json.dumps(report, indent=2), encoding='utf-8')
    return apply_recess_contract(root, contract)[0], report
