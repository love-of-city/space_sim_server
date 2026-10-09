"""One passive outer-panel hinge, leaving the existing model and platform untouched.

This is a motion-preview MJCF, NOT calibrated dynamics. The supplied CAD hinge
is a merged solid; its original surface triangles are partitioned through the
measured axis for display only. This does not reconstruct internal knuckles,
pins, bearings or contact geometry. No real limits or mass data are inferred.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

SOURCE_SHA256 = 'a165279b773a15c19c6c600144b0e51f309fb76c3a46cae4b3871faac36076d9'
OUTPUT_XML = 'ground_validation_satellite_articulated.xml'
OUTER_BODY = 'satellite_outer_panel'
INNER_BODY = 'satellite_inner_panel'
JOINT = 'outer_panel_hinge'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def nums(values):
    return ' '.join(f'{float(v):.14g}' for v in values)


def quat_matrix(quaternion):
    q = np.asarray(quaternion, float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-12:
        raise ValueError('Invalid quaternion')
    w, x, y, z = q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def axis_rotation(axis, angle):
    axis = np.asarray(axis, float)
    axis = axis/np.linalg.norm(axis)
    x, y, z = axis
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3)+math.sin(angle)*k+(1-math.cos(angle))*(k@k)


def descendants(manifest, root):
    children = {}
    for n in manifest['assembly']:
        children.setdefault(n['parent'], []).append(n['id'])
    result, pending = set(), [root]
    while pending:
        key = pending.pop()
        if key in result:
            raise ValueError('Invalid assembly graph')
        result.add(key)
        pending.extend(children.get(key, []))
    return result


def read_obj_triangles(path):
    vertices, normals, faces = [], [], []
    for line in path.read_text(encoding='utf-8').splitlines():
        row = line.split()
        if not row:
            continue
        if row[0] == 'v':
            vertices.append([float(x) for x in row[1:4]])
        elif row[0] == 'vn':
            normals.append([float(x) for x in row[1:4]])
        elif row[0] == 'f':
            if len(row) != 4:
                raise ValueError('Expected triangular source OBJ')
            face = []
            for corner in row[1:]:
                fields = corner.split('/')
                if len(fields) != 3 or not fields[2]:
                    raise ValueError('Expected explicit corner normals')
                face.append(np.r_[vertices[int(fields[0])-1], normals[int(fields[2])-1]])
            faces.append(face)
    result = np.asarray(faces, float)
    if result.ndim != 3 or result.shape[1:] != (3, 6) or not np.all(np.isfinite(result)):
        raise ValueError('Invalid OBJ triangles')
    return result


def area(triangles):
    if len(triangles) == 0:
        return 0.
    t = np.asarray(triangles)
    return float(np.linalg.norm(np.cross(t[:, 1, :3]-t[:, 0, :3], t[:, 2, :3]-t[:, 0, :3]), axis=1).sum()/2)


def clip_polygon(polygon, normal, offset, positive):
    """Sutherland-Hodgman clipping, retaining winding and corner-normal data."""
    output = []
    for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
        da, db = float(a[:3]@normal-offset), float(b[:3]@normal-offset)
        inside_a = da >= 0 if positive else da <= 0
        inside_b = db >= 0 if positive else db <= 0
        if inside_a:
            output.append(a.copy())
        if inside_a != inside_b:
            t = da/(da-db)
            c = a+t*(b-a)
            c[:3] -= (c[:3]@normal-offset)*normal
            output.append(c)
    return output


def split_surface(triangles, normal, offset):
    """Partition surfaces, never invent cut caps or silently discard whole faces.

The positive half-space moves. Entirely coplanar triangles belong only to the
fixed half, avoiding duplicated area. Corner-normal data is interpolated at cut edges; the consuming renderer may
renormalize it. Only surface geometry, not identical shading, is guaranteed.
"""
    normal = np.asarray(normal, float)
    length = np.linalg.norm(normal)
    if length < 1e-12 or not np.all(np.isfinite(normal)) or not math.isfinite(offset):
        raise ValueError('Invalid clipping plane')
    normal, offset = normal/length, offset/length
    fixed, moving = [], []
    crossing = 0
    for triangle in triangles:
        distances = triangle[:, :3]@normal-offset
        if np.all(np.abs(distances) < 1e-13):
            fixed.append(triangle.copy())
            continue
        if distances.min() < 0 < distances.max():
            crossing += 1
        for positive, destination in ((False, fixed), (True, moving)):
            polygon = clip_polygon(triangle, normal, offset, positive)
            for i in range(1, len(polygon)-1):
                tri = np.array([polygon[0], polygon[i], polygon[i+1]])
                if area(tri[None]) > 1e-24:
                    destination.append(tri)
    fixed, moving = np.asarray(fixed), np.asarray(moving)
    error = abs(area(fixed)+area(moving)-area(triangles))
    if not len(fixed) or not len(moving) or error > max(1e-15, area(triangles)*1e-10):
        raise ValueError('Surface partition failed area conservation')
    return fixed, moving, dict(source_triangles=len(triangles), fixed_triangles=len(fixed),
                               moving_triangles=len(moving), cut_triangles=crossing,
                               source_area_m2=area(triangles), area_error_m2=error,
                               generated_cut_caps=False, mechanical_internals_reconstructed=False)


def obj_text(triangles):
    lines = ['# Visual-only partition of original CAD surfaces. Not contact/collision geometry.']
    corners = np.asarray(triangles).reshape(-1, 6)
    lines += ['v '+nums(row[:3]) for row in corners]
    lines += ['vn '+nums(row[3:]) for row in corners]
    lines += [f'f {i}//{i} {i+1}//{i+1} {i+2}//{i+2}' for i in range(1, len(corners)+1, 3)]
    return '\n'.join(lines)+'\n'


def world_vertices(model, data, gid):
    mid = int(model.geom_dataid[gid])
    va, nv = int(model.mesh_vertadr[mid]), int(model.mesh_vertnum[mid])
    return np.asarray(model.mesh_vert[va:va+nv], float) @ data.geom_xmat[gid].reshape(3, 3).T+data.geom_xpos[gid]


def validate(original, output, mapping, pivot, axis):
    import mujoco
    src = mujoco.MjModel.from_xml_path(str(original))
    model = mujoco.MjModel.from_xml_path(str(output))
    sd, data = mujoco.MjData(src), mujoco.MjData(model)
    mujoco.mj_forward(src, sd); mujoco.mj_forward(model, data)
    if (model.nq, model.nv, model.nu, model.njnt) != (1, 1, 0, 1):
        raise AssertionError('Expected exactly one passive hinge')
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, JOINT)
    if jid != 0 or model.jnt_type[jid] != mujoco.mjtJoint.mjJNT_HINGE or model.jnt_limited[jid]:
        raise AssertionError('Unexpected joint type or invented angle limits')
    outer = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, OUTER_BODY)
    inner = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, INNER_BODY)
    assert model.body_parentid[outer] == inner and model.body_jntnum[inner] == 0
    assert not np.any(model.geom_contype) and not np.any(model.geom_conaffinity)
    baseline = {}
    errors = []
    for record in mapping:
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, record['output_geom'])
        sid = mujoco.mj_name2id(src, mujoco.mjtObj.mjOBJ_GEOM, record['source_geom'])
        assert gid >= 0 and sid >= 0
        vertices = world_vertices(model, data, gid)
        baseline[gid] = vertices
        if record['partition'] is None:
            reference = world_vertices(src, sd, sid)
            assert vertices.shape == reference.shape
            errors.append(float(np.max(np.abs(vertices-reference))))
        if record['moving']:
            assert model.geom_bodyid[gid] == outer
        else:
            assert model.geom_bodyid[gid] != outer
    for source_name in {r['source_geom'] for r in mapping if r['partition'] is not None}:
        sid = mujoco.mj_name2id(src, mujoco.mjtObj.mjOBJ_GEOM, source_name)
        reference = world_vertices(src, sd, sid)
        combined = np.concatenate([baseline[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, r['output_geom'])]
                                   for r in mapping if r['source_geom'] == source_name])
        errors.append(float(np.max(np.abs(np.array([combined.min(0), combined.max(0)])-
                                             np.array([reference.min(0), reference.max(0)])))))
    if max(errors) > 5e-8:
        raise AssertionError('Zero-pose geometry moved')
    motion_error = 0.
    test_angles = [-45, 0, 30, 60, 90, 135, 180]
    for angle in test_angles:
        data.qpos[0] = math.radians(angle)
        mujoco.mj_forward(model, data)
        rot = axis_rotation(axis, data.qpos[0])
        for gid, before in baseline.items():
            expected = (before-pivot)@rot.T+pivot if model.geom_bodyid[gid] == outer else before
            error = float(np.max(np.abs(world_vertices(model, data, gid)-expected)))
            motion_error = max(motion_error, error)
    if motion_error > 5e-8:
        raise AssertionError('Incorrect movement or fixed parts moved')
    mujoco.mj_resetData(model, data)
    data.qvel[0] = .2
    for _ in range(500):
        mujoco.mj_step(model, data)
    assert np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel))
    assert not any(int(w.number) for w in data.warning)
    return dict(nq=model.nq, nv=model.nv, nu=model.nu, body_count_excluding_world=model.nbody-1,
                geom_count=model.ngeom, neutral_pose_max_vertex_or_bounds_error_m=max(errors),
                rigid_motion_max_vertex_error_m=motion_error, tested_angles_deg=test_angles,
                tested_angles_are_not_joint_limits=True, finite_passive_steps=500,
                timestep_s=float(model.opt.timestep), simulation_warnings=0)


def build(folder):
    folder = Path(folder).resolve()
    source_xml = folder/'ground_validation_satellite.xml'
    manifest_file = folder/'conversion_manifest.json'
    manifest = json.loads(manifest_file.read_text(encoding='utf-8'))
    source = folder.parent/manifest['source']['filename']
    review = json.loads((folder/'articulation_review/candidates.json').read_text(encoding='utf-8'))
    if manifest['source']['sha256'] != SOURCE_SHA256 or digest(source) != SOURCE_SHA256 or review['source_sha256'] != SOURCE_SHA256:
        raise ValueError('Curated instance assignments require the inspected STEP')
    protected = [source, source_xml, manifest_file, folder/'satellite_free_preview.xml', folder.parent/'SARM/platform/sarm_platform.xml']
    hashes = {str(p): digest(p) for p in protected if p.exists()}
    for mesh in manifest['meshes']:
        if digest(folder/'meshes'/mesh['file']) != mesh['sha256']:
            raise ValueError('An original mesh changed: '+mesh['file'])
    a = next(c for c in review['candidates'] if c['id'] == 'A')
    pivot = np.asarray(a['preliminary_axis']['point_model_m'])
    axis = np.asarray(a['preliminary_axis']['direction_model'])
    axis /= np.linalg.norm(axis)
    if not np.allclose(axis, [0, 0, 1], atol=1e-9):
        raise ValueError('Unexpected measured hinge axis')
    inner_ids = descendants(manifest, 'instance_0131') | {'instance_0194', 'instance_0195'}
    outer_ids = descendants(manifest, 'instance_0198') | {'instance_0214'}
    hinge_ids = {'instance_0196', 'instance_0197'}
    assert not inner_ids & outer_ids
    gm = {g['name']: g for g in manifest['geoms']}
    hinges = [g for g in manifest['geoms'] if g['source_instance_id'] in hinge_ids]
    assert len(hinges) == 2 and len({g['mesh'] for g in hinges}) == 1
    planes = []
    for g in hinges:
        rot = quat_matrix(g['quaternion_wxyz'])
        normal = rot.T@np.array([-1., 0, 0])
        local_point = rot.T@(pivot-g['position_m'])
        planes.append((normal, float(local_point@normal)))
    if not np.allclose(planes[0][0], planes[1][0], atol=1e-10) or abs(planes[0][1]-planes[1][1]) > 1e-10:
        raise ValueError('Hinge occurrences require different surface partitions')
    triangles = read_obj_triangles(folder/'meshes'/(hinges[0]['mesh']+'.obj'))
    fixed, moving, split_info = split_surface(triangles, *planes[0])
    fixed_rel, moving_rel = 'articulated_meshes/hinge_fixed_visual.obj', 'articulated_meshes/hinge_moving_visual.obj'
    tree = ET.parse(source_xml, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
    root = tree.getroot()
    root.set('model', 'ground_validation_satellite_single_wing_hinge_preview')
    for comment in list(root):
        if comment.tag is ET.Comment:
            root.remove(comment)
    root.insert(0, ET.Comment(' One passive panel hinge. 1 kg synthetic moving-body mass and AABB inertia: NOT engineering data. No real joint limits. Hinge surfaces are display-only partitions, not mechanical internals. '))
    compiler = root.find('compiler'); compiler.set('meshdir', '.')
    asset = root.find('asset')
    for mesh in list(asset.findall('mesh')):
        if mesh.get('name') == hinges[0]['mesh']:
            asset.remove(mesh)
        else:
            mesh.set('file', 'meshes/'+mesh.get('file'))
    ET.SubElement(asset, 'mesh', name='hinge_fixed_visual', file=fixed_rel, inertia='shell')
    ET.SubElement(asset, 'mesh', name='hinge_moving_visual', file=moving_rel, inertia='shell')
    base = root.find('worldbody/body')
    inner = ET.Element('body', name=INNER_BODY)
    outer = ET.SubElement(inner, 'body', name=OUTER_BODY, pos=nums(pivot))
    outer.append(ET.Comment(' q=0 is the imported CAD pose, not a measured stowage zero. Unlimited only because actual stops are unknown. '))
    ET.SubElement(outer, 'joint', name=JOINT, type='hinge', pos='0 0 0', axis=nums(axis), limited='false')
    outer.append(ET.Comment(' Synthetic motion-preview inertia only. All visual geoms remain massless and non-colliding. '))
    moving_bounds = np.asarray([g['bounds_centered_m'] for g in manifest['geoms'] if g['source_instance_id'] in outer_ids])
    lo, hi = moving_bounds[:, 0].min(0), moving_bounds[:, 1].max(0)
    for g in hinges:
        rot = quat_matrix(g['quaternion_wxyz'])
        points = moving[:, :, :3].reshape(-1, 3)@rot.T+g['position_m']
        lo = np.minimum(lo, points.min(0)); hi = np.maximum(hi, points.max(0))
    center, size = (lo+hi)/2, hi-lo
    inertia = np.array([size[1]**2+size[2]**2, size[0]**2+size[2]**2, size[0]**2+size[1]**2])/12
    ET.SubElement(outer, 'inertial', pos=nums(center-pivot), mass='1', diaginertia=nums(inertia))
    mapping = []
    for geom in list(base.findall('geom')):
        g = gm[geom.get('name')]; instance = g['source_instance_id']
        if instance in hinge_ids:
            base.remove(geom)
            for partition, parent, moves in [('fixed', inner, False), ('moving', outer, True)]:
                copy = deepcopy(geom)
                copy.set('name', instance+'_hinge_'+partition+'_visual')
                copy.set('mesh', 'hinge_'+partition+'_visual')
                if moves:
                    copy.set('pos', nums(np.asarray(g['position_m'])-pivot))
                parent.append(copy)
                mapping.append(dict(source_geom=g['name'], output_geom=copy.get('name'), instance=instance,
                                    moving=moves, partition=partition))
        else:
            moves = instance in outer_ids
            if moves or instance in inner_ids:
                base.remove(geom)
                if moves:
                    geom.set('pos', nums(np.asarray(g['position_m'])-pivot))
                (outer if moves else inner).append(geom)
            mapping.append(dict(source_geom=g['name'], output_geom=geom.get('name'), instance=instance,
                                moving=moves, partition=None))
    base.append(inner)
    keyframe = ET.SubElement(root, 'keyframe')
    keyframe.append(ET.Comment(' Demonstration poses only, not mechanical limits. '))
    for name, angle in [('cad_pose', 0), ('demo_fold_45deg', 45), ('demo_fold_90deg', 90)]:
        ET.SubElement(keyframe, 'key', name=name, qpos=nums([math.radians(angle)]))
    ET.indent(tree, space='  ')
    generated = {fixed_rel: obj_text(fixed), moving_rel: obj_text(moving),
                 OUTPUT_XML: ET.tostring(root, encoding='unicode', xml_declaration=True)}
    marker_path = folder/'wing_articulation/build_manifest.json'
    previous = json.loads(marker_path.read_text(encoding='utf-8')) if marker_path.exists() else {}
    for rel in generated:
        dest = folder/rel
        if dest.exists() and digest(dest) != previous.get('generated_sha256', {}).get(rel):
            raise FileExistsError('Refusing to overwrite an unrecognized or manually edited file: '+str(dest))
    for rel, text in generated.items():
        dest = folder/rel; dest.parent.mkdir(exist_ok=True)
        dest.write_text(text, encoding='utf-8')
    generated_hashes = {rel: digest(folder/rel) for rel in generated}
    record = dict(schema='single-wing-motion-preview/1', source_sha256=SOURCE_SHA256,
                  model_file=OUTPUT_XML, generated_sha256=generated_hashes, protected_sha256=hashes,
                  joint=dict(name=JOINT, type='hinge', point_model_m=pivot.tolist(), direction_model=axis.tolist(),
                             actual_range_deg=None, limited=False, zero_pose='unchanged source CAD pose', actuator=None),
                  bodies=dict(inner=INNER_BODY, outer=OUTER_BODY),
                  synthetic_inertia=dict(mass_kg=1., center_model_m=center.tolist(), size_m=size.tolist(),
                                         diaginertia_kg_m2=inertia.tolist(), method='uniform AABB; NOT real mass or COM'),
                  hinge_visual_partition=dict(**split_info, plane_normal_mesh_local=planes[0][0].tolist(),
                                              plane_offset_mesh_local_m=planes[0][1],
                                              method='surface clipping through measured axis; not physical leaf/pin reconstruction'),
                  moving_leaf_instances=sorted({r['instance'] for r in mapping if r['moving'] and r['partition'] is None}),
                  attachment_note='Unlock device 02 follows its outer-panel mount; no internal unlock DOF/event. 01/03 remain fixed.',
                  collision_enabled=False, platform_integration_done=False, geometry_mapping=mapping,
                  validation_status='pending')
    marker_path.parent.mkdir(exist_ok=True)
    marker_path.write_text(json.dumps(record, indent=2, ensure_ascii=False), encoding='utf-8')
    record['validation'] = validate(source_xml, folder/OUTPUT_XML, mapping, pivot, axis)
    if any(digest(Path(p)) != h for p, h in hashes.items()):
        raise RuntimeError('A protected input changed during generation')
    record['protected_inputs_unchanged'] = True
    record['validation_status'] = 'passed'
    marker_path.write_text(json.dumps(record, indent=2, ensure_ascii=False), encoding='utf-8')
    print(json.dumps(record['validation'], indent=2), flush=True)
    print('Created:', folder/OUTPUT_XML, flush=True)
    return record


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('model_folder', type=Path)
    build(parser.parse_args().model_folder)
