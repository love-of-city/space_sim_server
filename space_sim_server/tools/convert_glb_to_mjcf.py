"""Convert a static, untextured Draco GLB 2.0 to a fixed, visual-only MJCF.

This deliberately does not infer mechanical joints, collision shapes, physical
scale, mass, or inertia. Node world transforms are baked into OBJ coordinates;
the source node tree remains as fixed grouping bodies with identity transforms.
Supported subset: embedded buffer, triangle primitives, POSITION/NORMAL, plain
baseColorFactor materials, no animation, skinning, morphs, or textures.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import re
import struct
import xml.etree.ElementTree as ET

import numpy as np

DRACO_EXTENSION = 'KHR_draco_mesh_compression'
# Proper rotation (det=+1): glTF +Y up -> MuJoCo +Z up, no reflection.
Y_UP_TO_Z_UP = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])


def safe_name(text: str) -> str:
    return re.sub(r'[^A-Za-z0-9_]+', '_', text).strip('_') or 'unnamed'


def numbers(values) -> str:
    return ' '.join(format(float(v), '.12g') for v in values)


def read_glb(source: Path):
    raw = source.read_bytes()
    if len(raw) < 20:
        raise ValueError('Truncated GLB header')
    magic, version, total = struct.unpack_from('<4sII', raw)
    if magic != b'glTF' or version != 2 or total != len(raw):
        raise ValueError('Expected a complete GLB 2.0 file')
    chunks = []
    offset = 12
    while offset < total:
        if offset + 8 > total:
            raise ValueError('Truncated GLB chunk header')
        length, kind = struct.unpack_from('<II', raw, offset)
        offset += 8
        if length % 4 or offset + length > total:
            raise ValueError('Invalid GLB chunk length/alignment')
        chunks.append((kind, raw[offset:offset + length]))
        offset += length
    if [kind for kind, _ in chunks] != [0x4E4F534A, 0x004E4942]:
        raise ValueError('Expected exactly one JSON chunk followed by one BIN chunk')
    doc = json.loads(chunks[0][1])
    if doc.get('asset', {}).get('version') != '2.0':
        raise ValueError('Expected glTF asset version 2.0')
    buffers = doc.get('buffers', [])
    binary = chunks[1][1]
    if len(buffers) != 1 or 'uri' in buffers[0]:
        raise ValueError('Only one embedded GLB buffer is supported')
    length = buffers[0]['byteLength']
    if length < 0 or not 0 <= len(binary) - length <= 3:
        raise ValueError('Embedded buffer length does not match the BIN chunk')
    unsupported = set(doc.get('extensionsUsed', [])) | set(doc.get('extensionsRequired', []))
    unsupported -= {DRACO_EXTENSION}
    if unsupported:
        raise ValueError(f'Unsupported glTF extensions: {sorted(unsupported)}')
    for feature in ('images', 'textures', 'animations', 'skins'):
        if doc.get(feature):
            raise ValueError(f'{feature} are not supported by this visual-only converter')
    for material in doc.get('materials', []):
        pbr = material.get('pbrMetallicRoughness', {})
        if (material.get('alphaMode', 'OPAQUE') != 'OPAQUE'
                or any('Texture' in key for key in (*material, *pbr))
                or any(material.get('emissiveFactor', [0, 0, 0]))):
            raise ValueError('Only opaque, untextured, non-emissive materials are supported')
    return doc, binary[:length], hashlib.sha256(raw).hexdigest(), len(raw)


def node_matrix(node: dict) -> np.ndarray:
    if 'matrix' in node:
        if any(key in node for key in ('rotation', 'translation', 'scale')):
            raise ValueError('A glTF node cannot mix matrix and TRS transforms')
        transform = np.asarray(node['matrix'], dtype=float).reshape(4, 4, order='F')
    else:
        x, y, z, w = np.asarray(node.get('rotation', [0, 0, 0, 1]), dtype=float)
        norm = math.sqrt(x*x + y*y + z*z + w*w)
        if not math.isfinite(norm) or norm == 0:
            raise ValueError('Invalid node rotation quaternion')
        x, y, z, w = np.array([x, y, z, w]) / norm
        rotation = np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
        ])
        transform = np.eye(4)
        transform[:3, :3] = rotation @ np.diag(node.get('scale', [1, 1, 1]))
        transform[:3, 3] = node.get('translation', [0, 0, 0])
    if (not np.isfinite(transform).all()
            or not np.allclose(transform[3], [0, 0, 0, 1], rtol=0, atol=1e-12)
            or np.linalg.slogdet(transform[:3, :3])[0] == 0):
        raise ValueError('Node transform must be finite, affine, and nonsingular')
    return transform


def scene_nodes(doc: dict):
    scenes = doc.get('scenes', [])
    selected = doc.get('scene', 0)
    if not scenes or not 0 <= selected < len(scenes):
        raise ValueError('No valid default scene')
    nodes = doc.get('nodes', [])
    visited = set()
    result = []

    def visit(index, parent_index, parent_world):
        if not 0 <= index < len(nodes) or index in visited:
            raise ValueError('Invalid, cyclic, or multiply-parented scene node')
        visited.add(index)
        node = nodes[index]
        if 'skin' in node or 'weights' in node:
            raise ValueError('Skinned/morphed nodes are not supported')
        world = parent_world @ node_matrix(node)
        result.append(dict(index=index, parent=parent_index, source=node,
                           body_name=f'node_{index:03d}_{safe_name(node.get("name", "node"))}',
                           world=world))
        for child in node.get('children', []):
            visit(child, index, world)

    for index in scenes[selected].get('nodes', []):
        visit(index, None, np.eye(4))
    return result


def decode_primitive(doc: dict, binary: bytes, primitive: dict):
    import DracoPy

    if primitive.get('mode', 4) != 4 or primitive.get('targets'):
        raise ValueError('Only unmorphed triangle primitives are supported')
    attributes = primitive.get('attributes', {})
    if 'POSITION' not in attributes or set(attributes) - {'POSITION', 'NORMAL'}:
        raise ValueError('Only POSITION and optional NORMAL attributes are supported')
    ext = primitive.get('extensions', {}).get(DRACO_EXTENSION)
    if ext is None or set(ext.get('attributes', {})) != set(attributes):
        raise ValueError('All primitive attributes must be present in the Draco payload')
    view = doc['bufferViews'][ext['bufferView']]
    start, length = view.get('byteOffset', 0), view['byteLength']
    if view.get('buffer', 0) != 0 or start < 0 or length <= 0 or start + length > len(binary):
        raise ValueError('Draco bufferView lies outside the embedded buffer')
    decoded = DracoPy.decode(binary[start:start + length])
    if not hasattr(decoded, 'faces'):
        raise ValueError('Draco point clouds are not supported')

    def attribute(semantic):
        # IDs in KHR_draco_mesh_compression are unique IDs, not array ordinals.
        attr = decoded.get_attribute_by_unique_id(ext['attributes'][semantic])
        if attr is None:
            raise ValueError(f'Missing Draco attribute {semantic}')
        values = np.asarray(attr['data'], dtype=np.float64)
        accessor = doc['accessors'][attributes[semantic]]
        if (accessor['type'] != 'VEC3' or accessor['componentType'] != 5126
                or values.shape != (accessor['count'], 3) or not np.isfinite(values).all()):
            raise ValueError(f'Decoded {semantic} does not match its glTF accessor')
        return values

    vertices = attribute('POSITION')
    faces = np.asarray(decoded.faces, dtype=np.int64)
    index_accessor = doc['accessors'][primitive['indices']]
    if (faces.ndim != 2 or faces.shape[1] != 3 or len(faces) == 0
            or faces.size != index_accessor['count'] or index_accessor['type'] != 'SCALAR'
            or faces.min() < 0 or faces.max() >= len(vertices)):
        raise ValueError('Decoded triangle indices do not match glTF or are out of bounds')
    source_faces = len(faces)
    twice_area = np.linalg.norm(np.cross(vertices[faces[:, 1]] - vertices[faces[:, 0]],
                                         vertices[faces[:, 2]] - vertices[faces[:, 0]]), axis=1)
    # Remove ONLY exactly zero-area triangles already present after decompression.
    # No visual decimation, epsilon-based removal, or invented replacement surfaces.
    faces = faces[twice_area > 0]
    if len(faces) == 0:
        raise ValueError('Primitive has no non-degenerate triangles')
    if 'NORMAL' in attributes:
        normals = attribute('NORMAL')
    else:
        normals = np.zeros_like(vertices)
        cross = np.cross(vertices[faces[:, 1]] - vertices[faces[:, 0]],
                         vertices[faces[:, 2]] - vertices[faces[:, 0]])
        for corner in range(3):
            np.add.at(normals, faces[:, corner], cross)
    used, remap = np.unique(faces, return_inverse=True)
    vertices, normals, faces = vertices[used], normals[used], remap.reshape(-1, 3)
    lengths = np.linalg.norm(normals, axis=1)
    if np.any(lengths == 0):
        raise ValueError('A used vertex has an undefined normal')
    normals /= lengths[:, None]
    return dict(vertices=vertices, normals=normals, faces=faces,
                decoded_triangles=source_faces, decoded_vertices=int(decoded.points.shape[0]),
                dropped_zero_area_triangles=source_faces - len(faces))


def transform_geometry(vertices, normals, faces, transform):
    linear = transform[:3, :3]
    v = vertices @ linear.T + transform[:3, 3]
    n = normals @ np.linalg.inv(linear)
    n /= np.linalg.norm(n, axis=1)[:, None]
    f = faces[:, [0, 2, 1]] if np.linalg.det(linear) < 0 else faces.copy()
    return v, n, f


def obj_text(vertices, normals, faces):
    rows = ['# Decoded GLB visual surface; transformed positions and source normals.']
    rows.extend('v ' + numbers(v) for v in vertices)
    rows.extend('vn ' + numbers(n) for n in normals)
    rows.extend('f ' + ' '.join(f'{int(i)+1}//{int(i)+1}' for i in face) for face in faces)
    return '\n'.join(rows) + '\n'


def prepare(doc, binary, scale):
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError('Scale must be a positive finite multiplier')
    mapping = np.eye(4)
    mapping[:3, :3] = scale * Y_UP_TO_Z_UP
    nodes = scene_nodes(doc)
    meshes = []
    cache = {}
    for node in nodes:
        if 'mesh' not in node['source']:
            continue
        mi = node['source']['mesh']
        for pi, primitive in enumerate(doc['meshes'][mi]['primitives']):
            key = (mi, pi)
            if key not in cache:
                cache[key] = decode_primitive(doc, binary, primitive)
            decoded = cache[key]
            v, n, f = transform_geometry(decoded['vertices'], decoded['normals'],
                                          decoded['faces'], mapping @ node['world'])
            material_index = primitive.get('material')
            material = doc['materials'][material_index] if material_index is not None else {}
            rgba = material.get('pbrMetallicRoughness', {}).get('baseColorFactor', [1, 1, 1, 1])
            if len(rgba) != 4 or not np.isfinite(rgba).all() or np.any(np.array(rgba) < 0) or np.any(np.array(rgba) > 1):
                raise ValueError('Invalid baseColorFactor')
            rgba = [*rgba[:3], 1.]  # OPAQUE means ignore baseColorFactor alpha.
            name = f'n{node["index"]:03d}_m{mi:02d}_p{pi:02d}'
            meshes.append(dict(name=name, node_index=node['index'], mesh_index=mi,
                               primitive_index=pi, vertices=v, normals=n, faces=f,
                               rgba=rgba, material_index=material_index,
                               decoded_triangles=decoded['decoded_triangles'],
                               decoded_vertices=decoded['decoded_vertices'],
                               dropped_zero_area_triangles=decoded['dropped_zero_area_triangles']))
    if not meshes:
        raise ValueError('Selected scene contains no triangle meshes')
    return nodes, meshes


def create_mjcf(name, doc, nodes, meshes):
    root = ET.Element('mujoco', model=name)
    root.append(ET.Comment('FIXED VISUAL MODEL ONLY. No recovered joints, collision, mass, or inertia.'))
    root.append(ET.Comment('Source world transforms baked into OBJ. glTF (x,y,z) -> MJCF (x,-z,y).'))
    ET.SubElement(root, 'compiler', angle='radian', meshdir='meshes',
                  inertiafromgeom='false', fusestatic='false')
    ET.SubElement(root, 'option', gravity='0 0 0', timestep='0.002')
    default = ET.SubElement(root, 'default')
    ET.SubElement(default, 'geom', type='mesh', group='2', contype='0', conaffinity='0', mass='0')
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'headlight', ambient='0.3 0.3 0.3', diffuse='0.7 0.7 0.7', specular='0.1 0.1 0.1')
    asset = ET.SubElement(root, 'asset')
    for i, mat in enumerate(doc.get('materials', [])):
        rgba = mat.get('pbrMetallicRoughness', {}).get('baseColorFactor', [1, 1, 1, 1])
        ET.SubElement(asset, 'material', name=f'mat_{i:02d}_{safe_name(mat.get("name", "material"))}',
                      rgba=numbers([*rgba[:3], 1.]), specular='0.15', shininess='0.2')
    for mesh in meshes:
        # Shell integration allows open/planar visual surfaces. This is NOT
        # engineering inertia: inference is disabled and all bodies are fixed.
        ET.SubElement(asset, 'mesh', name=mesh['name'], file=mesh['name']+'.obj', inertia='shell')
    world = ET.SubElement(root, 'worldbody')
    assembly = ET.SubElement(world, 'body', name=name)
    bodies = {}
    for node in nodes:
        parent = assembly if node['parent'] is None else bodies[node['parent']]
        bodies[node['index']] = ET.SubElement(parent, 'body', name=node['body_name'])
    for mesh in meshes:
        attrs = dict(name='visual_'+mesh['name'], mesh=mesh['name'], rgba=numbers(mesh['rgba']))
        i = mesh['material_index']
        if i is not None:
            attrs['material'] = f'mat_{i:02d}_{safe_name(doc["materials"][i].get("name", "material"))}'
        ET.SubElement(bodies[mesh['node_index']], 'geom', **attrs)
    return root


def validate(xml, meshes):
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    if (model.njnt or model.nq or model.nv or model.nu or model.ngeom != len(meshes)
            or np.any(model.geom_contype) or np.any(model.geom_conaffinity)
            or np.any(model.body_mass) or np.any(model.body_inertia)):
        raise AssertionError('Expected fixed, zero-inertia, contact-disabled visual model')
    errors, normal_errors, bounds = [], [], []
    material_error = 0.
    for mesh in meshes:
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'visual_'+mesh['name'])
        mid = model.geom_dataid[gid]
        va, nv = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
        fa, nf = model.mesh_faceadr[mid], model.mesh_facenum[mid]
        na, nn = model.mesh_normaladr[mid], model.mesh_normalnum[mid]
        rotation = data.geom_xmat[gid].reshape(3, 3)
        vertices = model.mesh_vert[va:va+nv].astype(float) @ rotation.T + data.geom_xpos[gid]
        normals = model.mesh_normal[na:na+nn].astype(float) @ rotation.T
        faces = model.mesh_face[fa:fa+nf]
        normal_faces = model.mesh_facenormal[fa:fa+nf]
        if nf != len(mesh['faces']):
            raise AssertionError('Compiled mesh lost or gained triangles')
        error = float(np.linalg.norm(vertices[faces] - mesh['vertices'][mesh['faces']], axis=2).max())
        nerror = float(np.linalg.norm(normals[normal_faces] - mesh['normals'][mesh['faces']], axis=2).max())
        tolerance = max(1e-6, float(np.ptp(mesh['vertices'], axis=0).max())*2e-6)
        if error > tolerance or nerror > 2e-5:
            raise AssertionError(f'Compiled geometry/normal mismatch in {mesh["name"]}: {error}, {nerror}')
        errors.append(error)
        normal_errors.append(nerror)
        bounds.append([vertices.min(0), vertices.max(0)])
        material_error = max(material_error, float(np.max(np.abs(model.geom_rgba[gid]-mesh['rgba']))))
        if mesh['material_index'] is not None:
            material_error = max(material_error, float(np.max(np.abs(model.mat_rgba[model.geom_matid[gid]]-mesh['rgba']))))
    if material_error > 1e-6:
        raise AssertionError('Material base colors were not preserved')
    for _ in range(100):
        mujoco.mj_step(model, data)
    warning_counts = [int(warning.number) for warning in data.warning]
    if any(warning_counts) or data.ncon or not np.isfinite(data.xpos).all():
        raise AssertionError('MuJoCo fixed-scene smoke test reported a problem')
    return dict(mujoco_version=mujoco.__version__, compiled=True,
                bodies_including_world=model.nbody, mesh_assets=model.nmesh,
                visual_geoms=model.ngeom, joints=model.njnt, dofs=model.nv, actuators=model.nu,
                max_triangle_vertex_error_m=max(errors), max_normal_vector_error=max(normal_errors),
                max_rgba_error=material_error, all_body_masses_and_inertias_zero=True,
                compiled_bounds_m=[np.min(np.array(bounds)[:, 0], axis=0).tolist(),
                                   np.max(np.array(bounds)[:, 1], axis=0).tolist()],
                static_smoke_test_steps=100, warning_counts=warning_counts,
                contacts=int(data.ncon), dynamics_calibrated=False)


def convert(source: Path, output: Path, *, name='nasa_lro_a', scale=1.0):
    source, output = source.resolve(), output.resolve()
    if name != safe_name(name):
        raise ValueError('Model name must contain only ASCII letters, digits, and underscores')
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise FileExistsError('Refusing to overwrite a nonempty output directory; choose a new directory')
    doc, binary, sha256, source_bytes = read_glb(source)
    nodes, meshes = prepare(doc, binary, scale)
    vertices = np.concatenate([m['vertices'] for m in meshes])
    lo, hi = vertices.min(0), vertices.max(0)
    dimensions = hi-lo
    output.mkdir(parents=True, exist_ok=True)
    (output/'meshes').mkdir()
    assets = []
    for mesh in meshes:
        path = output/'meshes'/(mesh['name']+'.obj')
        path.write_text(obj_text(mesh['vertices'], mesh['normals'], mesh['faces']), encoding='ascii', newline='\n')
        assets.append(dict(file=path.relative_to(output).as_posix(), sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    root = create_mjcf(name, doc, nodes, meshes)
    ET.indent(root, space='  ')
    xml = output/(name+'.xml')
    ET.ElementTree(root).write(xml, encoding='utf-8', xml_declaration=True)
    validation = validate(xml, meshes)
    if hashlib.sha256(source.read_bytes()).hexdigest() != sha256:
        raise AssertionError('Source changed during conversion')
    primitive_records = []
    for mesh in meshes:
        record = {k: v for k, v in mesh.items() if k not in ('vertices', 'normals', 'faces')}
        record.update(exported_vertices=len(mesh['vertices']), exported_triangles=len(mesh['faces']),
                      bounds_m=[mesh['vertices'].min(0).tolist(), mesh['vertices'].max(0).tolist()])
        primitive_records.append(record)
    report = dict(
        status='complete', model_file=xml.name, visual_only=True,
        source=dict(file=str(source), bytes=source_bytes, sha256=sha256,
                    format='GLB 2.0', generator=doc.get('asset', {}), modified=False),
        versions={key: importlib.metadata.version(key) for key in ('DracoPy', 'numpy', 'mujoco')},
        coordinates=dict(mapping='glTF (x,y,z) -> MJCF (x,-z,y)',
                         uniform_scale_multiplier=scale, meters_per_source_unit=scale,
                         physical_scale_verified=False, original_origin_preserved=True,
                         source_world_transforms_baked_into_obj=True,
                         fixed_body_frames_are_not_mechanical_pivots=True),
        bounds_m=[lo.tolist(), hi.tolist()], dimensions_m=dimensions.tolist(),
        counts=dict(source_mesh_groups=len(doc['meshes']), source_nodes=len(doc['nodes']),
                    scene_nodes=len(nodes), source_materials=len(doc.get('materials', [])),
                    visual_primitives=len(meshes),
                    decoded_triangles=sum(m['decoded_triangles'] for m in meshes),
                    dropped_zero_area_triangles=sum(m['dropped_zero_area_triangles'] for m in meshes),
                    exported_triangles=sum(len(m['faces']) for m in meshes)),
        assumptions=[
            'Static visual asset; no recovered joints, collision shapes, actuators, mass, or inertia.',
            'Preserve file scale by default; actual satellite dimensions have not been calibrated.',
            'All selected-scene node transforms baked into OBJ vertices; grouping bodies have identity transforms.',
            'Base colors and vertex normals retained; glTF PBR metallic/roughness appearance is not reproduced.',
            'Only exactly zero-area triangles removed; no simplification or invented surface thickness.',
            'Mesh shell integration is a compiler aid, not physical inertia; body inertia inference disabled.',
            'No platform/UE integration or native OpenGL rendering has been tested.',
        ],
        nodes=[dict(index=n['index'], parent=n['parent'], body_name=n['body_name'], source=n['source'],
                    source_world_transform=n['world'].tolist()) for n in nodes],
        source_materials=doc.get('materials', []), primitives=primitive_records,
        assets=assets, model_sha256=hashlib.sha256(xml.read_bytes()).hexdigest(), validation=validation,
    )
    (output/'conversion_manifest.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    write_readme(output, report)
    return report


def write_readme(output, report):
    counts, val = report['counts'], report['validation']
    dimensions = ' × '.join(f'{v:.6f}' for v in report['dimensions_m'])
    text = f'''# NASA LRO (A) — 固定结构 MJCF 视觉模型

## 打开什么

- 主文件：`{report['model_file']}`；必须与 `meshes/` 一起保留，不能只复制 XML。
- `preview.html` / `preview/overview.png`：运行预览脚本后生成的离线几何预览。
- `conversion_manifest.json`：源文件校验和、节点/材质映射、逐网格统计和验证结果。

## 本版范围

- 原始 GLB 未修改，SHA-256：`{report['source']['sha256']}`。
- {counts['source_mesh_groups']} 个源网格组、{counts['scene_nodes']} 个源节点、{counts['source_materials']} 个基础色材质，导出 {counts['visual_primitives']} 个 OBJ / visual geom。
- 源文件解码得到 {counts['decoded_triangles']:,} 个三角形，其中 {counts['dropped_zero_area_triangles']:,} 个面积严格为零。仅剔除这些无可见面积的退化面，保留 {counts['exported_triangles']:,} 个非退化三角形；没有减面或补厚。
- 保留分组层级，但所有 body 都固定。`Layer` / `Pivot` 名称不是机械关节证据。
- 不添加关节、自由基座、执行器、碰撞体或虚构的质量/惯量；关闭几何惯量推断。`inertia="shell"` 仅用于开放/平面网格的编译处理，不代表卫星真实惯量。
- 保留基础色和顶点法线，不保证 glTF 金属/粗糙度 PBR 材质与 MuJoCo 显示完全相同。

## 坐标与尺寸（重要）

- glTF 的 Y 向上转换为 MJCF 的 Z 向上：`(x, y, z) -> (x, -z, y)`，右手旋转，不镜像。
- 所选场景的节点世界变换已烘焙入 OBJ；固定 body 的坐标变换均为单位变换。不能把 body 原点当作转轴位置。
- 原点不居中、不移动；统一缩放倍率为 `{report['coordinates']['uniform_scale_multiplier']}`，即每个源坐标单位按 `{report['coordinates']['meters_per_source_unit']}` 米解释。
- 导出轴向包围盒 X/Y/Z 为 **{dimensions} m**。
- **上述数字是文件尺度，不是经实测确认的 LRO 尺寸。真实尺寸尚未校准。** 确认一个已知长度后可使用转换器 `--scale` 参数在新目录重新生成，不能直接把此版用于真实动力学计算。

## 验证

- MuJoCo `{val['mujoco_version']}` 加载/编译成功。
- {val['visual_geoms']} 个 visual geom；关节 / 自由度 / 执行器 = 0 / 0 / 0；所有 body 质量和惯量均为 0（固定视觉节点）。
- 编译后逐三角形位置对照已通过，最大顶点误差 `{val['max_triangle_vertex_error_m']:.3g} m`；法线和基础色对照通过。
- 100 步固定场景 smoke test 无警告、无接触；这不是活动机构或真实动力学验证。
- 未改动平台场景/默认配置；尚未测试平台或 UE 导入，也未使用原生 OpenGL 渲染器验证。

## 复现（在仓库根目录执行；建议独立 Python 3.11 环境）

```powershell
python -m pip install --only-binary=:all: -r tools/requirements-glb-to-mjcf.txt
python tools/convert_glb_to_mjcf.py "run/nasa-lro-a/Lunar Reconnaissance Orbiter (A).glb" --output model/nasa_lro_a_new --name nasa_lro_a
python tools/render_glb_preview.py model/nasa_lro_a_new
```

转换器拒绝覆盖已有非空目录。本工具仅支持静态、无纹理、Draco 压缩的三角网格 GLB 子集，不是通用 glTF 场景/动画导入器。
预览使用已有的 CPU z-buffer 绘制 MuJoCo 编译后的几何，非 AI 图片，非平台/UE 截图。

## 格式依据

- Khronos glTF 2.0 specification，Coordinate System and Units。
- Khronos `KHR_draco_mesh_compression` extension，属性按 unique ID 解码。
- MuJoCo XML Reference，mesh / material / compiler。
- DracoPy 官方解码库（转换记录中列出确切版本）。
'''
    (output/'README.md').write_text(text, encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--name', default='nasa_lro_a')
    parser.add_argument('--scale', type=float, default=1., help='Explicit uniform size multiplier; default preserves source scale')
    args = parser.parse_args()
    report = convert(args.source, args.output, name=args.name, scale=args.scale)
    print(json.dumps({key: report[key] for key in ('status', 'model_file', 'counts', 'dimensions_m', 'validation')}, indent=2))


if __name__ == '__main__':
    main()
