"""Evidence-only component review of this satellite; never adds simulation joints.

Renders compiled CAD meshes through the existing CPU z-buffer, then uses native
SVG/HTML for labels. No generative imagery, external services, OpenGL or new
packages. Colors identify inspection groups, NOT established moving rigid bodies.
"""
from __future__ import annotations
import argparse
import base64
import hashlib
import html
import json
import math
from pathlib import Path
import re
import sys

import numpy as np

SOURCE_SHA256 = 'a165279b773a15c19c6c600144b0e51f309fb76c3a46cae4b3871faac36076d9'
COLORS = {'A': '#23cdb1', 'B': '#ffb84c', 'C': '#d99aff', 'D': '#70baff'}


def decode_step_name(value):
    return re.sub(r'_X2_([0-9A-F]+)_X0_',
                  lambda m: bytes.fromhex(m[1]).decode('utf-16-be'), value)


def build_inventory(manifest):
    nodes = {n['id']: dict(n, decoded_name=decode_step_name(n['instance_name']),
                           geoms=[], leaf_instances=set()) for n in manifest['assembly']}
    geoms = {g['name']: g for g in manifest['geoms']}
    for g in geoms.values():
        key = g['source_instance_id']
        visited = set()
        while key is not None:
            if key in visited:
                raise ValueError('Assembly cycle')
            visited.add(key)
            node = nodes[key]
            node['geoms'].append(g['name'])
            node['leaf_instances'].add(g['source_instance_id'])
            key = node['parent']
    for node in nodes.values():
        node['leaf_instances'] = sorted(node['leaf_instances'])
        if node['geoms']:
            bounds = np.asarray([geoms[g]['bounds_centered_m'] for g in node['geoms']])
            lo, hi = bounds[:, 0].min(0), bounds[:, 1].max(0)
            node.update(bounds_m=[lo.tolist(), hi.tolist()], center_m=((lo+hi)/2).tolist(),
                        size_m=(hi-lo).tolist())
    return nodes


def canonical_axis(point, direction):
    """A line has no preferred axial origin or sign. Express it reproducibly."""
    p, d = np.asarray(point, float), np.asarray(direction, float)
    length = np.linalg.norm(d)
    if (p.shape != (3,) or d.shape != (3,) or not np.all(np.isfinite(p))
            or not np.all(np.isfinite(d)) or not np.isfinite(length) or length < 1e-12):
        raise ValueError('Invalid axis')
    d = d / length
    if d[np.argmax(np.abs(d))] < 0:
        d = -d
    return p - np.dot(p, d)*d, d


def transform_cylinder(cylinder, transform, origin):
    t = np.asarray(transform, float)
    p = t[:3, :3] @ (np.asarray(cylinder['axis_point_local_mm'])*.001) + t[:3, 3] - origin
    d = t[:3, :3] @ np.asarray(cylinder['axis_direction_local'])
    p, d = canonical_axis(p, d)
    return dict(face_index=cylinder['face_index'], radius_m=cylinder['radius_mm']*.001,
                line_point_model_m=p.tolist(), direction_model=d.tolist(),
                area_mm2=cylinder['area_mm2'])


def hex_linear(value):
    x = np.array([int(value[i:i+2], 16)/255 for i in (1, 3, 5)])
    return np.where(x <= .04045, x/12.92, ((x+.055)/1.055)**2.4)


def gather(xml, manifest, groups):
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(xml))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    if model.nq != 0:
        raise ValueError('Review expects the original fixed visual model')
    mapping = {g['name']: g['source_instance_id'] for g in manifest['geoms']}
    out = {k: [] for k in ('vertices', 'normals', 'faces', 'normal_faces', 'colors', 'owners')}
    vo = no = 0
    for gid in np.where(model.geom_group == 2)[0]:
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)
        instance = mapping[name]
        mid = model.geom_dataid[gid]
        va, nv = int(model.mesh_vertadr[mid]), int(model.mesh_vertnum[mid])
        na, nn = int(model.mesh_normaladr[mid]), int(model.mesh_normalnum[mid])
        fa, nf = int(model.mesh_faceadr[mid]), int(model.mesh_facenum[mid])
        r = data.geom_xmat[gid].reshape(3, 3)
        out['vertices'].append(np.asarray(model.mesh_vert[va:va+nv], float) @ r.T + data.geom_xpos[gid])
        out['normals'].append(np.asarray(model.mesh_normal[na:na+nn], float) @ r.T)
        out['faces'].append(np.asarray(model.mesh_face[fa:fa+nf], np.int32)+vo)
        out['normal_faces'].append(np.asarray(model.mesh_facenormal[fa:fa+nf], np.int32)+no)
        code = next((code for code, members in groups.items() if instance in members), None)
        color = hex_linear(COLORS[code]) if code else hex_linear('#77889a')
        out['colors'].append(np.tile(color, (nf, 1)))
        out['owners'].append(np.full(nf, int(instance.split('_')[1]), dtype=np.int32))
        vo += nv; no += nn
    result = {k: np.concatenate(v) for k, v in out.items()}
    result['normals'] = result['normals'].astype(np.float32)
    result['colors'] = result['colors'].astype(np.float32)
    if len(result['faces']) != manifest['counts']['instanced_triangles']:
        raise ValueError('Unexpected geometry count')
    return result


def select_geometry(geometry, instances):
    if instances is None:
        return geometry
    mask = np.isin(geometry['owners'], [int(s.split('_')[1]) for s in instances])
    if not np.any(mask):
        raise ValueError('No triangles selected')
    v, faces = np.unique(geometry['faces'][mask], return_inverse=True)
    n, normals = np.unique(geometry['normal_faces'][mask], return_inverse=True)
    return dict(vertices=geometry['vertices'][v], normals=geometry['normals'][n],
                faces=faces.reshape(-1, 3).astype(np.int32),
                normal_faces=normals.reshape(-1, 3).astype(np.int32),
                colors=geometry['colors'][mask], owners=geometry['owners'][mask])


def project(points, camera):
    cam = (np.asarray(points)-camera['eye']) @ np.asarray(camera['basis']).T
    if np.any(cam[:, 2] <= 0):
        raise ValueError('Projection behind camera')
    return np.column_stack((camera['width']/2+camera['focal']*cam[:, 0]/cam[:, 2],
                            camera['height']/2-camera['focal']*cam[:, 1]/cam[:, 2], 1/cam[:, 2]))


def render(geometry, destination, azimuth, elevation, width=1000, height=800):
    from PIL import Image
    from cpu_mjcf_preview import rasterize
    vertices = geometry['vertices']
    lo, hi = vertices.min(0), vertices.max(0)
    center = (lo+hi)/2
    az, el = math.radians(azimuth), math.radians(elevation)
    radial = np.array([math.cos(el)*math.cos(az), math.cos(el)*math.sin(az), math.sin(el)])
    forward = -radial
    right = np.cross(forward, [0, 0, 1]); right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    relative = vertices-center
    half_y = math.tan(math.radians(38)/2)*.76
    half_x = half_y*width/height
    distance = max(float(np.max(relative@radial+np.abs(relative@up)/half_y)),
                   float(np.max(relative@radial+np.abs(relative@right)/half_x)))
    distance += .02*float(np.linalg.norm(hi-lo))
    camera = dict(eye=(center+distance*radial).tolist(), basis=np.array([right, up, forward]).tolist(),
                  focal=height/(2*math.tan(math.radians(38)/2)), width=width, height=height,
                  azimuth_deg=azimuth, elevation_deg=elevation)
    projected = project(vertices, camera)
    if (np.any(projected[:, :2] < 0) or np.any(projected[:, 0] >= width)
            or np.any(projected[:, 1] >= height)):
        raise ValueError('Clipped geometry')
    key = np.array([1., -2, 3]); key /= np.linalg.norm(key)
    fill = np.array([-2., 1, 1]); fill /= np.linalg.norm(fill)
    pixels, depth = rasterize(projected, geometry['normals'], geometry['faces'],
                              geometry['normal_faces'], geometry['colors'], width, height, radial, key, fill)
    camera['foreground_pixels'] = int(np.count_nonzero(depth))
    if camera['foreground_pixels'] < 300:
        raise ValueError('Empty geometry preview')
    srgb = np.where(pixels <= .0031308, pixels*12.92, 1.055*np.maximum(pixels, 0)**(1/2.4)-.055)
    destination.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.uint8(np.clip(srgb*255, 0, 255))).save(destination)
    print('Rendered', destination.name, camera['foreground_pixels'], flush=True)
    return camera

def axis_feature(manifest, nodes, parts, instance, seed_face):
    node = nodes[instance]
    pid = next(p['id'] for p in manifest['parts'] if p['source_label'] == node['definition_label'])
    cylinders = parts[pid]['cylinders']
    seed = next(c for c in cylinders if c['face_index'] == seed_face)
    origin = np.asarray(manifest['bounds']['origin_shift_source_m'])
    t = np.asarray(node['transform_source_m'])
    base = transform_cylinder(seed, t, origin)
    p, d = np.asarray(base['line_point_model_m']), np.asarray(base['direction_model'])
    matched = []
    for c in cylinders:
        w = transform_cylinder(c, t, origin)
        if (np.linalg.norm(np.cross(d, w['direction_model'])) < 1e-5
                and np.linalg.norm(np.cross(np.asarray(w['line_point_model_m'])-p, d)) < 1e-6):
            matched.append(w)
    center_local = np.mean(seed['bounds_local_mm'], axis=0)*.001
    center = t[:3, :3] @ center_local + t[:3, 3] - origin
    anchor = p + np.dot(center-p, d)*d
    return dict(instance=instance, part_id=pid, source_label=node['definition_label'],
                seed_face_index=seed_face, point_model_m=anchor.tolist(), direction_model=d.tolist(),
                line_point_model_m=p.tolist(), matched_cylindrical_faces=matched,
                interpretation='analytic cylindrical feature; physical joint unconfirmed')


def make_findings(manifest, nodes, evidence):
    parts = evidence['parts']
    a_upper = axis_feature(manifest, nodes, parts, 'instance_0196', 1)
    a_lower = axis_feature(manifest, nodes, parts, 'instance_0197', 1)
    p = np.asarray(a_upper['line_point_model_m'])
    d = np.asarray(a_upper['direction_model'])
    line_error = float(np.linalg.norm(np.cross(np.asarray(a_lower['line_point_model_m'])-p, d)))
    if line_error > 1e-6 or abs(np.dot(d, a_lower['direction_model'])) < .999999:
        raise ValueError('Expected two coaxial hinge features were not found')
    middle = (np.asarray(nodes['instance_0196']['center_m'])+nodes['instance_0197']['center_m'])/2
    hinge_point = p + np.dot(middle-p, d)*d
    a_upper['point_model_m'] = (p+np.dot(np.asarray(nodes['instance_0196']['center_m'])-p, d)*d).tolist()
    a_lower['point_model_m'] = (p+np.dot(np.asarray(nodes['instance_0197']['center_m'])-p, d)*d).tolist()
    b_features = [dict(id=f'B{i}', **axis_feature(manifest, nodes, parts, 'instance_0114', face))
                  for i, face in enumerate((99, 228, 289), 1)]
    for i, word in ((194, '解锁器01'), (195, '解锁器03'), (214, '解锁器02')):
        if nodes[f'instance_{i:04}']['decoded_name'] != word:
            raise ValueError('Unexpected assembly mapping')
    groups = {
        'A': nodes['instance_0198']['leaf_instances']+['instance_0196', 'instance_0197'],
        'B': nodes['instance_0111']['leaf_instances'],
        'C': ['instance_0194', 'instance_0195', 'instance_0214'],
        'D': ['instance_0030'],
    }
    candidates = [
        dict(id='A', title='外侧板及板间铰链', confidence='较强的折叠关节候选', category='articulation_candidate',
             assembly_roots=['instance_0198'], inspection_instances=groups['A'],
             evidence=['外侧板与内侧板属于不同子装配；外侧板板面约 360 × 414.5 mm。',
                       '板缝上下各有一套 WXJL180-1 几何件；两件的圆柱面轴线共线，方向平行模型 Z 轴。',
                       '每套找到 10 个同轴圆柱面，两套合计 20 个；不是靠板缝外观猜轴。'],
             limitations=['这里只将两个铰链视为同一候选转轴的证据，并未确认自由度或驱动方式。',
                          '铰链件在 STEP 中各为一个 SOLID，内部固定叶片、活动叶片及销轴还需细分。',
                          '内侧板暂作固定参考 R；没有确认它与星体之间存在第二条活动转轴。',
                          '型号中的 180 不能直接作为运动限位；展开方向、收拢零位、角度范围均未知。'],
             ask='外侧板是否相对内侧板折叠？内侧板是否固定在星体上？',
             preliminary_axis=dict(point_model_m=hinge_point.tolist(), direction_model=d.tolist(),
                                   upper=a_upper, lower=a_lower, max_line_offset_m=line_error),
             proposed_joint_type='hinge_if_confirmed'),
        dict(id='B', title='底部自拍相机 / 支杆组件', confidence='中等：疑似展开连杆机构', category='articulation_candidate',
             assembly_roots=['instance_0111'], inspection_instances=groups['B'],
             evidence=['原始装配名解码为“自拍相机”，不是仅凭外观猜测设备名称。',
                       '孤立显示可见底座、分段支杆和末端结构；沿支杆找到三处平行于模型 X 轴的圆柱轴线特征 B1–B3。'],
             limitations=['B1–B3 只是疑似转接位置，不等于三个已确认的独立关节。',
                          '是否为联动、闭环、锁定或仅安装调节结构，需要实际机构说明。',
                          '现有两份叶级形状为实体/曲面汇总，不是按活动连杆划分；高亮整套不表示底座也应运动。'],
             ask='自拍相机支杆是否能够展开或折叠？请指出固定底座与活动杆段。',
             preliminary_cylindrical_features=b_features, proposed_joint_type=None),
        dict(id='C', title='解锁器 01 / 02 / 03', confidence='名称明确；运动形式待确认', category='release_mechanism_review',
             assembly_roots=['instance_0194', 'instance_0195', 'instance_0214'], inspection_instances=groups['C'],
             evidence=['原始装配名分别为“解锁器01”“解锁器02”“解锁器03”。',
                       '01/03 位于内侧板外缘同一区域，02 位于外侧板远端；外形和装配位置已单独标出。'],
             limitations=['名称可支持解锁用途，但无法据此确定内部是销轴滑动、转动还是一次性释放。',
                          '若只关心板展开，可能采用锁定/解锁状态，而不是给每个外壳加活动关节。'],
             ask='是否需要模拟解锁器内部运动，还是只需要锁定/解锁事件？', proposed_joint_type=None),
        dict(id='D', title='正面分离机构 · 星体端', confidence='接口位置明确；不是已确认的内部关节', category='separation_interface_review',
             assembly_roots=['instance_0029'], inspection_instances=groups['D'],
             evidence=['原始装配名为“分离机构”，叶级名称为“WF10_星体端”。',
                       '对应星体正面的方形接口，本次高亮的是已有的星体端形状。'],
             limitations=['该名称不能证明方形接口本体要相对卫星运动。',
                          '本次没有据此建立对接端、解锁部件或额外自由度；默认仍固定。'],
             ask='这里只保留固定接口，还是需要模拟卫星与外部对象的分离过程？', proposed_joint_type=None),
    ]
    for c in candidates:
        c.update(confirmed=False, confirmed_joint_type=None, range_deg=None, zero_pose=None, motion_sense=None)
        c['source_instances'] = [dict(id=i, name=nodes[i]['decoded_name'],
                                      definition_label=nodes[i]['definition_label']) for i in c['inspection_instances']]
        c['visual_geoms'] = sorted({g for i in c['inspection_instances'] for g in nodes[i]['geoms']})
    return groups, dict(schema='satellite-component-review/1', source_sha256=SOURCE_SHA256,
                        status='awaiting_user_confirmation', no_model_edits=True, no_joints_added=True,
                        coordinate_frame='current fixed MJCF, metres; NOT source STEP mm and NOT measured COM',
                        source_origin_shift_m=manifest['bounds']['origin_shift_source_m'],
                        all_range_values_unconfirmed=True, candidates=candidates,
                        fixed_reference=dict(id='R', instance='instance_0131', title='内侧板：暂作固定参考'),
                        exclusions=[dict(instance=i, title=nodes[i]['decoded_name'], reason='本次未找到足够证据将其列为外部活动关节')
                                    for i in ['instance_0036', 'instance_0043', 'instance_0075', 'instance_0081', 'instance_0115', 'instance_0121']])


def png_uri(path):
    return 'data:image/png;base64,'+base64.b64encode(path.read_bytes()).decode('ascii')


def svg_view(image, camera, callouts=(), segments=()):
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="800" viewBox="0 0 1000 800">',
             '<style>text{font-family:"Microsoft YaHei","Noto Sans CJK SC",sans-serif}</style>',
             f'<image width="1000" height="800" href="{png_uri(image)}"/>']
    for a, b, color in segments:
        x, y = project([a, b], camera)[:, :2]
        parts.append(f'<path d="M{x[0]:.2f},{x[1]:.2f} L{y[0]:.2f},{y[1]:.2f}" stroke="#10202d" stroke-width="6"/>')
        parts.append(f'<path d="M{x[0]:.2f},{x[1]:.2f} L{y[0]:.2f},{y[1]:.2f}" stroke="{color}" stroke-width="3" stroke-dasharray="8 6"/>')
    for point, x, y, label, color in callouts:
        px, py = project([point], camera)[0, :2]
        if not (0 <= px < 1000 and 0 <= py < 800):
            raise ValueError(f'Offscreen annotation: {label}')
        w = max(110, 24+sum(22 if ord(c)>127 else 13 for c in label))
        ax = x if px < x else min(x+w, max(x, px))
        ay = y+22
        parts.extend([f'<path d="M{ax},{ay} L{px:.2f},{py:.2f}" fill="none" stroke="{color}" stroke-width="2"/>',
                      f'<circle cx="{px:.2f}" cy="{py:.2f}" r="5" fill="{color}" stroke="#0a1623" stroke-width="2"/>',
                      f'<rect x="{x}" y="{y}" width="{w}" height="44" rx="9" fill="#102033" stroke="{color}" stroke-width="1.5"/>',
                      f'<text x="{x+12}" y="{y+29}" font-size="22" fill="{color}">{html.escape(label)}</text>'])
    parts.append('</svg>')
    return ''.join(parts)


def write_diagrams(folder, nodes, findings, cameras):
    a, b = findings['candidates'][:2]
    axis = a['preliminary_axis']; p = np.asarray(axis['point_model_m']); d = np.asarray(axis['direction_model'])
    line = (p-.16*d, p+.16*d, '#b3fff3')
    b_anchors = [f['point_model_m'] for f in b['preliminary_cylindrical_features']]
    rear = svg_view(folder/'images/rear.png', cameras['rear'], [
        (nodes['instance_0199']['center_m'], 65, 55, 'A 外侧板', COLORS['A']),
        (nodes['instance_0214']['center_m'], 40, 560, 'C2 解锁器02', COLORS['C']),
        (nodes['instance_0195']['center_m'], 690, 55, 'C1/C3 解锁器01/03', COLORS['C']),
        (nodes['instance_0132']['center_m'], 640, 680, 'R 内侧板：先按固定', '#bac6d4'),
        (p, 355, 720, 'A 上下同轴铰链', COLORS['A']),
    ], [line])
    front = svg_view(folder/'images/front.png', cameras['front'], [
        (nodes['instance_0030']['center_m'], 40, 55, 'D 分离机构·星体端', COLORS['D']),
        (b_anchors[-1], 220, 690, 'B 自拍相机 / 支杆', COLORS['B']),
        (nodes['instance_0199']['center_m'], 735, 630, 'A 外侧板', COLORS['A']),
    ])
    pair = svg_view(folder/'images/hinge_pair.png', cameras['hinge_pair'], [
        (axis['upper']['point_model_m'], 50, 140, 'A 上铰链', COLORS['A']),
        (axis['lower']['point_model_m'], 50, 565, 'A 下铰链', COLORS['A']),
        (nodes['instance_0186']['center_m'], 650, 95, 'R 内侧安装座', '#bac6d4'),
    ], [line])
    b_side = svg_view(folder/'images/selfie_camera_side.png', cameras['selfie_camera_side'],
                      [(anchor, 30, yy, f'B{i} 疑似转接处', COLORS['B'])
                       for i, (anchor, yy) in enumerate(zip(b_anchors, [270, 425, 600]), 1)])
    for name, svg in [('rear_annotated', rear), ('front_annotated', front), ('hinge_pair_annotated', pair), ('selfie_camera_annotated', b_side)]:
        (folder/(name+'.svg')).write_text(svg, encoding='utf-8')
    overview = f'''<svg xmlns="http://www.w3.org/2000/svg" width="1600" height="1030" viewBox="0 0 1600 1030">
<style>text{{font-family:"Microsoft YaHei","Noto Sans CJK SC",sans-serif}}</style>
<rect width="1600" height="1030" fill="#0c1623"/>
<text x="32" y="48" fill="#edf5ff" font-size="30" font-weight="bold">地面验证星｜疑似活动组件确认图</text>
<text x="32" y="82" fill="#b5c7da" font-size="18">只标候选，不添加关节｜灰色为其余几何；高亮整组不表示组内每个零件都应运动</text>
<text x="30" y="126" fill="#dae8f7" font-size="22">背面：板间铰链、解锁器</text>
<text x="830" y="126" fill="#dae8f7" font-size="22">正面：自拍相机、分离接口</text>
<g transform="translate(0,140) scale(.8)">{rear}</g>
<g transform="translate(800,140) scale(.8)">{front}</g>
<rect x="24" y="800" width="1552" height="166" rx="12" fill="#142234"/>
<text x="46" y="839" fill="{COLORS['A']}" font-size="23">A 较强候选：外侧板折叠</text>
<text x="46" y="871" fill="#b8c9db" font-size="18">上下两处同轴；先按一条候选转轴理解，待机构确认。</text>
<text x="46" y="913" fill="{COLORS['B']}" font-size="23">B 中等候选：自拍相机支杆</text>
<text x="46" y="945" fill="#b8c9db" font-size="18">可见分段结构；是否展开、如何联动尚未确认。</text>
<text x="824" y="839" fill="{COLORS['C']}" font-size="23">C 解锁器：释放形式待确认</text>
<text x="824" y="871" fill="#b8c9db" font-size="18">不因名称或圆柱外壳就添加滑动 / 旋转关节。</text>
<text x="824" y="913" fill="{COLORS['D']}" font-size="23">D 分离接口：默认固定</text>
<text x="824" y="945" fill="#b8c9db" font-size="18">“星体端”并不意味着接口本体相对星体活动。</text>
<text x="32" y="1007" fill="#93abc4" font-size="18">依据：原始 STEP 装配名、解析圆柱面和 MuJoCo 编译几何。未确定角度范围；非 AI 生成图。</text>
</svg>'''
    (folder/'overview.svg').write_text(overview, encoding='utf-8')
    return dict(overview=overview, rear=rear, front=front, pair=pair, b_side=b_side)

def write_report(folder, manifest, nodes, findings, diagrams):
    cards = []
    detail_images = {'A': diagrams['pair'], 'B': diagrams['b_side'],
                     'C': f'<img alt="解锁器01和03局部" src="{png_uri(folder/"images/release_latch.png")}">',
                     'D': f'<img alt="分离机构星体端" src="{png_uri(folder/"images/separation_interface.png")}">'}
    for c in findings['candidates']:
        cid = c['id']
        observed = ''.join(f'<li>{html.escape(t)}</li>' for t in c['evidence'])
        limits = ''.join(f'<li>{html.escape(t)}</li>' for t in c['limitations'])
        names = ', '.join(f'{i}: {nodes[i]["decoded_name"]}' for i in c['assembly_roots'])
        extra = ''
        if cid == 'C':
            extra = f'<details><summary>查看外侧板远端的解锁器02</summary><img alt="解锁器02" src="{png_uri(folder/"images/release_latch_02.png")}"></details>'
        elif cid == 'B':
            extra = f'<details><summary>查看整套自拍相机组件斜视图</summary><img alt="自拍相机组件斜视图" src="{png_uri(folder/"images/selfie_camera.png")}"></details>'
        cards.append(f'''<section class="card" id="{cid}" style="--accent:{COLORS[cid]}">
<h2>{cid} · {c['title']}</h2><p class="badge">{c['confidence']}</p>
<div class="detail"><div class="drawing">{detail_images[cid]}{extra}</div><div>
<h3>观察到的证据</h3><ul>{observed}</ul><h3>仍不能确认</h3><ul>{limits}</ul>
<p class="question">请确认：{html.escape(c['ask'])}</p>
<label>你的判断 <select data-id="{cid}"><option value="unconfirmed">暂不确定 / 待确认</option><option value="fixed">保持固定</option><option value="articulated">需要关节运动</option><option value="event_only">仅模拟解锁 / 分离事件</option></select></label>
<textarea data-notes="{cid}" placeholder="填写活动部件、固定部件、运动方式或图纸依据；不知道角度时留空。"></textarea>
<details><summary>源装配索引</summary><p class="mono">{html.escape(names)}</p><p>{len(c['inspection_instances'])} 个高亮叶级实例 / {len(c['visual_geoms'])} 个外观 geom；不代表已划分的刚体数。</p></details>
</div></div></section>''')
    a = findings['candidates'][0]['preliminary_axis']
    exclusions = '、'.join(html.escape(x['title']) for x in findings['exclusions'])
    axis_text = ', '.join(f'{x:.8f}' for x in a['point_model_m'])
    source = html.escape(manifest['source']['filename'])
    encoded_findings = json.dumps(findings, ensure_ascii=False).replace('</', r'<\/')
    body = f'''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>地面验证星 · 活动组件待确认</title><style>
*{{box-sizing:border-box}}body{{margin:0;background:#0c1623;color:#deebf9;font:16px/1.65 "Microsoft YaHei",sans-serif}}main{{max-width:1540px;margin:auto;padding:24px}}h1{{font-size:30px;margin:0 0 12px}}h2{{color:var(--accent);font-size:25px;margin:0}}h3{{font-size:18px;margin:15px 0 8px}}p{{margin:10px 0}}.muted,small{{color:#a7bdd4}}.notice{{border-left:4px solid #efba65;background:#1f2b3b;padding:14px 18px}}.overview > svg{{width:100%;height:auto}}.card{{margin:24px 0;border:1px solid #31445b;border-top:3px solid var(--accent);border-radius:12px;padding:20px}}.badge{{color:var(--accent)}}.detail{{display:grid;grid-template-columns:minmax(0,1fr) minmax(0,1fr);gap:25px}}.drawing > svg,.drawing img{{display:block;width:100%;height:auto;background:#1a2634;border-radius:8px}}li{{margin:8px 0}}ul{{padding-left:22px}}select,textarea,button{{font:inherit;border:1px solid #4b6582;border-radius:6px;background:#132237;color:#e7f1ff;padding:8px 12px}}select{{margin-left:8px;max-width:100%}}textarea{{display:block;width:100%;height:100px;margin-top:12px;resize:vertical}}button{{cursor:pointer;background:#1c5572;margin:12px 0}}.question{{padding:10px;border-left:3px solid var(--accent);background:#142438}}.mono{{font-family:Consolas,monospace;overflow-wrap:anywhere}}details{{margin:12px 0}}summary{{cursor:pointer;color:#a5c4e5}}a{{color:#7acaff}}.notes{{padding:20px;background:#142234;border-radius:10px}}@media(max-width:900px){{.detail{{grid-template-columns:1fr}}main{{padding:12px}}}}@media print{{body{{background:white;color:#111}}select,textarea,button{{display:none}}.card{{break-inside:avoid}}}}
</style></head><body><main><h1>地面验证星 · 疑似活动组件确认</h1>
<p class="muted">来源：{source}｜装配名已解码｜独立检查报告，不是可动 MJCF</p>
<div class="notice"><strong>优先确认 A 和 B。</strong> C 为解锁机构检查项，D 为分离接口检查项，不能都当成普通活动关节。所有角度范围仍为空；没有修改 STEP、现有 MJCF 或 SARM 平台配置。</div>
<div class="overview">{diagrams['overview']}</div>
<p>高亮色是检查分组，灰色只是未高亮的几何，并非已经证明固定。R 内侧板暂作固定参考；若实际也能运动，请指出它与星体之间的连接部位。</p>
{''.join(cards)}
<section class="notes"><h2 style="--accent:#91bbdf">识别方法与边界</h2>
<ul><li>读取已有装配树、实例位姿、网格映射；重新读取原始 STEP 的解析曲面。共检查 74 个形状定义，CAD 面数与转换清单一致。</li>
<li>A 的候选轴点（当前 MJCF 坐标 / 米）为 <code>[{axis_text}]</code>，方向约 <code>[0, 0, 1]</code>。轴线方向正负是数学等价表示，不代表已经确定展开方向。</li>
<li>轴上点不是质心，也不是机构零位。当前 CAD 姿态只是显示参考；不能据此认定收拢角或展开角。</li>
<li>B1–B3 标注来自解析圆柱轴线，不是由 AI 绘制的关节；螺孔和销轴也会产生圆柱面，因此还需要实际连接关系确认。</li>
<li>本次未列为活动候选：{exclusions}。尤其没有把顶部圆柱设备自动当成反作用轮，也没有把星敏感器当成云台。</li>
<li>图片直接使用 MuJoCo 编译后的真实网格和位姿，由已有 CPU 深度缓冲渲染器生成；标注由 SVG 投影添加。不是原生 OpenGL / UE 截图。</li>
<li>证据文件：<a href="candidates.json">candidates.json</a>、<a href="assembly_inventory.json">assembly_inventory.json</a>、<a href="cad_surface_evidence.json">cad_surface_evidence.json</a>；技术方法见 <a href="https://dev.opencascade.org/doc/refman/html/class_b_rep_adaptor___surface.html">Open CASCADE 解析曲面 API</a>。</li></ul>
<p><strong>下一步：</strong>确认活动组件后，再划分运动刚体与关节。此处没有套用任何假定的质量、角度或驱动参数。</p>
<button id="export" type="button">导出我的确认意见（JSON）</button><p class="muted">表单不会自动保存。导出只生成你的反馈文件，不会修改模型；也可以直接按 A / B / C / D 在聊天中回复。</p></section>
<script id="review-data" type="application/json">{encoded_findings}</script>
<script>document.getElementById('export').addEventListener('click',()=>{{const original=JSON.parse(document.getElementById('review-data').textContent);const review={{schema:'satellite-component-user-review/1',source_sha256:original.source_sha256,created_at:new Date().toISOString(),decisions:[...document.querySelectorAll('select[data-id]')].map(s=>({{id:s.dataset.id,decision:s.value,notes:document.querySelector('textarea[data-notes="'+s.dataset.id+'"]').value}}))}};const url=URL.createObjectURL(new Blob([JSON.stringify(review,null,2)],{{type:'application/json;charset=utf-8'}}));const a=document.createElement('a');a.href=url;a.download='satellite_component_confirmation.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}});</script>
</main></body></html>'''
    (folder/'review.html').write_text(body, encoding='utf-8')
    lines = ['# 地面验证星：疑似活动组件确认', '', '**只完成证据分析和标注，没有添加关节或修改原模型。**', '',
             '- 打开 `review.html` 查看总图、局部图、依据及待确认项（离线可用）。',
             '- `overview.svg` 为可放大的标注总图；浏览器可直接打开。`overview.png` 是该 SVG 的浏览器渲染快照。',
             '- `candidates.json` 为候选记录，所有 `range_deg` 仍为 `null`。',
             '- `assembly_inventory.json` 保留解码名称、源实例编号、geom 映射及边界。',
             '- `cad_surface_evidence.json` 是原始 STEP 的解析曲面证据，不是自动识别出的关节列表。', '',
             '| 编号 | 组件 | 初步判断 |', '|---|---|---|']
    lines += [f"| {c['id']} | {c['title']} | {c['confidence']} |" for c in findings['candidates']]
    lines += ['', '## 注意', '', '高亮整组不表示所有成员都应活动；R 内侧板暂作固定参考。',
              'A 的上下铰链共轴，但机械限位、运动方向及驱动均未知；不从 WXJL180 型号推断 180° 限位。',
              'B1–B3 是圆柱轴线特征，不能直接等同于三个独立关节。C/D 可能只需事件建模。',
              '本次检查不提供真实质量/惯量，不使用原自由预览的人为 1 kg 参数。', '', '## 复现（外层工作区 PowerShell）', '', '```powershell',
              r'.\run\step-converter-venv\Scripts\python.exe -X utf8 .\space_sim_server\tools\extract_step_surface_evidence.py .\model\ground_validation_satellite',
              r'.\run\step-converter-venv\Scripts\python.exe -X utf8 .\space_sim_server\tools\review_satellite_components.py .\model\ground_validation_satellite',
              '```', '', '报告使用已有 CPU 几何渲染与原生 SVG/HTML 标注；无 AI 图片、无 CAD 云端上传、无额外二进制下载。',
              '表单需点击导出按钮才会保存确认意见，不会直接修改模型。']
    (folder/'README.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('model_folder', type=Path)
    parser.add_argument('--reuse-images', action='store_true', help='Reuse existing review images and cameras')
    args = parser.parse_args()
    folder = args.model_folder.resolve()
    manifest_path = folder/'conversion_manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    source = folder.parent/manifest['source']['filename']
    if manifest['source']['sha256'] != SOURCE_SHA256 or hashlib.sha256(source.read_bytes()).hexdigest() != SOURCE_SHA256:
        raise ValueError('Curated candidate mappings apply only to the inspected source STEP')
    protected = [source, manifest_path, folder/'ground_validation_satellite.xml', folder/'satellite_free_preview.xml',
                 folder.parent/'SARM/platform/sarm_platform.xml']
    hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in protected if p.is_file()}
    out = folder/'articulation_review'
    evidence = json.loads((out/'cad_surface_evidence.json').read_text(encoding='utf-8'))
    if evidence['source_sha256'] != SOURCE_SHA256:
        raise ValueError('Mismatched analytic evidence')
    if any(evidence['parts'][p['id']]['cad_faces'] != p['cad_faces'] for p in manifest['parts']):
        raise ValueError('Analytic evidence face counts changed')
    nodes = build_inventory(manifest)
    groups, findings = make_findings(manifest, nodes, evidence)
    if args.reuse_images:
        cameras = json.loads((out/'view_cameras.json').read_text(encoding='utf-8'))
    else:
        geometry = gather(folder/'ground_validation_satellite.xml', manifest, groups)
        cameras = {}
        views = [('rear', None, -75, 15), ('front', None, 110, 18),
                 ('hinge_pair', [f'instance_{i:04}' for i in [186,187,189,190,196,197,208,209,211,212]], -65, 15),
                 ('selfie_camera', groups['B'], 125, 15), ('selfie_camera_side', groups['B'], 0, 0),
                 ('release_latch', ['instance_0194','instance_0195'], -65, 25),
                 ('release_latch_02', ['instance_0214'], -65, 25),
                 ('separation_interface', groups['D'], 100, 25)]
        for name, instances, az, el in views:
            cameras[name] = render(select_geometry(geometry, instances), out/'images'/(name+'.png'), az, el)
        (out/'view_cameras.json').write_text(json.dumps(cameras, indent=2), encoding='utf-8')
    diagrams = write_diagrams(out, nodes, findings, cameras)
    write_report(out, manifest, nodes, findings, diagrams)
    (out/'assembly_inventory.json').write_text(json.dumps(list(nodes.values()), ensure_ascii=False, indent=2), encoding='utf-8')
    (out/'candidates.json').write_text(json.dumps(findings, ensure_ascii=False, indent=2), encoding='utf-8')
    unchanged = all(hashlib.sha256(Path(p).read_bytes()).hexdigest() == h for p, h in hashes.items())
    if not unchanged:
        raise ValueError('A protected input changed during review generation')
    verification = dict(protected_inputs_sha256=hashes, protected_inputs_unchanged=True,
                        annotation_method='native SVG projection of compiled geometry', ai_generated=False,
                        all_candidates_unconfirmed=True, all_ranges_null=True,
                        part_face_counts_match=True, candidate_A_axis_separation_m=findings['candidates'][0]['preliminary_axis']['max_line_offset_m'],
                        render_views=list(cameras))
    (out/'verification.json').write_text(json.dumps(verification, ensure_ascii=False, indent=2), encoding='utf-8')
    print('Review saved:', out/'review.html', flush=True)


if __name__ == '__main__':
    main()
