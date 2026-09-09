"""CPU forward-kinematics previews for the single-panel hinge (no OpenGL).

All frames use one camera and compiled MuJoCo body poses. Demo angles are not
physical stops. The offline page plays precomputed poses, not a live simulator.
"""
import argparse
import base64
import hashlib
import json
import math
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image
from cpu_mjcf_preview import rasterize
from build_satellite_wing_joint import OUTPUT_XML, OUTER_BODY, JOINT


def geometry(model, data):
    out = {k: [] for k in ('vertices','normals','faces','normal_faces','colors','moving_v','moving_n')}
    outer = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, OUTER_BODY)
    vo = no = 0
    for gid in np.where(model.geom_group == 2)[0]:
        mid = int(model.geom_dataid[gid])
        va, nv = int(model.mesh_vertadr[mid]), int(model.mesh_vertnum[mid])
        na, nn = int(model.mesh_normaladr[mid]), int(model.mesh_normalnum[mid])
        fa, nf = int(model.mesh_faceadr[mid]), int(model.mesh_facenum[mid])
        rot = data.geom_xmat[gid].reshape(3, 3)
        out['vertices'].append(np.asarray(model.mesh_vert[va:va+nv], float)@rot.T+data.geom_xpos[gid])
        out['normals'].append(np.asarray(model.mesh_normal[na:na+nn], float)@rot.T)
        out['faces'].append(np.asarray(model.mesh_face[fa:fa+nf], np.int32)+vo)
        out['normal_faces'].append(np.asarray(model.mesh_facenormal[fa:fa+nf], np.int32)+no)
        moving = model.geom_bodyid[gid] == outer
        # Display emphasis only; the generated MJCF retains the original CAD RGBA.
        color = np.array([.025, .55, .42]) if moving else model.geom_rgba[gid, :3]
        out['colors'].append(np.tile(color, (nf, 1)))
        out['moving_v'].append(np.full(nv, moving)); out['moving_n'].append(np.full(nn, moving))
        vo += nv; no += nn
    result = {k: np.concatenate(v) for k, v in out.items()}
    result['colors'] = result['colors'].astype(np.float32)
    result['outer_pos_0'] = data.xpos[outer].copy()
    result['outer_rot_0'] = data.xmat[outer].reshape(3, 3).copy()
    result['outer_id'] = outer
    return result


def pose(template, model, data, degrees):
    data.qpos[0] = math.radians(degrees)
    mujoco.mj_forward(model, data)
    outer = template['outer_id']
    rot = data.xmat[outer].reshape(3, 3)@template['outer_rot_0'].T
    vertices, normals = template['vertices'].copy(), template['normals'].copy()
    vi, ni = template['moving_v'], template['moving_n']
    vertices[vi] = (vertices[vi]-template['outer_pos_0'])@rot.T+data.xpos[outer]
    normals[ni] = normals[ni]@rot.T
    return vertices, normals.astype(np.float32)


def fit_camera(lo, hi, width=1100, height=850, azimuth=-40, elevation=28):
    corners = np.array([[x,y,z] for x in (lo[0],hi[0]) for y in (lo[1],hi[1]) for z in (lo[2],hi[2])])
    center = (np.asarray(lo)+hi)/2
    az, el = math.radians(azimuth), math.radians(elevation)
    radial = np.array([math.cos(el)*math.cos(az), math.cos(el)*math.sin(az), math.sin(el)])
    forward = -radial
    right = np.cross(forward, [0,0,1]); right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    relative = corners-center
    half_y = math.tan(math.radians(38)/2)*.85
    half_x = half_y*width/height
    distance = max(float(np.max(relative@radial+abs(relative@up)/half_y)),
                   float(np.max(relative@radial+abs(relative@right)/half_x))) + .02*float(np.linalg.norm(hi-lo))
    return dict(target=center.tolist(), eye=(center+distance*radial).tolist(), basis=np.array([right,up,forward]).tolist(),
                radial=radial.tolist(), focal=height/(2*math.tan(math.radians(38)/2)),
                width=width, height=height, azimuth_deg=azimuth, elevation_deg=elevation)


def render(template, vertices, normals, camera):
    width, height = camera['width'], camera['height']
    cam = (vertices-camera['eye'])@np.asarray(camera['basis']).T
    projected = np.column_stack((width/2+camera['focal']*cam[:,0]/cam[:,2],
                                 height/2-camera['focal']*cam[:,1]/cam[:,2], 1/cam[:,2]))
    if np.any(cam[:,2] <= 0) or np.any(projected[:,:2] < 0) or np.any(projected[:,0] >= width) or np.any(projected[:,1] >= height):
        raise ValueError('Camera clips the posed model')
    key = np.array([1.,-2,3]); key /= np.linalg.norm(key)
    fill = np.array([-2.,1,1]); fill /= np.linalg.norm(fill)
    pixels, depth = rasterize(projected,normals,template['faces'],template['normal_faces'],template['colors'],
                              width,height,np.asarray(camera['radial']),key,fill)
    if np.count_nonzero(depth) < 1000:
        raise ValueError('Empty preview')
    srgb = np.where(pixels <= .0031308,pixels*12.92,1.055*np.maximum(pixels,0)**(1/2.4)-.055)
    return Image.fromarray(np.uint8(np.clip(srgb*255,0,255)))


def uri(path):
    return 'data:image/png;base64,'+base64.b64encode(path.read_bytes()).decode('ascii')


def create_preview(folder):
    folder = Path(folder).resolve()
    out = folder/'wing_articulation'; images = out/'preview'; images.mkdir(exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(folder/OUTPUT_XML))
    assert (model.nq,model.nv,model.nu)==(1,1,0)
    data = mujoco.MjData(model); mujoco.mj_forward(model,data)
    template = geometry(model,data)
    angles = list(range(0,91,5))
    lo, hi = np.full(3,np.inf), np.full(3,-np.inf)
    for angle in angles:
        v,_ = pose(template,model,data,angle)
        lo = np.minimum(lo,v.min(0)); hi = np.maximum(hi,v.max(0))
    camera = fit_camera(lo,hi)
    # Fit the union of the actual posed vertices, not impossible AABB corners.
    # Static geometry contributes once; only the small moving subset is swept.
    radial = np.asarray(camera['radial']); basis = np.asarray(camera['basis'])
    half_y = math.tan(math.radians(38)/2)*.85
    half_x = half_y*camera['width']/camera['height']
    directions = np.array([radial+basis[1]/half_y, radial-basis[1]/half_y,
                           radial+basis[0]/half_x, radial-basis[0]/half_x])
    moving = template['moving_v']
    extrema = (template['vertices'][~moving]@directions.T).max(0)
    for angle in angles:
        data.qpos[0] = math.radians(angle); mujoco.mj_forward(model,data)
        rot = data.xmat[template['outer_id']].reshape(3,3)@template['outer_rot_0'].T
        points = (template['vertices'][moving]-template['outer_pos_0'])@rot.T+data.xpos[template['outer_id']]
        extrema = np.maximum(extrema,(points@directions.T).max(0))
    target = np.asarray(camera['target'])
    distance = float(np.max(extrema-target@directions.T))+.01*float(np.linalg.norm(hi-lo))
    camera['eye'] = (target+distance*radial).tolist()
    camera['fit_method'] = 'exact static and posed moving vertices; fixed camera'
    frames = []; files = []
    for angle in angles:
        v,n = pose(template,model,data,angle)
        image = render(template,v,n,camera)
        dest = images/f'pose_{angle:03}.png'; image.save(dest)
        frames.append(image.resize((880,680),Image.Resampling.LANCZOS))
        files.append(dict(angle_deg=angle,file='preview/'+dest.name,uri=uri(dest)))
        print('Rendered demonstration pose',angle,flush=True)
    sequence = frames+frames[-2:0:-1]
    sequence[0].save(images/'motion.gif',save_all=True,append_images=sequence[1:],duration=110,loop=0,optimize=False)
    pictures=[]
    for x,angle in [(0,0),(533,45),(1066,90)]:
        picture=uri(images/f'pose_{angle:03}.png')
        label='0° · 原 CAD 姿态' if angle==0 else f'+{angle}° · 演示折转姿态'
        pictures.append(f'<text x="{x+24}" y="135" font-size="22" fill="#dcecf9">{label}</text><image x="{x}" y="155" width="533" height="412" href="{picture}"/>')
    svg='''<svg xmlns="http://www.w3.org/2000/svg" width="1600" height="660" viewBox="0 0 1600 660">
<style>text{font-family:"Microsoft YaHei",sans-serif}</style><rect width="1600" height="660" fill="#0c1623"/>
<text x="30" y="46" fill="#eef6ff" font-size="30" font-weight="bold">地面验证星｜外侧板单关节运动预览</text>
<text x="30" y="83" fill="#a8c3d9" font-size="20">绿色：随外侧板运动的外观几何；内侧板和星体固定。三幅图采用同一相机。</text>
'''+''.join(pictures)+'''
<text x="30" y="602" fill="#ffce87" font-size="21">0°–90° 仅为演示角度，不是真实机械限位。质量 / 惯量为测试占位值；未启用碰撞。</text>
<text x="30" y="637" fill="#a8c3d9" font-size="18">依据 MuJoCo 前向运动学生成的 CPU 几何图；不是 AI 生成图，也不是平台 / UE 运行截图。</text></svg>'''
    (out/'motion_overview.svg').write_text(svg,encoding='utf-8')
    frame_json = json.dumps(files,ensure_ascii=False).replace('</',r'<\/')
    page='''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>外侧板单关节预览</title>
<style>*{box-sizing:border-box}body{margin:0;background:#0c1623;color:#e3edf7;font:17px/1.7 "Microsoft YaHei",sans-serif}main{max-width:1250px;margin:auto;padding:24px}h1{margin:0;font-size:28px}.notice{background:#253044;border-left:4px solid #ffc876;padding:12px 18px;margin:14px 0}img{display:block;width:100%;max-height:710px;object-fit:contain;background:#1b2735}input{width:65%;vertical-align:middle}button{padding:8px 18px;color:#e8f8ff;background:#1b5864;border:1px solid #40899b;border-radius:6px;font:inherit;cursor:pointer}label{margin-left:18px}a{color:#7fd4ff}code{overflow-wrap:anywhere}.controls{padding:16px;background:#142638}small{color:#a9bdd1}li{margin:6px 0}</style></head>
<body><main><h1>外侧板单关节 · 离线运动预览</h1>
<div class="notice"><b>已在 MJCF 中建立一个旋转关节，但本页只播放预先计算的姿态。</b><br>演示滑块 0°–90° 不是关节限位，也不是已验证的安全运动范围。</div>
<img id="frame" alt="外侧板运动姿态"><div class="controls"><button id="play" type="button">播放 / 暂停</button><label><span id="angle">0°</span><input id="slider" type="range" min="0" max="18" value="0" step="1"></label></div>
<p>绿色是随动几何的预览高亮；MJCF 文件仍保留原 CAD 颜色。相机固定，只有外侧板组件绕板间轴线旋转。</p>
<ul><li>内侧板与星体固定；上下铰链对应同一个关节 <code>outer_panel_hinge</code>。</li>
<li>外侧板上的解锁器 02 随安装板移动，但没有模拟内部解锁运动；01/03、自拍相机、分离接口不新增自由度。</li>
<li>原始铰链为合并实体，本版将其表面沿测得轴线作固定/随动显示分区；没有完整还原铰链内部叶片、销轴和接触。</li>
<li>活动板使用人为 1 kg 质量与外接盒惯量，仅保证运动模型可加载；不可作为真实卫星动力学数据。</li>
<li>碰撞关闭、无驱动器、无实际限位；不等于已完成平台集成或接触仿真。</li></ul>
<p>模型：<a href="../ground_validation_satellite_articulated.xml">ground_validation_satellite_articulated.xml</a> · <a href="motion_overview.svg">姿态对比总图</a> · <a href="preview/motion.gif">循环动图</a></p>
<small>本地 MuJoCo 前向运动学 + CPU 深度缓冲渲染。无 OpenGL 要求、无联网资源、非 AI 图片。0° 是导入 CAD 的显示姿态，不是标定零位。</small>
<script type="application/json" id="frames">FRAME_DATA</script><script>
const frames=JSON.parse(document.getElementById('frames').textContent);const slider=document.getElementById('slider');let playing=false,direction=1;
function show(){const f=frames[Number(slider.value)];document.getElementById('frame').src=f.uri;document.getElementById('angle').textContent=f.angle_deg+'°';}slider.addEventListener('input',()=>{playing=false;show();});document.getElementById('play').addEventListener('click',()=>playing=!playing);
setInterval(()=>{if(!playing)return;let i=Number(slider.value)+direction;if(i>=frames.length){direction=-1;i=frames.length-2;}if(i<0){direction=1;i=1;}slider.value=i;show();},110);show();
</script></main></body></html>'''.replace('FRAME_DATA',frame_json)
    (out/'motion_preview.html').write_text(page,encoding='utf-8')
    metadata=dict(model_sha256=hashlib.sha256((folder/OUTPUT_XML).read_bytes()).hexdigest(),
                  renderer='CPU geometry from compiled MuJoCo and mj_forward body poses',native_opengl=False,ai_generated=False,
                  angles_deg=angles,demonstration_angles_only=True,camera_is_fixed=True,camera=camera,
                  xml_colors_changed=False,preview_moving_geometry_highlight=True,contact_or_limit_validation_done=False,
                  frames=[{k:v for k,v in f.items() if k!='uri'} for f in files])
    (out/'preview_manifest.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')
    print('Preview:',out/'motion_preview.html',flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('model_folder',type=Path)
    create_preview(parser.parse_args().model_folder)
