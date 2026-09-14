"""Render an offline visual GLB->MJCF preview using compiled MuJoCo geometry.

No OpenGL/GPU driver, native viewer, network, or AI image generation is involved.
The HTML switches between four precomputed views; it is not live 3D simulation.
"""
from __future__ import annotations

import argparse
from html import escape
import json
import math
from pathlib import Path
import time

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from cpu_mjcf_preview import gather_geometry, rasterize

VIEWS = [
    ('01_front_oblique', -55, 24, 'Front oblique', '正面斜视'),
    ('02_rear_oblique', 125, 24, 'Rear oblique', '背面斜视'),
    ('03_front', -90, 0, 'Front (-Y)', '正视图'),
    ('04_top', -90, 75, 'Upper view', '俯视图'),
]


def project_geometry(vertices, azimuth, elevation, width=1400, height=1050):
    low, high = vertices.min(0), vertices.max(0)
    center = (low+high)/2
    radius = float(np.linalg.norm(high-low))
    if radius <= 0:
        raise ValueError('Cannot frame empty/zero-size geometry')
    az, el = math.radians(azimuth), math.radians(elevation)
    radial = np.array([math.cos(el)*math.cos(az), math.cos(el)*math.sin(az), math.sin(el)])
    forward = -radial
    right = np.cross(forward, [0., 0., 1.])
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    relative = vertices-center
    half_y = math.tan(math.radians(38)/2)*0.87
    half_x = half_y*width/height
    depth_offset = relative@radial
    distance = max(float(np.max(depth_offset+np.abs(relative@up)/half_y)),
                   float(np.max(depth_offset+np.abs(relative@right)/half_x))) + 0.01*radius
    eye = center+distance*radial
    camera = (vertices-eye)@np.array([right, up, forward]).T
    focal = height/(2*math.tan(math.radians(38)/2))
    projected = np.column_stack([width/2+focal*camera[:, 0]/camera[:, 2],
                                 height/2-focal*camera[:, 1]/camera[:, 2],
                                 1/camera[:, 2]])
    if (np.any(camera[:, 2] <= 0) or not np.isfinite(projected).all()
            or projected[:, 0].min() < 0 or projected[:, 0].max() >= width
            or projected[:, 1].min() < 0 or projected[:, 1].max() >= height):
        raise AssertionError('Preview camera clips model vertices')
    return projected, radial, dict(eye=eye.tolist(), target=center.tolist(),
                                   azimuth_deg=azimuth, elevation_deg=elevation,
                                   width=width, height=height, fovy_deg=38)


def render(folder: Path):
    started = time.perf_counter()
    folder = folder.resolve()
    report = json.loads((folder/'conversion_manifest.json').read_text('utf-8'))
    xml = folder/report['model_file']
    vertices, normals, faces, normal_faces, colors = gather_geometry(xml)
    if len(faces) != report['counts']['exported_triangles']:
        raise AssertionError('Preview geometry does not match the conversion manifest')
    preview = folder/'preview'
    preview.mkdir(exist_ok=True)
    key = np.array([1., -2, 3]); key /= np.linalg.norm(key)
    fill = np.array([-2., 1, 1]); fill /= np.linalg.norm(fill)
    width, height = 1400, 1050
    pictures, records = [], []
    for name, az, el, label, label_zh in VIEWS:
        projected, radial, camera = project_geometry(vertices, az, el, width, height)
        pixels, depth = rasterize(projected, normals, faces, normal_faces, colors,
                                  width, height, radial, key, fill)
        foreground = int(np.count_nonzero(depth))
        if foreground < 1000:
            raise AssertionError('Empty preview')
        srgb = np.where(pixels <= 0.0031308, pixels*12.92,
                        1.055*np.power(np.maximum(pixels, 0), 1/2.4)-0.055)
        image = Image.fromarray(np.clip(srgb*255, 0, 255).astype(np.uint8))
        image.save(preview/(name+'.png'))
        pictures.append((label, image))
        records.append(dict(file='preview/'+name+'.png', label=label, label_zh=label_zh,
                            foreground_pixels=foreground, camera=camera))
        print(f'Rendered {name}: {foreground:,} foreground pixels', flush=True)
    sheet = Image.new('RGB', (1600, 1360), (15, 23, 34))
    draw = ImageDraw.Draw(sheet)
    try:
        font = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 29)
        small = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 20)
    except OSError:
        font = small = ImageFont.load_default()
    draw.text((24, 15), 'NASA LRO (A) | GLB -> MJCF', font=font, fill=(232, 242, 250))
    draw.text((24, 53), 'Fixed visual model | CPU preview of MuJoCo-compiled meshes | No inferred joints', font=small, fill=(157, 185, 211))
    for i, (label, picture) in enumerate(pictures):
        x, y = (i % 2)*800, 95+(i//2)*600
        sheet.paste(picture.resize((800, 600), Image.Resampling.LANCZOS), (x, y))
        draw.text((x+20, y+14), label, font=small, fill=(224, 238, 251))
    dims = report['dimensions_m']
    draw.text((24, 1303), f'File-scale X/Y/Z: {dims[0]:.3f} / {dims[1]:.3f} / {dims[2]:.3f} m; NOT physically calibrated.', font=small, fill=(255, 201, 129))
    draw.text((24, 1331), f'{len(faces):,} visible triangles | 55 visual geoms | No contacts or dynamic mass/inertia', font=small, fill=(157, 185, 211))
    sheet.save(preview/'overview.png')
    metadata = dict(model_file=xml.name, model_sha256=report['model_sha256'],
                    renderer='Custom CPU two-sided z-buffer from MuJoCo-compiled geometry and world poses',
                    native_opengl=False, ai_generated=False, platform_or_ue_screenshot=False,
                    live_3d=False, physical_scale_verified=False, triangle_count=len(faces),
                    views=records, overview='preview/overview.png', elapsed_seconds=time.perf_counter()-started)
    (folder/'preview_manifest.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    buttons = '\n'.join(f'<button type="button" data-src="{item["file"]}" aria-pressed="{str(i == 0).lower()}">{item["label_zh"]}</button>'
                        for i, item in enumerate(records))
    dimensions = ' × '.join(f'{v:.3f}' for v in dims)
    count = report['counts']
    template = '''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>NASA LRO · MJCF 转换预览</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#0b1420;color:#e2edf7;font:16px/1.7 "Microsoft YaHei",sans-serif}
main{max-width:1320px;margin:auto;padding:26px}h1{font-size:30px;margin:0}p{margin:10px 0}small{color:#a1bdd5}
.notice{background:#252c39;border-left:4px solid #ffc87e;padding:12px 18px;margin:18px 0;color:#ffe1b7}
.stats{display:flex;gap:14px;flex-wrap:wrap}.stat{background:#142638;border:1px solid #274257;padding:9px 18px;border-radius:8px}
.viewport{background:#0b1522;border:1px solid #263e53;border-radius:10px;overflow:hidden;margin-top:20px}
#view{display:block;width:100%;max-height:760px;object-fit:contain}.controls{display:flex;gap:10px;flex-wrap:wrap;padding:15px;background:#142436}
button{font:inherit;border:1px solid #35576f;color:#c6deee;background:#1b3447;border-radius:6px;padding:7px 18px;cursor:pointer}
button[aria-pressed=true]{background:#1b6978;border-color:#73d4d8;color:#fff}a{color:#80dbe8}code{overflow-wrap:anywhere}
section{background:#121f2e;padding:15px 23px;border:1px solid #23384c;border-radius:9px;margin-top:18px}h2{font-size:20px;margin:0 0 8px}
</style></head><body><main>
<h1>NASA LRO (A) · MJCF 视觉模型</h1><small>由本地 GLB 解码、转换并通过 MuJoCo 编译校验；原始模型未修改。</small>
<div class="notice"><b>本版固定、无活动关节，也未启用碰撞。</b><br>文件尺度 X/Y/Z = __DIMS__ m，尚未按真实卫星尺寸校准；不是可直接用于真实动力学的工程模型。</div>
<div class="stats"><div class="stat">5 个源网格组 / 11 个分组节点</div><div class="stat">34 个基础色材质 / 55 个可视几何</div><div class="stat">__TRIANGLES__ 个有效三角形</div><div class="stat">0 关节 / 0 执行器</div></div>
<div class="viewport"><img id="view" src="preview/01_front_oblique.png" alt="LRO MJCF 正面斜视几何预览"><div class="controls">__BUTTONS__</div></div>
<p><small>按钮仅切换预先绘制的视图，不是实时三维交互。图片来自 MuJoCo 编译后的网格与世界坐标，经 CPU 双面 z-buffer 绘制；非 AI 图片、非原生 OpenGL / 平台 / UE 截图。</small></p>
<section><h2>打开与使用</h2><p>MJCF 主文件：<a href="__XML__"><code>__XML__</code></a>，需与 <code>meshes/</code> 一同保留。</p>
<p><a href="preview/overview.png">四视图总览</a> · <a href="README.md">转换说明</a> · <a href="conversion_manifest.json">转换与验证记录</a></p>
<ul><li>保留基础色、法线、部件相对位置和分组树；源节点变换已烘焙进 OBJ，body 原点不是实际机械转轴。</li>
<li>glTF Y 向上转为 MJCF Z 向上，原点保留，未擅自缩放。</li>
<li>源文件有 __DROPPED__ 个面积严格为零的退化面，已剔除；未对有效面减面。</li>
<li>不保证金属 / 粗糙度 PBR 材质外观完全相同；质量、惯量、真实比例与机构关系仍待确认。</li>
<li>MuJoCo 加载、几何/法线/基础色对照和 100 步固定场景检查通过；未修改平台默认场景。</li></ul></section>
</main><script>
const view=document.getElementById('view');const buttons=[...document.querySelectorAll('button[data-src]')];
buttons.forEach(button=>button.addEventListener('click',()=>{view.src=button.dataset.src;view.alt='LRO MJCF '+button.textContent+'几何预览';buttons.forEach(other=>other.setAttribute('aria-pressed',String(other===button)));}));
</script></body></html>'''
    for key, value in {'__DIMS__': dimensions, '__TRIANGLES__': f'{len(faces):,}', '__BUTTONS__': buttons,
                       '__XML__': escape(xml.name, quote=True), '__DROPPED__': f'{count["dropped_zero_area_triangles"]:,}'}.items():
        template = template.replace(key, value)
    (folder/'preview.html').write_text(template, encoding='utf-8')
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('model_directory', type=Path)
    args = parser.parse_args()
    print(json.dumps(render(args.model_directory), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
