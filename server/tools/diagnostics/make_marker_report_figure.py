"""Annotate an unchanged original-stereo UE image with the installed marker IDs.

Model projections are used only for report labels, never as detector evidence.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation


SERVER = Path(__file__).resolve().parents[2]
OUTPUT = SERVER/'run/marker_report_20261029'
GROUPS = {
    .05: ('#29b6f6', '50 mm｜4块｜ID 0–3', '外围大靶标，白边总边长66.67 mm'),
    .02: ('#ffad32', '20 mm｜4块｜ID 4–7', '接口四周，白边总边长26.67 mm'),
    .01: ('#60d577', '10 mm｜9块｜ID 8、9、15、17–22', '接口邻近，白边总边长13.33 mm'),
    .006: ('#b891ff', '6 mm｜2块｜ID 14、16', '局部补充，白边总边长8 mm'),
    .0045: ('#ff7aaf', '4.5 mm｜4块｜ID 10–13', '接口较高表面，白边总边长6 mm'),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True,
                        help='Historical frame-112 actual_images directory, including calibration and render_frames')
    parser.add_argument('--output', type=Path, default=OUTPUT)
    parser.add_argument('--font', type=Path, default=Path('C:/Windows/Fonts/msyh.ttc'))
    args = parser.parse_args()
    output_directory = args.output.resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    source = args.source/'images/112_left.jpg'
    config = json.loads((args.source/'calibration.json').read_text(encoding='utf-8-sig'))
    with (args.source/'render_frames.jsonl').open(encoding='utf-8') as stream:
        frame = next(f for line in stream if (f := json.loads(line))['frame_id'] == '112')
    objects = {o['object_id']: o for o in frame['objects']}

    def rotation(name):
        return Rotation.from_quat(objects[name]['orientation_wxyz'], scalar_first=True).as_matrix()

    target_r, grip_r = rotation('teleop/capture_target'), rotation('teleop/gripper')
    target_p = np.array(objects['teleop/capture_target']['position_m'])
    grip_p = np.array(objects['teleop/gripper']['position_m'])
    mount = config['parent_from_optical']['left']
    camera_r = grip_r@np.array(mount['R_parent_from_optical'])
    camera_p = grip_p+grip_r@np.array(mount['position_parent_m'])
    k = np.array(config['calibration']['K_left'])

    def project(points):
        optical = (np.asarray(points)@target_r.T+target_p-camera_p)@camera_r
        assert np.all(optical[:, 2] > 0)
        pixels = optical@k.T
        return pixels[:, :2]/pixels[:, 2:3]

    raw = Image.open(source).convert('RGB')
    shutil.copy2(source, output_directory/'original_left_112.jpg')
    canvas = Image.new('RGB', (2400, 1740), '#eef2f6')
    draw = ImageDraw.Draw(canvas)
    font_path = str(args.font)
    font = lambda size: ImageFont.truetype(font_path, size)
    draw.text((55, 28), '23块靶标：仿真截图与尺寸对应', font=font(49), fill='#182b42')
    draw.text((58, 99), '原100°末端双目 · 左眼原始图像 · 原仿真灯光 · 仅裁剪放大并添加标注',
              font=font(27), fill='#526679')
    crop = (800, 685, 1270, 1450)
    scale = 1.9
    image = raw.crop(crop).resize((round((crop[2]-crop[0])*scale), round((crop[3]-crop[1])*scale)), Image.Resampling.LANCZOS)
    origin = np.array([55., 165.])
    canvas.paste(image, tuple(origin.astype(int)))
    draw = ImageDraw.Draw(canvas)

    def mapped(uv):
        return tuple(origin+(np.asarray(uv)-crop[:2])*scale)

    offsets = {0:(52,-50),1:(-100,-50),2:(52,35),3:(-100,35),4:(50,-5),5:(-80,10),
        6:(5,-38),7:(5,35),8:(-88,0),9:(-88,-15),10:(42,12),11:(42,-10),
        12:(-12,40),13:(-12,-25),14:(-88,7),15:(-70,30),16:(-88,-5),17:(-88,-5),
        18:(-12,50),19:(-5,-28),20:(-62,-20),21:(5,25),22:(12,-20)}
    records = []
    for marker in config['marker_rig']['markers']:
        identifier, side = marker['id'], marker['size_m']
        corners = project(config['layout']['markers'][str(identifier)])
        center = project([marker['center_m']])[0]
        color = GROUPS[side][0]
        mapped_corners = [mapped(p) for p in corners]
        draw.line(mapped_corners+[mapped_corners[0]], fill=color, width=3)
        anchor = mapped(center+offsets[identifier])
        draw.line([mapped(center), anchor], fill=color, width=2)
        text = 'ID '+str(identifier)
        bounds = draw.textbbox((0, 0), text, font=font(23))
        width = bounds[2]-bounds[0]+14
        draw.rounded_rectangle([anchor[0]-width/2, anchor[1]-18, anchor[0]+width/2, anchor[1]+18],
                               radius=6, fill='#102030', outline=color, width=2)
        draw.text((anchor[0]-width/2+7, anchor[1]-16), text, font=font(23), fill=color)
        records.append(dict(id=identifier, black_side_mm=side*1000, projected_corners_px=corners.tolist(),
                            projected_center_px=center.tolist()))

    draw.text((1040, 167), '与汇报表格逐项对应', font=font(36), fill='#182b42')
    for n, (color, title, purpose) in enumerate(GROUPS.values()):
        y = 237+n*100
        draw.rounded_rectangle([1040, y+4, 1060, y+63], radius=5, fill=color)
        draw.text((1081, y), title, font=font(31), fill='#21364b')
        draw.text((1081, y+46), purpose, font=font(25), fill='#586c80')

    def detail(box, location, size, title, ids):
        detail_image = raw.crop(box).resize(size, Image.Resampling.LANCZOS)
        x, y = location
        draw.text((x, y-48), title, font=font(29), fill='#21364b')
        canvas.paste(detail_image, location)
        for record in records:
            if record['id'] not in ids:
                continue
            corners = np.array(record['projected_corners_px'])
            points = ((corners-np.array(box[:2]))*np.array(size)/(np.array(box[2:])-box[:2]))+location
            color = GROUPS[record['black_side_mm']/1000][0]
            draw.line([tuple(p) for p in np.r_[points, points[:1]]], fill=color, width=3)
            label = points.mean(axis=0)+[3, -27]
            draw.text(tuple(label), str(record['id']), font=font(22), fill=color, stroke_width=1, stroke_fill='#101a24')

    detail((895, 870, 1055, 1000), (1040, 818), (640, 520), '局部放大：10 mm、6 mm与20 mm', {6,16,17,19,20,22})
    detail((956, 992, 1144, 1101), (1710, 818), (630, 365), '局部放大：4.5 mm，ID 10–13', {10,11,12,13})
    draw.text((1040, 1380), '颜色表示黑框尺寸；框线和ID按同帧模型安装位置投影。', font=font(26), fill='#44596e')
    draw.text((1040, 1430), '标注用于说明布局，不代表本帧23块全部被算法识别。', font=font(26), fill='#44596e')
    draw.text((1040, 1480), '原始图像另存；没有增加相机、改变光照或重绘靶标。', font=font(26), fill='#44596e')
    draw.text((55, 1660), '图源：原双目工具间隙0.5 m独立驻留试验，source frame 112，仿真t=0 s。',
              font=font(24), fill='#526679')
    output = output_directory/'target_marker_layout_annotated.png'
    canvas.save(output)
    audit = dict(source_image=str(source), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        figure_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
        camera='original stereo left, 100 degrees, 1920x1920',
        simulation_time_ns=frame['sim_time_ns'], source_frame_id=frame['frame_id'],
        annotation_uses_model_projection=True, all_markers_detected_claim=False,
        source_pixels_brightness_or_lighting_modified=False, markers=records)
    (output_directory/'figure_provenance.json').write_text(json.dumps(audit, ensure_ascii=False, indent=2), encoding='utf-8')
    print(output)


if __name__ == '__main__':
    main()
