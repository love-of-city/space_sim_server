"""Overlay read-only gripper alignment on a previously estimated camera frame.

Example (run with the vision Python environment):
  python tools/render_grasp_alignment.py --source D:/.../wrist_vision_0deg_v1 --output D:/.../wrist_alignment_v1
No connection to the simulation/control server is opened.
"""
from pathlib import Path
import argparse
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'backend'))
from space_arm_platform.grasp_alignment import load_capture_mount, alignment_from_pose


def render(source: Path, output: Path, model: Path, standoff: float):
    source,output,model=source.resolve(),output.resolve(),model.resolve()
    observation=json.loads((source/'result.json').read_text(encoding='utf-8'))
    raw=(source/'camera_input.png').read_bytes()
    if hashlib.sha256(raw).hexdigest()!=observation['input_sha256']:
        raise ValueError('source image does not match pose-estimation input')
    camera=ET.parse(model).find('.//camera[@name="capture_wrist_cam"]')
    expected=observation['calibration']
    if list(map(int,camera.get('resolution').split()))!=expected['native_image_size'] or float(camera.get('fovy'))!=expected['fovy_deg']:
        raise ValueError('current camera intrinsics differ from source calibration')
    mount=load_capture_mount(model)
    # This saved frame was taken after the verified zero-degree mount update.
    if not np.allclose(mount['camera_position_grip_m'],[.075,0,.031]) or not np.allclose(mount['R_grip_from_camera_optical'],[[0,-1,0],[-1,0,0],[0,0,-1]]):
        raise ValueError('saved frame requires its original zero-degree extrinsic calibration')
    alignment=alignment_from_pose(observation,mount,standoff)
    alignment.update(source_pose_file=str(source/'result.json'),source_input_sha256=observation['input_sha256'],
        model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
        source_mode='same saved single WebRTC frame as source pose, not live telemetry',
        dynamic_simulator_truth_used=False)
    output.mkdir(parents=True,exist_ok=True)
    (output/'alignment.json').write_text(json.dumps(alignment,indent=2,ensure_ascii=False),encoding='utf-8')
    image=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
    for detection in observation['detections']:
        points=np.rint(detection['corners']).astype(int)
        cv2.polylines(image,[points],True,(0,255,0),2,cv2.LINE_AA)
        cv2.putText(image,f"ID {detection['id']}",tuple(points[0]+[0,-8]),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,255,0),2,cv2.LINE_AA)
    lines=[('抓夹坐标下的视觉对准误差',25),('单帧观察 · 控制输出关闭',21),
           ('参考点：已有 grip_frame 抓夹中心',19),('显示坐标：X 向右／Y 向下／Z 向前',19)]
    if alignment['valid']:
        position=np.array(alignment['interface_position_display_m'])*1000
        error=np.array(alignment['position_error_display_mm'])
        angles=np.array(alignment['rotation_error_display_deg'])
        lines += [('接口中心相对抓夹中心',22),
            (f'X 横向偏心：{position[0]:+.2f} mm（右为正）',21),
            (f'Y 纵向偏心：{position[1]:+.2f} mm（下为正）',21),
            (f'Z 前方剩余间距：{position[2]:.2f} mm',21),
            (f'相对 {standoff*1000:g} mm 准备位：{error[2]:+.2f} mm',20),
            ('朝既有抓取对准姿态的旋转误差',22),
            (f'绕 X（右向）：{angles[0]:+.2f}°',21),
            (f'绕 Y（下向）：{angles[1]:+.2f}°',21),
            (f'绕 Z（前向）：{angles[2]:+.2f}°',21),
            (f"总姿态偏差：{alignment['total_angle_error_deg']:.2f}°",20),
            ('角度为旋转向量分量，按右手定则取正负',17)]
        K=np.array(expected['input_K'])
        pose=observation['pose']
        R_CT=np.array(pose['rotation_camera_from_target']);t_CT=np.array(pose['translation_m'])
        R_GC=np.array(mount['R_grip_from_camera_optical']);p_GC=np.array(mount['camera_position_grip_m'])
        p_GI=np.array(alignment['interface_position_grip_m'])
        p_CI=R_CT@np.array(mount['interface_origin_target_m'])+t_CT
        # Compare measured centre with where the physical grip axis intersects
        # this estimated depth plane, including both 75 mm and 31 mm offsets.
        p_Caxis=R_GC.T@(np.array([0.,0.,p_GI[2]])-p_GC)
        project=lambda p:(K@p)[:2]/p[2]
        measured,axis=project(p_CI),project(p_Caxis)
        pm,pa=tuple(np.rint(measured).astype(int)),tuple(np.rint(axis).astype(int))
        cv2.drawMarker(image,pa,(255,255,0),cv2.MARKER_CROSS,22,1,cv2.LINE_AA)
        cv2.circle(image,pm,4,(0,140,255),1,cv2.LINE_AA)
        cv2.line(image,pa,pm,(255,0,255),1,cv2.LINE_AA)
        # Add a clearly labelled 4x magnification; it is not a second camera.
        h,w=image.shape[:2]; cx,cy=np.rint((measured+axis)/2).astype(int)
        x0,y0=max(0,min(w-64,cx-32)),max(0,min(h-64,cy-32))
        inset=cv2.resize(image[y0:y0+64,x0:x0+64],(256,256),interpolation=cv2.INTER_NEAREST)
        image[72:328,12:268]=inset
        cv2.rectangle(image,(12,72),(268,328),(180,180,180),1)
        alignment['overlay_pixels']={'interface_center':measured.tolist(),'grip_axis_at_estimated_depth':axis.tolist()}
    else:
        lines += [('视觉结果无效，不显示旧误差',22)]
    lines += [('安装偏置已计入',21),('相机在抓夹中心上方 75 mm、后方 31 mm',18),
              ('青色十字：同深度处的抓夹轴线',18),('橙色圆圈：视觉接口中心；左上为 4 倍放大',18),
              ('绿色框：四个靶标；输入内参已随画面缩放',17),
              ('轴向间距是几何量，不是可直接执行的行程',17),
              ('沿用上一张识别帧，非实时误差／非真值精度',17)]
    row_height=sum(size+12 for _,size in lines)+38
    canvas=Image.new('RGB',(image.shape[1]+600,max(image.shape[0],row_height)),(20,27,37))
    canvas.paste(Image.fromarray(cv2.cvtColor(image,cv2.COLOR_BGR2RGB)),(0,0))
    draw=ImageDraw.Draw(canvas); x=image.shape[1]+22;y=20
    for text,size in lines:
        font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',size)
        draw.text((x,y),text,font=font,fill=(225,237,247)); y+=size+12
    draw.text((14,46),'接口附近 4 倍放大（非另一相机）',font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',17),fill=(255,255,255))
    canvas.save(output/'alignment_overlay.png')
    (output/'alignment.json').write_text(json.dumps(alignment,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps({k:alignment.get(k) for k in ('valid','interface_position_display_m','position_error_display_mm','rotation_error_display_deg','total_angle_error_deg','control_enabled')},indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--model',type=Path,default=ROOT/'model/SARM/platform/sarm_dual_capture26.xml')
    parser.add_argument('--standoff-m',type=float,default=.5)
    args=parser.parse_args()
    render(args.source,args.output,args.model,args.standoff_m)
