"""Encode synchronized real input JPEGs at their simulation sampling rate.

Only display resizing and a separate caption band are applied. No brightness,
material, background, detector input, temporal interpolation or retiming edits.
One video contains one initialized run; it is not a continuous mission claim.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import cv2
import imageio_ffmpeg
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--title',required=True)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    original=a.run/'actual_images'
    frames=[json.loads(l)for l in (original/'render_frames.jsonl').read_text().splitlines()]
    frames=[f for f in frames if f.get('capture_episode_id') and int(f['sim_time_ns'])>0]
    rows={round(r['time_s']*1e9):r for r in [json.loads(l)for l in (a.run/'trajectory.jsonl').read_text().splitlines()]}
    ns=np.array([int(f['sim_time_ns'])for f in frames],dtype=np.int64)
    assert len(frames)>1 and np.all(np.diff(ns)==100_000_000)
    summary=json.loads((a.run/'summary.json').read_text())
    config=json.loads((original/'calibration.json').read_text())
    fov=float(np.rad2deg(config['cameras'][0]['field_of_view_rad']))
    near_path=original/'near_calibration.json'
    near_fov=(float(np.rad2deg(json.loads(near_path.read_text())['cameras'][0]['field_of_view_rad']))
              if near_path.exists() else None)
    fontpath=Path('C:/Windows/Fonts/msyh.ttc')
    font=ImageFont.truetype(str(fontpath),25)
    small=ImageFont.truetype(str(fontpath),22)
    ffmpeg=imageio_ffmpeg.get_ffmpeg_exe()
    command=[ffmpeg,'-hide_banner','-loglevel','error','-f','rawvideo','-pixel_format','rgb24',
             '-video_size','1920x1080','-framerate','10','-i','pipe:0','-an','-c:v','libx264',
             '-threads','4','-preset','fast','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(a.output)]
    process=subprocess.Popen(command,stdin=subprocess.PIPE,stderr=subprocess.PIPE)
    source_hash=hashlib.sha256()
    try:
        for j,f in enumerate(frames):
            row=rows[int(f['sim_time_ns'])]
            near_active=row.get('active_image_channel')=='near'
            frame_fov=near_fov if near_active else fov
            aided=summary['image_uses_cooperative_attitude'] and not near_active
            canvas=Image.new('RGB',(1920,1080),(18,22,28))
            for i,eye in enumerate(('left','right')):
                path=original/'images'/(f['frame_id']+('_near_' if near_active else '_')+eye+'.jpg')
                data=path.read_bytes();source_hash.update(data)
                image=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR)
                assert image.shape[:2]==(1920,1920)
                image=cv2.cvtColor(cv2.resize(image,(960,960),interpolation=cv2.INTER_AREA),cv2.COLOR_BGR2RGB)
                canvas.paste(Image.fromarray(image),(i*960,76))
            draw=ImageDraw.Draw(canvas)
            draw.text((18,5),a.title,font=font,fill=(240,244,250))
            mode=(f'{frame_fov:g}°双目，姿态遥测辅助求位置' if aided
                  else f'{frame_fov:g}°双目，图像求完整位姿')
            line=(f"仿真时间 {row['time_s']:.1f} s    光学深度 {row['true_optical_depth_m']:.4f} m    "
                  f"导航位置误差 {row['filtered_position_error_mm']:.3f} mm    视觉{'就绪' if row['visual_ready'] else '积累有效观测中'}    {mode}")
            draw.text((18,41),line,font=small,fill=(195,216,237))
            draw.text((18,1042),'左眼                                        原仿真灯光、材质和23块方形靶标；本视频为独立验证片段                                        右眼',font=small,fill=(210,218,228))
            if j==0:canvas.save(a.output.with_suffix('.jpg'),quality=95)
            process.stdin.write(np.asarray(canvas).tobytes())
        process.stdin.close()
        error=process.stderr.read().decode(errors='replace')
        if process.wait()!=0:raise RuntimeError(error)
    finally:
        if process.poll() is None:process.kill();process.wait()
    subprocess.run([ffmpeg,'-hide_banner','-loglevel','error','-i',str(a.output),'-f','null','-'],check=True,
                   stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
    report=dict(source_run=str(a.run.resolve()),source_frame_count=len(frames),fps=10,
        simulation_time_window_s=[float(ns[0]*1e-9),float(ns[-1]*1e-9)],
        duration_s=len(frames)/10,source_image_sha256_concatenated=source_hash.hexdigest(),
        video_sha256=hashlib.sha256(a.output.read_bytes()).hexdigest(),
        source='actual synchronized image-estimator input JPEGs from this dynamics run',
        original_lighting_retained=True,display_resize_only=True,complete_decode_pass=True,
        newly_encoded=True,full_mission=False,independent_image_attitude=not summary['image_uses_cooperative_attitude'])
    if summary.get('range_transfer') is not None:
        report.update(displays_active_calibrated_pair=True,range_transfer=summary['range_transfer'])
    a.output.with_suffix('.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
