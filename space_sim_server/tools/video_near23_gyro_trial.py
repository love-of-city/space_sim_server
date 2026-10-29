"""Encode saved, uniformly sampled raw stereo frames as a portable H.264 MP4.

No missing frame is filled or synthesized. This displays the actual saved
1 Hz simulation samples at 5 fps, i.e. approximately five times simulation speed.
"""
import argparse
import json
from pathlib import Path
import subprocess

import imageio_ffmpeg
from PIL import Image, ImageDraw, ImageFont


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(); directory = args.directory.resolve()
    output = args.output or directory/'stereo_capture_h264.mp4'
    if output.exists(): raise FileExistsError(output)
    truth = {str(r['source_frame_id']): r for line in (directory/'truth.jsonl').read_text().splitlines()
             if (r := json.loads(line))}
    pairs = sorted((p for p in (directory/'images').glob('*_left.jpg')
                    if p.with_name(p.name.replace('_left', '_right')).exists()),
                   key=lambda p:int(p.name.split('_')[0]))
    if not pairs: raise ValueError('no saved image pairs')
    font_path = Path('C:/Windows/Fonts/msyh.ttc')
    font = ImageFont.truetype(str(font_path), 27) if font_path.exists() else ImageFont.load_default()
    command = [imageio_ffmpeg.get_ffmpeg_exe(), '-hide_banner', '-loglevel', 'error',
        '-f', 'rawvideo', '-pixel_format', 'rgb24', '-video_size', '1920x1080', '-framerate', '5',
        '-i', 'pipe:0', '-an', '-c:v', 'libx264', '-preset', 'medium', '-crf', '20',
        '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output)]
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        for left in pairs:
            frame = left.name.split('_')[0]; record = truth[frame]
            sheet = Image.new('RGB', (1920, 1080), '#15202b'); draw = ImageDraw.Draw(sheet)
            t = int(record['sim_time_ns'])/1e9; f = record.get('feedback') or {}
            gap = -record['interface_position_grip_m'][2]
            draw.text((18, 9), f'左右原始双目图像（缩放显示）  t={t:.1f} s  接口间隙={gap*1000:.1f} mm', font=font, fill='white')
            draw.text((18, 44), f'{record.get("phase", "preparation")}  压盘力={f.get("pad_force_n", 0):.2f} N  支撑爪={f.get("jaw_support_count", 0)}', font=font, fill='white')
            draw.text((18, 79), '原图按仿真1秒取样，5帧/秒播放；未补帧。环境及灯光参数沿用原仿真。', font=font, fill='#b0d8e8')
            for x, path in [(0, left), (960, left.with_name(left.name.replace('_left', '_right')))]:
                with Image.open(path) as image:
                    sheet.paste(image.convert('RGB').resize((960, 960)), (x, 120))
            process.stdin.write(sheet.tobytes())
        process.stdin.close(); error = process.stderr.read().decode(errors='replace')
        if process.wait() != 0: raise RuntimeError(error)
    finally:
        if process.poll() is None: process.kill(); process.wait()
    print(json.dumps(dict(output=str(output), frames=len(pairs), codec='H.264', pixel_format='yuv420p',
        faststart=True, source='actual saved stereo images', missing_frame_substitution=False)))


if __name__ == '__main__':
    main()
