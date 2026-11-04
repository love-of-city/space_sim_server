"""Make a labelled playback of one trial's actual original-camera inputs.

Sparse far images remain still until the next recorded image. No intermediate
views are generated. Playback speed and image sample time are always displayed.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess

import cv2
import imageio_ffmpeg
import numpy as np
from PIL import Image, ImageDraw, ImageFont


PHASES = dict(NavigationAcquire='导航初始化保持', Station17Approach='进入17.4m停泊',
              Station17Hold='17.4m停稳检查', TargetPointing='目标星调姿',
              GNSSBridge5p5Approach='GNSS诊断接近5.5m', GNSSBridge5p5Hold='5.5m导航停泊',
              GNSSBridgeNearApproach='GNSS衔接原双目', OriginalStereoAcquire='近距等待视觉就绪',
              VisualApproach='原双目参与导航', PlatformEntry='平台进入检查',
              PlatformHold='平台驻留', Grasp='机械臂图像/接触抓取',
              Complete='本次请求阶段完成', SafeHold='停止保持')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--playback-speed', type=float, default=10.)
    parser.add_argument('--start-s', type=float)
    parser.add_argument('--end-s', type=float)
    parser.add_argument('--allow-incomplete', action='store_true')
    args = parser.parse_args()
    if not np.isfinite(args.playback_speed) or args.playback_speed <= 0:
        parser.error('positive finite playback speed required')
    directory, output = args.run.resolve(), args.output.resolve()
    if output.exists():
        raise FileExistsError(output)
    summary_path = directory/'summary.json'
    if not summary_path.exists() and not args.allow_incomplete:
        raise ValueError('finished run required, or explicitly --allow-incomplete')
    calibration = json.loads((directory/'actual_images/calibration.json').read_text(encoding='utf-8'))
    assert len(calibration['marker_rig']['markers']) == 23
    for camera in calibration['cameras']:
        assert np.isclose(camera['field_of_view_rad'], np.deg2rad(100))
        assert camera['resolution'] == [1920, 1920]
    frames = []
    with (directory/'actual_images/render_frames.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            try:
                frame = json.loads(line)
            except json.JSONDecodeError:
                break
            if frame.get('capture_episode_id'):
                frames.append(dict(frame_id=frame['frame_id'], sim_time_ns=int(frame['sim_time_ns'])))
    rows = {}
    with (directory/'trajectory.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                break
            rows[round(row['time_s']*1e9)] = row
    frames = [f for f in frames if f['sim_time_ns'] in rows]
    times = np.array([f['sim_time_ns']/1e9 for f in frames])
    assert len(times) > 1 and np.all(np.diff(times) > 0)
    start = times[0] if args.start_s is None else max(times[0], args.start_s)
    end = times[-1] if args.end_s is None else min(times[-1], args.end_s)
    if not start < end:
        raise ValueError('nonempty video time window required')
    fps = 10
    count = math.ceil((end-start)/args.playback_speed*fps)+1
    output.parent.mkdir(parents=True, exist_ok=True)
    font = ImageFont.truetype('C:/Windows/Fonts/msyh.ttc', 25)
    small = ImageFont.truetype('C:/Windows/Fonts/msyh.ttc', 22)
    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    process = subprocess.Popen([ffmpeg, '-hide_banner', '-loglevel', 'error', '-f', 'rawvideo',
        '-pixel_format', 'rgb24', '-video_size', '1920x1080', '-framerate', str(fps),
        '-i', 'pipe:0', '-an', '-c:v', 'libx264', '-threads', '4', '-preset', 'fast',
        '-crf', '18', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output)],
        stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    source_hash = hashlib.sha256()
    used = []
    cached_id, cached = None, None
    repeated_source_frames = 0
    try:
        for index in range(count):
            now = min(end, start+index*args.playback_speed/fps)
            position = max(0, int(np.searchsorted(times, now+1e-9, side='right')-1))
            frame = frames[position]
            row = rows[frame['sim_time_ns']]
            identifier = frame['frame_id']
            repeated = identifier == cached_id
            repeated_source_frames += int(repeated)
            if identifier != cached_id:
                cached = Image.new('RGB', (1920, 1080), '#12161c')
                for eye, x in (('left', 0), ('right', 960)):
                    data = (directory/f'actual_images/images/{identifier}_{eye}.jpg').read_bytes()
                    source_hash.update(data)
                    raw = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                    assert raw.shape[:2] == (1920, 1920)
                    resized = cv2.cvtColor(cv2.resize(raw, (960, 960), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB)
                    cached.paste(Image.fromarray(resized), (x, 76))
                cached_id = identifier
                used.append(dict(source_frame_id=identifier, sample_time_s=times[position]))
            canvas = cached.copy()
            draw = ImageDraw.Draw(canvas)
            phase = PHASES.get(row['policy']['phase'], row['policy']['phase'])
            draw.text((18, 5), f'原100°双目连续诊断｜{args.playback_speed:g}倍播放｜{phase}', font=font, fill='#f0f4fa')
            draw.text((18, 41), f"画面采样 t={times[position]:.1f}s   光学深度 {row['true_optical_depth_m']:.3f}m   "
                      f"导航位置误差 {row['filtered_position_error_mm']:.3f}mm   "
                      f"视觉{'就绪' if row['visual_ready'] else '未就绪'}", font=small, fill='#c3d8ed')
            interval = (times[position+1]-times[position] if position+1 < len(times)
                        else times[position]-times[position-1])
            sample_status = '沿用该采样原图' if repeated else '显示新采样原图'
            draw.text((18, 1042), f'左眼 / 右眼｜原灯光、23块靶标｜源图间隔 {interval:g}s｜{sample_status}｜播放对应仿真 t={now:.1f}s',
                      font=small, fill='#d2dae4')
            if not index:
                canvas.save(output.with_suffix('.jpg'), quality=95)
            process.stdin.write(np.asarray(canvas).tobytes())
        process.stdin.close()
        error = process.stderr.read().decode(errors='replace')
        if process.wait() != 0:
            raise RuntimeError(error)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
    subprocess.run([ffmpeg, '-hide_banner', '-loglevel', 'error', '-i', str(output), '-f', 'null', '-'],
                   stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, check=True)
    report = dict(source_run=str(directory), simulation_time_window_s=[float(start), float(end)],
                  playback_speed=args.playback_speed, video_fps=fps, video_frames=count,
                  distinct_original_pairs_displayed=len(used), available_original_pairs=len(frames),
                  sparse_views_held_until_next_recorded_image=True, interpolated_views_generated=False,
                  caption_schema='actual_source_interval_and_repeated_source_status',
                  output_frames_reusing_previous_source=repeated_source_frames,
                  original_lighting_and_camera_retained=True, includes_caption_band_and_display_resize=True,
                  complete_decode_pass=True, source_image_sha256_concatenated=source_hash.hexdigest(),
                  video_sha256=hashlib.sha256(output.read_bytes()).hexdigest(), source_frames=used,
                  production_visual_policy_pass_claim=False)
    output.with_suffix('.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k: report[k] for k in ('simulation_time_window_s', 'playback_speed', 'video_frames',
                                           'distinct_original_pairs_displayed', 'complete_decode_pass')}))


if __name__ == '__main__':
    main()
