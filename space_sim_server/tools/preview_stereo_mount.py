"""Render a relocated stereo pair alongside the original on saved body states.

Only invisible camera descriptors are added. Recorded bodies, marker geometry,
environment and native work-light settings stay unchanged. This is a camera
comparison, not a new dynamics/grasp trial. The original right-eye work light
stays at its original mount during this comparison.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import queue
import socket
import subprocess
import sys
import time
import uuid

import numpy as np
from PIL import Image, ImageDraw, ImageFont


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-server', type=Path, required=True)
    parser.add_argument('--saved-run', type=Path, required=True)
    parser.add_argument('--camera-config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source, saved, out = (p.resolve() for p in (args.source_server, args.saved_run, args.output))
    out.mkdir(parents=True, exist_ok=False)
    (out/'images').mkdir()
    config = read(args.camera_config)
    original_config = read(saved/'calibration.json')
    original = read(saved/'manifest.json')
    manifest = copy.deepcopy(original)
    session = str(uuid.uuid4())
    manifest.update(session_id=session, revision='94')
    review_ids = {}
    for camera in manifest['cameras']:
        camera['capture_consumer'] = 'dataset' if camera['camera_id'] in original_config['camera_ids'].values() else 'vision'
    for eye in ('left', 'right'):
        camera = copy.deepcopy(next(c for c in config['cameras'] if c['camera_id']==config['camera_ids'][eye]))
        camera['camera_id'] = 'review/camera/front_'+eye
        camera['capture_consumer'] = 'dataset'
        camera['picture_in_picture'] = False
        review_ids[eye] = camera['camera_id']
        manifest['cameras'].append(camera)
    for key in ('settings', 'visuals', 'celestial_bodies', 'objects'):
        if manifest[key] != original[key]:
            raise ValueError('saved environment or body geometry changed: '+key)
    truth = {}
    with (saved/'truth.jsonl').open(encoding='utf-8-sig') as stream:
        for line in stream:
            row = json.loads(line)
            truth[str(row['source_frame_id'])] = row
    saved_frames = {}
    with (saved/'render_frames.jsonl').open(encoding='utf-8-sig') as stream:
        for line in stream:
            frame = json.loads(line)
            saved_frames[str(frame['frame_id'])] = frame
    # Truth is sampled faster than RGB. Select only recorded exposure epochs.
    truth = {key:row for key,row in truth.items() if key in saved_frames}
    gaps = [.5, .2, .12, .10, .05]
    selected = {}
    for gap in gaps:
        frame_id, row = min(truth.items(), key=lambda item: abs(-item[1]['interface_position_grip_m'][2]-gap))
        selected[frame_id] = dict(requested_gap_m=gap, truth=row)
    frames = [frame for key,frame in saved_frames.items() if key in selected]
    if len(frames) != len(selected):
        raise ValueError('selected truth has no matching saved render state')
    adapter = source.parent/'space_sim_UE_Adapter'
    sys.path[:0] = [str(Path(__file__).resolve().parents[1]/'backend'), str(adapter/'Adapters')]
    from bsk_render_adapter.protocol import RenderPublisher
    from space_arm_platform.capture_receiver import CaptureReceiver
    from space_arm_platform.multicamera_pose import MultiCameraPoseEstimator
    import cv2
    cv2.setNumThreads(2)
    estimator = MultiCameraPoseEstimator(config['calibration'], config['layout'])
    received, errors = queue.Queue(), []
    receiver = CaptureReceiver('127.0.0.1', 0, lambda m,p: received.put((m,p)), errors.append)
    receiver.start()
    deadline = time.monotonic()+10
    while receiver._listener is None:
        if time.monotonic()>deadline:
            raise TimeoutError('capture receiver startup')
        time.sleep(.05)
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    project = adapter/'Unreal/BskUnrealRenderer'
    streamer = 'StereoMountReview'+str(os.getpid())
    command = [r'C:/Program Files/Epic Games/UE_5.6/Engine/Binaries/Win64/UnrealEditor-Cmd.exe',
        str(project/'BskUnrealRenderer.uproject'), '-game', '-unattended', '-nop4', '-nosplash',
        '-RenderOffscreen', '-ForceRes', '-ResX=1280', '-ResY=800', '-BskListen=127.0.0.1',
        f'-BskPort={port}', '-BskCaptureHost=127.0.0.1', f'-BskCapturePort={receiver._listener.getsockname()[1]}',
        '-BskCaptureProducts=rgb', '-BskCaptureRate=10', '-ExecCmds=t.MaxFPS 30,r.VSync 0',
        '-DDC=InstalledNoZenLocalFallback', f'-abslog={out/"renderer.log"}',
        '-PixelStreamingUseMediaCapture=false', '-PixelStreamingDecoupleFramerate=false',
        '-PixelStreamingConnectionURL=ws://127.0.0.1:18888', '-PixelStreamingID='+streamer,
        '-BskPixelStreamingURL=ws://127.0.0.1:18888', '-BskPixelStreamingBaseId='+streamer,
        '-BskPixelStreamingCameras=teleop/camera/capture_stereo_left+teleop/camera/capture_stereo_right',
        '-BskPixelStreamingCameraWidth=1920', '-BskPixelStreamingCameraHeight=1920',
        '-BskPixelStreamingCameraFps=10', '-PixelStreamingWebRTCFps=30', '-PixelStreamingEncoderMinQuality=60']
    write(out/'command.json', command)
    write(out/'manifest.json', manifest)
    write(out/'calibration.json', config)
    env = os.environ.copy()
    env['UE-LocalDataCachePath'] = str(source/'run/ddc_capture26')
    process = browser = publisher = None
    records = []
    metadata_file = (out/'captures.jsonl').open('w', encoding='utf-8', buffering=1)
    def healthy():
        if errors:
            raise RuntimeError(errors[-1])
        if process is not None and process.poll() is not None:
            raise RuntimeError('private renderer exited')
    def publish(original_frame, capture=False):
        frame = copy.deepcopy(original_frame)
        frame.pop('vision_capture_requested', None)
        frame.pop('vision_capture_camera_ids', None)
        frame.update(session_id=session, manifest_revision='94', wall_time_ns=str(time.time_ns()),
            capture_episode_id=session if capture else '', capture_request_id='stereo-mount-review')
        publisher.publish_frame(frame)
        return frame
    try:
        process = subprocess.Popen(command, cwd=project, env=env, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, creationflags=subprocess.CREATE_NO_WINDOW)
        write(out/'process.json', dict(pid=process.pid, port=port, capture_port=receiver._listener.getsockname()[1]))
        deadline = time.monotonic()+240
        print('Starting private original-environment renderer', flush=True)
        while True:
            healthy()
            try:
                with socket.create_connection(('127.0.0.1', port), timeout=.2):
                    break
            except OSError:
                if time.monotonic()>deadline:
                    raise TimeoutError('private renderer startup')
                time.sleep(.2)
        publisher = RenderPublisher(port=port)
        hello = read(source/'run/capture_attempt_20261022/near_prepared.json')['hello']
        hello['session_id'] = session
        publisher.retain_hello(hello)
        publisher.retain_manifest(manifest)
        for _ in range(40):
            publish(frames[0]); time.sleep(.05)
        browser = subprocess.Popen(['node', str(source/'run/browser_probe_20261024/open_far_vision_lights_20261027.mjs'),
            str(out), streamer], cwd=source, stdout=(out/'browser.out.log').open('wb'),
            stderr=(out/'browser.err.log').open('wb'), creationflags=subprocess.CREATE_NO_WINDOW)
        deadline = time.monotonic()+210
        while not (out/'native_work_lights_ready.json').exists():
            healthy()
            if browser.poll() is not None:
                raise RuntimeError('native work-light verification failed; see browser logs')
            if time.monotonic()>deadline:
                raise TimeoutError('native work-light verification')
            publish(frames[0]); time.sleep(.2)
        print('Original native work lights verified; capturing five saved states', flush=True)
        expected_ids = set(review_ids.values()) | set(original_config['camera_ids'].values())
        for original_frame in frames:
            frame = publish(original_frame, capture=True)
            pairs = {}
            deadline = time.monotonic()+90
            while len(pairs)<4:
                healthy()
                if time.monotonic()>deadline:
                    raise TimeoutError('four synchronized camera images: '+str(list(pairs)))
                try:
                    meta, products = received.get(timeout=.1)
                except queue.Empty:
                    continue
                if meta.get('session_id')!=session or str(meta['source_frame_id'])!=str(frame['frame_id']):
                    raise ValueError('image session/source frame mismatch')
                if str(meta['sim_time_ns'])!=str(frame['sim_time_ns']):
                    raise ValueError('image exposure timestamp mismatch')
                cid = meta['camera_id']
                if cid not in expected_ids or cid in pairs:
                    raise ValueError('unexpected/duplicate camera')
                intr = meta['intrinsics']; k = config['calibration']['K_left']
                np.testing.assert_allclose([intr['fx_px'],intr['fy_px'],intr['cx_px'],intr['cy_px']],
                    [k[0][0],k[1][1],k[0][2],k[1][2]], atol=1e-5)
                pairs[cid] = products['rgb']
                metadata_file.write(json.dumps(meta)+'\n')
            frame_id = str(frame['frame_id'])
            for variant, ids in [('old',original_config['camera_ids']), ('front',review_ids)]:
                for eye,cid in ids.items():
                    (out/'images'/f'{frame_id}_{variant}_{eye}.jpg').write_bytes(pairs[cid])
            result = estimator.estimate(pairs[review_ids['left']],pairs[review_ids['right']])['methods']['joint_multicamera']
            entry = dict(source_frame_id=frame_id, sim_time_ns=frame['sim_time_ns'],
                requested_gap_m=selected[frame_id]['requested_gap_m'], result=result)
            if result['status']=='ok':
                pose=result['pose_target_to_left'];mount=config['parent_from_optical']['left']
                r=np.array(mount['R_parent_from_optical']);p=np.array(mount['position_parent_m'])
                point=r@(np.array(pose['R'])@config['marker_rig']['grasp_center_target_m']+pose['t_m'])+p
                entry['interface_position_error_mm']=float(np.linalg.norm(point-selected[frame_id]['truth']['interface_position_grip_m'])*1000)
            records.append(entry)
            print(json.dumps(entry), flush=True)
        font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',24)
        # Show the 0.5 m source state; later states are separate raw image pairs.
        record=min(records,key=lambda row:abs(row['requested_gap_m']-.5))
        frame_id=record['source_frame_id']
        sheet=Image.new('RGB',(1600,1710),'#15202b');draw=ImageDraw.Draw(sheet)
        for row,(variant,label) in enumerate([('old','移动前：X=60，Z=50 mm'),('front','移动后：X=75，Z=31 mm')]):
            y=45+row*835
            draw.text((16,y-34),label+'；左右眼Y=±60 mm',font=font,fill='white')
            for col,eye in enumerate(('left','right')):
                image=Image.open(out/'images'/f'{frame_id}_{variant}_{eye}.jpg').convert('RGB').resize((800,800))
                sheet.paste(image,(col*800,y))
        sheet.save(out/'mount_before_after.jpg',quality=94)
        write(out/'summary.json',dict(scope=__doc__, source_run=str(saved),
            source_manifest_sha256=hashlib.sha256((saved/'manifest.json').read_bytes()).hexdigest(),
            source_camera_config_sha256=hashlib.sha256(args.camera_config.read_bytes()).hexdigest(),
            same_saved_body_states=True, unchanged_environment_and_marker_geometry=True,
            native_work_light_parameters_unchanged=True, original_light_mounts_retained_for_comparison=True,
            full_grasp_rerun=False, records=records))
        (out/'completed.json').write_text('{}',encoding='utf-8')
    finally:
        (out/'stop_browser').touch()
        if browser and browser.poll() is None:
            try: browser.wait(timeout=10)
            except subprocess.TimeoutExpired: browser.terminate(); browser.wait(timeout=10)
        if publisher: publisher.close()
        if process and process.poll() is None:
            process.terminate()
            try: process.wait(timeout=15)
            except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=10)
        receiver.close(); metadata_file.close()


if __name__=='__main__':
    main()
