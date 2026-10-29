"""Isolated UE/image/LocalMujocoStepper trial of the merged terminal capture.

Uses an existing compatible native runtime and original prepared mesh catalog.
No live platform state, control socket or default renderer port is written.
Physics waits for actual image pairs: this is not a real-time latency test.
"""
import argparse
import copy
import importlib.util
import json
import os
from pathlib import Path
import queue
import socket
import subprocess
import sys
import time
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


def read(path):
    return json.loads(Path(path).read_text('utf-8-sig'))


def write(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-server', type=Path, required=True)
    parser.add_argument('--native-runtime', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=225.)
    parser.add_argument('--gap', type=float, default=.5)
    parser.add_argument('--hold-seconds', type=float, default=20.)
    parser.add_argument('--dropout', type=float, nargs=2, metavar=('START', 'STOP'))
    args = parser.parse_args()
    source = args.source_server.resolve(); out = args.output.resolve()
    if not out.is_relative_to((ROOT/'run').resolve()):
        raise ValueError('private trial output must be inside this server run directory')
    out.mkdir(parents=True, exist_ok=False); (out/'images').mkdir()
    adapter = source.parent/'space_sim_UE_Adapter'
    sys.path[:0] = [str(args.native_runtime.resolve()), str(ROOT), str(ROOT/'backend'),
                    str(adapter/'Adapters'), str(source/'run/end_stereo_20261023')]
    sys.modules['mujoco'] = None  # Native Basilisk ABI; never load standalone MuJoCo.
    os.environ['SPACE_SIM_POSTURE_BACKEND'] = 'native'
    import bsk_render_adapter
    from bsk_render_adapter import mjcf_assets
    from bsk_render_adapter.protocol import RenderPublisher
    from space_arm_platform.capture_receiver import CaptureReceiver
    from space_arm_platform.vision_service import _Worker
    from space_arm_platform.stereo_capture_packet import capture_packet, publish_packet
    from simulation import teleop_grasp_unreal as teleop
    from simulation.near23_capture import configure_scene, CONFIG
    # Load the existing benchmark harness; latest simulation modules above take
    # precedence, while source model/STL resources remain in the source tree.
    spec = importlib.util.spec_from_file_location('near23_benchmark',
        source/'run/end_stereo_20261023/runner_web_recording.py')
    runner = importlib.util.module_from_spec(spec); spec.loader.exec_module(runner)
    static = read(source/'run/capture_attempt_20261022/static_geometry.json')
    setup = read(ROOT/CONFIG)
    # Give each isolated trial its own configuration and atomic mailbox.
    # Otherwise a second test could overwrite another test's scene/session.
    setup['capture_export'] = (out/'observation.json').relative_to(ROOT).as_posix()
    config_path = out/'calibration.json'; write(config_path, setup)
    config_key = config_path.relative_to(ROOT).as_posix()
    estimator = _Worker(ROOT, config_path)
    scene = configure_scene(read(source/'run/capture_attempt_20261022/baseline_300s_input.scene.json'), ROOT, out)
    scene['render_profile']['vision_capture']['config_file'] = config_key
    scene['randomization']['capture_visual_test'].update(config_file=config_key,
        observation_file=setup['capture_export'])
    scene['instance_id'] = 'near23-gyro-'+out.name
    scene['runtime']['simulation_rate'] = 100.
    formation = scene['randomization']['coorbital_initialization']
    formation['grip_frame_standoff_m'] = args.gap
    formation['separation_m'] -= .5-args.gap
    scene_path = out/'input.scene.json'; write(scene_path, scene); write(out/'calibration.json', setup)
    received = queue.Queue(); errors = []
    receiver = CaptureReceiver('127.0.0.1', 0, lambda m,p: received.put((m,p)), errors.append,
                               on_vision_capture=lambda m,p: received.put((m,p)))
    receiver.start(); deadline = time.monotonic()+10
    while receiver._listener is None:
        if time.monotonic()>deadline: raise TimeoutError('capture listener')
        time.sleep(.02)
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0)); port = sock.getsockname()[1]
    project = adapter/'Unreal/BskUnrealRenderer'; streamer = 'Near23Gyro'+str(os.getpid())
    command = ['C:/Program Files/Epic Games/UE_5.6/Engine/Binaries/Win64/UnrealEditor-Cmd.exe',
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
    process = browser = wire = None
    logs = {name: (out/(name+'.jsonl')).open('w', encoding='utf-8', buffering=1)
            for name in ('measurements', 'render_frames', 'observations')}
    stats = dict(scored_pairs=0, rejected_pairs=0, injected_lost_pairs=0)
    mailbox = ROOT/setup['capture_export']
    last_observation = {}
    holding_since = None

    class TrialFinished(Exception):
        pass

    class IsolatedClient(teleop.SimulationControlClient):
        def start(self): pass
        def wait_until_ready(self, *a, **k): pass
        def has_observation_capacity(self, *a, **k): return True
        def send_observation(self, message):
            last_observation.clear(); last_observation.update(message)
            logs['observations'].write(json.dumps(message)+'\n')

    class ImagePublisher:
        def __init__(self, sink):
            self.sink = sink; self.lighting_confirmed = False
        def __getattr__(self, key): return getattr(wire, key)
        def retain_hello(self, msg): wire.retain_hello(msg)
        def retain_manifest(self, msg):
            msg = copy.deepcopy(msg)
            # The profile itself installs all 23 films and calibrated cameras.
            self.expected = set(setup['camera_ids'].values())
            write(out/'manifest.json', msg)
            write(out/'lighting.json', dict(settings=msg['settings'], visuals=msg['visuals'],
                renderer_config=read(project/'Config/bsk_unreal_scene.json'),
                lighting_overrides=False, original_native_work_light_parameters=True))
            wire.retain_manifest(msg)
        def publish_frame(self, frame):
            nonlocal holding_since
            self.sink.publish_frame(frame)
            msg = copy.deepcopy(frame)
            msg.pop('vision_capture_requested', None); msg.pop('vision_capture_camera_ids', None)
            ns = int(msg['sim_time_ns']); capture = ns%100000000 == 0 and ns>=200000000
            msg.update(capture_episode_id='', capture_request_id='near23-gyro-trial',
                       vision_capture_requested=capture, vision_capture_camera_ids=sorted(self.expected))
            wire.publish_frame(msg)
            if not self.lighting_confirmed:
                deadline = time.monotonic()+210
                while not (out/'native_work_lights_ready.json').exists():
                    if browser.poll() is not None: raise RuntimeError('native light setup failed; see browser.err.log')
                    if time.monotonic()>deadline: raise TimeoutError('native work light setup')
                    wire.publish_frame(msg)
                    time.sleep(.2)
                self.lighting_confirmed = True
                print('ORIGINAL_NATIVE_WORK_LIGHTS_VERIFIED', flush=True)
            if not capture: return
            pairs = {}; deadline = time.monotonic()+60
            while len(pairs)<len(self.expected):
                if errors: raise RuntimeError(errors[-1])
                if process.poll() is not None: raise RuntimeError('private renderer exited')
                if time.monotonic()>deadline: raise TimeoutError(f'frame {frame["frame_id"]}: {list(pairs)}')
                try: meta, products = received.get(timeout=.1)
                except queue.Empty: continue
                for key in ('session_id', 'source_frame_id', 'sim_time_ns'):
                    expected = frame['frame_id'] if key=='source_frame_id' else frame[key]
                    if str(meta[key])!=str(expected): raise ValueError('source mismatch: '+key)
                cid = meta['camera_id']
                if cid not in self.expected or cid in pairs: raise ValueError('duplicate/unexpected camera')
                pairs[cid] = (meta, products)
            blobs = {eye:pairs[setup['camera_ids'][eye]][1]['rgb'] for eye in ('left','right')}
            result = estimator(blobs['left'], blobs['right'])
            packet = capture_packet(result, setup, session_id=frame['session_id'],
                source_frame_id=frame['frame_id'], sim_time_ns=ns)
            dropped = bool(args.dropout and args.dropout[0]<=ns/1e9<args.dropout[1])
            if dropped: packet.update(valid=False, status='injected_visual_loss')
            publish_packet(mailbox, packet)
            logs['measurements'].write(json.dumps(dict(packet=packet, injected_loss=dropped,
                metadata={eye:pairs[setup['camera_ids'][eye]][0] for eye in ('left','right')}))+'\n')
            logs['render_frames'].write(json.dumps(frame)+'\n')
            # Raw images at 1 Hz plus every loss/recovery boundary, full poses at 10 Hz.
            if stats['scored_pairs']%10 == 0:
                for eye, blob in blobs.items(): (out/'images'/f'{frame["frame_id"]}_{eye}.jpg').write_bytes(blob)
            stats['scored_pairs'] += 1; stats['rejected_pairs'] += result['status']!='ok'
            stats['injected_lost_pairs'] += dropped
            if stats['scored_pairs']%50 == 0:
                print(json.dumps(dict(type='near23_gyro_images', t=ns/1e9, **stats,
                                      used_ids=result.get('used_ids'), status=result['status'])), flush=True)
            status = last_observation.get('capture_arm', {}).get('controller_status', '')
            if status.startswith('predictive_abort'):
                raise RuntimeError('capture controller stopped: '+status)
            if (out/'stop_trial').exists():
                raise RuntimeError('trial stopped by local stop_trial request')
            if status == 'predictive_hold_force':
                if holding_since is None: holding_since = ns/1e9
                if ns/1e9-holding_since >= args.hold_seconds: raise TrialFinished()
            else:
                holding_since = None
        def close(self): pass  # Outer scope owns and closes the private transport.

    original_bridge = bsk_render_adapter.BasiliskRenderBridge
    class LiveBridge(original_bridge):
        def __init__(self, **kwargs):
            sink = kwargs.pop('publisher')
            super().__init__(**kwargs, publisher=ImagePublisher(sink))
        def add_mj_scene(self, scene, **kwargs):
            kwargs['mesh_asset_catalog'] = Path(static['catalog'])
            return super().add_mj_scene(scene, **kwargs)
    try:
        env = os.environ.copy(); env['UE-LocalDataCachePath'] = str(source/'run/ddc_capture26')
        process = subprocess.Popen(command, cwd=project, env=env, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        write(out/'process.json', dict(pid=process.pid, port=port, streamer=streamer))
        deadline = time.monotonic()+240
        while True:
            if process.poll() is not None: raise RuntimeError('UE startup failed')
            try:
                with socket.create_connection(('127.0.0.1',port), timeout=.2): break
            except OSError:
                if time.monotonic()>deadline: raise TimeoutError('private UE startup')
                time.sleep(.2)
        browser = subprocess.Popen(['node', str(source/'run/browser_probe_20261024/open_far_vision_lights_20261027.mjs'),
            str(out), streamer], cwd=source, stdout=(out/'browser.out.log').open('wb'),
            stderr=(out/'browser.err.log').open('wb'), creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        wire = RenderPublisher(port=port)
        trial_args = runner.parser().parse_args(['--adapter-root',str(adapter),'--scene-instance',str(scene_path),
            '--output',str(out/'physics.json'),'--duration',str(args.duration),'--motion','hold',
            '--dynamics-backend','local','--posture-backend','native',
            '--model-path',str(source/'model/SARM/platform/sarm_dual_capture26.xml')])
        with patch.object(bsk_render_adapter,'BasiliskRenderBridge',LiveBridge), \
             patch.object(teleop,'SimulationControlClient',IsolatedClient), \
             patch.object(teleop,'RENDER_HZ',10), \
             patch.object(mjcf_assets,'_compiled_source_mesh_poses',lambda _:static['compiled_mesh_poses']):
            try:
                runner.benchmark(trial_args)
            except TrialFinished:
                pass
        write(out/'completed.json',dict(duration_sim_s=args.duration, **stats,
            physics='actual LocalMujocoStepper + native Basilisk + contact + shared 50 Hz gyro',
            image_feedback_only=True, simulation_code_root=str(ROOT), model_resource_root=str(source),
            no_live_control_connection=True, lockstep=True, measured_latency_test=False,
            physical_rate_hz=400, image_rate_hz=10, render_publish_rate_hz=10,
            opencv_threads=setup.get('opencv_threads',1), early_stop_after_force_hold_s=args.hold_seconds))
    finally:
        (out/'stop_browser').touch()
        if browser and browser.poll() is None:
            try: browser.wait(timeout=10)
            except subprocess.TimeoutExpired: browser.terminate(); browser.wait(timeout=10)
        if wire: wire.close()
        if process and process.poll() is None:
            process.terminate()
            try: process.wait(timeout=15)
            except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=10)
        receiver.close()
        estimator.close()
        for stream in logs.values(): stream.close()


if __name__=='__main__':
    main()
