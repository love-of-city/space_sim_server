"""Original-light private UE capture for authoritative dynamics experiments.

Truth readers are confined to render_frame(). Navigation receives only JPEGs
and source IDs/timestamps. The source scene and running preview are untouched.
Requires the explicitly supplied successful near-grasp fixture and UE project.
"""
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
from scipy.spatial.transform import Rotation

from simulation.dual_range_vision import long_focus_candidate


class NativeImageDynamicsFixture:
    def __init__(self, source_server, output, scene, orbit, near_config, *, long_focus_fov=None,
                 include_near=False):
        if long_focus_fov is not None or include_near:
            raise ValueError('Additional cameras withdrawn; original stereo capture only')
        self.source = Path(source_server).resolve()
        self.output = Path(output)/'actual_images'
        self.output.mkdir()
        self.session = str(uuid.uuid4())
        self.orbit = orbit
        fixture = self.source/'run/end_stereo_20261023/results/webpage_force_hold225_20261024'
        read = lambda path: json.loads(path.read_text(encoding='utf-8-sig'))
        self.manifest = read(fixture/'manifest.json')
        self.original = copy.deepcopy(self.manifest)
        self.config = copy.deepcopy(near_config)
        self.optics = dict(channel='original_wide', hardware_selected=False)
        if long_focus_fov is not None:
            self.config, self.manifest, self.optics = long_focus_candidate(
                self.config, self.manifest, fov_deg=long_focus_fov)
        self.ids = [self.config['camera_ids'][e] for e in ('left', 'right')]
        # A relocated pair must move in the renderer AND in hand-eye geometry.
        # The saved successful-run manifest describes the historical mount.
        # Keep its body geometry, environment and all other cameras intact.
        camera_mount_changes = []
        for spec in self.config['cameras']:
            matches = [c for c in self.manifest['cameras'] if c['camera_id'] == spec['camera_id']]
            if len(matches) != 1:
                raise ValueError('configured original camera missing from saved manifest')
            camera = matches[0]
            for key in ('parent_id', 'position_body_m', 'orientation_body_from_camera_wxyz',
                        'field_of_view_rad', 'resolution'):
                if camera[key] != spec[key]:
                    camera_mount_changes.append(dict(camera_id=spec['camera_id'], field=key,
                        previous=camera[key], current=spec[key]))
                camera[key] = copy.deepcopy(spec[key])
        self.camera_mount_changes = camera_mount_changes
        if include_near and long_focus_fov is None:
            raise ValueError('range transfer requires a separate long-focus channel')
        self.near_config = copy.deepcopy(near_config) if include_near else None
        self.near_ids = ([near_config['camera_ids'][e] for e in ('left', 'right')]
                         if include_near else [])
        self.last_near_capture = None
        for camera in self.manifest['cameras']:
            camera['capture_consumer'] = 'dataset' if camera['camera_id'] in self.ids+self.near_ids else 'vision'
        for key in ('settings', 'visuals', 'celestial_bodies', 'objects'):
            if self.manifest[key] != self.original[key]:
                raise ValueError('original environment or geometry changed: '+key)
        self.manifest.update(session_id=self.session, revision='91')
        with (fixture/'render_frames.jsonl').open(encoding='utf-8') as stream:
            self.seed = json.loads(next(stream))
        self.body_readers = {o['object_id']: scene.getBody(o['object_id'].removeprefix('teleop/'))
                             .getOrigin().stateOutMsg.addSubscriber() for o in self.seed['objects']}
        self.hello = read(self.source/'run/capture_attempt_20261022/near_prepared.json')['hello']
        self.hello['session_id'] = self.session
        adapter = self.source.parent/'space_sim_UE_Adapter'
        sys.path.insert(0, str(adapter/'Adapters'))
        from bsk_render_adapter.protocol import RenderPublisher
        from space_arm_platform.capture_receiver import CaptureReceiver
        self.publisher_type = RenderPublisher
        self.received, self.errors = queue.Queue(), []
        self.receiver = CaptureReceiver('127.0.0.1', 0,
            lambda m, p: self.received.put((m, p)), self.errors.append)
        self.project = adapter/'Unreal/BskUnrealRenderer'
        if read(self.project/'Config/bsk_unreal_scene.json') != read(fixture/'renderer_config.json'):
            raise ValueError('original renderer configuration has changed')
        self.process = self.browser = self.publisher = None
        self.index = 0
        self.trace = (self.output/'render_frames.jsonl').open('w', encoding='utf-8', buffering=1)
        self.captures = (self.output/'captures.jsonl').open('w', encoding='utf-8', buffering=1)
        self.image_dir = self.output/'images'; self.image_dir.mkdir()
        for name, data in [('calibration', self.config), ('manifest', self.manifest), ('optics', self.optics)]:
            (self.output/(name+'.json')).write_text(json.dumps(data, indent=2), encoding='utf-8')
        (self.output/'camera_mount_changes.json').write_text(
            json.dumps(camera_mount_changes, indent=2), encoding='utf-8')
        if self.near_config is not None:
            (self.output/'near_calibration.json').write_text(json.dumps(self.near_config,indent=2),encoding='utf-8')

    def start(self, first_frame):
        self.receiver.start()
        deadline = time.monotonic()+10
        while self.receiver._listener is None:
            if time.monotonic() > deadline: raise TimeoutError('capture receiver startup')
            time.sleep(.05)
        capture_port = self.receiver._listener.getsockname()[1]
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', 0)); port = sock.getsockname()[1]
        streamer = 'ImageDynamics'+str(os.getpid())
        command = [r'C:/Program Files/Epic Games/UE_5.6/Engine/Binaries/Win64/UnrealEditor-Cmd.exe',
            str(self.project/'BskUnrealRenderer.uproject'), '-game', '-unattended', '-nop4', '-nosplash',
            '-RenderOffscreen', '-ForceRes', '-ResX=1280', '-ResY=800', '-BskListen=127.0.0.1',
            f'-BskPort={port}', '-BskCaptureHost=127.0.0.1', f'-BskCapturePort={capture_port}',
            '-BskCaptureProducts=rgb', '-BskCaptureRate=10', '-ExecCmds=t.MaxFPS 30,r.VSync 0',
            '-DDC=InstalledNoZenLocalFallback', f'-abslog={self.output/"renderer.log"}',
            '-PixelStreamingUseMediaCapture=false', '-PixelStreamingDecoupleFramerate=false',
            '-PixelStreamingConnectionURL=ws://127.0.0.1:18888', '-PixelStreamingID='+streamer,
            '-BskPixelStreamingURL=ws://127.0.0.1:18888', '-BskPixelStreamingBaseId='+streamer,
            '-BskPixelStreamingCameras=teleop/camera/capture_stereo_left+teleop/camera/capture_stereo_right',
            '-BskPixelStreamingCameraWidth=1920', '-BskPixelStreamingCameraHeight=1920',
            '-BskPixelStreamingCameraFps=10', '-PixelStreamingWebRTCFps=30', '-PixelStreamingEncoderMinQuality=60']
        (self.output/'command.json').write_text(json.dumps(command, indent=2))
        env = os.environ.copy(); env['UE-LocalDataCachePath'] = str(self.source/'run/ddc_capture26')
        self.process = subprocess.Popen(command, cwd=self.project, env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, creationflags=subprocess.CREATE_NO_WINDOW)
        (self.output/'process.json').write_text(json.dumps(dict(pid=self.process.pid, publisher_pid=os.getpid(), port=port)))
        deadline = time.monotonic()+240
        while True:
            self._healthy()
            try:
                with socket.create_connection(('127.0.0.1', port), timeout=.2): break
            except OSError:
                if time.monotonic() > deadline: raise TimeoutError('private renderer startup')
                time.sleep(.2)
        self.publisher = self.publisher_type(port=port)
        self.publisher.retain_hello(self.hello); self.publisher.retain_manifest(self.manifest)
        for _ in range(60): self.send(first_frame, capture=False); time.sleep(1/30)
        self.browser = subprocess.Popen(['node', str(self.source/'run/browser_probe_20261024/open_far_vision_lights_20261027.mjs'),
            str(self.output), streamer], cwd=self.source,
            stdout=(self.output/'browser.out.log').open('wb'), stderr=(self.output/'browser.err.log').open('wb'),
            creationflags=subprocess.CREATE_NO_WINDOW)
        deadline = time.monotonic()+210
        while not (self.output/'native_work_lights_ready.json').exists():
            self._healthy()
            if self.browser.poll() is not None: raise RuntimeError('original work light setup failed')
            if time.monotonic() > deadline: raise TimeoutError('original work light startup')
            self.send(first_frame, capture=False); time.sleep(.2)

    def _healthy(self):
        if self.process is not None and self.process.poll() is not None:
            raise RuntimeError('private renderer exited')
        if self.errors: raise RuntimeError(str(self.errors[-1]))

    def render_frame(self, nanos):
        """Truth -> renderer only. Never return body transforms to navigation."""
        from Basilisk.utilities import RigidBodyKinematics as rbk
        frame = copy.deepcopy(self.seed)
        origin = np.array(self.body_readers['teleop/cubesat_bus']().r_BN_N)
        frame.update(session_id=self.session, manifest_revision='91', sim_time_ns=str(nanos),
                     origin_N_m=origin.tolist(), c_LN=np.eye(3).ravel().tolist())
        for obj in frame['objects']:
            state = self.body_readers[obj['object_id']]()
            obj['position_m'] = (np.array(state.r_BN_N)-origin).tolist()
            obj['orientation_wxyz'] = Rotation.from_matrix(rbk.MRP2C(state.sigma_BN).T).as_quat(scalar_first=True).tolist()
        for i, celestial in enumerate(frame['celestial_bodies']):
            payload = self.orbit.ephemeris.planetStateOutMsgs[i].read()
            celestial['position_m'] = (np.array(payload.PositionVector)-origin).tolist()
            celestial['velocity_mps'] = list(payload.VelocityVector)
            celestial['orientation_wxyz'] = Rotation.from_matrix(
                np.array(payload.J20002Pfix).reshape(3, 3).T).as_quat(scalar_first=True).tolist()
        return frame

    def send(self, frame, *, capture=True):
        current = copy.deepcopy(frame)
        current.pop('vision_capture_requested', None); current.pop('vision_capture_camera_ids', None)
        current.update(frame_id=str(self.index), wall_time_ns=str(time.time_ns()),
                       capture_episode_id=self.session if capture else '', capture_request_id='dynamics-image-feedback')
        self.index += 1
        self.publisher.publish_frame(current)
        if not capture:
            self.trace.write(json.dumps(current)+'\n')
            return None
        pairs = {}; deadline = time.monotonic()+90
        all_ids = self.ids+self.near_ids
        while len(pairs) < len(all_ids):
            self._healthy()
            if time.monotonic() > deadline: raise TimeoutError('synchronized actual JPEG pair')
            try: meta, products = self.received.get(timeout=.1)
            except queue.Empty: continue
            expected = dict(session_id=self.session, source_frame_id=current['frame_id'],
                            sim_time_ns=current['sim_time_ns'], capture_episode_id=self.session)
            if any(str(meta.get(k)) != str(v) for k, v in expected.items()):
                raise ValueError('capture timestamp/session/frame mismatch')
            cid = meta['camera_id']
            if cid not in all_ids or cid in pairs: raise ValueError('unexpected or duplicate camera image')
            is_near = cid in self.near_ids
            eye = ('left', 'right')[(self.near_ids if is_near else self.ids).index(cid)]
            # Check actual renderer intrinsics. Renderer extrinsics stay in
            # the audit file and never enter the image estimator/converter.
            config = self.near_config if is_near else self.config
            intr = meta['intrinsics']; k = config['calibration']['K_'+eye]
            np.testing.assert_allclose([intr['fx_px'], intr['fy_px'], intr['cx_px'], intr['cy_px']],
                                       [k[0][0], k[1][1], k[0][2], k[1][2]], atol=1e-5)
            blob = products['rgb']
            (self.image_dir/(current['frame_id']+('_near_' if is_near else '_')+eye+'.jpg')).write_bytes(blob)
            pairs[cid] = blob
            self.captures.write(json.dumps(meta)+'\n')
        self.trace.write(json.dumps(current)+'\n')
        def group(ids):
            return (dict(session_id=self.session, source_frame_id=int(current['frame_id']),
                         sim_time_ns=int(current['sim_time_ns']), camera_ids=ids),
                    {eye:pairs[cid] for eye,cid in zip(('left','right'),ids)})
        self.last_near_capture = group(self.near_ids) if self.near_ids else None
        return group(self.ids)

    def close(self):
        (self.output/'stop_browser').touch()
        if self.browser:
            try: self.browser.wait(timeout=10)
            except subprocess.TimeoutExpired: self.browser.terminate(); self.browser.wait(timeout=10)
        if self.publisher: self.publisher.close()
        if self.process and self.process.poll() is None:
            self.process.terminate()
            try: self.process.wait(timeout=15)
            except subprocess.TimeoutExpired: self.process.kill(); self.process.wait(timeout=10)
        self.receiver.close(); self.trace.close(); self.captures.close()
        (self.output/'scope.json').write_text(json.dumps(dict(
            source_environment_unchanged=True, running_scene_runtime_modified=False,
            camera_mount_changes=self.camera_mount_changes,
            light_parameters_modified=False, extra_camera_work_lights_created=False,
            physics_waits_for_actual_images=True, real_time_latency_validated=False,
            source_renderer_config_sha256=hashlib.sha256((self.project/'Config/bsk_unreal_scene.json').read_bytes()).hexdigest()
        ), indent=2))
