"""Owned UE image acquisition at the native simulation clock.

No OpenCV import in this process. Only fixed camera calibration and validated
PNG packets cross the method boundary; raw renderer transforms remain private.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import queue
import shutil
import socket
import time

import numpy as np
from scipy.spatial.transform import Rotation

from bsk_render_adapter import BasiliskRenderBridge, CameraVisual
from space_arm_platform.benchmark_camera import public_camera_image
from space_arm_platform.benchmark_execution import write_json
from space_arm_platform.benchmark_vision_resources import (
    graphics_identity, mirror_renderer_model, new_renderer_user_directory, renderer_paths, selected_graphics_identity,
)
from space_arm_platform.capture_receiver import CaptureReceiver
from space_arm_platform.capture_regression_runtime import ManagedProcesses, ensure_ports_free, wait_ready
from simulation.orbital_rendezvous.rendering import flight_scene_settings
from simulation.teleop_grasp_unreal import _register_celestial_bodies
from simulation.visual_attachments import attach_visual_geometries

FLU_TO_OPTICAL = np.array([[0.,-1.,0.],[0.,0.,-1.],[1.,0.,0.]])


def unused_ports(count):
    reservations=[]
    try:
        for _ in range(count):
            sock=socket.socket()
            sock.bind(('127.0.0.1',0))
            reservations.append(sock)
        return [sock.getsockname()[1] for sock in reservations]
    finally:
        for sock in reservations:
            sock.close()


class RenderedCameraSensor:
    def __init__(self,environment,cameras,*,warmup_wall_s=10.):
        self.environment,self.cameras=environment,tuple(cameras)
        self.output=environment.episode_directory/'rendered_sensor'
        self.output.mkdir(exist_ok=False)
        self.queue=queue.Queue(maxsize=8)
        self.bridge=self.receiver=self.renderer=None
        self.runtime_log=None
        self.managed=ManagedProcesses(self.output)
        self.last_time_ns=None
        self._log_offset=0
        self._log_tail=b''
        self._closed=False
        self._metadata=(self.output/'private_renderer_packets.jsonl').open('w',encoding='utf-8',newline='\n')
        try:
            self._start(warmup_wall_s)
        except BaseException:
            self.close()
            raise

    def _start(self,warmup_wall_s):
        env=self.environment
        project,editor=renderer_paths(env.repository)
        user_directory=new_renderer_user_directory(env.episode_directory)
        # UE's log writer does not reliably accept extended/deep Windows paths.
        # Use the same fresh short owner directory as its shader scratch, then
        # archive the completed log under the long Python evidence path.
        self.runtime_log=user_directory/'ue.log'
        render_model,model_sha256=mirror_renderer_model(env.episode_directory,user_directory)
        graphics=graphics_identity()
        self.render_port,self.capture_port=unused_ports(2)
        ensure_ports_free([self.render_port,self.capture_port])
        self.receiver=CaptureReceiver('127.0.0.1',self.capture_port,lambda *args:None,
            on_vision_capture=lambda metadata,products:self.queue.put_nowait((metadata,products)))
        self.receiver.start()
        arguments=[editor,project/'BskUnrealRenderer.uproject','-game','-RenderOffscreen','-ForceRes',
            f'-UserDir={user_directory}',
            '-unattended','-nosplash','-nosound','-ResX=1280','-ResY=720',
            '-BskListen=127.0.0.1',f'-BskPort={self.render_port}','-BskCaptureHost=127.0.0.1',
            f'-BskCapturePort={self.capture_port}','-BskCaptureProducts=rgb','-BskOfflineCapture','-BskVisionFastPng',
            '-ExecCmds=t.MaxFPS 20,r.VSync 0',
            '-DDC=InstalledDerivedDataBackendGraph',f'-abslog={self.runtime_log}']
        cache=os.environ.get('SPACESIM_BENCHMARK_DDC',str(Path(os.environ['LOCALAPPDATA'])/'BskDDC/capture-regression'))
        if cache:
            arguments.append(f'-LocalDataCachePath={Path(cache).resolve()}')
        write_json(self.output/'startup.json',dict(user_directory=str(user_directory),runtime_log=str(self.runtime_log),
            renderer_model=str(render_model),renderer_model_sha256=model_sha256,
            renderer_arguments=[str(value) for value in arguments],graphics_devices=graphics,
            render_port=self.render_port,capture_port=self.capture_port))
        self.renderer=self.managed.start('renderer',arguments,cwd=env.repository)
        wait_ready(self.renderer,self.render_port)
        selected_graphics=selected_graphics_identity(self.runtime_log.read_text('utf-8',errors='replace'),graphics)
        bridge=self.bridge=BasiliskRenderBridge(host='127.0.0.1',port=self.render_port,origin_object='team/cubesat_bus',
            vision_capture_rate_hz=self.cameras[0].sample_rate_hz,
            vision_capture_camera_ids=[camera.camera_id for camera in self.cameras],
            capture_state_provider=lambda:dict(capture_episode_id='',capture_request_id=''),
            reliable_frames=True,reliable_vision_frames=True)
        catalog=project/'Saved/AssetImport/sarm_platform.catalog.json'
        bridge.add_mj_scene(env._scene,namespace='team',source_path=render_model,
            mesh_asset_catalog=catalog,camera_picture_in_picture=False,camera_capture_products=())
        attachment=self._attachment_file()
        attach_visual_geometries(bridge._objects,[dict(parent_id='team/capture_target',geometry_file=attachment,
            sha256=self._attachment_sha256())],env.repository)
        for camera in self.cameras:
            bridge.add_camera(self._camera_visual(camera))
        _register_celestial_bodies(bridge,*env._assembly.extra['celestial'])
        bridge.set_scene_settings(flight_scene_settings(bodies=env.model_manifest['bodies']))
        bridge.Reset(0)
        bridge.vision_requests_enabled=False
        bridge.UpdateState(0)
        deadline=time.monotonic()+warmup_wall_s
        while time.monotonic()<deadline:
            self._check_alive()
            time.sleep(min(.1,max(0.,deadline-time.monotonic())))
        if not self.queue.empty():
            raise RuntimeError('camera warmup unexpectedly produced an authoritative dataset frame')
        bridge.vision_requests_enabled=True
        write_json(self.output/'sensor.json',dict(cameras=[camera.model_dump(mode='json') for camera in self.cameras],
            selected_graphics=selected_graphics,
            warmup=dict(wall_time_s=warmup_wall_s,physical_time_advanced_s=0,recorded_images=0),
            renderer_arguments=[str(value) for value in arguments],render_port=self.render_port,capture_port=self.capture_port,
            catalog_sha256=hashlib.sha256(catalog.read_bytes()).hexdigest(),
            geometry_sha256=hashlib.sha256((env.repository/attachment).read_bytes()).hexdigest()))

    def _attachment_file(self):
        return 'config/visual_attachments/target_aruco_flush_5cm_v2/geometry_metres.json'

    def _attachment_sha256(self):
        return 'ea51a2318029449f02d246e03f0ce6ee5e673c9f766541cc3e136344305e9fc0'

    def _camera_visual(self,camera):
        q=np.asarray(camera.orientation_body_from_optical_wxyz)
        optical=Rotation.from_quat(q[[1,2,3,0]]).as_matrix()
        flu_q=Rotation.from_matrix(optical@FLU_TO_OPTICAL).as_quat()[[3,0,1,2]]
        return CameraVisual(camera_id=camera.camera_id,parent_id='team/cubesat_bus',
            position_body_m=camera.position_body_m,orientation_body_from_camera_wxyz=tuple(flu_q),
            field_of_view_rad=camera.horizontal_fov_rad,resolution=(camera.width,camera.height),
            capture_rate_hz=camera.sample_rate_hz,capture_products=('rgb',),capture_consumer='vision',
            display_name='Benchmark calibrated camera')

    def _sample_period_ns(self):
        return self.environment.task.decision_period_ns

    def _public_image(self,raw,products,camera,source_frame,time_ns):
        return public_camera_image(raw,products,camera,session_id=self.bridge.session_id,
            source_frame_id=source_frame,sample_time_ns=time_ns)

    def _check_alive(self):
        self._check_log()
        if self.renderer.poll() is not None:
            raise RuntimeError('owned benchmark renderer exited')
        status=self.receiver.status()
        if status['last_vision_error'] or status['last_error']:
            raise RuntimeError('camera transport failed: '+str(status))
        if getattr(self.environment,'stop_requested',lambda:False)():
            raise InterruptedError('benchmark camera capture cancelled')

    def _check_log(self):
        path=self.runtime_log
        if path is None or not path.is_file():
            return
        with path.open('rb') as stream:
            stream.seek(self._log_offset)
            data=self._log_tail+stream.read()
            self._log_offset=stream.tell()
        if b'Ensure condition failed' in data or b'Fatal error:' in data:
            raise RuntimeError('owned renderer reported an ensure or fatal error; inspect rendered_sensor/ue.log')
        self._log_tail=data[-64:]

    def capture(self,time_ns):
        if self.last_time_ns is not None and time_ns!=self.last_time_ns+self._sample_period_ns():
            raise ValueError('camera samples must advance on the declared physical decision grid')
        self._check_alive()
        self.bridge.UpdateState(time_ns)
        source_frame=self.bridge.last_published_frame_id
        remaining={camera.camera_id:camera for camera in self.cameras}
        received,metadata={},{}
        deadline=time.monotonic()+120.
        while remaining:
            self._check_alive()
            if time.monotonic()>deadline:
                raise TimeoutError('authoritative camera image was not delivered')
            try:
                raw,products=self.queue.get(timeout=.25)
            except queue.Empty:
                continue
            camera=remaining.pop(raw.get('camera_id'),None)
            if camera is None:
                raise ValueError('duplicate or unexpected camera packet')
            public=self._public_image(raw,products,camera,source_frame,time_ns)
            received[camera.camera_id],metadata[camera.camera_id]=public,raw
            # Human-readable raw evidence is private; the policy receives only
            # the typed public image returned below, never this file path.
            self._metadata.write(json.dumps(raw,allow_nan=False,separators=(',',':'))+'\n')
        self.last_time_ns=time_ns
        self._check_alive()
        return tuple(received[camera.camera_id] for camera in self.cameras),tuple(metadata[camera.camera_id] for camera in self.cameras)

    def close(self):
        if self._closed:
            return
        self._closed=True
        try:
            if self.bridge is not None:
                self.bridge.close()
        finally:
            try:
                if self.receiver is not None:
                    self.receiver.close()
            finally:
                try:
                    self.managed.close()
                finally:
                    self._metadata.close()
                    if self.runtime_log is not None and self.runtime_log.is_file():
                        shutil.copyfile(self.runtime_log,self.output/'ue.log')
        self._check_log()
