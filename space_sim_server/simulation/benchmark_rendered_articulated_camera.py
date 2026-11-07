"""Owned rendered camera acquisition from the actual moving capture gripper.

The environment must prepare the full physical model as model/team_flight.xml
(the shared renderer mirror path), use that exact XML in LocalMujocoStepper,
and provide fresh body publications at every image sample. This component does
not create a capture task or a method observation containing private poses.
"""
from __future__ import annotations

from dataclasses import replace
import hashlib

from space_arm_platform.benchmark_articulated_camera import (
    MountedCaptureSequence, admit_mounted_camera_image,
)
from space_arm_platform.benchmark_execution import write_json
from simulation.benchmark_camera_mount import NativeCameraMountReader
from simulation.benchmark_rendered_camera import RenderedCameraSensor


class ArticulatedRenderedCameraSensor(RenderedCameraSensor):
    def __init__(self, environment, cameras, *, epoch_ns=0, warmup_wall_s=10.):
        self.mount_reader = NativeCameraMountReader(environment._assembly.ports, cameras,
            episode_id=environment.episode_id, epoch_ns=epoch_ns)
        self.sequence = MountedCaptureSequence(cameras, episode_id=environment.episode_id,
            model_xml_sha256=environment._stepper.model_xml_sha256, epoch_ns=epoch_ns)
        model = environment.episode_directory/'model/team_flight.xml'
        if hashlib.sha256(model.read_bytes()).hexdigest() != environment._stepper.model_xml_sha256:
            raise ValueError('articulated renderer XML must match the actually loaded native model')
        self._mounts = self._receipts = None
        super().__init__(environment, cameras, warmup_wall_s=warmup_wall_s)
        write_json(self.output/'moving_mount.json', dict(
            schema_version='spacesim-autonomy-mounted-renderer/1',
            model_xml_sha256=environment._stepper.model_xml_sha256,
            episode_id=environment.episode_id, epoch_ns=epoch_ns,
            sampling_period_ns=self.sequence.period_ns,
            reference='actual native named-body origin at exposure; not assembly COM or fixed bus',
            public_inputs='image bytes and fixed nominal camera calibration; no runtime transforms',
            scope='camera component; whole capture task acceptance is separate'))

    def _attachment_file(self):
        return 'config/visual_attachments/capture26_near23/geometry_metres.json'

    def _attachment_sha256(self):
        return 'e55cd66ad570ec9129cb344f22f506294ce2a0ae7ea70952052346feecd10a4f'

    def _camera_visual(self, camera):
        descriptor = super()._camera_visual(camera)
        return replace(descriptor, parent_id='team/'+camera.parent_body_name,
            exposure_time_s=camera.photometric_exposure_time_s, aperture_fnumber=camera.aperture_fnumber,
            work_light_profile='' if camera.work_light_profile == 'none' else camera.work_light_profile,
            display_name='Benchmark moving gripper camera')

    def _sample_period_ns(self):
        return self.sequence.period_ns

    def _public_image(self, raw, products, camera, source_frame, time_ns):
        index = self.cameras.index(camera)
        image, receipt = admit_mounted_camera_image(raw, products, camera, self._mounts[index],
            session_id=self.bridge.session_id, source_frame_id=source_frame, sample_time_ns=time_ns)
        self._receipts[camera.camera_id] = receipt
        return image

    def capture(self, time_ns):
        if self.sequence.failed:
            raise ValueError('moving-camera sequence failed; reset the environment')
        try:
            self._mounts = self.mount_reader.sample(time_ns)
            self._receipts = {}
            images, _private_packets = super().capture(time_ns)
            # Rendering must not advance native time or mutate articulated state.
            if self.mount_reader.sample(time_ns) != self._mounts:
                raise ValueError('physical moving mount changed while rendering an exposure')
            receipts = tuple(self._receipts[camera.camera_id] for camera in self.cameras)
            self.sequence.add(images, receipts)
        except BaseException:
            self.sequence.failed = True
            raise
        # The second result is explicitly private; the environment stores it in
        # physics evidence, while constructing method observations from images.
        return images, receipts
