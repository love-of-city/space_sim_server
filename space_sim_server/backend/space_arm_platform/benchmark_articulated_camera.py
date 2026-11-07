"""Moving camera calibration, public pixels and private exposure correspondence.

No native runtime or vision detector is imported. Body origins refer to the
authored MuJoCo body frame, not its centre of mass. Runtime transforms belong
only to the private receipt; a method receives pixels and fixed calibration.
"""
from __future__ import annotations

import base64
import hashlib
import json
import math
from pathlib import Path
from typing import Annotated, Literal

import numpy as np
from pydantic import Field, field_validator, model_validator
from scipy.spatial.transform import Rotation

from .benchmark_attitude import Quaternion, Tensor3, unit_quaternion
from .benchmark_camera import (
    CameraImage, PinholeCamera, Sha256, admit_pinhole_pixels, validate_image_pixels,
)
from .benchmark_contract import Contract, Count, Number, Vector3

MAX_ARTICULATED_PNG_BYTES = 16_000_000
MAX_ARTICULATED_SIDE = 1920
FLU_TO_OPTICAL = np.array([[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]])
POSITION_TOLERANCE_M = 1e-5
ORIENTATION_TOLERANCE_RAD = 1e-7
BodyName = Annotated[str, Field(pattern=r'^[A-Za-z_][A-Za-z0-9_.-]{0,127}$')]
CAPTURE26_CAMERA_CONFIG = 'config/vision/capture26_near23_front_mount_20261029.json'


def _rotation(quaternion):
    return Rotation.from_quat(np.asarray(quaternion)[[1, 2, 3, 0]]).as_matrix()


def _angle(first, second):
    relative = first @ second.T
    sine = np.linalg.norm([relative[2, 1]-relative[1, 2], relative[0, 2]-relative[2, 0],
                           relative[1, 0]-relative[0, 1]])/2
    return math.atan2(float(sine), float((np.trace(relative)-1)/2))


class MountedPinholeCamera(PinholeCamera):
    """Optics and fixed mount; no live body/target pose is public calibration."""
    schema_version: Literal['spacesim-autonomy-mounted-camera/1'] = 'spacesim-autonomy-mounted-camera/1'
    width: Annotated[int, Field(strict=True, ge=1, le=MAX_ARTICULATED_SIDE)]
    height: Annotated[int, Field(strict=True, ge=1, le=MAX_ARTICULATED_SIDE)]
    parent_body_name: BodyName
    mount_reference: Literal['named_body_origin_not_com'] = 'named_body_origin_not_com'
    calibration_status: Literal['nominal_simulation_not_measured_hardware_calibration'] = (
        'nominal_simulation_not_measured_hardware_calibration')
    # UE uses these for brightness. The image remains an instantaneous render;
    # there is no 2 ms motion-blur integration or simulated hardware shutter.
    photometric_exposure_time_s: Annotated[Number, Field(ge=0, le=1)] = .002
    aperture_fnumber: Annotated[Number, Field(ge=1, le=64)] = 4.
    work_light_profile: Literal['none', 'inspection-1500cd-v1'] = 'none'


class MountedCameraImage(CameraImage):
    schema_version: Literal['spacesim-autonomy-mounted-camera-image/1'] = 'spacesim-autonomy-mounted-camera-image/1'
    width: Annotated[int, Field(strict=True, ge=1, le=MAX_ARTICULATED_SIDE)]
    height: Annotated[int, Field(strict=True, ge=1, le=MAX_ARTICULATED_SIDE)]
    png_base64: str = Field(min_length=60, max_length=4*((MAX_ARTICULATED_PNG_BYTES+2)//3))

    @model_validator(mode='after')
    def intact_pixels(self):
        validate_image_pixels(self, maximum_width=MAX_ARTICULATED_SIDE, maximum_height=MAX_ARTICULATED_SIDE,
                              maximum_bytes=MAX_ARTICULATED_PNG_BYTES)
        return self


def capture26_mounted_cameras(repository):
    """Use the original gripper stereo calibration, only changing the namespace.

    Image size, sampling, FLU mount, photometric exposure and declared work light
    are preserved. The target's passive markings are specified separately.
    """
    config = json.loads((Path(repository)/CAPTURE26_CAMERA_CONFIG).read_text('utf-8'))
    if (config.get('schema') != 'capture26-stereo-vision/1'
            or config.get('calibration_status') != 'nominal_simulation_not_measured_hardware_calibration'
            or config.get('hardware', {}).get('measured_hardware_calibration') is not False
            or len(config.get('cameras', [])) != 2):
        raise ValueError('expected the original nominal capture26 stereo design')
    cameras = []
    for side, raw in zip(('left', 'right'), config['cameras'], strict=True):
        expected = 'teleop/camera/capture_stereo_'+side
        if (raw['parent_id'] != 'teleop/gripper' or raw['camera_id'] != expected
                or config['camera_ids'][side] != expected or raw['capture_rate_hz'] != config['trigger_rate_hz']
                or raw['capture_products'] != ['rgb'] or raw['capture_consumer'] != 'vision'):
            raise ValueError('capture26 stereo camera identity, mount or cadence changed')
        unit_quaternion(raw['orientation_body_from_camera_wxyz'])
        optical = _rotation(raw['orientation_body_from_camera_wxyz'])@FLU_TO_OPTICAL.T
        q = Rotation.from_matrix(optical).as_quat()[[3, 0, 1, 2]]
        camera = MountedPinholeCamera(camera_id=expected.replace('teleop/', 'team/', 1),
            parent_body_name='gripper', width=raw['resolution'][0], height=raw['resolution'][1],
            horizontal_fov_rad=raw['field_of_view_rad'], position_body_m=tuple(raw['position_body_m']),
            orientation_body_from_optical_wxyz=tuple(q), sample_rate_hz=raw['capture_rate_hz'],
            photometric_exposure_time_s=raw['exposure_time_s'], aperture_fnumber=raw['aperture_fnumber'],
            work_light_profile=raw.get('work_light_profile', 'none'))
        if (not np.allclose(camera.intrinsics, config['calibration']['K_'+side], rtol=0, atol=1e-10)
                or config['calibration']['image_size'] != [camera.width, camera.height]
                or config['calibration']['D_'+side] != [0, 0, 0, 0, 0]):
            raise ValueError('capture26 stereo intrinsics differ from the declared render optics')
        cameras.append(camera)
    left, right = cameras
    right_rotation = _rotation(right.orientation_body_from_optical_wxyz)
    relative_rotation = right_rotation.T@_rotation(left.orientation_body_from_optical_wxyz)
    relative_position = right_rotation.T@(np.asarray(left.position_body_m)-right.position_body_m)
    if (not np.allclose(relative_rotation, config['calibration']['R_right_from_left'], atol=1e-10, rtol=0)
            or not np.allclose(relative_position, config['calibration']['t_right_from_left_m'], atol=1e-10, rtol=0)):
        raise ValueError('capture26 stereo baseline differs from the original physical mounts')
    return tuple(cameras)


class CameraMountSample(Contract):
    """Private, same-epoch publication from the physical articulated body."""
    schema_version: Literal['spacesim-autonomy-camera-mount-sample/1'] = 'spacesim-autonomy-camera-mount-sample/1'
    episode_id: str = Field(min_length=1, max_length=128)
    model_xml_sha256: Sha256
    parent_body_name: BodyName
    epoch_ns: Count
    physics_time_ns: Count
    sample_time_ns: Count
    native_step_count: Count
    external_state_writes: Count
    origin_j2000_m: Vector3
    orientation_j2000_from_body_wxyz: Quaternion

    _unit = field_validator('orientation_j2000_from_body_wxyz')(unit_quaternion)

    @model_validator(mode='after')
    def physical_clock(self):
        if self.sample_time_ns != self.epoch_ns+self.physics_time_ns:
            raise ValueError('mount exposure epoch differs from its physical publication')
        if self.external_state_writes:
            raise ValueError('camera mount evidence contains post-initialization state assignment')
        return self


class MountedCameraCaptureEvidence(Contract):
    """Stored privately alongside physics; never passed to an external method."""
    record_type: Literal['mounted_camera_capture'] = 'mounted_camera_capture'
    schema_version: Literal['spacesim-autonomy-mounted-camera-evidence/1'] = 'spacesim-autonomy-mounted-camera-evidence/1'
    camera_id: str = Field(min_length=1, max_length=256)
    calibration_sha256: Sha256
    image_sha256: Sha256
    renderer_session_id: str = Field(min_length=1, max_length=128)
    source_frame_id: Count
    sample_time_ns: Count
    mount: CameraMountSample
    renderer_camera_origin_j2000_m: Vector3
    renderer_orientation_j2000_from_flu: Tensor3

    @model_validator(mode='after')
    def physical_rotation_and_epoch(self):
        matrix = np.asarray(self.renderer_orientation_j2000_from_flu)
        if (not np.allclose(matrix.T@matrix, np.eye(3), atol=1e-8, rtol=0)
                or not math.isclose(float(np.linalg.det(matrix)), 1., abs_tol=1e-8, rel_tol=0)):
            raise ValueError('renderer camera axes must form a right-handed orthonormal frame')
        if self.sample_time_ns != self.mount.sample_time_ns:
            raise ValueError('camera image and moving mount were not sampled at the same epoch')
        return self


def physical_mounted_camera(camera: MountedPinholeCamera, mount: CameraMountSample):
    camera, mount = MountedPinholeCamera.model_validate(camera), CameraMountSample.model_validate(mount)
    if camera.parent_body_name != mount.parent_body_name:
        raise ValueError('camera is attached to a different physical body')
    rotation = _rotation(mount.orientation_j2000_from_body_wxyz)
    return (np.asarray(mount.origin_j2000_m)+rotation@camera.position_body_m,
            rotation@_rotation(camera.orientation_body_from_optical_wxyz))


def verify_mounted_camera_capture(camera, image, receipt):
    """Pure regrade of a stored exposure. Run/asset authenticity is bound outside."""
    camera = MountedPinholeCamera.model_validate(camera)
    image = MountedCameraImage.model_validate(image)
    receipt = MountedCameraCaptureEvidence.model_validate(receipt)
    if (receipt.camera_id != camera.camera_id or image.camera_id != camera.camera_id
            or receipt.calibration_sha256 != camera.fingerprint()
            or receipt.image_sha256 != image.png_sha256
            or receipt.renderer_session_id != image.renderer_session_id
            or receipt.source_frame_id != image.source_frame_id or receipt.sample_time_ns != image.sample_time_ns
            or (image.width, image.height) != (camera.width, camera.height)):
        raise ValueError('image, moving camera calibration and private exposure receipt differ')
    position, optical = physical_mounted_camera(camera, receipt.mount)
    actual_optical = np.asarray(receipt.renderer_orientation_j2000_from_flu) @ FLU_TO_OPTICAL.T
    if (np.linalg.norm(position-receipt.renderer_camera_origin_j2000_m) > POSITION_TOLERANCE_M
            or _angle(optical, actual_optical) > ORIENTATION_TOLERANCE_RAD):
        raise ValueError('rendered camera pose differs from the physical moving mount at exposure')


def admit_mounted_camera_image(metadata, products, camera, mount, *, session_id, source_frame_id, sample_time_ns):
    """Admit actual rendered pixels plus a separate private mount receipt."""
    camera = MountedPinholeCamera.model_validate(camera)
    mount = CameraMountSample.model_validate(mount)
    png = admit_pinhole_pixels(metadata, products, camera, session_id=session_id,
        source_frame_id=source_frame_id, sample_time_ns=sample_time_ns,
        photometric_exposure_time_s=camera.photometric_exposure_time_s, aperture_fnumber=camera.aperture_fnumber,
        maximum_width=MAX_ARTICULATED_SIDE, maximum_height=MAX_ARTICULATED_SIDE,
        maximum_bytes=MAX_ARTICULATED_PNG_BYTES)
    image = MountedCameraImage(camera_id=camera.camera_id, renderer_session_id=session_id,
        source_frame_id=source_frame_id, sample_time_ns=sample_time_ns, width=camera.width, height=camera.height,
        png_sha256=hashlib.sha256(png).hexdigest(), png_base64=base64.b64encode(png).decode('ascii'))
    extrinsics = metadata.get('extrinsics')
    if not isinstance(extrinsics, dict) or extrinsics.get('camera_axes') != '+X_forward,+Y_left,+Z_up':
        raise ValueError('renderer must explicitly identify its private camera FLU axes')
    flat = extrinsics.get('c_NC')
    if (not isinstance(flat, list) or len(flat) != 9
            or any(type(value) not in (int, float) or not math.isfinite(value) for value in flat)):
        raise ValueError('renderer camera transform must contain nine finite row-major numbers')
    receipt = MountedCameraCaptureEvidence(camera_id=camera.camera_id, calibration_sha256=camera.fingerprint(),
        image_sha256=image.png_sha256, renderer_session_id=session_id, source_frame_id=source_frame_id,
        sample_time_ns=sample_time_ns, mount=mount,
        renderer_camera_origin_j2000_m=extrinsics.get('position_N_m'),
        renderer_orientation_j2000_from_flu=tuple(tuple(flat[i:i+3]) for i in (0, 3, 6)))
    verify_mounted_camera_capture(camera, image, receipt)
    return image, receipt


class MountedCaptureSequence:
    """A single episode's ordered rig receipts, with rejection latched on error."""
    def __init__(self, cameras, *, episode_id, model_xml_sha256, epoch_ns):
        self.cameras = tuple(MountedPinholeCamera.model_validate(camera) for camera in cameras)
        if (not self.cameras or len(self.cameras) > 2
                or len({camera.camera_id for camera in self.cameras}) != len(self.cameras)
                or len({camera.sample_rate_hz for camera in self.cameras}) != 1
                or 1_000_000_000 % self.cameras[0].sample_rate_hz):
            raise ValueError('moving rig requires one or two distinct cameras on one integer sampling grid')
        # Validate identity even before the first sample arrives.
        CameraMountSample(episode_id=episode_id, model_xml_sha256=model_xml_sha256,
            parent_body_name=self.cameras[0].parent_body_name, epoch_ns=epoch_ns, physics_time_ns=0,
            sample_time_ns=epoch_ns, native_step_count=0, external_state_writes=0,
            origin_j2000_m=(0., 0., 0.), orientation_j2000_from_body_wxyz=(1., 0., 0., 0.))
        self.episode_id, self.model_xml_sha256, self.epoch_ns = episode_id, model_xml_sha256, epoch_ns
        self.period_ns = 1_000_000_000//self.cameras[0].sample_rate_hz
        self.count = 0
        self.last_frame = self.session = self.last_steps = None
        self.failed = False

    def add(self, images, receipts):
        if self.failed:
            raise ValueError('moving-camera evidence sequence is invalid; start a new episode')
        try:
            images = tuple(MountedCameraImage.model_validate(image) for image in images)
            receipts = tuple(MountedCameraCaptureEvidence.model_validate(receipt) for receipt in receipts)
            if len(images) != len(self.cameras) or len(receipts) != len(self.cameras):
                raise ValueError('moving-camera rig has missing or extra exposures')
            first = receipts[0]
            expected = self.epoch_ns+self.count*self.period_ns
            bodies = {}
            for camera, image, receipt in zip(self.cameras, images, receipts, strict=True):
                verify_mounted_camera_capture(camera, image, receipt)
                mount = receipt.mount
                if (receipt.sample_time_ns != expected or mount.epoch_ns != self.epoch_ns
                        or mount.episode_id != self.episode_id or mount.model_xml_sha256 != self.model_xml_sha256
                        or receipt.source_frame_id != first.source_frame_id
                        or receipt.renderer_session_id != first.renderer_session_id
                        or mount.native_step_count != first.mount.native_step_count):
                    raise ValueError('moving-camera rig is stale, mixed-session or not simultaneous')
                if mount.parent_body_name in bodies and bodies[mount.parent_body_name] != mount:
                    raise ValueError('cameras on one body disagree about their physical mount publication')
                bodies[mount.parent_body_name] = mount
            if (self.session is not None and (first.renderer_session_id != self.session
                    or first.source_frame_id != self.last_frame+1
                    or first.mount.native_step_count <= self.last_steps)):
                raise ValueError('moving-camera clock, physical integration or source frame did not advance')
        except (TypeError, ValueError):
            self.failed = True
            raise
        self.count += 1
        self.session, self.last_frame = first.renderer_session_id, first.source_frame_id
        self.last_steps = first.mount.native_step_count
