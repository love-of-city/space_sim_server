"""Moving-mount correspondence and image boundaries; no rendered success claim."""
import base64
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import struct
import zlib

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from space_arm_platform.benchmark_articulated_camera import (
    CAPTURE26_CAMERA_CONFIG, FLU_TO_OPTICAL, CameraMountSample, MountedCameraCaptureEvidence,
    MountedCameraImage, MountedCaptureSequence, MountedPinholeCamera, admit_mounted_camera_image,
    capture26_mounted_cameras, verify_mounted_camera_capture,
)
from space_arm_platform.benchmark_camera import CameraImage, PNG_SIGNATURE, inspect_rgb_png

SERVER = Path(__file__).resolve().parents[1]


def chunk(kind, data):
    return struct.pack('!I', len(data))+kind+data+struct.pack('!I', zlib.crc32(kind+data)&0xffffffff)


def png(width, height):
    raw = (b'\0'+bytes([23, 41, 89])*width)*height
    return (PNG_SIGNATURE+chunk(b'IHDR', struct.pack('!IIBBBBB', width, height, 8, 2, 0, 0, 0))
            +chunk(b'IDAT', zlib.compress(raw))+chunk(b'IEND', b''))


def exposure(*, index=0, side='left', width=3, height=2, body_angle=.4):
    mount_rotation = Rotation.from_rotvec([.2, body_angle, -.3])
    optical_rotation = Rotation.from_euler('xyz', [.1, -.2, .6])
    camera = MountedPinholeCamera(camera_id='team/camera/'+side, parent_body_name='gripper',
        width=width, height=height, horizontal_fov_rad=1.2, position_body_m=(.075, .06 if side=='left' else -.06, .031),
        orientation_body_from_optical_wxyz=tuple(optical_rotation.as_quat()[[3, 0, 1, 2]]), sample_rate_hz=10)
    mount = CameraMountSample(episode_id='episode-a', model_xml_sha256='a'*64, parent_body_name='gripper',
        epoch_ns=8_000_000_000_000, physics_time_ns=index*100_000_000,
        sample_time_ns=8_000_000_000_000+index*100_000_000, native_step_count=index*40,
        external_state_writes=0, origin_j2000_m=(6_900_001.+index*.003, -12., 45.),
        orientation_j2000_from_body_wxyz=tuple(mount_rotation.as_quat()[[3, 0, 1, 2]]))
    # Independently compose the two authored transforms, using a nonzero lever
    # arm and noncommuting rotations. A fixed bus or an optical/FLU mixup fails.
    position = np.array(mount.origin_j2000_m)+mount_rotation.apply(camera.position_body_m)
    flu = (mount_rotation*optical_rotation).as_matrix()@FLU_TO_OPTICAL
    k = camera.intrinsics
    raw = dict(protocol='bsk-capture/1', type='camera_frame', stream_kind='vision', state_kind='authoritative',
        sensor_model='render_pinhole', color_model='rgb', camera_mode='rgb', session_id='renderer-a',
        camera_id=camera.camera_id, source_frame_id=str(index+8), sim_time_ns=str(mount.sample_time_ns),
        resolution=[width, height], temporal_exposure_integration=False, exposure_semantics='render_photometric_only',
        exposure_time_s=.002, aperture_fnumber=4, sampling_fps=10, sampling_clock='simulation',
        intrinsics=dict(model='pinhole', distortion_model='none', fx_px=k[0][0], fy_px=k[1][1], cx_px=k[0][2],
            cy_px=k[1][2], fov_x_rad=1.2, fov_y_rad=2*math.atan(height/(2*k[1][1]))),
        extrinsics=dict(camera_axes='+X_forward,+Y_left,+Z_up', position_N_m=position.tolist(), c_NC=flu.ravel().tolist()),
        private_truth='NEVER_POLICY_INPUT', floating_origin=dict(origin_N_m=[6_900_000., 0., 0.]))
    return camera, mount, raw, {'rgb': png(width, height)}


def admit(value):
    camera, mount, raw, products = value
    return admit_mounted_camera_image(raw, products, camera, mount, session_id='renderer-a',
        source_frame_id=int(raw['source_frame_id']), sample_time_ns=mount.sample_time_ns)


def sequence(cameras):
    return MountedCaptureSequence(cameras, episode_id='episode-a', model_xml_sha256='a'*64,
                                  epoch_ns=8_000_000_000_000)


def test_public_moving_image_contains_pixels_but_no_runtime_pose_or_body_truth():
    value = exposure()
    image, receipt = admit(value)
    assert image.png_bytes() == value[3]['rgb']
    assert image.png_sha256 == hashlib.sha256(value[3]['rgb']).hexdigest()
    assert MountedCameraImage.model_validate_json(image.model_dump_json()) == image
    assert MountedCameraCaptureEvidence.model_validate_json(receipt.model_dump_json()) == receipt
    assert set(image.model_dump()) == {'schema_version', 'camera_id', 'renderer_session_id', 'source_frame_id',
        'sample_time_ns', 'width', 'height', 'png_sha256', 'png_base64'}
    assert not any(word in image.model_dump_json() for word in ('NEVER_POLICY', 'origin_j2000', '"mount":', 'position_N'))
    verify_mounted_camera_capture(value[0], image, receipt)
    with pytest.raises(ValueError):
        MountedCameraImage.model_validate(dict(image.model_dump(), mount=receipt.mount.model_dump()))


def test_original_capture26_calibration_keeps_resolution_baseline_exposure_and_lamp():
    cameras = capture26_mounted_cameras(SERVER)
    raw = json.loads((SERVER/CAPTURE26_CAMERA_CONFIG).read_text('utf-8'))
    assert len(cameras) == 2 and all(camera.parent_body_name == 'gripper' for camera in cameras)
    assert all((camera.width, camera.height, camera.sample_rate_hz) == (1920, 1920, 10) for camera in cameras)
    assert all(camera.photometric_exposure_time_s == .002 and camera.aperture_fnumber == 4 for camera in cameras)
    assert [camera.work_light_profile for camera in cameras] == ['none', 'inspection-1500cd-v1']
    assert math.isclose(math.dist(cameras[0].position_body_m, cameras[1].position_body_m), .12, abs_tol=1e-15)
    for camera, original in zip(cameras, raw['cameras'], strict=True):
        optical = Rotation.from_quat(np.array(camera.orientation_body_from_optical_wxyz)[[1, 2, 3, 0]])
        flu = Rotation.from_quat(np.array(original['orientation_body_from_camera_wxyz'])[[1, 2, 3, 0]])
        assert np.allclose(optical.as_matrix()@FLU_TO_OPTICAL, flu.as_matrix(), atol=1e-15, rtol=0)


def test_full_design_resolution_has_separate_version_and_preserves_old_bounds():
    image, _ = admit(exposure(width=1920, height=1920))
    assert image.width == image.height == 1920
    with pytest.raises(ValueError):
        inspect_rgb_png(image.png_bytes())
    with pytest.raises(ValueError):
        CameraImage.model_validate(dict(image.model_dump(), schema_version='spacesim-autonomy-camera-image/1'))
    malformed = image.model_dump()
    malformed['png_base64'] += '\n'
    with pytest.raises(ValueError):
        MountedCameraImage.model_validate(malformed)


@pytest.mark.parametrize('change', ['wrong-body', 'fixed-pose', 'flipped-axis', 'reflection', 'nonunit',
                                  'missing-axes', 'bool-matrix', 'nan-matrix', 'missing-matrix', 'stale-sample'])
def test_wrong_moving_mount_or_camera_transform_is_rejected(change):
    camera, mount, raw, products = exposure()
    if change == 'wrong-body':
        mount = CameraMountSample.model_validate(dict(mount.model_dump(), parent_body_name='cubesat_bus'))
    elif change == 'fixed-pose':
        raw['extrinsics']['position_N_m'] = list(mount.origin_j2000_m)
    elif change == 'flipped-axis':
        raw['extrinsics']['c_NC'] = (np.array(raw['extrinsics']['c_NC']).reshape(3, 3)@FLU_TO_OPTICAL.T).ravel().tolist()
    elif change in {'reflection', 'nonunit'}:
        matrix = np.array(raw['extrinsics']['c_NC']).reshape(3, 3)
        matrix[:, 0] *= -1 if change == 'reflection' else 1.001
        raw['extrinsics']['c_NC'] = matrix.ravel().tolist()
    elif change == 'missing-axes':
        del raw['extrinsics']['camera_axes']
    elif change == 'missing-matrix':
        del raw['extrinsics']['c_NC']
    elif change == 'stale-sample':
        raw['sim_time_ns'] = str(mount.sample_time_ns-1)
    else:
        raw['extrinsics']['c_NC'][0] = True if change == 'bool-matrix' else float('nan')
    with pytest.raises(ValueError):
        admit((camera, mount, raw, products))


@pytest.mark.parametrize('field,value', [('aperture_fnumber', 2.), ('exposure_time_s', 0.),
    ('temporal_exposure_integration', True), ('sampling_fps', 2), ('state_kind', 'presentation'),
    ('session_id', 'old-renderer'), ('sim_time_ns', 8_000_000_000_000.)])
def test_photometric_and_temporal_contract_cannot_be_substituted(field, value):
    data = exposure()
    data[2][field] = value
    with pytest.raises(ValueError):
        admit(data)


def test_sequence_tracks_moving_stereo_and_rejects_previous_frame_after_a_valid_pair():
    first, right = exposure(), exposure(side='right')
    gate = sequence((first[0], right[0]))
    for index in range(3):
        pair = [admit(exposure(index=index, side=side, body_angle=.4+index*.02)) for side in ('left', 'right')]
        images, receipts = zip(*pair, strict=True)
        gate.add(images, receipts)
    assert gate.count == 3 and not gate.failed
    with pytest.raises(ValueError):
        gate.add(images, receipts)
    assert gate.failed and gate.count == 3
    valid = [admit(exposure(index=3, side=side)) for side in ('left', 'right')]
    with pytest.raises(ValueError, match='invalid'):
        gate.add(*zip(*valid, strict=True))


@pytest.mark.parametrize('field,value', [('episode_id', 'episode-b'), ('model_xml_sha256', 'b'*64),
                                      ('epoch_ns', 0), ('external_state_writes', 1), ('physics_time_ns', 1)])
def test_wrong_source_or_clock_mount_cannot_enter_a_sequence(field, value):
    data = exposure()
    raw_mount = dict(data[1].model_dump(), **{field: value})
    with pytest.raises(ValueError):
        mount = CameraMountSample.model_validate(raw_mount)
        image, receipt = admit((data[0], mount, data[2], data[3]))
        sequence((data[0],)).add((image,), (receipt,))


def test_two_cameras_cannot_claim_different_poses_for_the_same_physical_mount():
    left, right = exposure(), exposure(side='right', body_angle=.9)
    images, receipts = zip(admit(left), admit(right), strict=True)
    with pytest.raises(ValueError, match='one body'):
        sequence((left[0], right[0])).add(images, receipts)


def test_mutated_replay_and_extra_public_fields_are_rejected():
    data = exposure()
    image, receipt = admit(data)
    for key, value in [('image_sha256', '0'*64), ('calibration_sha256', '0'*64),
                       ('source_frame_id', 3), ('renderer_session_id', 'changed')]:
        changed = dict(receipt.model_dump(), **{key: value})
        with pytest.raises(ValueError):
            verify_mounted_camera_capture(data[0], image, changed)
    changed = deepcopy(receipt.model_dump())
    changed['mount']['origin_j2000_m'] = (0., 0., 0.)
    with pytest.raises(ValueError, match='moving mount'):
        verify_mounted_camera_capture(data[0], image, changed)
    malformed = image.model_dump()
    raw = image.png_bytes()+b'target pose'
    malformed.update(png_sha256=hashlib.sha256(raw).hexdigest(), png_base64=base64.b64encode(raw).decode())
    with pytest.raises(ValueError):
        MountedCameraImage.model_validate(malformed)


@pytest.mark.parametrize('change', ['stopped-physics', 'frame-gap', 'session-change', 'missing-camera'])
def test_sequence_rejects_motion_clock_and_rig_discontinuities(change):
    first, second = exposure(), exposure(index=1, body_angle=.5)
    gate = sequence((first[0],))
    image, receipt = admit(first)
    gate.add((image,), (receipt,))
    image, receipt = admit(second)
    raw_image, raw_receipt = image.model_dump(), receipt.model_dump()
    if change == 'stopped-physics':
        raw_receipt['mount']['native_step_count'] = 0
    elif change == 'frame-gap':
        raw_receipt['source_frame_id'] = raw_image['source_frame_id'] = 100
    elif change == 'session-change':
        raw_receipt['renderer_session_id'] = raw_image['renderer_session_id'] = 'renderer-b'
    with pytest.raises(ValueError):
        gate.add(() if change == 'missing-camera' else (raw_image,), (raw_receipt,))
    assert gate.failed and gate.count == 1
