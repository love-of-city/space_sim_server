"""Image-only policy boundary: freshness, calibration, pixels and private data."""
import base64
from copy import deepcopy
import hashlib
import math
import struct
import zlib

import pytest

from space_arm_platform.benchmark_camera import (
    CameraImage, PinholeCamera, PNG_SIGNATURE, inspect_rgb_png, public_camera_image,
)


def chunk(kind, data):
    return struct.pack('!I',len(data))+kind+data+struct.pack('!I',zlib.crc32(kind+data) & 0xffffffff)


def png(*, width=3, height=2, pixels=None, extra=b''):
    pixels = bytes([0,90,120,160,90,120,160,90,120,160])*2 if pixels is None else pixels
    return (PNG_SIGNATURE+chunk(b'IHDR',struct.pack('!IIBBBBB',width,height,8,2,0,0,0))
            +extra+chunk(b'IDAT',zlib.compress(pixels))+chunk(b'IEND',b''))


@pytest.fixture
def capture():
    camera = PinholeCamera(camera_id='test/left',width=3,height=2,horizontal_fov_rad=.7,
        position_body_m=(.1,-.9,.3),orientation_body_from_optical_wxyz=(1.,0.,0.,0.))
    k = camera.intrinsics
    metadata = dict(protocol='bsk-capture/1',type='camera_frame',stream_kind='vision',
        state_kind='authoritative',sensor_model='render_pinhole',color_model='rgb',camera_mode='rgb',
        session_id='session',camera_id=camera.camera_id,source_frame_id='8',sim_time_ns='500000000',
        temporal_exposure_integration=False,exposure_semantics='render_photometric_only',exposure_time_s=0,
        sampling_fps=2,sampling_clock='simulation',resolution=[3,2],
        intrinsics=dict(model='pinhole',distortion_model='none',fx_px=k[0][0],fy_px=k[1][1],
            cx_px=1.,cy_px=.5,fov_x_rad=.7,fov_y_rad=2*math.atan(2/(2*k[1][1]))),
        extrinsics=dict(position_N_m=[12345.,67890.,-100.]),
        floating_origin=dict(origin_N_m=[9999.,9999.,9999.]),private_truth='MUST_NOT_REACH_THE_METHOD')
    return camera,metadata,dict(rgb=png())


def admit(capture):
    camera,metadata,products = capture
    return public_camera_image(metadata,products,camera,session_id='session',source_frame_id=8,sample_time_ns=500_000_000)


def test_public_packet_preserves_pixels_and_excludes_runtime_transforms(capture):
    image = admit(capture)
    assert image.png_bytes() == capture[2]['rgb']
    assert image.png_sha256 == hashlib.sha256(capture[2]['rgb']).hexdigest()
    assert CameraImage.model_validate_json(image.model_dump_json()) == image
    assert set(image.model_dump()) == {'schema_version','camera_id','renderer_session_id','source_frame_id',
                                      'sample_time_ns','width','height','png_sha256','png_base64'}
    assert not any(value in image.model_dump_json() for value in ('MUST_NOT_REACH','position_N_m','floating_origin'))
    with pytest.raises(ValueError):
        CameraImage.model_validate(dict(image.model_dump(),target_pose=[0.,0.,0.]))


@pytest.mark.parametrize('key,value', [
    ('session_id','old-session'),('camera_id','test/right'),('source_frame_id','7'),
    ('sim_time_ns','0'),('sim_time_ns',500000000.0),('source_frame_id','08'),('source_frame_id',True),
    ('stream_kind','preview'),('state_kind','interpolated'),('resolution',[2,3]),
    ('temporal_exposure_integration',True),('exposure_time_s',False),('sampling_fps',True),
    ('sampling_clock','wall'),('sensor_model','truth_projection'),
])
def test_stale_wrong_sensor_and_unphysical_clock_are_rejected(capture,key,value):
    capture[1][key] = value
    with pytest.raises(ValueError):
        admit(capture)


@pytest.mark.parametrize('key,value', [('fx_px',1.),('cx_px',1.5),('fy_px',float('nan')),
                                     ('fov_y_rad',True),('distortion_model','brown')])
def test_packet_cannot_substitute_another_camera_calibration(capture,key,value):
    capture[1]['intrinsics'][key] = value
    with pytest.raises(ValueError):
        admit(capture)


def test_hash_and_png_must_match_even_if_outer_manifest_is_rewritten(capture):
    image = admit(capture).model_dump()
    for mutated in (dict(image,png_sha256='0'*64),dict(image,width=4),
                    dict(image,png_base64=image['png_base64']+'\n')):
        with pytest.raises(ValueError):
            CameraImage.model_validate(mutated)
    # A private pose must not enter through an ancillary PNG text chunk.
    secret = png(extra=chunk(b'tEXt',b'target_pose\x001,2,3'))
    image.update(png_sha256=hashlib.sha256(secret).hexdigest(),png_base64=base64.b64encode(secret).decode())
    with pytest.raises(ValueError,match='unexpected PNG chunk'):
        CameraImage.model_validate(image)


@pytest.mark.parametrize('data', [
    png()[:-1], png()+b'ignored-data', png()[:20]+b'broken'+png()[26:],
    png(pixels=b'\x00'*10000),png(pixels=b'\x05'+b'\x00'*19),
    png(width=641),png(height=481),
])
def test_malformed_compressed_images_are_bounded_and_rejected(data):
    with pytest.raises(ValueError):
        inspect_rgb_png(data)


def test_packet_does_not_accept_extra_products_or_malformed_body_calibration(capture):
    capture[2]['depth'] = b'undeclared modality'
    with pytest.raises(ValueError):
        admit(capture)
    data = deepcopy(capture[0].model_dump())
    for name,value in [('orientation_body_from_optical_wxyz',(2.,0.,0.,0.)),
                       ('sample_rate_hz',True),('width',641),('horizontal_fov_rad',float('inf'))]:
        with pytest.raises(ValueError):
            PinholeCamera.model_validate(dict(data,**{name:value}))
