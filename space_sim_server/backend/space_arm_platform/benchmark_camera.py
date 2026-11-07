"""Public pinhole images and a strict boundary around private renderer packets.

The renderer also reports authoritative world transforms. They are deliberately
absent from these method-input types. Image bytes are portable, lossless PNG;
camera extrinsics here are fixed engineering calibration, never runtime pose.
"""
from __future__ import annotations

import base64
import binascii
import hashlib
import math
import struct
from typing import Annotated, Literal
import zlib

from pydantic import Field, field_validator, model_validator

from .benchmark_attitude import Quaternion, unit_quaternion
from .benchmark_contract import Contract, Count, Number, Vector3

MAX_PNG_BYTES = 1_350_000
PNG_SIGNATURE = b'\x89PNG\r\n\x1a\n'
Sha256 = Annotated[str, Field(pattern=r'^[0-9a-f]{64}$')]


def inspect_rgb_png(encoded: bytes) -> tuple[int, int]:
    """Check bounded RGB/RGBA pixels without importing a vision/native runtime.

    Decode the PNG's DEFLATE stream to validate row count and filter bytes. This
    checks integrity rather than interpreting pixels or changing the image.
    Ancillary text/EXIF chunks are rejected so image containers cannot smuggle
    renderer metadata into otherwise typed policy inputs.
    """
    return inspect_bounded_rgb_png(encoded, maximum_width=640, maximum_height=480, maximum_bytes=MAX_PNG_BYTES)


def inspect_bounded_rgb_png(encoded: bytes, *, maximum_width: int, maximum_height: int,
                            maximum_bytes: int) -> tuple[int, int]:
    """Use explicit frozen bounds; never accept limits from packet metadata.

    Existing camera contracts retain their 640x480/1.35 MB limits.
    """
    if any(type(value) is not int or value <= 0 for value in (maximum_width, maximum_height, maximum_bytes)):
        raise ValueError('PNG limits must be positive integers')
    if not isinstance(encoded, bytes) or not 45 <= len(encoded) <= maximum_bytes or not encoded.startswith(PNG_SIGNATURE):
        raise ValueError('expected a bounded PNG image')
    offset, size, image_data, ended, data_ended = 8, None, bytearray(), False, False
    color_channels = None
    while offset < len(encoded):
        if len(encoded)-offset < 12:
            raise ValueError('truncated PNG chunk')
        length = struct.unpack_from('!I', encoded, offset)[0]
        if length > len(encoded)-offset-12:
            raise ValueError('PNG chunk exceeds image length')
        kind = encoded[offset+4:offset+8]
        data = encoded[offset+8:offset+8+length]
        checksum = struct.unpack_from('!I', encoded, offset+8+length)[0]
        if zlib.crc32(kind+data) & 0xffffffff != checksum:
            raise ValueError('PNG chunk checksum differs')
        if size is None:
            if kind != b'IHDR' or length != 13:
                raise ValueError('PNG must start with one IHDR')
            width, height, depth, color, compression, filtering, interlace = struct.unpack('!IIBBBBB', data)
            if (not (1 <= width <= maximum_width and 1 <= height <= maximum_height)
                    or (depth, compression, filtering, interlace) != (8,0,0,0) or color not in (2,6)):
                raise ValueError('camera PNG exceeds its dimensions or is not 8-bit noninterlaced RGB/RGBA')
            size, color_channels = (width, height), 3 if color == 2 else 4
        elif kind == b'IDAT' and not data_ended:
            image_data.extend(data)
        elif kind == b'IEND' and length == 0 and image_data:
            ended = True
        elif kind in {b'sRGB', b'gAMA', b'cHRM', b'pHYs'} and not image_data:
            # Numerical colour/scale metadata only, before compressed pixels.
            if length != {b'sRGB':1, b'gAMA':4, b'cHRM':32, b'pHYs':9}[kind]:
                raise ValueError('invalid PNG colour/scale chunk')
        else:
            raise ValueError('unexpected PNG chunk or pixel-data ordering')
        if image_data and kind != b'IDAT':
            data_ended = True
        offset += 12+length
        if ended:
            if offset != len(encoded):
                raise ValueError('bytes after PNG end')
            break
    if not ended or size is None:
        raise ValueError('incomplete PNG')
    stride = size[0]*color_channels+1
    expected = size[1]*stride
    decoder = zlib.decompressobj()
    try:
        raw = decoder.decompress(bytes(image_data), expected+1)
    except zlib.error as error:
        raise ValueError('invalid compressed PNG pixels') from error
    if len(raw) != expected or not decoder.eof or decoder.unused_data or decoder.unconsumed_tail:
        raise ValueError('PNG pixel length or compression stream differs from its dimensions')
    if any(raw[index] > 4 for index in range(0, expected, stride)):
        raise ValueError('invalid PNG row filter')
    return size


class PinholeCamera(Contract):
    camera_id: str = Field(min_length=1, max_length=256)
    width: Annotated[int, Field(strict=True, ge=1, le=640)]
    height: Annotated[int, Field(strict=True, ge=1, le=480)]
    horizontal_fov_rad: Annotated[Number, Field(gt=0.05, lt=math.pi-0.05)]
    position_body_m: Vector3
    orientation_body_from_optical_wxyz: Quaternion
    optical_axes: Literal['x_right_y_down_z_forward'] = 'x_right_y_down_z_forward'
    distortion_model: Literal['none'] = 'none'
    exposure_model: Literal['instantaneous_render_pinhole'] = 'instantaneous_render_pinhole'
    sample_rate_hz: Literal[1,2,5,10,25,30,50] = 2

    @field_validator('sample_rate_hz', mode='before')
    @classmethod
    def integer_rate(cls, value):
        if type(value) is not int:
            raise ValueError('camera sample rate must be an integer')
        return value

    @field_validator('orientation_body_from_optical_wxyz')
    @classmethod
    def normalized_mount(cls, value):
        unit_quaternion(value)
        return value

    @property
    def intrinsics(self):
        focal = self.width/(2*math.tan(self.horizontal_fov_rad/2))
        return ((focal,0.,(self.width-1)/2), (0.,focal,(self.height-1)/2), (0.,0.,1.))


class CameraImage(Contract):
    schema_version: Literal['spacesim-autonomy-camera-image/1'] = 'spacesim-autonomy-camera-image/1'
    camera_id: str = Field(min_length=1, max_length=256)
    renderer_session_id: str = Field(min_length=1, max_length=128)
    source_frame_id: Count
    sample_time_ns: Count
    width: Annotated[int, Field(strict=True, ge=1, le=640)]
    height: Annotated[int, Field(strict=True, ge=1, le=480)]
    png_sha256: Sha256
    png_base64: str = Field(min_length=60, max_length=4*((MAX_PNG_BYTES+2)//3))

    @model_validator(mode='after')
    def intact_pixels(self):
        validate_image_pixels(self, maximum_width=640, maximum_height=480, maximum_bytes=MAX_PNG_BYTES)
        return self

    def png_bytes(self):
        return base64.b64decode(self.png_base64, validate=True)


def validate_image_pixels(image, *, maximum_width, maximum_height, maximum_bytes):
    try:
        pixels = base64.b64decode(image.png_base64, validate=True)
    except (ValueError, binascii.Error) as error:
        raise ValueError('invalid base64 image') from error
    if base64.b64encode(pixels).decode('ascii') != image.png_base64:
        raise ValueError('noncanonical base64 image')
    if (inspect_bounded_rgb_png(pixels, maximum_width=maximum_width, maximum_height=maximum_height,
                               maximum_bytes=maximum_bytes) != (image.width, image.height)
            or hashlib.sha256(pixels).hexdigest() != image.png_sha256):
        raise ValueError('PNG dimensions or hash differ from the public image')


def _wire_count(value):
    # The renderer transmits uint64 values as decimal strings. A bool, float,
    # signed number or rounding conversion must not pass the freshness check.
    if type(value) is int and value >= 0:
        return value
    if isinstance(value, str) and value.isascii() and value.isdecimal() and str(int(value)) == value:
        return int(value)
    raise ValueError('invalid renderer integer')


def public_camera_image(metadata: dict, products: dict, camera: PinholeCamera, *, session_id: str,
                        source_frame_id: int, sample_time_ns: int) -> CameraImage:
    """Admit only the exact requested authoritative frame and frozen optics."""
    png = admit_pinhole_pixels(metadata, products, camera, session_id=session_id,
        source_frame_id=source_frame_id, sample_time_ns=sample_time_ns)
    return CameraImage(camera_id=camera.camera_id, renderer_session_id=session_id,
        source_frame_id=source_frame_id, sample_time_ns=sample_time_ns,
        width=camera.width, height=camera.height, png_sha256=hashlib.sha256(png).hexdigest(),
        png_base64=base64.b64encode(png).decode('ascii'))


def admit_pinhole_pixels(metadata, products, camera, *, session_id, source_frame_id, sample_time_ns,
                         photometric_exposure_time_s=0., aperture_fnumber=None,
                         maximum_width=640, maximum_height=480, maximum_bytes=MAX_PNG_BYTES):
    """Validate transport and optics; return pixels without any private pose."""
    identity = dict(protocol='bsk-capture/1', type='camera_frame', stream_kind='vision',
                    state_kind='authoritative', sensor_model='render_pinhole', color_model='rgb',
                    camera_mode='rgb', session_id=session_id, camera_id=camera.camera_id)
    if any(metadata.get(key) != value for key,value in identity.items()):
        raise ValueError('camera packet provenance differs from the requested sensor')
    if (_wire_count(metadata.get('source_frame_id')) != source_frame_id
            or _wire_count(metadata.get('sim_time_ns')) != sample_time_ns):
        raise ValueError('camera packet is stale or has the wrong physical sample time')
    if (metadata.get('resolution') != [camera.width,camera.height]
            or metadata.get('temporal_exposure_integration') is not False
            or metadata.get('exposure_semantics') != 'render_photometric_only'
            or type(metadata.get('exposure_time_s')) not in (int,float)
            or metadata.get('exposure_time_s') != photometric_exposure_time_s
            or type(metadata.get('sampling_fps')) not in (int,float)
            or metadata.get('sampling_fps') != camera.sample_rate_hz
            or metadata.get('sampling_clock') != 'simulation'
            or metadata.get('intrinsics', {}).get('model') != 'pinhole'
            or metadata.get('intrinsics', {}).get('distortion_model') != 'none'):
        raise ValueError('camera packet sensor model differs from the declared pinhole')
    if (aperture_fnumber is not None and (type(metadata.get('aperture_fnumber')) not in (int,float)
                                        or metadata['aperture_fnumber'] != aperture_fnumber)):
        raise ValueError('camera packet aperture differs from the declared photometric setting')
    k = camera.intrinsics
    expected = dict(fx_px=k[0][0], fy_px=k[1][1], cx_px=k[0][2], cy_px=k[1][2],
                    fov_x_rad=camera.horizontal_fov_rad,
                    fov_y_rad=2*math.atan(camera.height/(2*k[1][1])))
    for key,value in expected.items():
        actual = metadata['intrinsics'].get(key)
        if type(actual) not in (float,int) or not math.isfinite(actual) or not math.isclose(actual,value,abs_tol=1e-7,rel_tol=0):
            raise ValueError('camera packet calibration differs from the fixed task camera')
    if set(products) != {'rgb'}:
        raise ValueError('camera packet must contain exactly one RGB PNG')
    png = products['rgb']
    if inspect_bounded_rgb_png(png, maximum_width=maximum_width, maximum_height=maximum_height,
                              maximum_bytes=maximum_bytes) != (camera.width,camera.height):
        raise ValueError('camera pixels differ from the configured resolution')
    return png
