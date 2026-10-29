"""Calibrated stereo mounts, attached to the named spacecraft or gripper body."""
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def load_stereo_setup(root: Path, spec: dict | None):
    if not spec or not spec.get('enabled') or not spec.get('stereo_enabled',True):
        return None
    root = Path(root).resolve()
    path = (root / spec['config_file']).resolve()
    if not path.is_relative_to(root):
        raise ValueError('stereo configuration must be inside server directory')
    config = json.loads(path.read_text('utf-8-sig'))
    if len(config['cameras']) != 2 or set(config['camera_ids']) != {'left', 'right'}:
        raise ValueError('stereo requires exactly two named cameras')
    ids = [c['camera_id'] for c in config['cameras']]
    if len(set(ids)) != 2 or set(ids) != set(config['camera_ids'].values()):
        raise ValueError('stereo camera IDs must be unique and match calibration eyes')
    parent = config.get('mount_parent_id', 'teleop/cubesat_bus')
    if parent not in ('teleop/cubesat_bus', 'teleop/gripper'):
        raise ValueError('unsupported stereo mount parent')
    calibration = config.get('calibration')
    rendered_mounts = {}
    for eye in ('left', 'right'):
        camera = next(c for c in config['cameras'] if c['camera_id'] == config['camera_ids'][eye])
        if camera.get('parent_id') != parent:
            raise ValueError('stereo camera parent differs from calibrated mount')
        size = camera.get('resolution')
        if (camera.get('capture_consumer') != 'vision' or not isinstance(size, list)
                or len(size) != 2 or any(type(v) is not int or v <= 0 for v in size)):
            raise ValueError('stereo must use full-resolution vision capture')
        if calibration is not None:
            if size != calibration['image_size']:
                raise ValueError('render resolution differs from stereo calibration')
            k = np.asarray(calibration['K_' + eye], float)
            focal = size[0] / (2 * math.tan(camera['field_of_view_rad'] / 2))
            if (k.shape != (3, 3) or not np.isfinite(k).all()
                    or not np.allclose([k[0, 0], k[1, 1]], focal, atol=1e-7)
                    or not np.allclose(k[:2, 2], [(size[0]-1)/2, (size[1]-1)/2])):
                raise ValueError('renderer pinhole optics differ from calibration')
            conv = np.array([[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]])
            q = np.asarray(camera['orientation_body_from_camera_wxyz'], float)
            r = Rotation.from_quat(np.roll(q, -1)).as_matrix() @ conv.T
            p = np.asarray(camera['position_body_m'], float)
            rendered_mounts[eye] = (r, p)
            mounting = config.get('parent_from_optical', config.get('body_from_optical'))
            if mounting is not None:
                m = mounting[eye]
                rm = m.get('R_parent_from_optical', m.get('R_body_from_optical'))
                pm = m.get('position_parent_m', m.get('position_body_m'))
                if not np.allclose(r, rm, atol=1e-8) or not np.allclose(p, pm, atol=1e-8):
                    raise ValueError('rendered camera mount differs from hand-eye calibration')
    if calibration is not None:
        rl, pl = rendered_mounts['left']; rr, pr = rendered_mounts['right']
        if (not np.allclose(rr.T@rl, calibration['R_right_from_left'], atol=1e-8)
                or not np.allclose(rr.T@(pl-pr), calibration['t_right_from_left_m'], atol=1e-8)):
            raise ValueError('rendered camera pair differs from stereo extrinsic calibration')
    return config


def register_stereo_cameras(bridge, config):
    if config is None:
        return
    from bsk_render_adapter import CameraVisual
    for camera in config['cameras']:
        bridge.add_camera(CameraVisual(**camera))


def configure_wrist_capture(bridge, enabled, *, calibration=None):
    """Sample identical optics separately, preserving preview/dataset routing."""
    if not enabled:return
    from dataclasses import replace
    from space_arm_platform.wrist_vision_config import CAMERA_ID,VISION_CAMERA_ID
    camera=next((c for c in bridge._cameras if c.camera_id==CAMERA_ID),None)
    if camera is None:raise ValueError('wrist alignment camera is missing')
    if calibration is not None:
        # Apply the same optics to the visible view and the native vision image.
        # Body attachment and optical orientation stay exactly as registered.
        camera.resolution=tuple(calibration['image_size'])
        camera.field_of_view_rad=calibration['horizontal_fov_rad']
        camera.vertical_field_of_view_rad=calibration['vertical_fov_rad']
        camera.exposure_time_s=calibration['exposure_time_s']
    bridge.add_camera(replace(camera,camera_id=VISION_CAMERA_ID,
        display_name='Capture wrist image-only alignment',picture_in_picture=False,
        capture_consumer='vision',capture_products=('rgb',),
        capture_rate_hz=calibration['capture_rate_hz'] if calibration else 10.))
