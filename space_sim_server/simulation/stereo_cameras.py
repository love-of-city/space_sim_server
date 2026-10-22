"""Explicit body-mounted stereo descriptors; no changes to MJCF physics/cameras."""
import json
from pathlib import Path


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
    for camera in config['cameras']:
        if camera.get('parent_id') != 'teleop/cubesat_bus':
            raise ValueError('capture26 stereo must be fixed to service spacecraft body')
        if camera.get('capture_consumer') != 'vision' or camera.get('resolution') != [1920, 1200]:
            raise ValueError('stereo must use full-resolution vision capture')
    return config


def register_stereo_cameras(bridge, config):
    if config is None:
        return
    from bsk_render_adapter import CameraVisual
    for camera in config['cameras']:
        bridge.add_camera(CameraVisual(**camera))


def configure_wrist_capture(bridge, enabled):
    """Sample identical optics separately, preserving preview/dataset routing."""
    if not enabled:return
    from dataclasses import replace
    from space_arm_platform.wrist_vision_config import CAMERA_ID,VISION_CAMERA_ID
    camera=next((c for c in bridge._cameras if c.camera_id==CAMERA_ID),None)
    if camera is None:raise ValueError('wrist alignment camera is missing')
    bridge.add_camera(replace(camera,camera_id=VISION_CAMERA_ID,
        display_name='Capture wrist image-only alignment',picture_in_picture=False,
        capture_consumer='vision',capture_products=('rgb',),capture_rate_hz=10.))
