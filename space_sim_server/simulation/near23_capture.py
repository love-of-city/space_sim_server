"""Explicit opt-in scene profile for the merged terminal capture experiment."""
import copy
import hashlib
import json
from pathlib import Path

CONFIG = 'config/vision/capture26_near23_front_mount_20261029.json'
GEOMETRY = 'config/visual_attachments/capture26_near23/geometry_metres.json'


def configure_scene(scene, root, output_directory):
    root = Path(root).resolve()
    scene = copy.deepcopy(scene)
    config = json.loads((root / CONFIG).read_text('utf-8-sig'))
    profile = scene['render_profile']
    profile['vision_capture'] = dict(enabled=True, config_file=CONFIG,
                                     stereo_enabled=True, wrist_alignment_enabled=False)
    profile['visual_attachments'] = [dict(parent_id='teleop/capture_target', geometry_file=GEOMETRY,
        sha256=hashlib.sha256((root / GEOMETRY).read_bytes()).hexdigest())]
    scene['randomization']['capture_window_trial'] = True
    scene['randomization']['capture_visual_test'] = dict(enabled=True, mode='predictive_grasp',
        input_source='near23_stereo', held_target_pose=True, config_file=CONFIG,
        observation_file=config['capture_export'], output_directory=str(output_directory))
    return scene
