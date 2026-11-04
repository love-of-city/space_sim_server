"""Local accepted task instances exposed through the normal platform scene menu.

The installed index is generated data, never a hard-coded workstation path.
Model/camera definitions are not rewritten by this integration.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def entries(project_root):
    path = Path(project_root)/'run/imported_scene_library.json'
    if not path.exists():
        return []
    index = read(path)
    if index.get('schema') != 'space-arm-platform-library/1':
        raise ValueError('Unsupported installed scene-library index')
    return index['entries']


def lookup(project_root, template_id):
    return next((entry for entry in entries(project_root) if entry['id']==template_id), None)


def verified_source(entry):
    for key in ('source_instance', 'receipt', 'catalog', 'camera_baseline'):
        if key in entry and digest(entry[key]) != entry[key+'_sha256']:
            raise ValueError(f'Imported scene evidence changed: {key}')
    source = read(entry['source_instance'])
    for dependency in source['dependencies']:
        if digest(dependency['path']) != dependency['sha256']:
            raise ValueError(f'Imported scene dependency changed: {dependency["path"]}')
    return source


def apply_instance(instance, entry):
    source = verified_source(entry)
    initial = source['initial_scene']
    instance['randomization'] = deepcopy(initial['randomization'])
    instance['seed'] = initial.get('seed', source.get('seed', 42))
    instance['arm_preparation_required'] = False
    instance['initialization_enabled'] = False
    instance['randomize_orbit_phase'] = False
    instance['randomize_overview_camera'] = False
    instance.pop('overview_camera', None)
    instance['library_scene'] = deepcopy(entry)
    instance['library_scene']['model'] = source['model']
    instance['library_scene']['model_sha256'] = digest(source['model'])
    instance['capture_target']['runtime_model'] = source['model']
    instance['capture_target']['runtime_warning'] = (
        '场景库固定初态，沿用原系统相机。抓取臂通过已连接约束保持独立靶星，操作臂用于作业；本场景不验证接近捕获过程。'
        if 'preheld_capture_joint_position' in initial['randomization'] else
        '场景库固定初态，沿用原系统相机；已加载对应任务模型和初始机械臂姿态。'
    )
    if initial.get('preheld_capture'):
        instance['preheld_capture'] = deepcopy(initial['preheld_capture'])
    if 'render_profile' in initial:
        override = initial['render_profile']
        if not isinstance(override, dict) or set(override) - {'visual_attachments', 'vision_capture'}:
            raise ValueError('Imported render overrides may only select visual attachments and vision calibration')
        profile = deepcopy(instance.get('render_profile', {}))
        if 'visual_attachments' in override:
            if not isinstance(override['visual_attachments'], list):
                raise ValueError('Imported visual attachments must be a list')
            profile['visual_attachments'] = deepcopy(override['visual_attachments'])
        if 'vision_capture' in override:
            vision = override['vision_capture']
            if (not isinstance(vision, dict) or set(vision) != {'fiducial_calibration_valid'}
                    or type(vision['fiducial_calibration_valid']) is not bool):
                raise ValueError('Imported vision override must declare fiducial calibration validity only')
            profile['vision_capture'] = {**profile.get('vision_capture', {}), **vision}
        instance['render_profile'] = profile
    return instance


def model_for_instance(instance):
    entry = instance.get('library_scene')
    if not entry:
        return None
    source = verified_source(entry)
    if (entry['model'] != source['model'] or digest(entry['model']) != entry['model_sha256']
            or instance['template_id'] != entry['base_template_id']):
        raise ValueError('Imported scene model or base template changed')
    return Path(source['model']).resolve()
