"""Private renderer receipts, kept outside image-solver and navigation inputs."""
import hashlib
import json
from pathlib import Path


SCHEMA = 'space-sim-render-lighting/1'
PROFILE = 'inspection-1500cd-v1'


def session_directory(root, session_id):
    root = Path(root).resolve()
    path = (root/str(session_id)).resolve()
    if path.parent != root or not str(session_id):
        raise ValueError('image evidence session must be one directory inside its task')
    return path


def append_lighting_receipt(root, result, images, metadata):
    directory = session_directory(root, result['session_id'])
    directory.mkdir(parents=True, exist_ok=True)
    receipt = {key: result[key] for key in (
        'mission_id', 'session_id', 'calibration_id', 'source_frame_id', 'sim_time_ns')}
    receipt.update(schema=SCHEMA, images={})
    for eye in ('left', 'right'):
        raw = metadata[eye]
        receipt['images'][eye] = {key: raw.get(key) for key in (
            'session_id', 'camera_id', 'source_frame_id', 'sim_time_ns', 'state_kind', 'camera_work_lights')}
        receipt['images'][eye]['rgb_sha256'] = hashlib.sha256(images[eye]).hexdigest()
    with (directory/'lighting.jsonl').open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(receipt, allow_nan=False, separators=(',', ':'))+'\n')


def score_work_lights(folder, handoff, config):
    """Check actual per-image lamps and cover every visual result consumed by control."""
    folder = Path(folder)
    identity = handoff.get('navigation', {}).get('vision_identity')
    if (not isinstance(identity, (list, tuple)) or len(identity) != 3
            or not all(isinstance(value, str) and value for value in identity)
            or (identity[0], identity[2]) != (handoff['mission_id'], handoff['calibration_id'])):
        return dict(passed=False, errors=['missing_or_mismatched_navigation_vision_identity'], pairs=0)
    identity = tuple(identity)
    session = identity[1]
    paths = [session_directory(folder/name, session)/'lighting.jsonl'
             for name in ('vision-evidence', 'images')]
    paths = [path for path in paths if path.is_file()]
    if len(paths) != 1:
        return dict(passed=False, errors=['missing_or_ambiguous_render_lighting_receipts'], pairs=0)
    declared = [camera['camera_id'] for camera in config['cameras'] if camera.get('work_light_profile') == PROFILE]
    if declared != [config['camera_ids']['right']]:
        return dict(passed=False, errors=['near23_requires_declared_right_camera_profile'], pairs=0)
    errors, recorded, image_hashes = [], set(), {}
    previous = -1
    try:
        with paths[0].open(encoding='utf-8') as stream:
            for line in stream:
                row = json.loads(line)
                stamp = int(row['sim_time_ns'])
                key = (str(row['source_frame_id']), stamp)
                if (row['schema'] != SCHEMA or tuple(row[k] for k in
                    ('mission_id', 'session_id', 'calibration_id')) != identity
                        or key in recorded or stamp <= previous):
                    errors.append('receipt_identity_or_order')
                previous = stamp
                recorded.add(key)
                for eye in ('left', 'right'):
                    image = row['images'][eye]
                    if (image['session_id'] != session or image['camera_id'] != config['camera_ids'][eye]
                            or image['state_kind'] != 'authoritative'
                            or (str(image['source_frame_id']), int(image['sim_time_ns'])) != key):
                        errors.append('image_source_binding')
                    digest = image['rgb_sha256']
                    if not isinstance(digest, str) or len(digest) != 64 or any(c not in '0123456789abcdef' for c in digest):
                        errors.append('image_hash')
                    image_hashes[(key, eye)] = digest
                    lamps = image['camera_work_lights']
                    if not isinstance(lamps, list):
                        errors.append('missing_native_lamps')
                        continue
                    active = [lamp for lamp in lamps if lamp.get('enabled') is True]
                    expected = dict(camera_id=declared[0], profile=PROFILE, enabled=True,
                        attached_to_camera=True, identity_camera_mount=True, intensity_units='cd',
                        intensity_cd=1500., range_m=10., inner_cone_deg=35., outer_cone_deg=65., cast_shadows=False)
                    if len(active) != 1 or any((active[0].get(k) is not v) if type(v) is bool
                            else active[0].get(k) != v for k, v in expected.items()):
                        errors.append('actual_lamp_differs_from_declared_profile')
        consumed = set()
        with (folder/'measurements.jsonl').open(encoding='utf-8') as stream:
            for line in stream:
                row = json.loads(line)
                if row.get('kind') != 'visual':
                    continue
                result = row['result']
                key = (str(result['source_frame_id']), int(result['sim_time_ns']))
                if tuple(result[k] for k in ('mission_id', 'session_id', 'calibration_id')) != identity:
                    errors.append('consumed_image_identity')
                consumed.add(key)
        if not consumed or not consumed.issubset(recorded):
            errors.append('consumed_images_missing_receipts')
        saved = 0
        for sample in paths[0].parent.iterdir():
            if not sample.is_dir():
                continue
            result = json.loads((sample/'result.json').read_text('utf-8'))
            key = (str(result['source_frame_id']), int(result['sim_time_ns']))
            for eye in ('left', 'right'):
                digest = hashlib.sha256((sample/(eye+'.png')).read_bytes()).hexdigest()
                if image_hashes.get((key, eye)) != digest:
                    errors.append('saved_rgb_does_not_match_receipt')
            saved += 1
        if not saved:
            errors.append('no_actual_rgb_samples')
    except (OSError, ValueError, TypeError, KeyError) as error:
        errors.append('invalid_evidence: '+str(error))
        consumed, saved = set(), 0
    return dict(passed=not errors, errors=sorted(set(errors)), pairs=len(recorded),
        consumed_frames=len(consumed), saved_pairs=saved, receipt_path=str(paths[0]),
        profile=PROFILE, scope='actual renderer lamp receipts and sampled RGB; no hardware photometry claim')
