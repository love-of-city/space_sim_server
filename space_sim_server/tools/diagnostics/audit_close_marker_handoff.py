"""Compare near-grasp marker availability on identical saved image detections.

This is an offline replay of the original gripper-camera trial, not a new
physical grasp or a validation of another camera installation. Ground truth
selects and scores frames only; it never enters either pose estimator.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'backend'))

from space_arm_platform.multicamera_pose import MultiCameraPoseEstimator


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def replay(source, requested_distances):
    config = read_json(source / 'calibration.json')
    if config.get('mount_parent_id') != 'teleop/gripper':
        raise ValueError('this replay requires the original gripper-mounted stereo calibration')
    if set(config['layout']['markers']) != {str(i) for i in range(23)}:
        raise ValueError('the source must contain the unchanged 23-marker layout')
    poses = {}
    with (source / 'truth.jsonl').open(encoding='utf-8-sig') as stream:
        for line in stream:
            row = json.loads(line)
            poses[(row['session_id'], str(row['source_frame_id']), str(row['sim_time_ns']))] = row
    candidates = []
    with (source / 'measurements.jsonl').open(encoding='utf-8-sig') as stream:
        for line in stream:
            row = json.loads(line)
            identity = (row['session_id'], str(row['source_frame_id']), str(row['sim_time_ns']))
            truth = poses.get(identity)
            if truth is None or 'detected_corners_px' not in row.get('result', {}):
                continue
            distance = float(np.linalg.norm(truth['interface_position_grip_m']))
            candidates.append((distance, row, truth))
    if not candidates:
        raise ValueError('no same-session, same-frame, same-epoch detection/truth pairs')
    large = copy.deepcopy(config['layout'])
    large['markers'] = {key: value for key, value in large['markers'].items() if int(key) < 4}
    estimators = {
        'original_23': MultiCameraPoseEstimator(config['calibration'], config['layout']),
        'large_4_only': MultiCameraPoseEstimator(config['calibration'], large),
    }
    mount = config['parent_from_optical']['left']
    grip_from_left = np.array(mount['R_parent_from_optical'])
    left_position = np.array(mount['position_parent_m'])
    center = np.array(config['marker_rig']['grasp_center_target_m'])
    records = []
    for requested in requested_distances:
        distance, packet, truth = min(candidates, key=lambda value: abs(value[0] - requested))
        observations = [packet['result']['detected_corners_px'][eye] for eye in ('left', 'right')]
        record = dict(requested_distance_m=requested, distance_m=distance,
                      axial_gap_m=-float(truth['interface_position_grip_m'][2]),
                      session_id=packet['session_id'], source_frame_id=packet['source_frame_id'],
                      sim_time_ns=packet['sim_time_ns'], methods={})
        for name, estimator in estimators.items():
            # Inputs are image corners and fixed calibration/model only.
            result = estimator.estimate_from_detections(*observations)['methods']['joint_multicamera']
            entry = dict(status=result['status'], reason=result['reason'],
                         used_ids=result.get('used_ids', {}))
            if result['status'] == 'ok':
                pose = result['pose_target_to_left']
                measured = grip_from_left @ (np.array(pose['R']) @ center + pose['t_m']) + left_position
                entry['position_error_mm'] = float(np.linalg.norm(
                    measured - truth['interface_position_grip_m']) * 1000)
                entry['reprojection_rms_px'] = result['reprojection_error_px']
            record['methods'][name] = entry
        records.append(record)
    return dict(source_run=str(source), scope=__doc__,
                distance_definition='Euclidean distance: gripper origin to layout grasp_center_target_m',
                reference_target_m=center.tolist(), new_render=False, new_physical_grasp=False,
                same_saved_detections_both_methods=True,
                small_markers_still_present_in_source_images=True,
                truth_used_as_estimator_input=False,
                camera_parent=config['mount_parent_id'], image_size=config['calibration']['image_size'],
                source_hashes={name: hashlib.sha256((source / name).read_bytes()).hexdigest()
                               for name in ('calibration.json', 'measurements.jsonl', 'truth.jsonl')},
                records=records)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--distances', type=float, nargs='+', default=[.2, .15, .12, .1, .08, .05])
    args = parser.parse_args()
    if not all(np.isfinite(value) and value > 0 for value in args.distances):
        parser.error('distances must be finite and positive')
    source = args.source_run.resolve()
    output = args.output.resolve()
    if output == source or output.is_relative_to(source):
        parser.error('write the replay evidence outside the source experiment')
    report = replay(source, args.distances)
    output.mkdir(parents=True, exist_ok=True)
    (output / 'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n',
                                       encoding='utf-8')
    for row in report['records']:
        print(json.dumps(dict(distance_m=row['distance_m'], source_frame_id=row['source_frame_id'],
                              methods=row['methods']), ensure_ascii=False))


if __name__ == '__main__':
    main()
