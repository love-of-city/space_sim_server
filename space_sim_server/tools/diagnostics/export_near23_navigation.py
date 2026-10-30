"""Re-solve an actual recorded pair and export the V-bar navigation input.

Only RGB, static calibration and same-epoch arm encoders enter the bridge.
No renderer object poses, target truth or old pose solutions are used.
This is an interface replay, not a new rendezvous or capture acceptance run.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT/'backend')]


def export(recording, frame, output, *, geometry_prior):
    import numpy as np
    from simulation.near23_navigation import Near23NavigationBridge
    from space_arm_platform.multicamera_pose import MultiCameraPoseEstimator
    from space_arm_platform.visual_navigation import calibration_id

    config = json.loads((recording/'calibration.json').read_text('utf-8-sig'))
    current = json.loads((ROOT/'config/vision/capture26_near23_front_mount_20261029.json').read_text('utf-8-sig'))
    for key in ('cameras', 'calibration', 'camera_ids', 'mount_parent_id', 'parent_from_optical', 'layout', 'marker_rig'):
        if config[key] != current[key]:
            raise ValueError('recorded pair differs from the current gripper stereo: '+key)
    with (recording/'observations.jsonl').open(encoding='utf-8-sig') as stream:
        observation = next((x for line in stream if
            str((x := json.loads(line)).get('render_frame_id')) == str(frame)), None)
    if observation is None:
        raise ValueError('no actual encoder observation for requested frame')
    model = ROOT/'model/SARM/platform/sarm_dual_capture26.xml'
    if observation['model_sha256'] != hashlib.sha256(model.read_bytes()).hexdigest():
        raise ValueError('recorded encoders belong to another model')
    images = {eye: recording/'images'/f'{frame}_{eye}.jpg' for eye in ('left', 'right')}
    estimator = MultiCameraPoseEstimator(config['calibration'], config['layout'])
    measurement = estimator.estimate(*(images[eye].read_bytes() for eye in ('left', 'right')))
    measurement = measurement['methods']['joint_multicamera']
    measurement.update(mission_id='recording-interface-replay', session_id=observation['render_session_id'],
        source_frame_id=str(frame), sim_time_ns=observation['sim_time_ns'], calibration_id=calibration_id(config))
    bridge = Near23NavigationBridge(model, config,
        joint_covariance_rad2=geometry_prior['joint_covariance_rad2'],
        mount_covariance=geometry_prior['mount_covariance'], uncertainty_source=geometry_prior['source'])
    bridge.remember_encoders(int(observation['sim_time_ns']), observation['capture_arm']['joint_position_rad'])
    packet = bridge.image_result(measurement)
    packet['diagnostic_source'] = dict(scope='recorded real RGB interface replay; no closed-loop flight claim',
        recording=str(recording), image_sha256={eye: hashlib.sha256(path.read_bytes()).hexdigest()
                                               for eye, path in images.items()})
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x', encoding='utf-8') as stream:
        json.dump(packet, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(dict(status=packet['status'], reason=packet['reason'], output=str(output),
        counts=packet.get('counts'), optical_depth_sigma_m=packet.get('optical_depth_sigma_m'),
        body_origin_position_m=packet.get('pose_target_to_service', {}).get('t_m'),
        largest_position_sigma_m=(float(np.sqrt(np.linalg.eigvalsh(
            np.array(packet['pose_covariance']['matrix'])[:3, :3])[-1])) if packet['status'] == 'ok' else None)),
        ensure_ascii=False))
    return packet


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recording', type=Path, required=True)
    parser.add_argument('--frame', required=True)
    parser.add_argument('--output', type=Path, required=True)
    uncertainty = parser.add_mutually_exclusive_group(required=True)
    uncertainty.add_argument('--geometry-prior', type=Path,
        help='JSON source, joint_covariance_rad2 (6x6), mount_covariance (6x6)')
    uncertainty.add_argument('--ideal-sim-geometry', action='store_true',
        help='explicitly treat nominal simulated mounts and encoder readings as exact')
    args = parser.parse_args()
    if args.ideal_sim_geometry:
        zeros = [[0.]*6 for _ in range(6)]
        prior = dict(source='ideal simulation mounts/encoders; pixel prior only',
                     joint_covariance_rad2=zeros, mount_covariance=zeros)
    else:
        prior = json.loads(args.geometry_prior.read_text('utf-8-sig'))
    export(args.recording.resolve(), args.frame, args.output.resolve(), geometry_prior=prior)


if __name__ == '__main__':
    main()
