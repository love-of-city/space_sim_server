"""Replay recorded image poses and encoder/attitude telemetry through the adapter.

Truth is loaded only after conversion, for scoring. No navigation/control
replay or covariance calibration is claimed. Original input images and poses
are unchanged. Run products go under ignored run/.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys

import numpy as np

SERVER = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(SERVER),str(SERVER/'backend')]
from simulation.end_stereo_pointing import EndStereoGeometry
from simulation.image_relative_navigation import ImageRelativeNavigation
from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix


def records(path):
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            yield json.loads(line)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-directory',type=Path,required=True)
    p.add_argument('--scene-model',type=Path,required=True)
    p.add_argument('--camera-config',type=Path,required=True)
    p.add_argument('--interface-test-sigma-m',type=float,required=True,
                   help='API exercise only; NOT a validated image covariance')
    p.add_argument('--output',type=Path,default=SERVER/'run/image_navigation_replay')
    args = p.parse_args()
    if not np.isfinite(args.interface_test_sigma_m) or args.interface_test_sigma_m<=0:
        p.error('interface test sigma must be positive')
    source,output = args.source_directory.resolve(),args.output.resolve()
    output.mkdir(parents=True,exist_ok=True)
    g = EndStereoGeometry.from_files(args.scene_model,args.camera_config)
    # Explicit whitelist: discard renderer extrinsics and diagnostic metadata.
    packets = [{k:row[k] for k in ('session_id','source_frame_id','sim_time_ns','camera_ids','measurement')}
               for row in records(source/'measurements.jsonl')]
    needed = {int(row['sim_time_ns']) for row in packets}
    telemetry = {}
    for row in records(source/'physics.observations.jsonl'):
        t = int(row['sim_time_ns'])
        if t not in needed:
            continue
        attitude = row['attitude_control']
        if int(attitude['state_time_ns']) != t:
            continue
        telemetry[t] = (np.array(row['capture_arm']['joint_position_rad']),
                        quaternion_wxyz_to_matrix(attitude['orientation_inertial_wxyz']))
    adapter = ImageRelativeNavigation(g,packets[0]['session_id'])
    covariance = np.eye(3)*args.interface_test_sigma_m**2
    outputs,rejections = [],Counter()
    for packet in packets:
        t = int(packet['sim_time_ns'])
        if t not in telemetry:
            rejections['missing_image_epoch_telemetry'] += 1
            continue
        q,service = telemetry[t]
        try:
            nav = adapter.convert(packet,now_ns=t+150_000_000,joint_position_rad=q,
                kinematics_stamp_ns=t,service_rotation_n_from_body=service,service_attitude_stamp_ns=t,
                position_covariance_n_m2=covariance)
        except ValueError as error:
            rejections[str(error)] += 1
            continue
        outputs.append(dict(packet=nav,q=q,service=service,stamp=t))
    # Separate evaluation pass: these values never enter the adapter.
    origin_truth = {int(row['sim_time_ns']):np.array(row['target_relative_state']['position_inertial_m'])
                    for row in records(source/'physics.observations.jsonl') if int(row['sim_time_ns']) in needed}
    center_truth = {int(row['sim_time_ns']):np.array(row['interface_position_grip_m'])
                    for row in records(source/'truth.jsonl') if int(row['sim_time_ns']) in needed}
    scores = []
    for entry in outputs:
        nav,t = entry['packet'],entry['stamp']
        if t not in origin_truth or t not in center_truth:
            continue
        pg,rg = g.chain.forward(entry['q'])
        center_n = nav['z']+nav['rotation_n_from_target']@g.center
        center_grip = rg.T@(entry['service'].T@center_n-pg)
        scores.append(dict(stamp_ns=t,frame_id=nav['source_frame_id'],
            body_origin_position_error_mm=float(np.linalg.norm(nav['z']-origin_truth[t])*1000),
            mechanism_center_error_mm=float(np.linalg.norm(center_grip-center_truth[t])*1000),
            gripper_to_mechanism_center_m=float(np.linalg.norm(center_truth[t]))))
    def statistics(subset):
        result = dict(n=len(subset))
        for name in ('body_origin_position_error_mm','mechanism_center_error_mm'):
            e = np.array([s[name] for s in subset])
            result[name] = (dict(rmse=float(np.sqrt(np.mean(e*e))),max=float(e.max()),
                                p95=float(np.percentile(e,95))) if e.size else None)
        return result
    all_scores = statistics(scores)
    numerical_ok = bool(scores) and all(
        all_scores[key]['max'] < 3. for key in ('body_origin_position_error_mm','mechanism_center_error_mm'))
    summary = dict(source_directory=str(source),scope='recorded-image coordinate and epoch handoff only',
        total_image_pairs=len(packets),converted=len(outputs),rejected=dict(rejections),
        exact_epoch_telemetry=len(telemetry),scored=len(scores),
        service_attitude_model='recorded simulation attitude, treated as ideal telemetry',
        interface_test_sigma_m=args.interface_test_sigma_m,covariance_validated_for_live_use=False,
        replay_delivery_delay_s=.15,delay_is_test_assumption=True,
        all_converted=all_scores,requested_grasp_range=statistics(
            [s for s in scores if .05<=s['gripper_to_mechanism_center_m']<=.501]),
        coordinate_error_check=dict(threshold_mm=3.,passed=numerical_ok),
        complete_mission_validated=False,
        passed=numerical_ok and len(scores)==len(outputs)>0 and rejections.get('missing_image_epoch_telemetry',0)==0)
    (output/'scores.jsonl').write_text(''.join(json.dumps(s)+'\n' for s in scores),encoding='utf-8')
    (output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    print(json.dumps(summary,indent=2))
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
