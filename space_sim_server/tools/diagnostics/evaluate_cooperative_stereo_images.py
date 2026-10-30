"""Evaluate an explicitly attitude-aided route on saved UE optical sweeps.

Stored true rotation stands in for IDEAL cooperative attitude telemetry in this
diagnostic, with separate deterministic attitude-error perturbations. It is not
an independent image-only six-DOF validation. Stored translation is scoring only.
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

SERVER = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SERVER/'backend'))
from space_arm_platform.cooperative_stereo_translation import CooperativeStereoTranslation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs', type=Path, nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--attitude-errors-deg', type=float, nargs='+', default=[0., .1, .5])
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    import cv2
    cv2.setNumThreads(1)
    groups = defaultdict(list)
    with (args.output/'observations.jsonl').open('w', encoding='utf-8') as stream:
        for source in args.inputs:
            config = json.loads((source/'calibration.json').read_text())
            estimator = CooperativeStereoTranslation(config['calibration'], config['layout'])
            center = np.array(config['marker_rig']['grasp_center_target_m'])
            for line in (source/'observations.jsonl').read_text().splitlines():
                original = json.loads(line); row = original['metrics']
                stem = f'{row["trial"]:03d}_{row["repeat"]}'
                left, right = [(source/(stem+'_'+e+'.jpg')).read_bytes() for e in ('left','right')]
                for error in args.attitude_errors_deg:
                    # Deterministic oblique direction prevents testing a single
                    # favourable rotation axis. Both signs tested on repeats.
                    axis = np.array([1., 2., 3.]); axis /= np.linalg.norm(axis)
                    perturb = Rotation.from_rotvec(axis*np.deg2rad(error)*(-1 if row['repeat']%2 else 1)).as_matrix()
                    cooperative_rotation = perturb@np.array(original['truth_R_target_to_left'])
                    result = estimator.estimate_aided(left, right, rotation_target_to_left=cooperative_rotation,
                        attitude_stamp_ns=original['sim_time_ns'], image_stamp_ns=original['sim_time_ns'])
                    chosen = result['measurement']
                    metrics = dict(candidate=source.name, optical_depth_m=row['gap_m'],
                        angle_deg=row['angle_deg'], azimuth_deg=row['azimuth_deg'],
                        attitude_error_assumption_deg=error, status=chosen['status'], reason=chosen['reason'])
                    if chosen['status']=='ok':
                        pose = chosen['pose_target_to_left']; r, t = np.array(pose['R']), np.array(pose['t_m'])
                        truth = np.array(original['truth_R_target_to_left'])@center+original['truth_t_target_to_left_m']
                        metrics['center_position_error_mm'] = float(np.linalg.norm(r@center+t-truth)*1000)
                        metrics['origin_position_error_mm'] = float(np.linalg.norm(t-np.array(original['truth_t_target_to_left_m']))*1000)
                    groups[(source.name,row['gap_m'],error)].append(metrics)
                    stream.write(json.dumps(dict(metrics=metrics,result=result))+'\n')
    summaries = []
    for (candidate,depth,error), rows in groups.items():
        valid = [r for r in rows if r['status']=='ok']
        e = np.array([r['center_position_error_mm'] for r in valid])
        summaries.append(dict(candidate=candidate,optical_depth_m=depth,attitude_error_assumption_deg=error,
            total=len(rows),accepted=len(valid),position_rmse_mm=float(np.sqrt(np.mean(e**2))) if len(e) else None,
            max_mm=float(e.max()) if len(e) else None))
    report = dict(scope=__doc__, groups=summaries, independent_image_attitude_validated=False,
                  translated_truth_used_as_estimator_input=False)
    (args.output/'summary.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__ == '__main__': main()
