"""Run actual UE stereo distance/pointing probes without freezing mission nodes.

UE must already be running. Runs are serial so physics, images and evidence
writing share the same machine load. A quality candidate still needs collision,
braking, statistical consistency and full real-time mission acceptance.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'backend')]


def summarize(folder):
    from score_capture26_navigation import score
    from space_arm_platform.visual_navigation import covariance
    summary = json.loads((folder/'summary.json').read_text('utf-8'))
    independent = score(folder, warmup_s=1.)
    results = [p['result'] for p in map(json.loads, (folder/'measurements.jsonl').read_text().splitlines())
               if p.get('kind') == 'visual' and int(p['result']['sim_time_ns']) >= 1_000_000_000]
    quality = []
    for r in results:
        good = r.get('status') == 'ok' and not r.get('ambiguous', True)
        if good:
            try:
                c = covariance(r['pose_covariance']['matrix'], 6)
            except (TypeError, ValueError, np.linalg.LinAlgError):
                quality.append(False)
                continue
            good = (min(r['counts'][key] for key in ('used_markers_left', 'used_markers_right', 'common_markers')) >= 2
                    and all(r['reprojection_error_px'][eye] <= 2.5 and r['max_corner_error_px'][eye] <= 5
                            for eye in ('left', 'right'))
                    and np.sqrt(c[2, 2]) < .03)
        quality.append(bool(good))
    return dict(output=str(folder), depth_m=summary['settings']['depth'],
                pointing_error_deg=summary['settings']['pointing_error_deg'],
                processed_after_warmup=len(quality), measurement_quality_fraction=float(np.mean(quality)) if quality else 0.,
                every_measurement_qualified=bool(quality) and all(quality),
                ready_observed=independent['any_visual_ready_after_warmup'],
                real_time_factor=independent['processing_real_time_factor_after_warmup'],
                lag_growth_s=independent['lag_growth_after_warmup_s'],
                maximum_exposure_age_s=independent['maximum_consumed_exposure_age_s'],
                mean_pose_nees_6d=independent['mean_pose_nees_6d'],
                maximum_image_position_error_m=independent['maximum_image_position_error_m'],
                maximum_image_rotation_error_deg=independent['maximum_image_rotation_error_deg'],
                external_state_writes=summary['dynamics']['external_state_writes'])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--catalog', type=Path, required=True)
    p.add_argument('--duration', type=float, default=4.)
    p.add_argument('--far-depth', type=float, default=4.)
    p.add_argument('--near-depth', type=float, default=1.5)
    p.add_argument('--pointing-depth', type=float, default=1.5)
    p.add_argument('--render-port', type=int, default=5558)
    p.add_argument('--capture-port', type=int, default=8767)
    p.add_argument('--seed', type=int, default=7)
    p.add_argument('--simulation-rate', type=float, default=1.)
    args = p.parse_args()
    if not (2 <= args.duration <= 30 and .5 <= args.near_depth <= args.far_depth <= 4
            and 0 <= args.seed < 2**32 and 0 < args.simulation_rate <= 100):
        p.error('invalid scan bounds, duration or seed')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    catalog = args.catalog.resolve()
    cases = [(f'depth-{i:02d}', float(d), [0., 0., 0.])
             for i, d in enumerate(np.arange(args.far_depth, args.near_depth-.001, -.25))]
    cases += [(f'pointing-{axis}-{sign:+d}', args.pointing_depth,
               [float(sign*5) if j == axis else 0. for j in range(3)])
              for axis in range(3) for sign in (-1, 1)]
    results = []
    for name, depth, pointing in cases:
        destination = output/name
        command = [sys.executable, str(ROOT/'tools/validate_capture26_navigation.py'),
                   '--output', str(destination), '--ue', '--realtime', '--save-all-images',
                   '--catalog', str(catalog), '--duration', str(args.duration), '--depth', str(depth),
                   '--seed', str(args.seed), '--render-port', str(args.render_port),
                   '--simulation-rate', str(args.simulation_rate),
                   '--capture-port', str(args.capture_port), '--pointing-error-deg', *map(str, pointing)]
        with (output/f'{name}.log').open('wb') as log:
            completed = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                       creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        result = (summarize(destination) if completed.returncode == 0
                  else dict(output=str(destination), error='probe failed; see case log', exit_code=completed.returncode))
        results.append(result)
        report = dict(schema='space-sim-capture26-UE-prevalidation/1', integration_complete=False,
                      vision_nodes_validated=False, cases=results,
                      remaining=['braking and collision bounds', 'statistical calibration validation',
                                 'full near/far/handoff and fault acceptance'])
        (output/'scan.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
        print(json.dumps(dict(case=name, **result)), flush=True)
        if completed.returncode:
            return completed.returncode
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
