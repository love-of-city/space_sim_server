"""Independent image error and timing score for an actual native probe.

No estimator/controller code is imported. Truth is consumed here after the run,
at the exposure epoch, and cannot be used as a replacement observation.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def rotation_error(observed, actual):
    """SO(3) log for independent scoring; needs only the native NumPy runtime."""
    r = np.asarray(observed)@np.asarray(actual).T
    angle = float(np.arccos(np.clip((np.trace(r)-1)/2, -1., 1.)))
    vee = np.array([r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1]])
    if angle < 1e-7:
        return .5*vee
    if np.pi-angle < 1e-5:
        _, vectors = np.linalg.eigh((r+r.T)/2)
        axis = vectors[:, -1]
        return axis*angle*(-1 if axis@vee < 0 else 1)
    return vee*(angle/(2*np.sin(angle)))


def score(folder, warmup_s=2.):
    folder = Path(folder)
    summary = json.loads((folder/'summary.json').read_text('utf-8'))
    configuration = (summary.get('vision') or {}).get('configuration')
    if configuration is None:
        raise ValueError('no actual image configuration in this run')
    digest = hashlib.sha256(json.dumps(configuration, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
    if digest != summary['calibration_id']:
        raise ValueError('scoring calibration does not match the image session')
    truth = {round(r['time_s']*1e9): r for r in map(json.loads, (folder/'score.jsonl').read_text().splitlines())}
    com = [np.array(summary['body_properties'][n]['com_body_m']) for n in ('cubesat_bus', 'capture_target')]
    camera = configuration['body_from_optical']['left']
    camera_r = np.array(camera['R_body_from_optical'])
    camera_offset = np.array(camera['position_body_m'])-com[0]
    errors, ages, wall_ages, accepted_epochs, accepted_receives = [], [], [], [], []
    wall_receives, wall_sources = [], []
    live_clock_samples = 0
    pipeline_ms = {name: [] for name in ('source_to_readback', 'readback_to_capture',
                                        'capture_to_receiver', 'source_to_solution')}
    for packet in map(json.loads, (folder/'measurements.jsonl').read_text().splitlines()):
        if packet.get('kind') != 'visual':
            continue
        image = packet['result']
        exposure_ns = int(image['sim_time_ns'])
        age = (packet['control_time_ns']-exposure_ns)*1e-9
        if ('receive_wall_time_ns' in packet and all('source_wall_time_ns' in image.get('capture_timing', {}).get(eye, {})
                                                    for eye in ('left', 'right'))):
            live_clock_samples += 1
            source = min(int(image['capture_timing'][eye]['source_wall_time_ns']) for eye in ('left', 'right'))
            wall_ages.append((packet['receive_wall_time_ns']-source)*1e-9)
            if packet['accepted']:
                wall_receives.append(packet['receive_wall_time_ns'])
                wall_sources.append(source)
        ages.append(age)
        if packet['accepted']:
            accepted_epochs.append(exposure_ns)
            accepted_receives.append(packet['control_time_ns'])
        for eye in image.get('capture_timing', {}).values():
            intervals = [('source_to_readback', eye.get('source_wall_time_ns'), eye.get('readback_completed_wall_time_ns')),
                         ('readback_to_capture', eye.get('readback_completed_wall_time_ns'), eye.get('capture_wall_time_ns')),
                         ('capture_to_receiver', eye.get('capture_wall_time_ns'), eye.get('received_wall_time_ns')),
                         ('source_to_solution', eye.get('source_wall_time_ns'), image.get('solved_wall_time_ns'))]
            for name, start, end in intervals:
                if start is not None and end is not None:
                    pipeline_ms[name].append((int(end)-int(start))/1e6)
        if image.get('status') != 'ok' or exposure_ns not in truth:
            continue
        physical = truth[exposure_ns]
        local_com = [np.array(x) for x in physical.get('truth_com_body_m', com)]
        camera_offset = np.array(camera['position_body_m'])-local_com[0]
        positions = np.array(physical['truth_com_position_n_m'])
        rs, rt = np.array(physical['truth_rotation_n_from_body'])
        nc = rs@camera_r
        target_origin = positions[1]-rt@local_com[1]
        camera_origin = positions[0]+rs@camera_offset
        actual_t, actual_r = nc.T@(target_origin-camera_origin), nc.T@rt
        observed = image['pose_target_to_left']
        residual = np.r_[np.array(observed['t_m'])-actual_t, rotation_error(observed['R'], actual_r)]
        c = np.array(image['pose_covariance']['matrix'])
        nees = float(residual@np.linalg.solve(c, residual))
        errors.append(dict(source_frame_id=image['source_frame_id'], exposure_ns=exposure_ns,
                           position_error_m=residual[:3].tolist(), rotation_error_rad=residual[3:].tolist(), nees_6d=nees,
                           navigation_accepted=packet['accepted']))
    windows = json.loads((folder/'performance.json').read_text())
    warm = next((w for w in windows if w['sim_time_s'] >= warmup_s), windows[0])
    end = windows[-1]
    elapsed = end['sim_time_s']-warm['sim_time_s']
    compute = end['compute_s']-warm['compute_s']
    after_warmup = [row['visual'] for epoch, row in truth.items() if epoch >= warmup_s*1e9]
    report = dict(schema='space-sim-capture26-independent-score/1', integration_complete=False,
        source_run=str(folder.resolve()), calibration_id=digest, actual_image_solutions=len(errors),
        accepted_image_solutions=sum(e['navigation_accepted'] for e in errors),
        maximum_consumed_exposure_age_s=max(ages, default=None),
        maximum_accepted_exposure_interval_s=float(max(np.diff(accepted_epochs), default=0))*1e-9,
        maximum_effective_receive_interval_s=float(max(np.diff(accepted_receives), default=0))*1e-9,
        maximum_wall_pipeline_age_s=max(wall_ages, default=None),
        maximum_wall_receive_interval_s=float(max(np.diff(wall_receives), default=0))*1e-9,
        clock_scope='simulation time; wall clock diagnostics separate',
        live_clock_samples=live_clock_samples,
        pipeline_ms={name: dict(median=float(np.median(values)), maximum=float(max(values)))
                     for name, values in pipeline_ms.items() if values},
        maximum_image_position_error_m=max((float(np.linalg.norm(e['position_error_m'])) for e in errors), default=None),
        maximum_image_rotation_error_deg=max((float(np.rad2deg(np.linalg.norm(e['rotation_error_rad']))) for e in errors), default=None),
        mean_pose_nees_6d=float(np.mean([e['nees_6d'] for e in errors])) if errors else None,
        mean_position_bias_m=np.mean([e['position_error_m'] for e in errors], axis=0).tolist() if errors else None,
        mean_rotation_bias_rad=np.mean([e['rotation_error_rad'] for e in errors], axis=0).tolist() if errors else None,
        fraction_inside_95percent_6d_ellipsoid=float(np.mean([e['nees_6d'] <= 12.592 for e in errors])) if errors else None,
        fraction_inside_99percent_6d_ellipsoid=float(np.mean([e['nees_6d'] <= 16.812 for e in errors])) if errors else None,
        warmup_s=warm['sim_time_s'], processing_real_time_factor_after_warmup=elapsed/compute if compute > 0 else None,
        performance_required_for_acceptance=False,
        requested_simulation_rate=summary.get('requested_simulation_rate', 1.),
        pacing_backlog_s=summary.get('pacing_backlog_s'),
        lag_growth_after_warmup_s=(end['wall_s']-end['sim_time_s'])-(warm['wall_s']-warm['sim_time_s']),
        any_visual_ready_after_warmup=any(r['ready'] for r in after_warmup),
        model_parameters_unchanged=summary['model_assets_modified'] is False,
        external_state_writes=summary['dynamics']['external_state_writes'],
        scope='short native probe only; no mission, handoff, collision or statistical consistency certification', errors=errors)
    (folder/'independent-score.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    parser.add_argument('--warmup', type=float, default=2.)
    args = parser.parse_args()
    report = score(args.folder, args.warmup)
    print(json.dumps({key: value for key, value in report.items() if key != 'errors'}, indent=2))


if __name__ == '__main__':
    main()
