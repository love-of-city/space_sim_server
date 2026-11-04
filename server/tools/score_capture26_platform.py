"""Recompute platform dwell from recorded estimates without control imports.

This audits every navigation update, not the slower webpage or score refresh.
Physical truth and image covariance are scored separately. No state is written.
"""
import argparse
import json
from pathlib import Path

import numpy as np


def audit_rows(rows, configuration, vision, inherited=None):
    mount = vision['body_from_optical']
    camera = .5*(np.array(mount['left']['position_body_m'])+mount['right']['position_body_m'])
    optical = np.array([[-1., 0., 0.], [0., 0., -1.], [0., -1., 0.]])
    marker = np.array([.1755, .05475, .0420787001299893])
    start = previous = reference = None
    if inherited:
        start, previous, reference = inherited['start_ns'], inherited['last_ns'], inherited['reference']
    first_stable = first_release = None
    mismatches, premature_release, rows_seen = [], [], 0
    stable = False
    for row in rows:
        now, depth, node = row['time_ns'], row['depth_m'], row['node']
        p, v, r, w, com, dc = (np.array(row[name]) for name in ('position_n_m', 'velocity_n_m_s',
            'rotation_n_from_body', 'rates_body_rad_s', 'com_body_m', 'com_velocity_body_m_s'))
        axis = r[1]@optical[:, 2]
        target_lever = r[1]@(marker-com[1]-depth*optical[:, 2])
        service_lever = r[0]@(camera-com[0])
        offset = target_lever-service_lever
        lever_rate = (np.cross(r[1]@w[1], target_lever)-np.cross(r[0]@w[0], service_lever)
                      +r[0]@dc[0]-r[1]@dc[1])
        current_reference = ['optical_depth_m', depth]
        at_node = True
        if node == 'COM50m':
            distance = row['reference_com_distance_m']
            current_reference = ['com_distance_m', distance]
            at_node = (distance == configuration['com_hold_m']
                       and abs(float(np.linalg.norm(offset))-distance) < 1e-7)
            lever_rate -= float(offset@lever_rate/(offset@axis))*axis
        position_error = float(np.linalg.norm(p[1]+offset-p[0]))
        # Reference motion is disqualified below; parked velocity includes all
        # timestamped encoder lever and target angular-rate terms.
        velocity_error = float(np.linalg.norm(v[0]-v[1]-lever_rate))
        terminal = node == 'PlatformStandoff' or row['stage'] == 'operations'
        targets = {'GNSS17p4m': configuration['gnss_hold_depth_m'], 'VisualAcquire': configuration['acquisition_depth_m'],
                   'VisualReady': configuration['readiness_depth_m'], 'PlatformStandoff': configuration['terminal_depth_m']}
        if node != 'COM50m' and row['stage'] != 'operations':
            at_node = depth == targets[node]
        p_cov, a_cov = (np.array(row[key]) for key in ('position_covariance_m2', 'attitude_covariance_rad2'))
        covariance_valid = all(c.shape == (3, 3) and np.isfinite(c).all()
            and np.allclose(c, c.T, atol=1e-10) and np.linalg.eigvalsh(c)[0] > 0 for c in (p_cov, a_cov))
        healthy = (all(row['gyro_health']) and row['process_healthy'] and not row['safe_hold']
                   and row['last_gnss_ns'] is not None and 0 <= now-row['last_gnss_ns'] <= 350_000_000
                   and row['last_cooperative_ns'] is not None and 0 <= now-row['last_cooperative_ns'] <= 60_000_000)
        conditions = (healthy and position_error <= (.006 if terminal else .05) and velocity_error <= .002
                      and float(np.linalg.norm(w[0])) <= configuration['service_angular_rate_limit_rad_s']
                      and covariance_valid and np.linalg.eigvalsh(p_cov)[-1] < .04**2 and at_node)
        if (not conditions or current_reference != reference
                or previous is not None and not 0 <= now-previous <= 150_000_000):
            start = None
        if conditions and start is None:
            start = now
        stable = start is not None and now-start >= 5_000_000_000
        if stable != row['reported_platform_stable'] and len(mismatches) < 20:
            mismatches.append(now)
        if stable and first_stable is None:
            first_stable = now
        if row['released']:
            if first_release is None:
                first_release = now
            if not (terminal and stable and row['visual_ready'] and row['visual_count'] >= 10) and len(premature_release) < 20:
                premature_release.append(now)
        reference, previous = current_reference, now
        rows_seen += 1
    return dict(schema='space-sim-capture26-platform-audit/1', navigation_updates=rows_seen,
        first_platform_stable_ns=first_stable, first_release_ns=first_release,
        final_platform_stable=stable, final_stable_duration_s=0. if start is None else (previous-start)*1e-9,
        stability_mismatch_epochs_ns=mismatches, premature_release_epochs_ns=premature_release,
        audit_passed=bool(rows_seen) and not mismatches and not premature_release, integration_complete=False)


def score(folder):
    folder = Path(folder)
    summary = json.loads((folder/'summary.json').read_text('utf-8'))
    root = Path(__file__).resolve().parents[1]
    vision = json.loads((root/'config/vision/capture26_rendezvous_stereo.json').read_text('utf-8'))
    source = summary['settings'].get('handoff')
    inherited = (json.loads(Path(source).read_text('utf-8'))['control'].get('platform_stability')
                 if source and summary['settings'].get('mission_stage') == 'operations' else None)
    with (folder/'platform-estimates.jsonl').open(encoding='utf-8') as handle:
        report = audit_rows(map(json.loads, handle), summary['mission_configuration'], vision, inherited)
    (folder/'platform-audit.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    print(json.dumps(score(parser.parse_args().folder), indent=2))
