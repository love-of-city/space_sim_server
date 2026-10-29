"""Reconstruct actual parking intervals from physical feedback publications.

No controller, navigation or stability implementation is imported. The scorer
checks raw positions/velocities, body rotations, folded FK and the physical
interface plane against the frozen mission geometry.
"""
import json
from pathlib import Path
import sys

import numpy as np

NODE_IDS = ('Vbar50m', 'Optical17p4m', 'VisualReady5p5m', 'PlatformStandoff')


def node_offset(row, config, geometry, vision):
    p, v, r = (np.asarray(row[k], float) for k in ('position_n_m', 'velocity_n_m_s', 'rotation_n_from_body'))
    cs, ct = np.asarray(row['com_body_m'])
    name = row['node']
    radial = p[1]/np.linalg.norm(p[1])
    normal = np.cross(p[1], v[1]); normal /= np.linalg.norm(normal)
    tangent = np.cross(normal, radial)
    if name == 'PlatformStandoff':
        return (r[1]@(np.array(geometry['surface_point_target_m'])+.5*np.array(geometry['surface_normal_target'])
                +geometry['lateral_target_m']-ct)-r[0]@(np.array(geometry['folded_tcp_body_m'])-cs))
    distance = 50.
    if name != 'Vbar50m':
        optical = r[0]@np.array(vision['body_from_optical']['left']['R_body_from_optical'])[:, 2]
        camera = .5*sum(np.asarray(vision['body_from_optical'][eye]['position_body_m']) for eye in ('left', 'right'))
        marker = np.array([.1755, .05475, .0420787001299893])
        depth = config['gnss_hold_depth_m'] if name == 'Optical17p4m' else config['visual_hold_depth_m']
        distance = (depth-optical@(r[1]@(marker-ct)-r[0]@(camera-cs)))/(optical@tangent)
    return -distance*tangent


def audit_near(rows, config, geometry, vision):
    records, violations = [], []
    index, start_ns, previous, max_position, max_velocity, max_gap = 0, None, None, 0., 0., 0.
    last_ns = None
    for row in rows:
        if row.get('node') not in NODE_IDS:
            continue
        now = row['time_ns']
        if row.get('feedback_source') != 'authoritative_simulation':
            violations.append('parking feedback was not authoritative physical state')
            break
        if last_ns is not None and (now <= last_ns or now-last_ns > 150_000_000):
            start_ns = None
            violations.append('physical parking evidence was interrupted')
        last_ns = now
        if index == 4:
            continue
        if row['node'] != NODE_IDS[index]:
            if row['node'] in NODE_IDS[index+1:]:
                violations.append('node advanced without a complete physical dwell: '+NODE_IDS[index])
                break
            continue
        offset = node_offset(row, config, geometry, vision)
        position = np.asarray(row['position_n_m']); velocity = np.asarray(row['velocity_n_m_s'])
        error = float(np.linalg.norm(position[0]-position[1]-offset))
        # A backward derivative independently checks moving target/attitude/COM
        # lever arms; no reported velocity reference or qualification is used.
        if previous is None or previous[0]['node'] != row['node']:
            speed = float('inf')
        else:
            dt = (now-previous[0]['time_ns'])*1e-9
            speed = float(np.linalg.norm(velocity[0]-velocity[1]-(offset-previous[1])/dt))
        previous = row, offset
        rs, rt = np.asarray(row['rotation_n_from_body'])
        cs, ct = np.asarray(row['com_body_m'])
        tcp = np.asarray(row['tcp_body_m'])
        gap = float((position[0]-position[1]+rs@(tcp-cs)
            -rt@(np.asarray(geometry['surface_point_target_m'])-ct))@(rt@geometry['surface_normal_target']))
        qualified = (row.get('at_node') and not row['safe_hold'] and error <= .05 and speed <= .010002
                     and (index != 3 or abs(gap-.5) <= .05))
        if not qualified:
            start_ns = None
            max_position = max_velocity = max_gap = 0.
            continue
        if start_ns is None:
            start_ns = now
        max_position, max_velocity = max(max_position, error), max(max_velocity, speed)
        max_gap = max(max_gap, abs(gap-.5))
        if now-start_ns >= 5_000_000_000:
            records.append(dict(node=NODE_IDS[index], dwell_start_ns=start_ns, release_time_ns=now,
                actual_dwell_s=(now-start_ns)*1e-9, maximum_position_error_m=max_position,
                maximum_velocity_error_m_s=max_velocity,
                maximum_tcp_gap_error_m=max_gap if index == 3 else None))
            index += 1
            start_ns = None
    return dict(schema='space-sim-capture26-demo-parking-audit/1',
        acceptance_profile=config['acceptance_profile'], records=records, violations=violations,
        all_four_physical_dwells=len(records) == 4 and not violations,
        audit_passed=len(records) == 4 and not violations)


def score_near(folder, config, geometry, vision):
    folder = Path(folder)
    with (folder/'platform-estimates.jsonl').open(encoding='utf-8') as source:
        result = audit_near((json.loads(line) for line in source), config, geometry, vision)
    (folder/'parking-audit.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    return result


def main():
    import argparse
    root = Path(__file__).resolve().parents[1]
    sys.path[:0] = [str(root), str(root/'backend')]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True, type=Path, help='completed near-stage output directory')
    args = parser.parse_args()
    summary = json.loads((args.run/'summary.json').read_text('utf-8'))
    from simulation.capture26_mission import Capture26Mission, load_vision
    from simulation.capture26_demo_geometry import load_geometry
    config = Capture26Mission(**summary['mission_configuration'])
    if not config.is_demo:
        parser.error('parking audit requires the explicit simulation-state demonstration profile')
    geometry = load_geometry(root, config)
    result = score_near(args.run, config.to_dict(), geometry, load_vision(root))
    print(json.dumps(result, indent=2))
    return 0 if result['audit_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
