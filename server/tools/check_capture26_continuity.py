"""Audit source checkpoint against the receiving native scene's first epoch.

This is diagnostic evidence only, never authorization for a mission handoff.
No simulation or navigation implementation is imported by this scorer.
"""
import argparse
import json
from pathlib import Path

import numpy as np


def check(source, receiving):
    before = json.loads(Path(source).read_text('utf-8-sig'))
    after = json.loads((Path(receiving)/'restored-state.json').read_text('utf-8'))
    checks, errors = {}, {}
    for key in ('qpos_inertial', 'qvel_inertial', 'origin_position_m', 'origin_velocity_m_s'):
        a, b = (np.asarray(d['physical'][key], float) for d in (before, after))
        errors[key] = float(np.max(np.abs(a-b))) if a.shape == b.shape else None
        checks[key] = a.shape == b.shape and errors[key] <= 1e-9
    checks['common_epoch'] = (before['physical']['time_ns'] == after['physical']['time_ns']
                              == before['navigation']['state']['time_ns'] == after['navigation']['state']['time_ns'])
    # Session rebinding changes only ingress identity, not posterior/history.
    for key in ('state', 'base', 'events', 'snapshots', 'accepted', 'base_committed', 'last_visual_epoch'):
        checks['navigation/'+key] = before['navigation'][key] == after['navigation'][key]
    for key in ('gyros', 'gnss_receiver', 'gnss_pending', 'retention'):
        checks[key] = before[key] == after[key]
    for key in ('operating_reference', 'capture_reference'):
        checks[key] = before['control'][key] == after['control'][key]
    for key in ('gyro_pending', 'last_cooperative_ns', 'platform_stability', 'preparation'):
        if key in before['control'] or key in after['control']:
            source_value, receiving_value = before['control'].get(key), after['control'].get(key)
            if key == 'platform_stability' and source_value is None and before.get('phase') == 'FarReady':
                # The far hold has no near-node stability accumulator. A new
                # near stage starts an empty clock; it cannot inherit dwell
                # credit from a different reference.
                checks['control/new_stage_stability_unstarted'] = receiving_value is None or (
                    isinstance(receiving_value, dict) and receiving_value.get('schema') == 'space-sim-platform-stability/1'
                    and receiving_value.get('start_ns') is None and receiving_value.get('last_ns') is None
                    and receiving_value.get('reference') is None and receiving_value.get('failures') == [])
            else:
                checks['control/'+key] = source_value == receiving_value
    if before.get('phase') == 'NearParked' and before.get('acceptance_profile') == 'simulation_state_demo_v1':
        checks['control/parking_completion'] = (before['control'].get('parking_completion') is not None
            and before['control']['parking_completion'] == after['control'].get('parking_completion'))
    for key in ('mission_configuration', 'sensor_delivery'):
        if key in before or key in after:
            a, b = before.get(key), after.get(key)
            if key == 'sensor_delivery' and a and b:
                # Monotonic host clocks are diagnostics; packet identities and
                # physical delivery epochs must be identical across processes.
                for document in (a, b):
                    for packet in document['pending']:
                        packet.pop('age_wall_s', None)
            checks[key] = a == b
    # Forces/environment may be freshly recomputed at the same epoch, but
    # their stochastic state and last sampling epoch must remain identical.
    checks['environment'] = before['environment'] == after['environment']
    report = dict(schema='space-sim-capture26-continuity-audit/1', checks=checks,
                  maximum_absolute_errors=errors, continuity_passed=all(checks.values()),
                  source=str(Path(source).resolve()), receiving=str(Path(receiving).resolve()),
                  mission_accepted=False, integration_complete=False)
    (Path(receiving)/'continuity-audit.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('source', type=Path)
    p.add_argument('receiving', type=Path)
    args = p.parse_args()
    report = check(args.source, args.receiving)
    print(json.dumps(report, indent=2))
    return 0 if report['continuity_passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
