"""Score truth-only node windows separately from estimated release gates."""
import argparse
import json
from pathlib import Path

import numpy as np


def node_window(rows, completion, reference, *, tool_gap=None):
    samples = [r for r in rows if completion is not None
               and completion-5.-1e-8 <= r['time_s'] <= completion+1e-8]
    if len(samples) < 101:
        return dict(passed=False, reason='missing continuous five-second evidence', samples=len(samples))
    times = np.array([r['time_s'] for r in samples])
    errors = [np.linalg.norm(np.array(r['true_com_position_m'])-reference) for r in samples]
    speeds = [np.linalg.norm(r['true_com_velocity_m_s']) for r in samples]
    gaps = ([abs(-r['true_center_gripper_m'][2]-tool_gap) for r in samples]
            if tool_gap is not None else [])
    tool_errors = ([np.linalg.norm(np.array(r['true_center_gripper_m'])-[0, 0, -tool_gap])
                    for r in samples] if tool_gap is not None else [])
    return dict(passed=bool(np.max(np.diff(times)) <= .05000001
                           and times[-1]-times[0] >= 5.-1e-8
                           and max(errors) <= .05 and max(speeds) <= .01
                           and (not gaps or max(gaps) <= .05 and max(tool_errors) <= .05)),
        samples=len(samples), start_s=float(times[0]), end_s=float(times[-1]),
        max_position_error_m=float(max(errors)), max_speed_m_s=float(max(speeds)),
        max_tool_gap_error_m=float(max(gaps)) if gaps else None,
        max_tool_position_error_m=float(max(tool_errors)) if tool_errors else None)


def score(directory):
    directory = Path(directory)
    summary = json.loads((directory/'summary.json').read_text('utf-8'))
    rows = [json.loads(line) for line in (directory/'trajectory.jsonl').read_text('utf-8').splitlines()]
    evidence = summary.get('vbar_integration', {}).get('station17_evidence') or {}
    checks = {}
    checks['vbar_platform_ownership'] = dict(passed=bool(rows) and all(
        r['policy'].get('reference_owner') == 'vbar' for r in rows))
    for name in ('Vbar50m', 'Optical17p4m'):
        gate = evidence.get(name, {})
        checks[name] = node_window(rows, gate.get('completed_at_s'), gate.get('reference_com_m'))
    events = summary.get('pointing_handoff', {}).get('events', [])
    sequence = summary.get('continuous_original_stereo', {})
    visual_frames = set()
    release, parked = sequence.get('image_release_s'), sequence.get('platform_completed_s')
    for row in rows:
        image = row.get('image') or {}
        if (release is not None and parked is not None and release <= row['time_s'] <= parked
                and image.get('time_s') == row['time_s'] and image.get('ekf_accepted')
                and (image.get('body_navigation') or {}).get('status') == 'ok'
                and row['policy'].get('reference_owner') == 'vbar'
                and np.allclose(row['policy'].get('state'), row['estimated_com_state'], atol=1e-12, rtol=0)):
            visual_frames.add(image['frame_id'])
    checks['visual_navigation_used_by_vbar'] = dict(passed=len(visual_frames) >= 10,
                                                   distinct_accepted_frames=len(visual_frames))
    next_leg = next((e['time_s'] for e in events if e['phase'] == 'GNSSBridgeNearApproach'), None)
    # Recover the frozen 5.5 m COM station from its actual hold references.
    station5 = next((r['policy']['reference_com_m'] for r in rows
                     if r['policy']['phase'] == 'GNSSBridge5p5Hold'), None)
    checks['GNSS5p5m'] = node_window(rows, next_leg, station5)
    checks['PlatformHold'] = node_window(rows, sequence.get('platform_completed_s'),
        sequence.get('platform_com_reference_m'), tool_gap=sequence.get('platform_gap_m', .5))
    capture = sequence.get('capture') or {}
    checks['force_hold'] = dict(passed=bool(capture.get('completed') and not capture.get('failure')
                                           and capture.get('longest_force_s', 0) >= 20.),
                                longest_force_s=capture.get('longest_force_s', 0),
                                failure=capture.get('failure'))
    passed = bool(summary.get('physical_run_valid') and summary.get('requested_stage_completed')
                  and all(c['passed'] for c in checks.values())
                  and summary['stepper'].get('external_state_writes') == 0)
    result = dict(schema='vbar-near23-physical-acceptance/1', passed=passed, checks=checks,
        inherited_far_handoff=False, full_orbit_to_capture_validated=False,
        scope='initialized V-bar entry, original RGB, same dynamics/navigation and physical force hold')
    (directory/'acceptance.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    result = score(parser.parse_args().directory)
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
