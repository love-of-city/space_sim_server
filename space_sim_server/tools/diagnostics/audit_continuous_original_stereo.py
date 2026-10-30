"""Independently score the original-stereo continuous trial's saved history."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def stats(values):
    values = np.asarray(values, float)
    if not len(values):
        return None
    return dict(samples=len(values), rmse=float(np.sqrt(np.mean(values**2))),
                maximum=float(np.max(values)), p95=float(np.percentile(values, 95)))


def audit(directory):
    path = directory/'trajectory.jsonl'
    rows = []
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                if not line.endswith('\n'):
                    break
                raise
    if not rows:
        raise ValueError('no trajectory samples')
    times = np.array([r['time_s'] for r in rows])
    assert np.allclose(np.diff(times), .05, atol=1e-9, rtol=0)
    phases = [r['policy']['phase'] for r in rows]
    transitions = []
    for i, phase in enumerate(phases):
        if not i or phase != phases[i-1]:
            transitions.append(dict(time_s=float(times[i]), phase=phase))
    actual = np.array([r['true_com_position_m'] for r in rows])
    velocity = np.gradient(actual, times, axis=0)
    averaged = np.diff(actual, axis=0)/np.diff(times)[:, None]

    def window(phase, title, *, platform=False):
        begins = next((i for i, name in enumerate(phases) if name == phase), None)
        if begins is None:
            return dict(node=title, result='not_reached')
        ends = next((i for i in range(begins+1, len(phases)) if phases[i] != phase), None)
        if ends is None:
            return dict(node=title, result='in_progress')
        # A SafeHold transition is not a successful station admission.
        if phases[ends] == 'SafeHold':
            return dict(node=title, result='failed_before_release')
        start_s = times[ends]-5.
        indices = np.flatnonzero((times >= start_s-1e-8) & (times <= times[ends]+1e-8))
        if len(indices) != 101:
            return dict(node=title, result='insufficient_dwell_samples')
        reference = np.array(rows[indices[0]]['reference_com_m'])
        assert all(np.allclose(rows[i]['reference_com_m'], reference, rtol=0, atol=1e-9) for i in indices)
        errors = np.linalg.norm(actual[indices]-reference, axis=1)*1000
        central = np.linalg.norm(velocity[indices], axis=1)*1000
        intervals = np.linalg.norm(averaged[indices[0]:indices[-1]], axis=1)*1000
        speed = max(float(central.max()), float(intervals.max()))
        result = dict(node=title, result='scored', window_s=[float(times[indices[0]]), float(times[indices[-1]])],
            samples=len(indices), actual_fixed_station_position_error_mm=stats(errors),
            maximum_sampled_speed_mm_s=speed,
            navigation_position_error_mm=stats([rows[i]['filtered_position_error_mm'] for i in indices]),
            position_pass=bool(errors.max() <= 50), speed_pass=bool(speed <= 10),
            five_second_window_pass=True,
            all_rows_visual_ready=all(rows[i]['visual_ready'] for i in indices),
            all_rows_prealigned=all(rows[i]['prealignment']['aligned'] for i in indices))
        if platform:
            tool = np.array([rows[i]['true_center_gripper_m'] for i in indices])
            gap = json.loads((directory/'summary.json').read_text())['continuous_original_stereo']['platform_gap_m']
            result['actual_tool_position_error_mm'] = stats(np.linalg.norm(tool-[0., 0., -gap], axis=1)*1000)
            result['actual_tool_gap_error_mm'] = stats(np.abs(-tool[:, 2]-gap)*1000)
            result['tool_position_pass'] = result['actual_tool_position_error_mm']['maximum'] <= 50
            result['tool_gap_pass'] = result['actual_tool_gap_error_mm']['maximum'] <= 50
        return result

    images = []
    seen = set()
    for row in rows:
        image = row.get('image')
        if image is None or image['frame_id'] in seen:
            continue
        seen.add(image['frame_id'])
        images.append(dict(image, true_optical_depth_m=row['true_optical_depth_m']))
    accepted = [i for i in images if i['ekf_accepted']]
    ready = next((r for r in rows if r['visual_ready']), None)
    moving = [r for r in rows if r['policy']['phase'] in {'GNSSBridge5p5Approach', 'GNSSBridge5p5Hold',
              'GNSSBridgeNearApproach', 'OriginalStereoAcquire', 'VisualApproach', 'PlatformEntry', 'PlatformHold'}]
    result = dict(source_trajectory_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        sampled_physics_history_contiguous=True, sample_hz=20,
        original_camera_only=True, extra_long_focus_camera_used=False,
        initialized_at_17p9m_not_far_flight=True, production_policy_pass_claim=False,
        diagnostic_gnss_bridge_changes_12m_and_5p5m_visual_assumptions=True,
        phases=transitions, last_time_s=float(times[-1]), last_phase=phases[-1],
        navigation_error_all_mm=stats([r['filtered_position_error_mm'] for r in rows]),
        navigation_error_after20s_mm=stats([r['filtered_position_error_mm'] for r in rows if r['time_s']>=20]),
        image_pairs=len(images), image_pairs_accepted=len(accepted),
        first_filter_accepted_image=next(iter(accepted), None),
        first_visual_ready=(None if ready is None else dict(time_s=ready['time_s'],
            true_optical_depth_m=ready['true_optical_depth_m'], image=ready['image'])),
        maximum_attitude_tracking_error_during_approach_deg=max(
            (r['prealignment']['attitude_error_deg'] for r in moving), default=None),
        prealignment_continuously_valid_during_approach=bool(moving) and all(r['prealignment']['aligned'] for r in moving),
        nodes=[window('Station17Hold','17.4m GNSS stop'),
               window('GNSSBridge5p5Hold','5.5m GNSS-only diagnostic stop'),
               window('OriginalStereoAcquire','near original-camera visual acquisition')],
        velocity_method='20Hz actual COM position differences in rotating guidance frame; no claim about between-sample peaks')
    summary_path = directory/'summary.json'
    failure_path = directory/'failure.json'
    if failure_path.exists():
        result['run_failure'] = json.loads(failure_path.read_text(encoding='utf-8'))
        result['requested_stage_completed'] = False
    if summary_path.exists():
        summary = json.loads(summary_path.read_text())
        result['nodes'] += [window('PlatformEntry','0.5m platform entry',platform=True),
                            window('PlatformHold','0.5m platform residence',platform=True)]
        result['requested_stage_completed'] = summary.get('requested_stage_completed',False)
        result['continuous_original_stereo'] = summary['continuous_original_stereo']
        result['external_physical_state_writes'] = summary['stepper'].get('external_state_writes')
        result['all_scored_station_position_speed_components_pass'] = all(
            n.get('position_pass',False) and n.get('speed_pass',False) and n.get('five_second_window_pass',False)
            for n in result['nodes'])
        force_path = directory/'contact_audit_force.jsonl'
        force_hz = 400
        if not force_path.exists():
            force_path, force_hz = directory/'grasp_feedback.jsonl', 100
        if force_path.exists():
            force_rows = [json.loads(line) for line in force_path.read_text(encoding='utf-8').splitlines()]
            force_times = np.array([r['time_s'] for r in force_rows])
            if len(force_times) > 1:
                assert np.allclose(np.diff(force_times), 1/force_hz, atol=1e-8, rtol=0)
            since, longest = None, 0.
            for row in force_rows:
                feedback = row['feedback']
                qualified = (row['phase'] == 'hold_force' and feedback['jaw_support_count'] >= 4
                             and 4.5 <= feedback['pad_force_n'] <= 5.5)
                if not qualified:
                    since = None
                elif since is None:
                    since = row['time_s']
                if since is not None:
                    longest = max(longest, row['time_s']-since)
            result['grasp_force_limits'] = dict(
                samples=len(force_rows), sample_hz=force_hz,
                source_file=force_path.name, source_sha256=hashlib.sha256(force_path.read_bytes()).hexdigest(),
                maximum_peak_contact_force_n=max((r['feedback']['peak_target_contact_force_n'] for r in force_rows),default=0),
                maximum_pad_compression_m=max((r['feedback']['pad_compression_m'] for r in force_rows),default=0),
                maximum_jaw_force_n=max((r['feedback']['jaw_force_n'] for r in force_rows),default=0),
                maximum_sampled_four_jaw_force_streak_s=longest,
                self_contact_samples=sum(r.get('self_contact_count', 0) > 0 for r in force_rows))
            force = result['grasp_force_limits']
            force['force_and_compression_limits_pass'] = bool(force['maximum_peak_contact_force_n'] <= 10
                and force['maximum_pad_compression_m'] <= .002 and force['maximum_jaw_force_n'] <= 20
                and force['self_contact_samples'] == 0)
            force['four_jaw_force_hold_10s_pass'] = bool(longest >= 10-1e-8)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args = parser.parse_args()
    result = audit(args.directory)
    (args.directory/'independent_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:result[k] for k in ['last_time_s','last_phase','image_pairs','image_pairs_accepted','nodes']},
                     ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
