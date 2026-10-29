"""Score saved image/control output against independent same-exposure truth."""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def rows(path):
    return [json.loads(line) for line in path.read_text('utf-8').splitlines() if line.strip()]


def metrics(values):
    a = np.asarray(values, float)
    return dict(count=len(a), rmse=float(np.sqrt(np.mean(a*a))), maximum=float(a.max())) if len(a) else None


def longest_hold(truth, support):
    start = previous = None; longest = 0.
    for row in truth:
        t = int(row['sim_time_ns'])/1e9; f = row.get('feedback') or {}
        good = (row.get('phase') == 'predictive_hold_force'
                and 4.5 <= f.get('pad_force_n', 0) <= 5.5
                and f.get('jaw_support_count', 0) >= support)
        if good:
            if start is None or (previous is not None and t-previous > .151): start = t
            longest = max(longest, t-start)
        else: start = None
        previous = t
    return longest


def contact_metrics(path):
    """Check every physics tick; a force excursion between images breaks hold."""
    starts = {2: None, 4: None}; longest = {2: 0., 4: 0.}
    previous = None; count = 0; peak = 0.; aborted = False
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            if row.get('type') != 'step':
                continue
            t = float(row['t']); f = row['clamp']; count += 1
            if previous is not None and t <= previous:
                raise ValueError('contact audit time must increase')
            peak = max(peak, float(f['peak_target_contact_force_n']))
            aborted |= row['phase'] == 'abort'
            for support in starts:
                good = (row['phase'] == 'hold_force'
                        and 4.5 <= f['pad_force_n'] <= 5.5
                        and f['jaw_support_count'] >= support)
                if good:
                    if starts[support] is None or (previous is not None and t-previous > .002501):
                        starts[support] = t
                    longest[support] = max(longest[support], t-starts[support])
                else:
                    starts[support] = None
            previous = t
    return dict(samples=count, peak_individual_target_contact_force_n=peak,
                aborted=aborted, supported_force_hold_2plus_jaws_s=longest[2],
                supported_force_hold_all4_jaws_s=longest[4])


def score(directory):
    truth = rows(directory/'truth.jsonl'); images = rows(directory/'measurements.jsonl')
    indexed = {(r['session_id'], str(r['source_frame_id']), str(r['sim_time_ns'])):r for r in truth}
    image_pos = []; image_rot = []; used_small = 0; mismatched = 0
    for row in images:
        p = row['packet']
        key = (p['session_id'], str(p['source_frame_id']), str(p['sim_time_ns']))
        if key not in indexed:
            mismatched += 1; continue
        ref = indexed[key]
        # Deliberate loss is excluded from delivered-vision accuracy.
        if not p.get('valid'): continue
        a = p['result']['alignment']
        image_pos.append(np.linalg.norm(np.array(a['interface_position_grip_m'])-ref['interface_position_grip_m'])*1000)
        image_rot.append(np.rad2deg(np.linalg.norm(Rotation.from_matrix(
            np.array(a['R_grip_from_interface'])@np.array(ref['R_grip_from_interface']).T).as_rotvec())))
        used_small += any(i >= 8 for ids in p['measurement']['used_ids'].values() for i in ids)
    prediction = {'visual': [], 'blind': []}; angles = {'visual': [], 'blind': []}
    phases = Counter(); pad = []; supports = []; peak = []; recoveries = []
    previous_phase = None
    for row in truth:
        phase = row.get('phase', 'unknown'); phases[phase] += 1
        if previous_phase == 'predictive_approach_encoder' and phase == 'predictive_approach_visual':
            recoveries.append(int(row['sim_time_ns'])/1e9)
        previous_phase = phase
        f = row.get('feedback') or {}
        pad.append(f.get('pad_force_n', 0)); supports.append(f.get('jaw_support_count', 0))
        peak.append(f.get('peak_target_contact_force_n', 0))
        if phase not in ('predictive_approach_visual', 'predictive_approach_encoder'): continue
        p = row.get('prediction') or {}
        if p.get('predicted_position_grip_m') is None: continue
        # Prediction is issued before this tick's physical integration. At
        # 4 mm/s the 2.5 ms timing offset can contribute about 0.01 mm.
        if abs(float(row['prediction_control_time_s'])-int(row['sim_time_ns'])/1e9) > .002501: continue
        group = 'blind' if phase.endswith('encoder') else 'visual'
        prediction[group].append(np.linalg.norm(np.array(p['predicted_position_grip_m'])-row['interface_position_grip_m'])*1000)
        angles[group].append(np.rad2deg(np.linalg.norm(Rotation.from_matrix(
            np.array(p['predicted_rotation_grip'])@np.array(row['R_grip_from_interface']).T).as_rotvec())))
    hold2 = longest_hold(truth, 2); hold4 = longest_hold(truth, 4)
    completed = (directory/'completed.json').exists()
    audit_path = directory/'contacts_400hz.jsonl'
    audit = contact_metrics(audit_path) if audit_path.exists() else None
    usable = bool(completed and images and image_pos and mismatched == 0
                  and audit and audit['samples'] and not audit['aborted'])
    result = dict(completed=completed, paired_images=len(images), unmatched_source_frames=mismatched,
        last_truth_sim_time_s=int(truth[-1]['sim_time_ns'])/1e9 if truth else None,
        delivered_valid_images=sum(bool(r['packet'].get('valid')) for r in images),
        estimator_rejected=sum(r['packet']['measurement'].get('status') != 'ok' for r in images),
        deliberately_lost_images=sum(bool(r.get('injected_loss')) for r in images),
        valid_frames_using_small_markers=used_small,
        image_interface_position_error_mm=metrics(image_pos), image_interface_rotation_error_deg=metrics(image_rot),
        approach_prediction_position_error_mm={k:metrics(v) for k,v in prediction.items()},
        approach_prediction_rotation_error_deg={k:metrics(v) for k,v in angles.items()},
        phases=dict(phases), terminal_phase=truth[-1].get('phase') if truth else None,
        supported_force_hold_2plus_jaws_s=hold2, supported_force_hold_all4_jaws_s=hold4,
        peak_individual_target_contact_force_n=max(peak, default=0),
        final_pad_force_n=pad[-1] if pad else None, final_jaw_support_count=supports[-1] if supports else None,
        feedback_record_rate_hz=10, overload_guard_rate_hz=400,
        visual_recovery_times_s=recoveries,
        final_interface_position_grip_m=truth[-1]['interface_position_grip_m'] if truth else None,
        final_lateral_alignment_error_mm=float(np.linalg.norm(truth[-1]['interface_position_grip_m'][:2])*1000) if truth else None,
        final_interface_rotation_from_aligned_deg=float(np.rad2deg(np.linalg.norm(Rotation.from_matrix(
            np.array(truth[-1]['R_grip_from_interface'])).as_rotvec()))) if truth else None,
        physics_tick_contact_audit=audit,
        physical_grasp_hold_pass=bool(audit and not audit['aborted']
                                     and audit['supported_force_hold_2plus_jaws_s'] >= 10),
        grasp_pass=bool(usable and audit['supported_force_hold_2plus_jaws_s'] >= 10),
        all_four_jaws_supported_hold_pass=bool(usable and audit['supported_force_hold_all4_jaws_s'] >= 10),
        scope='single seeded local native dynamics trial; lockstep 10 Hz actual UE images; not real-time/hardware/multi-seed qualification')
    (directory/'score.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    print(json.dumps(score(parser.parse_args().directory), indent=2))
