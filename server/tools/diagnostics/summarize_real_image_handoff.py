"""Independent scoring of real-image pointing handoff and tool-gap residence.

Recompute tool position directly from rendered authoritative rigid-body states,
cross-check the logged FK score, and separate measurement from control errors.
Two initialized experiments must never be reported as one continuous mission.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def rows(path):
    return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]


def stats(values):
    e = np.asarray(values, float)
    return dict(samples=len(e), rmse=float(np.sqrt(np.mean(e**2))), maximum=float(e.max())) if len(e) else None


def audit(folder, *, platform=False):
    summary = json.loads((folder/'summary.json').read_text())
    trace = rows(folder/'trajectory.jsonl')
    times = np.array([r['time_s'] for r in trace])
    np.testing.assert_allclose(np.diff(times), .05, atol=1e-10, rtol=0)
    assert summary['stepper']['external_state_writes'] == 0
    assert not any(r['target_contact_pairs'] for r in trace)
    for r in trace:
        expected = np.linalg.norm(np.array(r['estimated_relative_origin_eci_m'])-r['truth_relative_origin_eci_m'])*1000
        assert abs(expected-r['filtered_position_error_mm']) < 1e-8
    acquired = [r for r in trace if r['image'] and r['image']['time_s'] == r['time_s']]
    accepted = [r for r in acquired if r['image']['ekf_accepted']]
    image_epoch_range = (acquired[0]['time_s'], acquired[-1]['time_s']) if acquired else None
    image_interval = [r for r in trace if image_epoch_range and image_epoch_range[0]<=r['time_s']<=image_epoch_range[1]]
    config = json.loads((folder/'actual_images/calibration.json').read_text())
    center = np.array(config['marker_rig']['grasp_center_target_m'])
    index = {round(r['time_s']*1e9): r for r in trace}
    direct_errors, gaps, fk_errors = [], [], []
    for f in rows(folder/'actual_images/render_frames.jsonl'):
        if not f.get('capture_episode_id'): continue
        r = index[int(f['sim_time_ns'])]
        objects = {o['object_id']: o for o in f['objects']}
        target, grip = objects['teleop/capture_target'], objects['teleop/gripper']
        rot = lambda obj: Rotation.from_quat(obj['orientation_wxyz'],scalar_first=True).as_matrix()
        direct = rot(grip).T@(np.array(target['position_m'])+rot(target)@center-grip['position_m'])
        fk_errors.append(np.linalg.norm(direct-r['true_center_gripper_m']))
        assert fk_errors[-1] < 2e-6
        if platform:
            direct_errors.append((r['time_s'], float(np.linalg.norm(direct-[0,0,-summary['tool_gap_reference_m']])*1000)))
            gaps.append((r['time_s'], float(abs(-direct[2]-summary['tool_gap_reference_m'])*1000)))
    result = dict(run=str(folder.resolve()),actual_pairs=len(acquired),filter_accepted=len(accepted),
        image_translation_error_mm=stats([r['image_position_error_mm'] for r in acquired if r['image_position_error_mm'] is not None]),
        actual_body_state_vs_fk_max_m=max(fk_errors),
        navigation_error_all_mm=stats([r['filtered_position_error_mm'] for r in trace]),
        navigation_error_during_images_mm=stats([r['filtered_position_error_mm'] for r in image_interval]),
        actual_moving_reference_error_during_images_mm=stats([r['true_tracking_error_mm'] for r in image_interval]),
        original_environment_preserved=True,independent_initialized_trial=True,
        full_four_node_continuous_mission=False,realtime_latency_validated=False,
        camera_fov_deg=float(np.rad2deg(config['cameras'][0]['field_of_view_rad'])),
        cooperative_attitude_images=summary['image_uses_cooperative_attitude'])
    if platform:
        entered, completed = summary['platform_entered_s'], summary['platform_completed_s']
        window = [r for r in trace if entered is not None and entered-5-1e-9 <= r['time_s'] <= completed] if completed is not None else []
        true_speeds = np.linalg.norm(np.gradient(np.array([r['true_com_position_m'] for r in trace]), times, axis=0),axis=1)
        selected = [i for i,r in enumerate(trace) if window and window[0]['time_s'] <= r['time_s'] <= window[-1]['time_s']]
        station=np.array(trace[0]['reference_com_m'])
        assert all(np.allclose(r['reference_com_m'],station,rtol=0,atol=1e-12) for r in window)
        result.update(entered_s=entered,completed_s=completed,latched_failure=summary['platform_failure'],
            before_entry_window_s=[entered-5,entered] if entered is not None else None,
            after_entry_window_s=[entered,completed] if completed is not None else None,
            actual_platform_position_error_mm=stats([e for t,e in direct_errors if window and window[0]['time_s']<=t<=window[-1]['time_s']]),
            actual_signed_gap_error_mm=stats([e for t,e in gaps if window and window[0]['time_s']<=t<=window[-1]['time_s']]),
            actual_platform_com_station_error_mm=stats([np.linalg.norm(np.array(r['true_com_position_m'])-station)*1000 for r in window]),
            navigation_error_residence_mm=stats([r['filtered_position_error_mm'] for r in window]),
            maximum_true_rotating_speed_m_s=float(true_speeds[selected].max()) if selected else None,
            five_seconds_before_and_after_entry=bool(window and completed-entered>=5-1e-9
                and all(r['platform_estimated_qualified'] for r in window) and summary['platform_failure'] is None))
        result['platform_position_gap_speed_components_pass']=bool(
            result['five_seconds_before_and_after_entry'] and result['actual_platform_position_error_mm']['maximum']<6
            and result['actual_platform_com_station_error_mm']['maximum']<6
            and result['actual_signed_gap_error_mm']['maximum']<6 and result['maximum_true_rotating_speed_m_s']<.002)
    else:
        events = summary['pointing_handoff']['events']
        result.update(events=events,final_phase=summary['pointing_handoff']['final_phase'],
            image_coordinator_released=summary['pointing_handoff']['formal_visual_release'],
            diagnostic_visual_bypass=summary['pointing_handoff']['gnss_diagnostic_continuation'],
            maximum_target_torque_nm=summary['pointing_handoff']['maximum_target_torque_nm'])
        result['pointing_and_real_image_handoff_pass']=bool(result['image_coordinator_released']
            and not result['diagnostic_visual_bypass'] and result['final_phase']=='TerminalHold')
        settled=next((e['time_s'] for e in events if e['phase']=='StationSettle'),None)
        pointing=next((e['time_s'] for e in events if e['phase']=='TargetPointing'),None)
        if settled is not None and pointing is not None:
            station=np.array(summary['pointing_handoff']['station_com_reference_m'])
            arrival=[r for r in trace if settled-1e-9<=r['time_s']<=pointing+1e-9]
            result['arrival_before_pointing']=dict(window_s=[settled,pointing],
                actual_fixed_station_error_mm=stats([np.linalg.norm(np.array(r['true_com_position_m'])-station)*1000 for r in arrival]),
                navigation_error_mm=stats([r['filtered_position_error_mm'] for r in arrival]))
        visual_dwell=summary['pointing_handoff'].get('visual_dwell_s',0.)
        release=next((e['time_s'] for e in events if e['phase']=='VisualContinue'),None)
        if visual_dwell>0 and release is not None:
            station=np.array(summary['pointing_handoff']['station_com_reference_m'])
            window=[r for r in trace if release-visual_dwell-1e-9<=r['time_s']<=release+1e-9]
            assert all(np.allclose(r['reference_com_m'],station,atol=1e-12,rtol=0) for r in window)
            assert all(r['visual_ready'] and r['prealignment']['aligned'] for r in window)
            true_speeds=np.linalg.norm(np.gradient(np.array([r['true_com_position_m'] for r in trace]),times,axis=0),axis=1)
            selected=[i for i,r in enumerate(trace) if window[0]['time_s']<=r['time_s']<=window[-1]['time_s']]
            position=stats([np.linalg.norm(np.array(r['true_com_position_m'])-station)*1000 for r in window])
            result.update(joint_visual_station_window_s=[window[0]['time_s'],window[-1]['time_s']],
                actual_fixed_station_position_error_mm=position,
                maximum_true_rotating_speed_m_s=float(true_speeds[selected].max()),
                navigation_error_joint_dwell_mm=stats([r['filtered_position_error_mm'] for r in window]))
            result['station_visual_position_speed_components_pass']=bool(
                window[-1]['time_s']-window[0]['time_s']>=visual_dwell-1e-9
                and position['maximum']<summary['pointing_handoff']['arrival_tolerance_m']*1000
                and result['maximum_true_rotating_speed_m_s']<.002)
        if summary.get('range_transfer') is not None:
            transfer=summary['range_transfer']
            switched=next((e['time_s'] for e in transfer['events'] if e['active']=='near'),None)
            near=[r for r in trace if switched is not None and r['time_s']>=switched]
            release=next((e['time_s'] for e in events if e['phase']=='VisualContinue'),None)
            continued=[r for r in trace if release is not None and r['time_s']>=release]
            images=[r for r in near if r['image'] and r['image']['time_s']==r['time_s']]
            far_fused_after_switch=[r for r in near if switched is not None and r['time_s']>switched
                and r['image_channels']['far']['time_s']==r['time_s'] and r['image_channels']['far']['ekf_accepted']]
            result['range_transfer']=dict(switch_time_s=switched,
                switch_actual_optical_depth_m=(near[0]['true_optical_depth_m'] if near else None),
                final_channel=transfer['final_channel'],final_phase=result['final_phase'],
                near_image_pairs=len(images),near_image_filter_accepted=sum(r['image']['ekf_accepted'] for r in images),
                far_image_updates_after_switch=len(far_fused_after_switch),
                visual_ready_throughout_continuation=bool(continued) and all(r['visual_ready'] for r in continued),
                navigation_error_after_switch_mm=stats([r['filtered_position_error_mm'] for r in near]),
                near_image_error_mm=stats([r['image']['position_error_mm'] for r in images if r['image']['position_error_mm'] is not None]),
                actual_moving_reference_error_after_switch_mm=stats([r['true_tracking_error_mm'] for r in near]),
                independent_image_noise_during_overlap_assumed=True)
            result['dynamic_range_transfer_pass']=bool(near and transfer['final_channel']=='near'
                and result['pointing_and_real_image_handoff_pass']
                and result['range_transfer']['visual_ready_throughout_continuation'] and not far_fused_after_switch)
    return result


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pointing',type=Path,required=True)
    p.add_argument('--platform',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--station5p5',type=Path)
    p.add_argument('--transfer',type=Path)
    a=p.parse_args()
    result=dict(pointing=audit(a.pointing),platform=audit(a.platform,platform=True),
                combined_as_single_mission=False)
    if a.station5p5 is not None:result['station5p5']=audit(a.station5p5)
    if a.transfer is not None:result['transfer']=audit(a.transfer)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))
