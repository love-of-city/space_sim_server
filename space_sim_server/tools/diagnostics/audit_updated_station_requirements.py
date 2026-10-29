"""Re-score saved stationary trials against the 2026-10-29 requirements.

No dynamics or mission limits are changed. Fixed-reference control errors,
navigation estimation errors and sampled rotating-frame speeds remain separate.
Independent trials do not establish a continuous mission or far-flight handoff.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def audit(node, folder, start, end, *, platform=False):
    summary=json.loads((folder/'summary.json').read_text(encoding='utf-8'))
    rows=[json.loads(line) for line in (folder/'trajectory.jsonl').read_text(encoding='utf-8').splitlines()]
    times=np.array([r['time_s'] for r in rows])
    np.testing.assert_allclose(np.diff(times),.05,rtol=0,atol=1e-10)
    selected=(times>=start-1e-9)&(times<=end+1e-9)
    indices=np.flatnonzero(selected)
    assert indices.size>=101 and times[indices[-1]]-times[indices[0]]>=5-1e-9
    assert indices[0]>0 and indices[-1]<len(rows)-1
    station=np.array(rows[indices[0]]['reference_com_m'])
    assert all(np.allclose(rows[i]['reference_com_m'],station,atol=1e-12,rtol=0) for i in indices)
    positions=np.array([r['true_com_position_m'] for r in rows])
    error=np.linalg.norm(positions[selected]-station,axis=1)
    # Desired rotating-frame station velocity is zero. Differentiate the logged
    # truth positions, without substituting estimated EKF velocity for truth.
    central=np.linalg.norm(np.gradient(positions,times,axis=0)[selected],axis=1)
    one_sided=np.diff(positions,axis=0)/np.diff(times)[:,None]
    interval_speed=np.linalg.norm(one_sided[indices[0]-1:indices[-1]+1],axis=1)
    conservative_sampled_speed=float(max(central.max(),interval_speed.max()))
    assert summary['stepper']['external_state_writes']==0
    assert not any(r['target_contact_pairs'] for r in rows)
    navigation=np.array([rows[i]['filtered_position_error_mm'] for i in indices])
    result=dict(node=node,source_run=str(folder.resolve()),window_s=[start,end],samples=int(indices.size),
        continuous_sampled_duration_s=float(times[indices[-1]]-times[indices[0]]),sample_hz=20,
        reference='fixed COM station, rotating target-orbit guidance frame',
        position_requirement_m=.05,speed_requirement_m_s=.01,dwell_requirement_s=5.,
        maximum_actual_fixed_station_position_error_mm=float(error.max()*1000),
        maximum_actual_central_difference_speed_mm_s=float(central.max()*1000),
        maximum_adjacent_interval_average_speed_mm_s=float(interval_speed.max()*1000),
        conservative_sampled_speed_mm_s=conservative_sampled_speed*1000,
        maximum_estimated_rotating_speed_mm_s=max(rows[i]['estimated_rotating_speed_m_s'] for i in indices)*1000,
        maximum_navigation_estimation_error_mm=float(navigation.max()),
        position_pass=bool(error.max()<=.05),speed_pass=bool(conservative_sampled_speed<=.01),
        five_second_sampled_window_pass=True,independently_initialized_trial=True,
        all_mission_common_gates_validated=False,continuous_mission_validated=False,
        source_trajectory_sha256=hashlib.sha256((folder/'trajectory.jsonl').read_bytes()).hexdigest())
    if platform:
        assert summary['tool_gap_reference_m']==.5
        center=np.array([rows[i]['true_center_gripper_m'] for i in indices])
        relative_error=np.linalg.norm(center-[0,0,-.5],axis=1)
        gap_error=np.abs(-center[:,2]-.5)
        assert summary['platform_failure'] is None
        result.update(final_working_point_assumption='same h=0.5 m tool-gap target as preceding plan',
            maximum_actual_tool_relative_position_error_mm=float(relative_error.max()*1000),
            maximum_actual_TCP_axial_gap_error_mm=float(gap_error.max()*1000),
            tool_position_pass=bool(relative_error.max()<=.05),TCP_gap_pass=bool(gap_error.max()<=.05),
            original_before_after_five_second_residence_completed=bool(
                summary['platform_completed_s']-summary['platform_entered_s']>=5-1e-9))
    result['new_position_speed_dwell_components_pass']=bool(result['position_pass'] and result['speed_pass']
        and result.get('tool_position_pass',True) and result.get('TCP_gap_pass',True))
    result['uses_only_original_stereo_or_GNSS']=summary.get('image_channel') not in {'long','dual'}
    result['real_image_updates']=summary['vision_updates']
    result['visual_readiness_components_pass']=bool(summary['vision_updates'] and all(rows[i]['visual_ready'] for i in indices))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--server-root',type=Path,default=Path(__file__).resolve().parents[2])
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    root=args.server_root.resolve()
    base=root/'run/dual_range_vision_20261028'
    cases=[('50 m COM',root/'run/node_acceptance_20261028/node50_com_short_leg_s7',41.25,46.25,False),
           ('17.4 m optical',root/'run/pointing_handoff_20261028/station17p4_90_s7',41.25,46.25,False),
           ('5.50 m optical',root/'run/pointing_handoff_20261028/station5p5_90_acquire_s7',41.25,46.25,False),
           ('final work station (h=0.5 m)',base/'platform_gap0p5_epoch_fixed_s7',1.,11.,True)]
    near=[audit(n,f,s,e,platform=p) for n,f,s,e,p in cases]
    assert all(r['uses_only_original_stereo_or_GNSS'] for r in near)
    result=dict(requirements_date='2026-10-29',new_limits_applied_to_production=False,
        near_station_results=near,all_four_near_station_components_pass=all(r['new_position_speed_dwell_components_pass'] for r in near),
        far_110m=dict(position_requirement_m=2.,speed_requirement_m_s=.01,dwell_requirement_s=5.,
            result='not_verified',reason='no saved far-flight trajectory found in available workspaces',
            existing_code_limits=dict(range_error_m=5.,rotating_relative_speed_m_s=.2,hold_s=120.),
            existing_limits_are_not_proof_of_new_acceptance=True),
        velocity_scoring_scope='finite differences of 20 Hz truth COM positions; not independently sampled instantaneous velocities or a proof between samples',
        retained_assumptions='original stereo and saved scene, ideal cooperative attitude and encoders, single GNSS seed; far station components are GNSS-only',
        extra_camera_candidate_withdrawn=True,far_real_image_readiness_verified=False,
        separate_grasp_accuracy_requirement_changed=False,full_five_stage_continuous_mission_pass=False)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps(result,indent=2,ensure_ascii=False))


if __name__=='__main__':
    main()
