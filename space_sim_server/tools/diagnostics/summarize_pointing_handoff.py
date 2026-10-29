"""Audit sequential prealignment traces independently of their summary values."""
import argparse
import json
from pathlib import Path

import numpy as np


def stats(rows, key):
    values=np.array([r[key] for r in rows],float)
    if not len(values):
        return None
    return dict(samples=len(values),rmse=float(np.sqrt(np.mean(values**2))),
                p95=float(np.percentile(values,95)),maximum=float(values.max()))


def audit(path):
    summary=json.loads((path/'summary.json').read_text())
    rows=[json.loads(line) for line in (path/'trajectory.jsonl').read_text().splitlines()]
    info=summary['pointing_handoff']
    times=np.array([r['time_s'] for r in rows])
    assert np.allclose(np.diff(times),.05,atol=1e-10,rtol=0),'control epoch discontinuity'
    for row in rows:
        error=(np.array(row['estimated_relative_origin_eci_m'])-row['truth_relative_origin_eci_m'])*1000
        assert np.allclose(error,row['filtered_position_error_xyz_mm'],atol=1e-9,rtol=0)
        assert np.isclose(np.linalg.norm(error),row['filtered_position_error_mm'],atol=1e-9,rtol=0)
        actual=np.array(row['true_target_rotation_n_from_body'])
        reference=np.array(row['reference_target_rotation_n_from_body'])
        angle=np.rad2deg(np.arccos(np.clip((np.trace(actual.T@reference)-1)/2,-1,1)))
        assert abs(angle-row['true_target_reference_error_deg'])<1e-7
        assert np.linalg.norm(row['target_torque_body_nm'])<=.02+1e-12
        tracking=np.linalg.norm(np.array(row['true_com_position_m'])-row['reference_com_m'])*1000
        assert abs(tracking-row['true_tracking_error_mm'])<1e-8
        assert not row['policy']['formal_visual_release'],'no image input may grant visual release'
    phases=[e['phase'] for e in info['events']]
    required=['NavigationAcquire','StationApproach','StationSettle','TargetPointing','VisualWait']
    assert phases[:5]==required,phases
    if info['gnss_diagnostic_continuation']:
        assert phases[5:]==['GNSSDiagnosticContinue','TerminalHold'],phases
    else:
        assert phases[5:]==['SafeHold'] and info['failure_reason']=='visual_not_ready_after_pointing',phases
    started=info['events'][3]['time_s']
    waiting=info['events'][4]['time_s']
    assert started-info['events'][2]['time_s']>=5-1e-9
    first_aligned=next(r for r in rows if r['prealignment']['aligned'])
    assert first_aligned['prealignment']['dwell_s']>=5-1e-9
    assert waiting-first_aligned['time_s']>=5-1e-9
    before=[r for r in rows if r['time_s']<started]
    active=[r for r in rows if r['policy']['phase'] in {'TargetPointing','VisualWait'}]
    settled=[r for r in rows if r['policy']['phase']=='VisualWait']
    for row in active:
        assert np.allclose(row['reference_com_m'],info['station_com_reference_m'],atol=1e-12,rtol=0)
        assert np.allclose(row['policy']['reference_velocity_m_s'],0,atol=1e-12,rtol=0)
    assert max(r['true_target_reference_error_deg'] for r in before)<1.,'target slewed before arrival'
    assert summary['stepper']['external_state_writes']==0
    assert summary['vision_updates']==0
    assert not any(r['target_contact_pairs'] for r in rows)
    assert summary['physical_run_valid']
    release=next((r for r in rows if r['time_s']==waiting),None)
    return dict(run=path.name,seed=summary['seed'],station_nominal_optical_depth_m=info['station_nominal_optical_depth_m'],
        initial_target_tilt_deg=info['initial_target_tilt_deg'],audited_epochs=len(rows),
        events=info['events'],slew_started_s=started,first_prealigned_s=first_aligned['time_s'],
        slew_to_first_prealigned_s=first_aligned['time_s']-started,
        slew_to_visual_wait_s=waiting-started,
        at_visual_wait=dict(attitude_error_deg=release['prealignment']['attitude_error_deg'],
            rate_error_rad_s=release['prealignment']['rate_error_rad_s'],
            true_tracking_error_mm=release['true_tracking_error_mm'],
            true_optical_depth_m=release['true_optical_depth_m']),
        during_pointing_and_wait=dict(position_error_mm=stats(active,'filtered_position_error_mm'),
            true_com_tracking_error_mm=stats(active,'true_tracking_error_mm')),
        settled_wait=dict(position_error_mm=stats(settled,'filtered_position_error_mm'),
            true_com_tracking_error_mm=stats(settled,'true_tracking_error_mm')),
        all_position_error_mm=stats(rows,'filtered_position_error_mm'),
        max_service_attitude_error_deg=max(r['service_attitude_error_deg'] for r in rows),
        maximum_target_torque_nm=max(np.linalg.norm(r['target_torque_body_nm']) for r in rows),
        diagnostic_continuation=info['gnss_diagnostic_continuation'],
        final_phase=info['final_phase'],final_reason=info['failure_reason'],
        terminal_true_optical_depth_m=rows[-1]['true_optical_depth_m'],
        terminal_true_tracking_error_mm=rows[-1]['true_tracking_error_mm'],
        physical_run_valid=summary['physical_run_valid'],formal_visual_release=False,
        scope='actual articulated dynamics; ideal attitude/rate/encoders; GNSS navigation; no images; no web mission takeover')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs',nargs='+',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    results=[audit(path) for path in args.runs]
    args.output.write_text(json.dumps(dict(results=results),indent=2),encoding='utf-8')
    for result in results:
        print(json.dumps(result,indent=2))
