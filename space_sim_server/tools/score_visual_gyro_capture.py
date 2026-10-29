"""Fail-closed independent scorer, run in the separate OpenCV environment."""
from pathlib import Path
import argparse
import json
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'backend'),str(ROOT)]
from space_arm_platform.capture_regression import physical_fingerprint
from space_arm_platform.capture_regression_score import records,phase_evidence,score_hold,target_motion,angle,gyro_evidence


def qualify(output):
    recipe=json.loads((output/'recipe.json').read_text('utf-8'))
    infrastructure=json.loads((output/'required-processes.json').read_text('utf-8'))
    end_t=float(infrastructure['checked_sim_t'])
    controls=[r for r in records(output/'capture/visual_control.jsonl') if r['t']<=end_t]
    if not controls:raise ValueError('no visual controller evidence')
    phases=phase_evidence(controls)
    hold=score_hold(r for r in records(output/'capture/contact_audit.jsonl') if r.get('t',0)<=end_t)
    contact=hold['first_target_contact']
    if contact is None:raise ValueError('no actual inter-spacecraft contact')
    motion=target_motion(output/'capture/contact_audit.jsonl',output/'free_reference.jsonl',contact['t'],end_t=end_t)
    frozen=json.loads((output/'physical-before.json').read_text('utf-8'))
    physics_unchanged=frozen==physical_fingerprint(ROOT)==json.loads((output/'physical-after.json').read_text('utf-8'))
    initial=next(records(output/'capture/truth.jsonl'))
    initial_gap_error=float(np.linalg.norm(np.array(initial['interface_position_grip_m'])-[0,0,-recipe['initial_gap_m']]))
    initial_angle=angle(np.array(initial['R_grip_from_interface']))
    gyros=list(records(output/'capture/consumed_service_gyro.jsonl'))
    session=controls[-1]['session_id']
    gyro=gyro_evidence(gyros,controls,session,end_t)
    infrastructure_ok=(set(p['role'] for p in infrastructure['processes'])=={'backend','renderer','simulation'}
        and all(p['returncode'] is None for p in infrastructure['processes'])
        and infrastructure['checked_sim_t']>=controls[-1]['t']-.01)
    from space_arm_platform.wrist_vision_worker import WristEstimator
    calibration=json.loads((output/'calibration.json').read_text('utf-8'))
    estimator=WristEstimator(calibration)
    image_result=None;image_metadata=None
    for row in records(output/'vision/capture_samples.jsonl'):
        if row['session_id']!=session:continue
        result=estimator(Path(row['image']).read_bytes())
        if result.get('alignment',{}).get('valid') and len(result.get('detections',[]))==4:
            image_result=result;image_metadata=row;break
    if image_result is None:raise ValueError('no native four-marker PNG can be independently decoded')
    intr=image_metadata['intrinsics']
    native_K=[[intr['fx_px'],0,intr['cx_px']],[0,intr['fy_px'],intr['cy_px']],[0,0,1]]
    optics_ok=image_metadata['resolution']==[1920,1200] and np.allclose(native_K,calibration['K'],atol=1e-8)
    consumption=[r for r in controls if r.get('prediction')]
    blind=max((r['prediction'].get('blind_path_mm',0) or 0 for r in consumption),default=0)
    checks=dict(phase_sequence=phases['complete'],continuous_four_support_hold=hold['continuous_hold_s']>=recipe['hold_s'],
        no_service_self_collision=hold['self_contact'] is None,physics_unchanged=physics_unchanged,
        initial_05m_gap=initial_gap_error<.00005 and initial_angle<.01,
        real_image_and_matching_intrinsics=bool(optics_ok),gyro_contiguous_and_valid=gyro['valid'],
        all_required_processes_alive_through_hold=infrastructure_ok,
        observed_blind_prediction=0<blind<=180,
        free_reference_precontact_agreement=motion['precontact_translation_mm']<=recipe['reference_precontact_translation_mm']
            and motion['precontact_rotation_deg']<=recipe['reference_precontact_rotation_deg'],
        bounded_target_motion=motion['max_translation_mm']<=recipe['max_target_translation_mm']
            and motion['max_rotation_deg']<=recipe['max_target_rotation_deg'])
    # Preserve truth only in diagnostics, never export it back to a runtime path.
    (output/'verified_image_pose.json').write_text(json.dumps(image_result,indent=2),encoding='utf-8')
    return dict(success=all(checks.values()),checks=checks,session_id=session,scored_until_sim_s=end_t,
        continuous_four_support_hold_s=hold['continuous_hold_s'],hold_force_range_n=hold['hold_force_range_n'],
        self_contact=hold['self_contact'],first_contact=contact,phase_transitions=phases['transitions'],
        target_motion=motion,initial_gap_error_mm=initial_gap_error*1000,initial_angle_deg=initial_angle,
        gyro_evidence=gyro,maximum_blind_path_mm=blind,native_image=image_metadata,
        target_assembly_mass_kg=frozen['target_assembly_mass_kg'],target_truth_for_guidance=False,
        inter_satellite_fixed_constraints=frozen['inter_satellite_fixed_constraints'],
        scope='One retained nominal UE-image trial; not a hardware or reliability qualification.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    try:result=qualify(args.output)
    except Exception as error:result=dict(success=False,error=str(error))
    (args.output/'result.json').write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False))
    raise SystemExit(0 if result['success'] else 1)


if __name__=='__main__':main()
