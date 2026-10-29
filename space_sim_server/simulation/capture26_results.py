"""Independent stage scoring and complete continuation serialization.

Truth records are consumed after control has stopped. They cannot replace a
sensor observation or alter a propagated state.
"""
import json
from pathlib import Path

import numpy as np

from simulation.capture26_handoff import SCHEMA, DEMO_SCHEMA, validate
from simulation.capture26_mission import MODEL_ID
from simulation.orbital_rendezvous.reference import FlightProfile


def write_stage_result(output, args, summary, rows, windows, nav, gyros, receiver, pending,
                       retention, capture, stepper, control, approach, contacts,
                       environment, mission_id, epoch_ns, ephemeris_epoch):
    stage = args.mission_stage
    final = rows[-1]
    now_ns = round(final['time_s']*1e9)
    guidance = final.get('guidance') or {}
    warm = next((w for w in windows if w['sim_time_s'] >= 2.), windows[0])
    end = windows[-1]
    elapsed, compute = end['sim_time_s']-warm['sim_time_s'], end['compute_s']-warm['compute_s']
    capacity = elapsed/compute if compute > 0 else 0.
    lag_growth = (end['wall_s']-end['sim_time_s'])-(warm['wall_s']-warm['sim_time_s'])
    checks = dict(completed_stage=bool(guidance.get('released')),
                  native_state_not_overwritten=summary['dynamics']['external_state_writes'] == 0,
                  no_spacecraft_contact=not contacts,
                  gyros_healthy=all(final['gyro_health']), gnss_healthy=final['gnss_healthy'],
                  common_sensor_epoch=now_ns % 100_000_000 == 0)
    demo = summary['mission_configuration'].get('acceptance_profile') == 'simulation_state_demo_v1'
    if demo:
        checks.pop('gyros_healthy')
        checks.pop('gnss_healthy')
        checks['authoritative_simulation_feedback'] = guidance.get('feedback_source') == 'authoritative_simulation'
    performance = dict(required_for_acceptance=False,
        requested_simulation_rate=getattr(args, 'simulation_rate', 1.),
        wall_clock_pacing=bool(args.realtime),
        processing_real_time_factor_after_warmup=capacity,
        lag_growth_against_1x_after_warmup_s=lag_growth,
        achieved_1x=capacity >= 1.)
    preparation = getattr(control, 'joint_preparation', None)
    if (summary.get('mission_configuration') or {}).get('geometry_certificate_sha256'):
        checks['certified_joint_preparation_complete'] = bool(preparation and preparation.complete and not preparation.failed)
    if demo and stage == 'near':
        from tools.score_capture26_demo import score_near
        from simulation.capture26_mission import load_vision
        audit = score_near(output, summary['mission_configuration'], summary['operating_geometry'],
                           load_vision(Path(__file__).resolve().parents[1]))
        checks['all_four_actual_five_second_dwells'] = audit['audit_passed']
        checks['working_reference_bound'] = guidance.get('working_reference_sha256') == summary['mission_configuration']['operating_geometry_sha256']
    elif stage == 'far':
        profile = FlightProfile(scenario=args.scenario, handoff_distance_m=args.flight_options.handoff_distance_m)
        dwell_ns = 5_000_000_000 if demo else 120_000_000_000
        hold_start_ns = max(round((profile.start_approach_s+profile.approach_duration_s)*1e9), now_ns-dwell_ns)
        hold = [r for r in rows if round(r['time_s']*1e9) >= hold_start_ns]
        errors, speeds, angular_rates = [], [], []
        for row in hold:
            t = row['time_s']
            target_p, target_v, _ = profile.target(t)
            service_p, service_v, _ = profile.chaser(t)
            actual = np.asarray(row['truth_com_position_n_m'])
            velocity = np.asarray(row['truth_com_velocity_n_m_s'])
            errors.append(float(np.linalg.norm(actual[0]-actual[1]-(service_p-target_p))))
            speeds.append(float(np.linalg.norm(velocity[0]-velocity[1]-(service_v-target_v))))
            angular_rates.append(float(np.linalg.norm(row['truth_angular_velocity_body_rad_s'][0])))
        checks.update(actual_120s_hold=bool(hold) and round(hold[-1]['time_s']*1e9)-round(hold[0]['time_s']*1e9) >= dwell_ns,
                      actual_tracking_2m=bool(errors) and max(errors) <= 2.,
                      actual_velocity_2cm_s=bool(speeds) and max(speeds) < .02,
                      actual_service_rate_0p001rad_s=bool(angular_rates) and bool(np.isfinite(angular_rates).all())
                      and max(angular_rates) <= summary['mission_configuration']['service_angular_rate_limit_rad_s'])
        if demo:
            checks['actual_5s_hold'] = checks.pop('actual_120s_hold')
            checks['actual_velocity_1cm_s'] = bool(speeds) and max(speeds) <= .01
            checks.pop('actual_velocity_2cm_s')
            checks.pop('actual_service_rate_0p001rad_s')
            checks['continuous_physical_evidence'] = all(0 < b['time_s']-a['time_s'] <= .050001 for a, b in zip(hold, hold[1:]))
    else:
        checks.update(current_visual_ready=bool(final['visual']['ready']),
                      vision_process_healthy=summary['sensor_delivery']['healthy'],
                      vision_nodes_prevalidated=bool(approach.config.vision_nodes_validated),
                      five_second_platform_stability=bool(guidance.get('platform_stable')),
                      five_second_dwell=guidance.get('platform_stable_duration_s', 0.) >= 5.)
        from tools.score_capture26_platform import score as score_platform
        platform_audit = score_platform(output)
        checks['independent_platform_stability'] = (platform_audit['audit_passed']
            and platform_audit['final_platform_stable'] and platform_audit['first_release_ns'] is not None)
        from simulation.capture26_mission import load_mission
        certified = load_mission(Path(__file__).resolve().parents[1])
        checks['prepared_arm_geometry_certified'] = bool(certified.geometry_certificate_sha256
            and approach.config == certified)
    accepted = all(checks.values())
    outcome = guidance.get('phase', 'NavigationAcquire')
    document = dict(schema=DEMO_SCHEMA if demo else SCHEMA, model_id=MODEL_ID, source_sha256=summary['source_sha256'],
        accepted=accepted, checks=checks, performance=performance,
        acceptance_profile=summary['mission_configuration']['acceptance_profile'],
        feedback_source=summary['mission_configuration']['feedback_source'],
        phase=outcome, time_ns=now_ns, mission_id=mission_id,
        calibration_id=summary['calibration_id'], ephemeris_epoch_utc=(
            '2026 SEPTEMBER 02 00:00:00.000' if epoch_ns else ephemeris_epoch),
        mission_options=args.flight_options.to_dict(), body_properties=summary['body_properties'],
        mission_configuration=summary.get('mission_configuration'),
        sensor_delivery=control.sensor_delivery.snapshot() if hasattr(control, 'sensor_delivery') else None,
        physical=stepper.physical_snapshot(epoch_ns=epoch_ns), navigation=nav.snapshot(),
        gyros=[g.sensor.snapshot() for g in gyros], gnss_receiver=receiver.snapshot(), gnss_pending=pending,
        retention=retention.snapshot() if retention else None,
        control=dict(operating_reference=getattr(control, 'operating_reference', np.zeros(8)).tolist(), capture_reference=capture.target.position.tolist(),
                     force_n=control.force_n.tolist(), last_gnss_ns=control.last_gnss_ns,
                     gyro_pending=getattr(control, 'gyro_pending', []),
                     last_cooperative_ns=getattr(control, 'last_cooperative_ns', None),
                     approach=approach.snapshot() if approach else None,
                     platform_stability=approach.stability.snapshot() if approach else None,
                     preparation=preparation.snapshot() if preparation else None),
        environment=[m.snapshot(epoch_ns=epoch_ns) for m in environment.models] if environment else [])
    if demo:
        document['control']['parking_completion'] = dict(acceptance_profile=summary['acceptance_profile'],
            working_reference_sha256=summary['mission_configuration']['operating_geometry_sha256'],
            records=list(approach.audit) if approach else [dict(node='FarHandoff110m',
                dwell_start_ns=guidance.get('dwell_start_ns'), release_time_ns=now_ns, dwell_s=guidance.get('dwell_s', 0.))])
    if accepted:
        validate(document, Path(__file__).resolve().parents[1])
    encode = lambda v: v.tolist() if isinstance(v, np.ndarray) else str(v)
    (output/'handoff.json').write_text(json.dumps(document, default=encode, indent=2)+'\n', encoding='utf-8')
    report = dict(schema='space-sim-capture26-stage/1', scenario='near-field' if stage == 'near' else args.scenario,
        model_id=MODEL_ID, mission_options=args.flight_options.to_dict(), outcome=outcome,
        acceptance_profile=summary['mission_configuration']['acceptance_profile'],
        feedback_source=summary['mission_configuration']['feedback_source'],
        working_reference_sha256=summary['mission_configuration']['operating_geometry_sha256'],
        final_time_s=now_ns*1e-9, final_center_distance_m=float(np.linalg.norm(final['truth_relative_position_m'])),
        basic_nominal_passed=accepted, mission_success=accepted, checks=checks, performance=performance,
        processing_real_time_factor_after_warmup=capacity,
        lag_growth_after_warmup_s=lag_growth,
        final=dict(visual_ready=final['visual']['ready'], visual=final['visual'],
                   **{key: guidance[key] for key in ('node', 'reason', 'completed_nodes', 'parking_records', 'optical_depth_m',
                      'tcp_gap_m', 'position_error_m', 'velocity_error_m_s', 'position_qualified', 'velocity_qualified',
                      'gap_qualified', 'at_node', 'dwell_s', 'dwell_remaining_s') if key in guidance},
                   gyro_health=final['gyro_health'], gnss_healthy=final['gnss_healthy']),
        unplanned_contacts=contacts, integration_complete=False)
    (output/'mission_report.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    return report
