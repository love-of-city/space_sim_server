"""Independent stage scoring and complete continuation serialization.

Truth records are consumed after control has stopped. They cannot replace a
sensor observation or alter a propagated state.
"""
import json
from pathlib import Path

import numpy as np

from simulation.capture26_handoff import SCHEMA, validate
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
    performance = dict(required_for_acceptance=False,
        requested_simulation_rate=getattr(args, 'simulation_rate', 1.),
        wall_clock_pacing=bool(args.realtime),
        processing_real_time_factor_after_warmup=capacity,
        lag_growth_against_1x_after_warmup_s=lag_growth,
        achieved_1x=capacity >= 1.)
    preparation = getattr(control, 'joint_preparation', None)
    if (summary.get('mission_configuration') or {}).get('geometry_certificate_sha256'):
        checks['certified_joint_preparation_complete'] = bool(preparation and preparation.complete and not preparation.failed)
    if stage == 'far':
        profile = FlightProfile(scenario=args.scenario, handoff_distance_m=args.flight_options.handoff_distance_m)
        hold = [r for r in rows if r['time_s'] >= profile.start_approach_s+profile.approach_duration_s]
        errors, speeds = [], []
        for row in hold:
            t = row['time_s']
            target_p, target_v, _ = profile.target(t)
            service_p, service_v, _ = profile.chaser(t)
            actual = np.asarray(row['truth_com_position_n_m'])
            velocity = np.asarray(row['truth_com_velocity_n_m_s'])
            errors.append(float(np.linalg.norm(actual[0]-actual[1]-(service_p-target_p))))
            speeds.append(float(np.linalg.norm(velocity[0]-velocity[1]-(service_v-target_v))))
        checks.update(actual_120s_hold=bool(hold) and hold[-1]['time_s']-hold[0]['time_s'] >= 120.,
                      actual_tracking_2m=bool(errors) and max(errors) < 2.,
                      actual_velocity_2cm_s=bool(speeds) and max(speeds) < .02)
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
    document = dict(schema=SCHEMA, model_id=MODEL_ID, source_sha256=summary['source_sha256'],
        accepted=accepted, checks=checks, performance=performance,
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
    if accepted:
        validate(document, Path(__file__).resolve().parents[1])
    encode = lambda v: v.tolist() if isinstance(v, np.ndarray) else str(v)
    (output/'handoff.json').write_text(json.dumps(document, default=encode, indent=2)+'\n', encoding='utf-8')
    report = dict(schema='space-sim-capture26-stage/1', scenario='near-field' if stage == 'near' else args.scenario,
        model_id=MODEL_ID, mission_options=args.flight_options.to_dict(), outcome=outcome,
        final_time_s=now_ns*1e-9, final_center_distance_m=float(np.linalg.norm(final['truth_relative_position_m'])),
        basic_nominal_passed=accepted, mission_success=accepted, checks=checks, performance=performance,
        processing_real_time_factor_after_warmup=capacity,
        lag_growth_after_warmup_s=lag_growth,
        final=dict(visual_ready=final['visual']['ready'], visual=final['visual'],
                   gyro_health=final['gyro_health'], gnss_healthy=final['gnss_healthy']),
        unplanned_contacts=contacts, integration_complete=False)
    (output/'mission_report.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    return report
