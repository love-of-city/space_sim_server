"""Prevent separate accepted experiments being labelled one continuous flight."""
from copy import deepcopy
import json

import numpy as np
import pytest

from simulation.orbital_rendezvous.reference import FlightProfile
from simulation.near_field.orbital_guidance import vbar_frame
from tools.check_capture26_continuous import identity_checks, score_activation, score_alignment_recovery, score_far


@pytest.mark.parametrize('damage', [None, 'mission_id', 'calibration_id', 'mission_options', 'checkpoint', 'clock'])
def test_cross_stage_identity_rejects_mixed_experiments(damage):
    far = dict(mission_id='same-task', mission_options=dict(seed=7), mission_configuration=dict(physics_hz=400),
        calibration_id='camera-A', source_sha256='model-A', ephemeris_epoch_utc='unchanged',
        phase='FarReady', accepted=True, time_ns=5_000_000_000,
        physical=dict(qpos_inertial=[1., 2., 3.]))
    received = deepcopy(far)
    near = dict(deepcopy(far), phase='GraspHold', time_ns=10_000_000_000)
    if damage in ('mission_id', 'calibration_id'):
        near[damage] = 'different-experiment'
    elif damage == 'mission_options':
        near['mission_options']['seed'] = 8
    elif damage == 'checkpoint':
        received['physical']['qpos_inertial'][0] += .01
    elif damage == 'clock':
        near['time_ns'] = 0
    checks = identity_checks(far, near, received)
    assert all(checks.values()) == (damage is None)


@pytest.mark.parametrize('damage', [None, '110m_start', 'missing_sample', 'position', 'velocity'])
def test_far_raw_evidence_requires_the_long_start_and_actual_hold(tmp_path, damage):
    profile = FlightProfile(handoff_distance_m=110.)
    rows = [dict(time_s=0., truth_relative_position_m=[110. if damage == '110m_start' else 7000., 0., 0.])]
    for k in range(101):
        if damage == 'missing_sample' and k == 50:
            continue
        t = 5000.+k*.05
        pt, vt, _ = profile.target(t)
        ps, vs, _ = profile.chaser(t)
        if k == 50 and damage == 'position':
            ps = ps+np.array([2.1, 0., 0.])
        if k == 50 and damage == 'velocity':
            vs = vs+np.array([.011, 0., 0.])
        rows.append(dict(time_s=t, truth_com_position_n_m=[ps.tolist(), pt.tolist()],
            truth_com_velocity_n_m_s=[vs.tolist(), vt.tolist()]))
    (tmp_path/'score.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows), encoding='utf-8')
    result = score_far(tmp_path, dict(time_ns=5_005_000_000_000,
        mission_options=dict(handoff_distance_m=110.)))
    assert result['passed'] == (damage is None)


@pytest.mark.parametrize('damage', [None, 'early_activation', 'duplicate_activation', 'missing_first',
    'missing_middle', 'missing_last', 'changed_delay', 'no_result', 'worker_fault', 'wrong_grid'])
def test_stereo_sampling_requires_actual_continuous_frames_after_qualified_dwell(tmp_path, damage):
    stamp = 5_600_050_000_000
    policy = 'after_qualified_GNSS5p5m_dwell'
    near = dict(time_ns=stamp+700_000_000, control=dict(
        stereo_activation=dict(time_ns=stamp, policy=policy, capture_rate_hz=10),
        approach=dict(epoch_ns=5_005_000_000_000, telemetry=dict(
            vbar_nodes=dict(GNSS5p5m=dict(completed_at_s=595.05))))))
    events = [dict(kind='stereo_activation', time_ns=stamp, policy=policy, rate_hz=10)]
    for k in range(7):
        exposure = 5_600_100_000_000+k*100_000_000
        events.append(dict(kind='delivery', exposure_ns=exposure, due_ns=exposure+200_000_000,
            result_available=True, process_healthy=True))
    if damage == 'early_activation':
        near['control']['stereo_activation']['time_ns'] -= 50_000_000
        events[0]['time_ns'] -= 50_000_000
    elif damage == 'duplicate_activation':
        events.insert(0, deepcopy(events[0]))
    elif damage in ('missing_first', 'missing_middle', 'missing_last'):
        events.pop(dict(missing_first=1, missing_middle=3, missing_last=-1)[damage])
    elif damage == 'changed_delay':
        events[3]['due_ns'] += 50_000_000
    elif damage == 'no_result':
        events[3]['result_available'] = False
    elif damage == 'worker_fault':
        events.append(dict(kind='process_fault', time_ns=stamp+300_000_000, reason='worker exited'))
    elif damage == 'wrong_grid':
        events[3]['exposure_ns'] += 50_000_000
        events[3]['due_ns'] += 50_000_000
    (tmp_path/'delivery.json').write_text(json.dumps(events), encoding='utf-8')
    result = score_activation(tmp_path, near)
    assert result['passed'] == (damage is None)


@pytest.mark.parametrize('damage', [None, 'position', 'velocity', 'missing_sample', 'unqualified',
    'missing_event', 'unpaired_event'])
def test_realign_release_requires_independent_stationary_five_seconds(tmp_path, damage):
    stop = np.array([16., 0., 0.])
    events = [dict(time_s=10.,event='brake_for_target_realign',node='GNSS5p5m',stop_com_m=stop.tolist()),
        dict(time_s=20.,event='resume_after_target_realign',node='GNSS5p5m',dwell=dict(samples=101,dwell_s=5.))]
    if damage == 'missing_event':
        events = []
    elif damage == 'unpaired_event':
        events.pop()
    rows = []
    profile = FlightProfile(handoff_distance_m=110.)
    for k in range(201):
        if k == 150 and damage == 'missing_sample':
            continue
        t = 1010.+k*.05
        pt,vt,_ = profile.target(t)
        frame,omega = vbar_frame(np.r_[pt,vt])
        dp = frame@stop
        ps,vs = pt-dp,vt-np.cross(omega,dp)
        if k == 150 and damage == 'position':
            ps += frame@np.array([.051,0.,0.])
        if k == 150 and damage == 'velocity':
            vs += frame@np.array([.011,0.,0.])
        rows.append(dict(time_s=t,truth_com_position_n_m=[ps.tolist(),pt.tolist()],
            truth_com_velocity_n_m_s=[vs.tolist(),vt.tolist()],
            guidance=dict(phase='TargetRealignment',vbar=dict(prealigned=not (k == 150 and damage == 'unqualified')))))
    (tmp_path/'score.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows), encoding='utf-8')
    near=dict(control=dict(approach=dict(epoch_ns=1_000_000_000_000,
        telemetry=dict(target_realignment=dict(events=events)))))
    result = score_alignment_recovery(tmp_path,near)
    assert result['passed'] == (damage is None)
