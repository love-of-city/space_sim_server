"""Ownership, physical-state continuity and real gate evidence at handoff."""
import numpy as np
import pytest

from simulation.vbar_stereo_sequence import VbarStereoSequence


def sequence(**kwargs):
    return VbarStereoSequence([51., 0., 0.], [49., 0., 0.], [5.5, 0., 0.],
                              [1.5, 0., 0.], [.6, .01, 0.], **kwargs)


def tick(seq, now, state, **overrides):
    inputs = dict(covariance=np.eye(6)*1e-6, optical_depth_m=state[0],
        available_acceleration_m_s2=.005, observation_age_s=0., position_sigma_m=.001,
        inputs_valid=True, service_ready=True, corridor_ready=True)
    inputs.update(overrides)
    return seq.update(now, state, **inputs)


def to_handoff(seq):
    state = np.array([51., 0., 0., 0., 0., 0.])
    now = seq.epoch
    for _ in range(8000):
        before = state.copy()
        ref, vel, _ = tick(seq, now, state)
        np.testing.assert_array_equal(state, before)  # coordinator cannot write physical/nav state
        if seq.handoff_s is not None:
            return now, state, ref
        assert seq.phase != 'SafeHold', seq.failure
        state = np.r_[ref, vel]
        now += .05
    raise AssertionError('handoff not reached')


def test_orders_actual_dwells_and_keeps_vbar_owner_for_target_pointing():
    seq = sequence(gnss_bridge_diagnostic=True)
    now, state, ref = to_handoff(seq)
    assert seq.owner == 'vbar' and seq.phase == 'TargetPointing'
    assert seq.previous == now
    evidence = seq.handoff_evidence
    assert evidence['acquire']['completed_at_s'] < evidence['Vbar50m']['completed_at_s'] < now
    for name in ('acquire', 'Vbar50m', 'Optical17p4m'):
        assert evidence[name]['samples'] >= 101 and evidence[name]['dwell_s'] >= 5.-1e-9
    np.testing.assert_allclose(ref, seq.station)
    np.testing.assert_allclose(state[:3], ref)
    assert not seq.image_release_s
    assert seq.target_pointing_enabled
    assert any(e['phase'] == 'TargetPointing' for e in seq.events)


def test_global_epoch_is_preserved_for_image_timestamps_and_events():
    seq = sequence(epoch_s=123., gnss_bridge_diagnostic=True)
    now, _, _ = to_handoff(seq)
    assert now > 123.
    assert seq.phase_start == now
    assert seq.handoff_evidence['Optical17p4m']['completed_at_s'] == pytest.approx(now)


def test_no_image_or_gyro_flag_can_skip_vbar_nodes_or_start_slew():
    seq = sequence(gnss_bridge_diagnostic=True)
    for i in range(50):
        tick(seq, i*.05, np.array([51., 0., 0., 0., 0., 0.]),
             visual_ready=True, visual_stamp_s=i*.05, prealigned=True)
    assert seq.handoff_s is None and not seq.target_pointing_enabled
    assert seq.vbar.gates['Vbar50m'].completed_at_s is None
    assert seq.vbar.pointing_gate.completed_at_s is None


def test_control_gap_latches_safe_stop_and_never_grants_near_owner():
    seq = sequence()
    state = np.array([51., 0., 0., -.01, 0., 0.])
    tick(seq, 0., state)
    tick(seq, .25, state)
    assert seq.failure == 'control_update_gap'
    tick(seq, .3, state, prealigned=True, visual_ready=True, visual_stamp_s=.3)
    assert seq.phase == 'SafeHold' and seq.owner == 'vbar'


def fly(seq, target, *, images=False, state=None, now=None):
    state = np.r_[seq.points[0], np.zeros(3)] if state is None else state
    now = seq.epoch if now is None else now
    for _ in range(65000):
        ref, vel, _ = tick(seq, now, state, prealigned=seq.target_pointing_enabled,
            visual_ready=images, visual_stamp_s=now if images else None,
            estimated_tool_center_gripper_m=[0., 0., -.5])
        state = np.r_[ref, vel]
        if seq.phase == target:
            return now+.05, state
        assert not seq.failure, seq.failure
        now += .05
    raise AssertionError('stage not reached: '+target)


@pytest.mark.parametrize('mission_profile', [None, 'near23_vbar_v1'])
def test_vbar_receives_updated_navigation_after_images_and_holds_for_arm_grasp(mission_profile):
    seq = VbarStereoSequence([50.02, 0., 0.], [17.4, 0., 0.], [5.5, 0., 0.],
                            [1.5, 0., 0.], [.6, .01, 0.],
                            gnss_bridge_diagnostic=True, with_grasp=True, mission_profile=mission_profile)
    now, state = fly(seq, 'OriginalStereoAcquire')
    assert seq.vbar.gates['GNSS5p5m'].completed_at_s is not None
    assert seq.image_release_s is None
    now, state = fly(seq, 'VisualApproach', images=True, now=now, state=state)
    assert seq.owner == 'vbar'
    # The same V-bar object consumes the new navigation posterior, including
    # position corrections from the camera, after its real-image release gate.
    corrected = state.copy()
    corrected[1] += .001
    tick(seq, now, corrected, prealigned=True, visual_ready=True, visual_stamp_s=now)
    np.testing.assert_allclose(seq.vbar.telemetry['state'], corrected)
    assert seq.telemetry['reference_owner'] == 'vbar'
    now, state = fly(seq, 'Grasp', images=True, now=now+.05, state=state)
    assert seq.platform_completed_s-seq.platform_entered_s >= 5.-1e-9
    tick(seq, now, state, prealigned=False, visual_ready=False)
    assert seq.phase == 'Grasp' and seq.owner == 'vbar' and not seq.failure
    tick(seq, now+.05, state, grasp_complete=True)
    assert seq.phase == 'Complete'


def test_missing_visual_never_admits_offset_and_loss_after_release_latches():
    seq = VbarStereoSequence([50.02, 0., 0.], [17.4, 0., 0.], [5.5, 0., 0.],
                            [1.5, 0., 0.], [.6, .01, 0.], gnss_bridge_diagnostic=True)
    now, state = fly(seq, 'OriginalStereoAcquire')
    for _ in range(110):
        ref, _, _ = tick(seq, now, state, prealigned=True)
        np.testing.assert_allclose(ref, seq.points[3])
        now += .05
    assert seq.image_release_s is None and seq.vbar.offset_progress == 0.
    now, state = fly(seq, 'VisualApproach', images=True, now=now, state=state)
    tick(seq, now, state, prealigned=True)
    assert seq.phase == 'SafeHold' and seq.failure == 'visual_readiness_lost_after_release'
    tick(seq, now+.05, state, prealigned=True, visual_ready=True, visual_stamp_s=now+.05)
    assert seq.phase == 'SafeHold' and seq.owner == 'vbar'


def formal_bridge():
    seq = VbarStereoSequence([50.02, 0., 0.], [17.4, 0., 0.], [5.5, 0., 0.],
        [1.5, 0., 0.], [.6, .01, 0.], mission_profile='near23_vbar_v1')
    now, state = fly(seq, 'GNSSBridge5p5Approach')
    state[3] = -.02
    return seq, now, state


def test_formal_alignment_loss_brakes_then_requires_a_new_complete_dwell():
    seq, now, state = formal_bridge()
    completed = {k: seq.vbar.gates[k].completed_at_s for k in ('Vbar50m', 'Optical17p4m')}
    before = state.copy()
    ref, vel, _ = tick(seq, now, state, prealigned=False)
    np.testing.assert_array_equal(state, before)
    np.testing.assert_allclose(ref, state[:3])
    np.testing.assert_allclose(vel, state[3:])
    assert seq.phase == 'TargetRealignment' and not seq.failure
    assert seq.owner == 'vbar'
    stop = seq.vbar.alignment_stop
    assert stop.endpoint[0] < state[0]  # brake along the current closing motion
    start = now
    state = np.r_[ref, vel]
    for _ in range(1000):
        now += .05
        elapsed = now-start
        aligned = elapsed >= stop.duration+1.
        previous_velocity = state[3:].copy()
        ref, vel, acc = tick(seq, now, state, prealigned=aligned)
        assert np.linalg.norm(acc) <= .005+1e-12
        assert np.linalg.norm(vel-previous_velocity)/.05 <= .005+1e-10
        state = np.r_[ref, vel]
        assert not seq.failure
        if seq.vbar.alignment_wait is None:
            break
        assert seq.phase == 'TargetRealignment'
        assert seq.vbar.gates['GNSS5p5m'].completed_at_s is None
    else:
        raise AssertionError('stable alignment did not release the paused leg')
    recovery = seq.vbar.alignment_recovery_gate
    assert recovery.dwell_s >= 5.-1e-9 and recovery.samples >= 101
    assert now-start >= stop.duration+6.-1e-9
    assert {k: seq.vbar.gates[k].completed_at_s for k in completed} == completed
    resume, velocity, _ = tick(seq, now+.05, state, prealigned=True)
    np.testing.assert_allclose(resume, stop.endpoint, atol=1e-12)
    np.testing.assert_allclose(velocity, 0., atol=1e-12)
    assert seq.phase == 'GNSSBridge5p5Approach'


@pytest.mark.parametrize('fault', ['persistent_misalignment', 'navigation_loss'])
def test_alignment_recovery_keeps_timeout_and_navigation_protection(fault):
    seq, now, state = formal_bridge()
    tick(seq, now, state, prealigned=False)
    for _ in range(4000):
        now += .05
        ref, vel, _ = tick(seq, now, state, prealigned=False,
            inputs_valid=fault != 'navigation_loss')
        state = np.r_[ref, vel]
        if seq.failure:
            break
    assert seq.failure == ('target_realignment_timeout' if fault == 'persistent_misalignment'
                           else 'navigation_health_lost')
    assert seq.phase == 'SafeHold' and seq.owner == 'vbar'


def test_first_image_attitude_correction_waits_for_service_tracking_without_losing_healthy_navigation():
    seq, now, state = formal_bridge()
    ref, vel, _ = tick(seq, now, state, prealigned=True, service_ready=False,
        visual_ready=True, visual_stamp_s=now)
    assert seq.phase == 'TargetRealignment' and not seq.failure
    assert not seq.telemetry['prealigned']
    assert seq.vbar.telemetry['conditions']['common']
    state = np.r_[ref,vel]
    for _ in range(400):
        now += .05
        ref,vel,_ = tick(seq, now,state,prealigned=True,service_ready=False,
            visual_ready=True,visual_stamp_s=now)
        state=np.r_[ref,vel]
        assert seq.phase == 'TargetRealignment' and not seq.failure
        assert seq.vbar.alignment_recovery_gate.samples == 0
    now,state = fly(seq,'GNSSBridge5p5Approach',images=True,now=now+.05,state=state)
    assert seq.vbar.alignment_recovery_gate.dwell_s >= 5.-1e-9


def test_service_attitude_still_blocks_initial_unqualified_entry():
    seq=sequence(mission_profile='near23_vbar_v1')
    state=np.array([51.,0.,0.,0.,0.,0.])
    for k in range(110):
        tick(seq,k*.05,state,service_ready=False)
    assert seq.vbar.acquire.completed_at_s is None
    assert not seq.vbar.acquire.telemetry(5.45)['completed']


def test_realign_on_gnss_leg_to_first_image_does_not_require_unavailable_vision():
    seq=VbarStereoSequence([50.02,0.,0.],[17.4,0.,0.],[5.5,0.,0.],
        [1.5,0.,0.],[.6,.01,0.],mission_profile='near23_vbar_v1',with_grasp=True)
    now,state=fly(seq,'GNSSBridgeNearApproach')
    assert seq.vbar.gates['GNSS5p5m'].completed_at_s is not None
    assert seq.image_release_s is None
    state[3]=-.02
    ref,vel,_=tick(seq,now,state,prealigned=False)
    state=np.r_[ref,vel]
    assert seq.phase=='TargetRealignment' and not seq.failure
    for _ in range(600):
        now+=.05
        ref,vel,_=tick(seq,now,state,prealigned=True)
        state=np.r_[ref,vel]
        assert not seq.failure
        if seq.phase=='GNSSBridgeNearApproach':
            break
    else:
        raise AssertionError('GNSS bridge deadlocked waiting for unavailable first vision')
    assert seq.image_release_s is None
    now,state=fly(seq,'OriginalStereoAcquire',state=state,now=now+.05)
    assert seq.image_release_s is None  # reaching the node never grants visual release
    now,state=fly(seq,'VisualApproach',images=True,state=state,now=now+.05)
    now,state=fly(seq,'Grasp',images=True,state=state,now=now+.05)
    assert seq.image_release_s is not None and seq.platform_completed_s is not None
    tick(seq,now,state,grasp_complete=True)
    assert seq.phase=='Complete' and not seq.failure


def formal_visual_leg():
    seq = VbarStereoSequence([50.02, 0., 0.], [17.4, 0., 0.], [5.5, 0., 0.],
        [1.5, 0., 0.], [.6, .01, 0.], mission_profile='near23_vbar_v1', with_grasp=True)
    now, state = fly(seq, 'VisualApproach', images=True)
    for _ in range(200):
        ref, vel, _ = tick(seq, now, state, prealigned=True,
            visual_ready=True, visual_stamp_s=now, estimated_tool_center_gripper_m=[0., 0., -.5])
        state = np.r_[ref, vel]
        now += .05
    return seq, now, state


def test_formal_visual_interruption_brakes_reacquires_and_resumes_from_full_stop():
    seq, now, state = formal_visual_leg()
    completed = {k: g.completed_at_s for k, g in seq.vbar.gates.items()}
    original = state.copy()
    ref, vel, _ = tick(seq, now, state, prealigned=True)
    np.testing.assert_array_equal(state, original)
    np.testing.assert_allclose(ref, state[:3])
    np.testing.assert_allclose(vel, state[3:])
    assert seq.phase == 'VisualRecovery' and not seq.failure
    stop, start = seq.vbar.alignment_stop, now
    assert stop.endpoint[0] < state[0]
    assert stop.endpoint[1] > 0.  # interruption occurs inside the transverse offset
    state = np.r_[ref, vel]
    for _ in range(400):
        now += .05
        fresh = now-start >= 2.
        previous_velocity = state[3:].copy()
        ref, vel, acc = tick(seq, now, state, prealigned=True,
            visual_ready=fresh, visual_stamp_s=now if fresh else None,
            estimated_tool_center_gripper_m=[0., 0., -.5])
        assert np.linalg.norm(acc) <= .005+1e-12
        assert np.linalg.norm(vel-previous_velocity) <= .005*.05+1e-10
        state = np.r_[ref, vel]
        assert not seq.failure
        if seq.vbar.alignment_wait is None:
            break
        assert seq.phase == 'VisualRecovery' and seq.platform_completed_s is None
    else:
        raise AssertionError('qualified images and five-second dwell did not release the stop')
    gate = seq.vbar.alignment_recovery_gate
    assert gate.dwell_s >= 5.-1e-9 and gate.samples >= 101
    assert now-start >= max(2., stop.duration)+5.-1e-9
    assert {k: g.completed_at_s for k, g in seq.vbar.gates.items()} == completed
    resume, velocity, acceleration = tick(seq, now+.05, state, prealigned=True,
        visual_ready=True, visual_stamp_s=now+.05, estimated_tool_center_gripper_m=[0., 0., -.5])
    np.testing.assert_allclose(resume, stop.endpoint, atol=1e-12)
    np.testing.assert_allclose(velocity, 0., atol=1e-12)
    np.testing.assert_allclose(acceleration, 0., atol=1e-12)
    assert seq.phase == 'VisualApproach' and seq.owner == 'vbar'
    now, state = fly(seq, 'Grasp', images=True, state=state, now=now+.1)
    assert seq.platform_completed_s-seq.platform_entered_s >= 5.-1e-9
    tick(seq, now, state, grasp_complete=True)
    assert seq.phase == 'Complete' and not seq.failure


@pytest.mark.parametrize('fault,expected', [
    ('persistent_images', 'visual_recovery_timeout'),
    ('navigation', 'navigation_health_lost'),
    ('braking', 'braking_authority_unverified'),
    ('boundary', 'optical_braking_margin'),
])
def test_visual_recovery_retains_deadline_navigation_and_braking_protection(fault, expected):
    seq, now, state = formal_visual_leg()
    kwargs = dict(prealigned=True)
    if fault == 'navigation':
        kwargs['inputs_valid'] = False
    elif fault == 'braking':
        kwargs['available_acceleration_m_s2'] = 0.
    elif fault == 'boundary':
        kwargs['optical_depth_m'] = 1.
    start = now
    for _ in range(1300):
        ref, vel, _ = tick(seq, now, state, **kwargs)
        state = np.r_[ref, vel]
        if seq.failure:
            break
        assert seq.phase == 'VisualRecovery' and seq.platform_completed_s is None
        now += .05
    assert seq.phase == 'SafeHold' and seq.failure == expected
    if fault == 'persistent_images':
        assert 60.-1e-9 <= now-start < 60.1


def test_repeated_image_loss_resets_recovery_dwell_without_resetting_deadline():
    seq, now, state = formal_visual_leg()
    ref, vel, _ = tick(seq, now, state, prealigned=True)
    state = np.r_[ref, vel]
    started = seq.vbar.alignment_wait
    for i in range(240):
        now += .05
        fresh = i % 40 != 0  # never sustains the original five-second dwell
        ref, vel, _ = tick(seq, now, state, prealigned=True,
            visual_ready=fresh, visual_stamp_s=now if fresh else None)
        state = np.r_[ref, vel]
        assert not seq.failure and seq.phase == 'VisualRecovery'
        assert seq.vbar.alignment_wait == started
        assert seq.vbar.alignment_recovery_gate.dwell_s < 2.
        assert seq.platform_completed_s is None
