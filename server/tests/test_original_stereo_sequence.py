import numpy as np

from simulation.original_stereo_sequence import OriginalStereoLimits, OriginalStereoSequence


def sequence(*, bridge=True):
    return OriginalStereoSequence([2., 0., 0.], [1.9, 0., 0.], [1.8, 0., 0.],
        [1.7, 0., 0.], [.6, .02, 0.],
        limits=OriginalStereoLimits(dwell_s=.1, image_timeout_s=.5),
        gnss_bridge_diagnostic=bridge)


def advance(seq, target_phase, *, visible=False, limit=20000):
    state = getattr(seq, '_test_state', np.r_[seq.points[0], np.zeros(3)])
    now = 0. if seq.previous is None else seq.previous+.05
    for _ in range(limit):
        ref, vel, _ = seq.update(now, state, position_sigma_m=.003,
            inputs_valid=True, service_ready=True, corridor_ready=True,
            prealigned=seq.target_pointing_enabled, visual_ready=visible,
            visual_stamp_s=now if visible else None,
            estimated_tool_center_gripper_m=[0., 0., -.5])
        state = np.r_[ref, vel]
        seq._test_state = state
        if seq.phase == target_phase:
            return
        if seq.phase == 'SafeHold':
            raise AssertionError(seq.failure)
        now += .05
    raise AssertionError('stage not reached: '+target_phase)


def test_bridge_is_not_enabled_by_default_and_missing_far_vision_stops():
    seq = sequence(bridge=False)
    advance(seq, 'OriginalPolicyVisualWait')
    for _ in range(12):
        t = seq.previous+.05
        ref, _, _ = seq.update(t, np.r_[seq.station, np.zeros(3)],
            position_sigma_m=.003, inputs_valid=True, service_ready=True,
            corridor_ready=True, prealigned=True)
        np.testing.assert_allclose(ref, seq.station)
    assert seq.phase == 'SafeHold'
    assert seq.failure == 'original_policy_far_visual_unavailable'
    assert not any('Bridge' in e['phase'] for e in seq.events)


def test_gnss_dwell_is_not_visual_release_and_requires_near_images():
    seq = sequence()
    advance(seq, 'OriginalStereoAcquire')
    assert any(e['phase'] == 'GNSSBridge5p5Hold' for e in seq.events)
    assert seq.image_release_s is None
    assert not seq.telemetry['formal_visual_release']
    for _ in range(12):
        t = seq.previous+.05
        seq.update(t, np.r_[seq.points[3], np.zeros(3)], position_sigma_m=.003,
            inputs_valid=True, service_ready=True, corridor_ready=True, prealigned=True)
    assert seq.phase == 'SafeHold' and seq.failure == 'original_stereo_not_ready'


def test_one_sequence_completes_offset_and_two_platform_dwell_windows():
    seq = sequence()
    advance(seq, 'OriginalStereoAcquire')
    advance(seq, 'TerminalHold', visible=True)
    assert seq.platform_completed_s-seq.platform_entered_s >= .1-1e-9
    assert seq.image_release_s < seq.platform_entered_s
    np.testing.assert_allclose(seq._test_state[:3], seq.points[4])
    assert seq.telemetry['trial_original_image_release']
    assert not seq.telemetry['formal_visual_release']


def test_visual_loss_after_release_latches_stop_and_never_auto_resumes():
    seq = sequence()
    advance(seq, 'OriginalStereoAcquire')
    advance(seq, 'VisualApproach', visible=True)
    for fresh in (False, True):
        t = seq.previous+.05
        seq.update(t, seq._test_state, position_sigma_m=.003, inputs_valid=True,
            service_ready=True, corridor_ready=True, prealigned=True,
            visual_ready=fresh, visual_stamp_s=t)
        assert seq.phase == 'SafeHold'
    assert seq.failure == 'visual_lost_after_release'
