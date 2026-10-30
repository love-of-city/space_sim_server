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


def test_vbar_receives_updated_navigation_after_images_and_holds_for_arm_grasp():
    seq = VbarStereoSequence([50.02, 0., 0.], [17.4, 0., 0.], [5.5, 0., 0.],
                            [1.5, 0., 0.], [.6, .01, 0.],
                            gnss_bridge_diagnostic=True, with_grasp=True)
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
