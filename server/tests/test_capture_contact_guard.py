"""Contact guards must distinguish free-base motion from contact disturbance."""
import numpy as np
from simulation import capture_sequence


def guard():
    cls = getattr(capture_sequence, 'CaptureContactGuard', None)
    assert cls is not None, 'capture guard must use a first-contact baseline'
    return cls()


def test_free_base_displacement_before_contact_is_not_an_impact():
    g = guard()
    assert g.observe(2., [0., 0., 0.], 0.) is None
    assert g.observe(140., [0., .08, 0.], 0.) is None
    assert g.first_contact_time is None


def test_first_contact_arms_persistent_50_mm_displacement_guard():
    g = guard()
    assert g.observe(140., [0., .08, 0.], 0.) is None
    assert g.observe(185., [0., .10, 0.], 1.) is None
    assert g.first_contact_time == 185.
    assert g.observe(190., [0., .13, 0.], 0.) is None
    reason = g.observe(195., [0., .151, 0.], 0.)
    assert reason['reason'] == 'post_contact_relative_displacement_over_50mm'
    assert np.isclose(reason['displacement_m'], .051)


def test_large_first_contact_force_aborts_without_waiting_for_displacement():
    reason = guard().observe(40., [0., 0., 0.], 10.1)
    assert reason['reason'] == 'target_contact_force_over_10n'


def test_invalid_feedback_stops_motion():
    assert guard().observe(40., [0., np.nan, 0.], 0.)['reason'] == 'invalid_capture_guard_feedback'
    assert guard().observe(40., [0., 0., 0.], np.nan)['reason'] == 'invalid_capture_guard_feedback'


def test_approach_horizon_allows_half_metre_low_speed_motion():
    s = capture_sequence.CaptureSequence()
    s.phase, s.start = 'approach', 0.
    q = np.zeros(12)
    q[6] = -.006
    action, phase = s.command(120., q, np.array([0., 0., .08]), np.eye(3))
    assert phase == 'approach'
    assert np.linalg.norm(action['end_effector_linear_velocity_body_m_s']) <= .004000001
    _, phase = s.command(241., q, np.array([0., 0., .08]), np.eye(3))
    assert phase == 'abort'
