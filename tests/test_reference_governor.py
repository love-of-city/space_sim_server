import numpy as np
import pytest

from simulation.reference_governor import JointReferenceGovernor, ReferenceGovernorSettings


def test_free_motion_keeps_direction_and_magnitude():
    governor = JointReferenceGovernor()
    velocity = np.array([0.2, -0.1, 0.3, 0.4, -0.5, 0.6])
    result = governor.apply(np.zeros(6), np.zeros(6), velocity, 0.01, enabled=True)
    np.testing.assert_array_equal(result.velocity, velocity)
    assert result.state == "active"


def test_soft_limit_scales_all_axes_uniformly():
    governor = JointReferenceGovernor()
    reference = np.array([0.06, 0, 0, 0, 0, 0])
    velocity = np.array([0.2, 0.1, -0.1, 0, 0, 0])
    result = governor.apply(reference, np.zeros(6), velocity, 0.01, enabled=True)
    assert result.scale == pytest.approx(0.5)
    np.testing.assert_allclose(result.velocity, velocity * result.scale)
    assert result.limited_joints == (1,)


def test_blocked_joint_stays_bounded_and_reverse_can_escape():
    governor = JointReferenceGovernor()
    reference = np.zeros(6)
    velocity = np.array([0.7, 0, 0, 0, 0, 0])
    for _ in range(1000):
        result = governor.apply(reference, np.zeros(6), velocity, 0.01, enabled=True)
        reference += result.velocity * 0.01
    assert reference[0] <= governor.hard[0]
    result = governor.apply(reference, np.zeros(6), -velocity, 0.01, enabled=True)
    assert result.scale == 1.0
    assert result.velocity[0] < 0


def test_external_displacement_blocks_only_worsening_direction():
    governor = JointReferenceGovernor()
    reference = np.array([0.3, 0, 0, 0, 0, 0])
    velocity = np.array([0.4, 0, 0, 0, 0, 0])
    blocked = governor.apply(reference, np.zeros(6), velocity, 0.01, enabled=True)
    recovery = governor.apply(reference, np.zeros(6), -velocity, 0.01, enabled=True)
    assert blocked.scale == 0
    assert recovery.scale == 1


def test_continuous_saturation_latches_with_hysteresis_and_recovery():
    governor = JointReferenceGovernor()
    reference = np.array([0.05, 0, 0, 0, 0, 0])
    velocity = np.array([0.3, 0, 0, 0, 0, 0])
    for _ in range(20):
        result = governor.apply(reference, np.zeros(6), velocity, 0.01, enabled=True, effort_ratio=np.ones(6))
    assert result.state == "saturation_limited"
    assert result.scale == 0
    still_blocked = governor.apply(reference, np.zeros(6), velocity, 0.01, enabled=True)
    assert still_blocked.scale == 0
    released = governor.apply(reference, np.zeros(6), velocity, 0.01, enabled=False)
    assert released.state == "holding"
    reverse = governor.apply(reference, np.zeros(6), -velocity, 0.01, enabled=True)
    assert reverse.scale == 1
    assert not governor.blocked.any()


def test_measured_catchup_automatically_clears_latch():
    governor = JointReferenceGovernor()
    governor.blocked[:] = True
    result = governor.apply(np.zeros(6), np.zeros(6), np.ones(6) * 0.1, 0.01, enabled=True)
    assert result.scale == 1
    assert not governor.blocked.any()


def test_short_saturation_does_not_latch_and_reset_clears_state():
    governor = JointReferenceGovernor()
    reference = np.full(6, 0.045)
    governor.apply(reference, np.zeros(6), np.ones(6), 0.01, enabled=True, effort_ratio=np.ones(6))
    assert not governor.blocked.any()
    governor.reset()
    assert not governor.blocked.any()
    assert not governor.saturation_seconds.any()


@pytest.mark.parametrize("enabled", [False, True])
def test_invalid_feedback_fails_closed_by_raising(enabled):
    with pytest.raises(ValueError):
        JointReferenceGovernor().apply(np.zeros(6), np.full(6, np.nan), np.ones(6), 0.01, enabled=enabled)


def test_zero_command_never_generates_motion_and_timeout_stops_immediately():
    governor = JointReferenceGovernor()
    for velocity, enabled in [(np.zeros(6), True), (np.ones(6), False)]:
        result = governor.apply(np.full(6, 0.02), np.zeros(6), velocity, 0.01, enabled=enabled)
        np.testing.assert_array_equal(result.velocity, np.zeros(6))


@pytest.mark.parametrize("settings", [
    {"soft_error_rad": (0.1,)}, {"hard_error_rad": (0.01,) * 6},
    {"saturation_hold_s": float("nan")}, {"saturation_threshold": 2},
])
def test_invalid_settings_are_rejected(settings):
    with pytest.raises(ValueError):
        ReferenceGovernorSettings(**settings)
