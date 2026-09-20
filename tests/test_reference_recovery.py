import numpy as np
import pytest

from simulation.reference_governor import JointReferenceGovernor
from simulation.reference_recovery import ReferenceRecoverySettings


def stuck_reference():
    return np.array([0.0, 0.06733, -0.06232, 0.0, 0.0, 0.0])


def test_opposing_joint_errors_recover_then_both_directions_are_available():
    governor = JointReferenceGovernor()
    reference = stuck_reference()
    governor.blocked[1:3] = True
    states = set()
    for _ in range(250):
        before = reference.copy()
        result = governor.apply(reference, np.zeros(6), np.zeros(6), 0.01, enabled=False, allow_recovery=True)
        reference += result.velocity * 0.01
        states.add(result.state)
        assert np.all(np.abs(reference) <= np.abs(before) + 1e-12)
        assert np.max(np.abs(result.velocity)) <= 0.08 + 1e-12
    assert "reference_recovery" in states
    assert np.max(np.abs(reference)) <= 0.010001
    for sign in (1, -1):
        result = governor.apply(reference, np.zeros(6), np.array([0, sign * 0.1, -sign * 0.1, 0, 0, 0]), 0.01, enabled=True)
        assert result.scale == 1
    assert not governor.blocked.any()


def test_no_implicit_recovery_without_explicit_permission():
    governor = JointReferenceGovernor()
    for _ in range(300):
        result = governor.apply(stuck_reference(), np.zeros(6), np.zeros(6), 0.01, enabled=False)
        np.testing.assert_array_equal(result.velocity, np.zeros(6))
        assert result.state == "holding"


def test_recovery_waits_for_neutral_dwell_and_cancels_immediately():
    governor = JointReferenceGovernor()
    reference = stuck_reference()
    for _ in range(10):
        result = governor.apply(reference, np.zeros(6), np.zeros(6), 0.01, enabled=False, allow_recovery=True)
        assert not result.velocity.any()
    for _ in range(20):
        result = governor.apply(reference, np.zeros(6), np.zeros(6), 0.01, enabled=False, allow_recovery=True)
        reference += result.velocity * 0.01
    assert result.velocity.any()
    cancelled = governor.apply(reference, np.zeros(6), np.zeros(6), 0.01, enabled=False, allow_recovery=False)
    assert not cancelled.velocity.any()
    assert cancelled.state == "holding"


def test_recovery_stops_after_fixed_budget_instead_of_chasing_forever():
    governor = JointReferenceGovernor()
    reference = stuck_reference() * 10
    initial = reference.copy()
    for _ in range(800):
        result = governor.apply(reference, np.zeros(6), np.zeros(6), 0.01, enabled=False, allow_recovery=True)
        reference += result.velocity * 0.01
    assert np.max(np.abs(reference - initial)) <= 0.120001
    assert not result.velocity.any()
    assert governor.recovery.consumed


def test_recovery_is_not_started_for_normal_small_holding_error():
    governor = JointReferenceGovernor()
    for _ in range(100):
        result = governor.apply(np.full(6, 0.005), np.zeros(6), np.zeros(6), 0.01, enabled=False, allow_recovery=True)
        assert not result.reference_rebased


def test_active_input_preempts_recovery_and_reset_clears_it():
    governor = JointReferenceGovernor()
    for _ in range(30):
        governor.apply(stuck_reference(), np.zeros(6), np.zeros(6), 0.01, enabled=False, allow_recovery=True)
    result = governor.apply(stuck_reference(), np.zeros(6), np.array([0.1, 0, 0, 0, 0, 0]), 0.01, enabled=True, allow_recovery=True)
    assert result.state != "reference_recovery"
    assert result.velocity[0] > 0
    governor.reset()
    assert governor.recovery.destination is None
    assert not governor.blocked.any()


@pytest.mark.parametrize("settings", [{"max_speed_rad_s": 0}, {"max_duration_s": float("nan")}, {"residual_fraction": 1}])
def test_invalid_recovery_settings_are_rejected(settings):
    with pytest.raises(ValueError):
        ReferenceRecoverySettings(**settings)
