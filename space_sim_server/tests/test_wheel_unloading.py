import numpy as np
import pytest

from simulation.wheel_unloading import WheelUnloading


def test_unloading_opposes_stored_momentum_with_bounded_external_torque():
    unloading = WheelUnloading()
    speeds = np.array([606., -500., 100.])
    torque = unloading.update(speeds, [.01]*3, [628.]*3)
    assert np.linalg.norm(torque) == pytest.approx(.02)
    assert torque[0] < 0 < torque[1]
    assert torque[2] == 0
    # With attitude held, total angular momentum changes only by the actual
    # external torque. The required wheel impulse removes stored momentum.
    assert torque@(speeds*.01) < 0


def test_hysteresis_survives_handoff_and_does_not_chatter():
    unloading = WheelUnloading()
    assert not unloading.update([69., 0., 0.], [.01]*3, [100.]*3).any()
    unloading.update([71., 0., 0.], [.01]*3, [100.]*3)
    saved = unloading.snapshot()
    restored = WheelUnloading()
    restored.restore(saved)
    assert restored.snapshot() == saved
    assert restored.update([60., 0., 0.], [.01]*3, [100.]*3)[0] < 0
    assert not restored.update([50., 0., 0.], [.01]*3, [100.]*3).any()
    assert not restored.update([60., 0., 0.], [.01]*3, [100.]*3).any()


@pytest.mark.parametrize('speeds,inertias,limits', [
    ([np.nan, 0, 0], [.01]*3, [100]*3),
    ([0, 0], [.01]*3, [100]*3),
    ([0]*3, [0]*3, [100]*3),
    ([0]*3, [.01]*3, [-1]*3),
])
def test_invalid_encoder_or_hardware_data_is_rejected(speeds, inertias, limits):
    with pytest.raises(ValueError):
        WheelUnloading().update(speeds, inertias, limits)


def test_changed_unloading_policy_is_rejected_at_handoff():
    saved = WheelUnloading().snapshot()
    saved['max_torque_nm'] = .2
    with pytest.raises(ValueError, match='policy changed'):
        WheelUnloading().restore(saved)


def test_corrupted_actuator_history_is_rejected_at_handoff():
    saved = WheelUnloading().snapshot()
    saved['torque_body_nm'] = [1., 0., 0.]
    with pytest.raises(ValueError, match='history'):
        WheelUnloading().restore(saved)
