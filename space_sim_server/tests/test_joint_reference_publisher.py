import numpy as np
import pytest

from simulation.joint_reference_publisher import HeldJointReferencePublisher


def test_reference_publisher_holds_messages_until_next_control_update():
    position = np.arange(8, dtype=float) / 10
    velocity = np.zeros(8)
    publisher = HeldJointReferencePublisher(lambda _seconds: (position, velocity), 8)
    publisher.Reset(0)
    assert publisher.positionOutMsgs[3].read().state == pytest.approx(0.3)
    position[3] = 0.4
    velocity[3] = 0.2
    assert publisher.positionOutMsgs[3].read().state == pytest.approx(0.3)
    publisher.UpdateState(10_000_000)
    assert publisher.positionOutMsgs[3].read().state == pytest.approx(0.4)
    assert publisher.velocityOutMsgs[3].read().state == pytest.approx(0.2)


def test_reference_publisher_reset_publishes_current_initial_state():
    position = np.zeros(8)
    publisher = HeldJointReferencePublisher(lambda _seconds: (position, np.zeros(8)), 8)
    publisher.UpdateState(10_000_000)
    position[0] = 0.3
    publisher.Reset(0)
    assert publisher.positionOutMsgs[0].read().state == pytest.approx(0.3)


@pytest.mark.parametrize("position", [np.zeros(7), np.full(8, np.nan)])
def test_invalid_reference_is_rejected(position):
    publisher = HeldJointReferencePublisher(lambda _seconds: (position, np.zeros(8)), 8)
    with pytest.raises(ValueError):
        publisher.UpdateState(0)
