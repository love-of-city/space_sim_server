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


def test_stationary_flight_reference_changes_only_when_command_or_reset_changes():
    position, velocity = np.arange(8, dtype=float), np.zeros(8)
    publisher = HeldJointReferencePublisher(lambda _: (position, velocity), 8, publish_on_change=True)
    publisher.Reset(0)
    reader = publisher.positionOutMsgs[2].addSubscriber()
    publisher.UpdateState(2_500_000)
    assert reader.timeWritten() == 0 and reader().state == 2.
    position[2] = 2.25
    publisher.UpdateState(5_000_000)
    assert reader.timeWritten() == 5_000_000 and reader().state == 2.25
    velocity[2] = .1
    publisher.UpdateState(7_500_000)
    assert publisher.velocityOutMsgs[2].read().state == .1
    publisher.Reset(0)
    assert reader.timeWritten() == 0 and reader().state == 2.25
