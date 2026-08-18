import time

import pytest

from space_arm_platform.models import OperatorAction
from space_arm_platform.safety import ActionRejected, SafetyController


def request(sequence: int, deadman: bool = True, linear_speed: float = 0.05) -> OperatorAction:
    return OperatorAction(
        client_sequence=sequence,
        client_time_ns="123456789",
        deadman=deadman,
        end_effector_linear_speed_m_s=linear_speed,
        end_effector_linear_velocity=[2.0, -2.0, 0.5],
        end_effector_angular_velocity=[0.5, 0.0, 0.25],
        gripper_velocity=-0.5,
        input_source="keyboard",
    )


def test_action_is_clipped_scaled_and_deadman_guarded() -> None:
    safety = SafetyController(timeout_s=1.0)
    action = safety.process("operator", request(1), "episode-test")
    assert action.limited
    assert action.end_effector_linear_velocity_body_m_s == pytest.approx([0.05, -0.05, 0.025])
    assert action.end_effector_angular_velocity_body_rad_s == pytest.approx([0.25, 0.0, 0.125])
    assert action.gripper_velocity_rad_s == pytest.approx(-0.4)
    assert action.applied_end_effector_linear_speed_m_s == pytest.approx(0.05)

    stopped = safety.process("operator", request(2, deadman=False), "episode-test")
    assert stopped.end_effector_linear_velocity_body_m_s == [0.0] * 3
    assert stopped.end_effector_angular_velocity_body_rad_s == [0.0] * 3
    assert stopped.gripper_velocity_rad_s == 0.0


def test_runtime_linear_speed_is_applied_and_safety_clamped() -> None:
    safety = SafetyController(timeout_s=1.0)
    faster = safety.process("operator", request(1, linear_speed=0.12), None)
    assert faster.end_effector_linear_velocity_body_m_s == pytest.approx([0.12, -0.12, 0.06])
    assert faster.requested_end_effector_linear_speed_m_s == pytest.approx(0.12)
    assert faster.applied_end_effector_linear_speed_m_s == pytest.approx(0.12)

    clamped = safety.process("operator", request(2, linear_speed=0.50), None)
    assert clamped.limited
    assert clamped.requested_end_effector_linear_speed_m_s == pytest.approx(0.50)
    assert clamped.applied_end_effector_linear_speed_m_s == pytest.approx(0.20)
    assert clamped.end_effector_linear_velocity_body_m_s == pytest.approx([0.20, -0.20, 0.10])


def test_duplicate_sequence_is_rejected() -> None:
    safety = SafetyController()
    safety.process("operator", request(7), None)
    with pytest.raises(ActionRejected):
        safety.process("operator", request(7), None)


def test_timeout_emits_only_one_neutral_action() -> None:
    safety = SafetyController(timeout_s=0.001)
    safety.process("operator", request(1), None)
    time.sleep(0.005)
    neutral = safety.timeout_action(None)
    assert neutral is not None
    assert not neutral.deadman
    assert neutral.reason == "input_timeout"
    assert safety.timeout_action(None) is None
