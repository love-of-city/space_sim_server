import numpy as np
import pytest
from space_arm_platform.models import OperatorAction
from space_arm_platform.safety import SafetyController


def test_capture_commands_are_routed_and_deadman_guarded():
    safety = SafetyController()
    data = dict(client_sequence=1, client_time_ns='1', deadman=True,
                active_arm='capture', capture_radial_velocity=1., capture_axial_velocity=-1.,
                end_effector_linear_velocity=[0.,0.,0.], end_effector_angular_velocity=[0.,0.,0.])
    action = safety.process('a', OperatorAction(**data), None)
    assert action.active_arm == 'capture'
    assert action.capture_radial_velocity_m_s == pytest.approx(.003)
    assert action.capture_axial_velocity_m_s == pytest.approx(-.003)
    for seq, change in enumerate([dict(deadman=False), dict(active_arm='operating')], 2):
        action = safety.process('a', OperatorAction(**(data | change | dict(client_sequence=seq))), None)
        assert action.capture_radial_velocity_m_s == action.capture_axial_velocity_m_s == 0


def test_capture_controller_holds_measured_pose_on_release_and_ignores_other_arm():
    from simulation.capture_arm_target import CaptureArmTarget
    from pathlib import Path
    target = CaptureArmTarget(Path(__file__).resolve().parents[1]/'model/SARM/platform/sarm_dual_capture26.xml')
    actual = target.position.copy()
    action = dict(active_arm='capture', deadman=True, joint6_velocity_rad_s=.2)
    target.step(.01, action, False, actual)
    assert target.position[5] > actual[5]
    actual[5] = .001
    actual[6] = .018
    actual[8:] = -.011
    target.step(.01, action | dict(active_arm='operating'), False, actual)
    assert target.position[5] == actual[5]
    np.testing.assert_allclose(target.position, actual)
    held = target.position.copy()
    for _ in range(3): target.step(.01, action, True, actual)
    np.testing.assert_array_equal(target.position, held)


def test_capture_tool_stroke_limits_and_cam_synchronization():
    from simulation.capture_arm_target import CaptureArmTarget
    from pathlib import Path
    target = CaptureArmTarget(Path(__file__).resolve().parents[1]/'model/SARM/platform/sarm_dual_capture26.xml')
    action = dict(active_arm='capture', deadman=True, capture_radial_velocity_m_s=.003,
                  capture_axial_velocity_m_s=.003)
    for _ in range(2000): target.step(.01, action, False, target.position.copy())
    assert target.position[8] == pytest.approx(0)
    assert target.position[7] == pytest.approx(0)
    assert np.all(target.position <= target.upper + 1e-9)
    assert np.all(target.position >= target.lower - 1e-9)


def test_arm_only_command_does_not_realign_measured_tool():
    from simulation.capture_arm_target import CaptureArmTarget
    from pathlib import Path
    target=CaptureArmTarget(Path(__file__).resolve().parents[1]/'model/SARM/platform/sarm_dual_capture26.xml')
    measured=target.position.copy()
    measured[7]=.9
    measured[8:]=[-.011,-.0111,-.0112,-.0113]
    position,_=target.step(.01,dict(active_arm='capture',deadman=True,joint6_velocity_rad_s=.1),False,measured)
    np.testing.assert_allclose(position[6:],measured[6:])
