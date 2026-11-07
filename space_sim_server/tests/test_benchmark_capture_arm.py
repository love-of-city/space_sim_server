"""Capture method boundary, causality, expiry and bounded reference checks."""
from pathlib import Path
import math

import numpy as np
import pytest

from simulation.benchmark_capture_arm_control import (
    COMMAND_LIFETIME_NS, PERIOD_NS, CaptureArmReferenceController,
)
from simulation.capture26_mission import MODEL_PATH
from space_arm_platform.benchmark_capture_arm import CaptureArmCommand, CaptureEncoders

SERVER = Path(__file__).resolve().parents[1]
EPOCH = 8_000_000_000_000


def reading(time=EPOCH, **changes):
    return CaptureEncoders(**dict(dict(episode_id='capture-a', sample_time_ns=time,
        arm_position_rad=(0.,)*6, arm_velocity_rad_s=(0.,)*6,
        axial_position_m=.02, axial_velocity_m_s=0., cam_angle_rad=math.pi/3, cam_velocity_rad_s=0.,
        jaw_position_m=(-.012,)*4, jaw_velocity_m_s=(0.,)*4), **changes))


def controller(**changes):
    return CaptureArmReferenceController(SERVER/MODEL_PATH,
        **dict(dict(episode_id='capture-a', epoch_ns=EPOCH), **changes))


def motion():
    return CaptureArmCommand(enabled=True, joint6_velocity_rad_s=.25,
                             axial_velocity_m_s=-.003, radial_velocity_m_s=.003)


def test_public_encoders_keep_mixed_units_and_no_pose_or_target_truth():
    measured = reading(arm_position_rad=(0., 0., 0., 0., 0., 7.))
    assert measured.positions() == (0., 0., 0., 0., 0., 7., .02, math.pi/3, *(-.012,)*4)
    # Physical joint overshoot must remain visible to the independent scorer.
    assert measured.arm_position_rad[-1] == 7.
    assert CaptureEncoders.model_validate_json(measured.model_dump_json()) == measured
    for private in ('target_pose', 'body_origin', 'native_step_count', 'joint_reference'):
        with pytest.raises(ValueError):
            CaptureEncoders.model_validate(dict(measured.model_dump(), **{private: [0., 0., 0.]}))


@pytest.mark.parametrize('change', [
    {'enabled': 1}, {'enabled': 'true'}, {'joint6_velocity_rad_s': True},
    {'joint6_velocity_rad_s': .251}, {'axial_velocity_m_s': -.0031}, {'radial_velocity_m_s': .004},
    {'linear_velocity_tcp_in_bus_m_s': (.02, .02, 0.)},
    {'angular_velocity_in_bus_rad_s': (.1, .1, 0.)},
    {'angular_velocity_in_tool_rad_s': (0., 0., float('nan'))},
    {'linear_velocity_tcp_in_bus_m_s': (0., 0., float('inf'))},
    {'target_pose': [0., 0., 0.]}, {'base_body': 'capture_target'}, {'tool_site': 'truth_com'},
    {'qpos': [0.]},
])
def test_invalid_or_undeclared_commands_are_rejected_before_control(change):
    with pytest.raises(ValueError):
        CaptureArmCommand.model_validate(dict(motion().model_dump(), **change))


def test_disabled_command_is_explicit_and_cannot_hide_motion():
    with pytest.raises(ValueError, match='disabled'):
        CaptureArmCommand(joint6_velocity_rad_s=.1)
    action = CaptureArmCommand().legacy_action()
    assert action['active_arm'] == 'capture' and action['deadman'] is False
    assert action['arm_preparation'] is False
    assert not {'target_pose', 'qpos', 'force', 'torque', 'initial_state'} & action.keys()


def test_original_mechanism_converts_units_and_couples_cam_with_four_jaws():
    control = controller()
    before = reading()
    control.submit(motion(), EPOCH)
    result = control.advance(EPOCH+PERIOD_NS, before)
    assert result.command_fresh and result.control_time_ns-result.encoder_sample_time_ns == PERIOD_NS
    assert result.position_si[5] == pytest.approx(.25*.0025, abs=1e-15)
    assert result.position_si[6] == pytest.approx(.02-.003*.0025, abs=1e-15)
    assert np.allclose(result.position_si[8:], -.012+.003*.0025, atol=1e-15, rtol=0)
    assert result.position_si[7] == pytest.approx(-result.position_si[8]/.012*(math.pi/3), abs=1e-15)
    assert result.velocity_si_s[5] == pytest.approx(.25)
    assert before == reading()  # References never rewrite measured encoder state.


def test_blocked_arm_cannot_accumulate_a_reference_beyond_original_lead_limit():
    control = controller()
    for step in range(160):
        now = EPOCH+step*PERIOD_NS
        if step%20 == 0:
            control.submit(motion(), now)
        result = control.advance(now+PERIOD_NS, reading(now))
        assert max(abs(value) for value in result.position_si[:6]) <= .05
        assert max(abs(value) for value in result.velocity_si_s[:6]) <= .25+1e-12
        assert -.006 <= result.position_si[6] <= .02
        assert all(-.012 <= value <= 0 for value in result.position_si[8:])
    assert result.position_si[5] == .05 and abs(result.velocity_si_s[5]) < 1e-12


def test_expired_action_latches_measured_pose_once_and_new_action_resumes_from_encoders():
    control = controller()
    control.submit(motion(), EPOCH)
    for step in range(1, COMMAND_LIFETIME_NS//PERIOD_NS):
        result = control.advance(EPOCH+step*PERIOD_NS, reading(EPOCH+(step-1)*PERIOD_NS))
        assert result.command_fresh
    now = EPOCH+COMMAND_LIFETIME_NS
    result = control.advance(now, reading(now-PERIOD_NS))
    assert not result.command_fresh and result.status == 'hold'
    assert result.position_si == reading().positions() and result.velocity_si_s == (0.,)*12
    drifting = reading(now, arm_position_rad=(0., 0., 0., 0., 0., .1))
    held = control.advance(now+PERIOD_NS, drifting)
    assert held.position_si == result.position_si
    control.submit(motion(), now+PERIOD_NS)
    resumed = control.advance(now+2*PERIOD_NS,
        reading(now+PERIOD_NS, arm_position_rad=drifting.arm_position_rad))
    assert resumed.position_si[5] == pytest.approx(.1+.25*.0025)


@pytest.mark.parametrize('case', ['future_encoder', 'old_encoder', 'wrong_episode', 'duplicate_tick', 'gap', 'bool_time'])
def test_invalid_clock_or_episode_does_not_advance_reference_history(case):
    control = controller()
    control.submit(motion(), EPOCH)
    stamp, encoder = EPOCH+PERIOD_NS, reading()
    if case == 'future_encoder':
        encoder = reading(EPOCH+PERIOD_NS)
    elif case == 'old_encoder':
        encoder = reading(EPOCH-PERIOD_NS)
    elif case == 'wrong_episode':
        encoder = reading(episode_id='another-episode')
    elif case == 'duplicate_tick':
        stamp = EPOCH
    elif case == 'gap':
        stamp += PERIOD_NS
    else:
        stamp = True
    initial = control.initial_reference
    with pytest.raises(ValueError, match='contiguous'):
        control.advance(stamp, encoder)
    assert control.last_reference_time_ns == EPOCH and control.initial_reference == initial
    assert control.advance(EPOCH+PERIOD_NS, reading()).command_fresh


def test_command_clock_rejects_duplicates_or_future_without_replacing_held_action():
    control = controller()
    control.submit(motion(), EPOCH)
    for stamp in (EPOCH, EPOCH-1, EPOCH+1, True):
        with pytest.raises(ValueError, match='new at the current'):
            control.submit(CaptureArmCommand(), stamp)
    result = control.advance(EPOCH+PERIOD_NS, reading())
    assert result.position_si[5] > 0


def test_fresh_controller_rebuild_clears_commands_and_reproduces_reference_sequence():
    def run():
        control = controller()
        control.submit(motion(), EPOCH)
        result = [control.advance(EPOCH+index*PERIOD_NS, reading(EPOCH+(index-1)*PERIOD_NS))
                  for index in range(1, 20)]
        with pytest.raises(ValueError, match='before physical stepping'):
            control.initial_reference
        return result
    assert run() == run()
    fresh = controller()
    held = fresh.advance(EPOCH+PERIOD_NS, reading())
    assert not held.command_fresh and held.position_si == reading().positions()


@pytest.mark.parametrize('initial', [[True]*6, ['0']*6, [0.]*5, [math.inf]*6, [20.]*6])
def test_invalid_initial_configuration_is_rejected(initial):
    with pytest.raises(ValueError):
        controller(initial_arm_rad=initial)
