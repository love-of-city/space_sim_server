import numpy as np
import pytest

from simulation.cooperative_pointing import (
    bounded_pointing_torque, camera_facing_reference, pointing_status,
)


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle)*cross + (1-np.cos(angle))*(cross@cross)


@pytest.mark.parametrize('depth', [110., 50., 17.4, 12., 5.5])
def test_correct_target_minus_service_sign_offsets_and_fixed_roll(depth):
    relative = np.array([depth, .3, -.2])
    service = rotation([1, 2, -3], .7)
    camera = np.array([.4, -.15, .35])
    marker = np.array([.1755, .05475, .0420787001299893])
    R = camera_facing_reference(relative_position_n_m=relative,
        service_rotation_n_from_body=service, camera_midpoint_service_m=camera,
        marker_center_target_m=marker, up_n=[0, 0, 1])
    sight = -relative + service@camera - R@marker
    np.testing.assert_allclose(R[:, 1], sight/np.linalg.norm(sight), atol=1e-10)
    np.testing.assert_allclose(R.T@R, np.eye(3), atol=1e-12)
    assert np.linalg.det(R) == pytest.approx(1)
    assert R[2, 2] > 0


def test_rotated_inertial_coordinates_and_moving_camera_change_reference():
    relative = np.array([17.4, 1., -.5]); camera = np.array([.4, -.1, .3])
    marker = np.array([.17, .05, .04]); up = np.array([0., 0., 1.])
    base = camera_facing_reference(relative_position_n_m=relative,
        service_rotation_n_from_body=np.eye(3),camera_midpoint_service_m=camera,
        marker_center_target_m=marker,up_n=up)
    Q = rotation([1, -2, 3], 1.2)
    transformed = camera_facing_reference(relative_position_n_m=Q@relative,
        service_rotation_n_from_body=Q,camera_midpoint_service_m=camera,
        marker_center_target_m=marker,up_n=Q@up)
    np.testing.assert_allclose(transformed,Q@base,atol=1e-10)
    moved = camera_facing_reference(relative_position_n_m=relative,
        service_rotation_n_from_body=np.eye(3),camera_midpoint_service_m=camera+[0,.3,0],
        marker_center_target_m=marker,up_n=up)
    assert np.linalg.norm(moved-base) > .01


def test_stationary_hold_needs_no_relative_velocity_and_rejects_undefined_roll():
    kwargs=dict(relative_position_n_m=[17.4,0,0],service_rotation_n_from_body=np.eye(3),
                camera_midpoint_service_m=[0,0,0],marker_center_target_m=[0,0,0])
    camera_facing_reference(**kwargs,up_n=[0,0,1])
    with pytest.raises(ValueError,match='parallel'):
        camera_facing_reference(**kwargs,up_n=[1,0,0])
    with pytest.raises(ValueError,match='at least 1 m'):
        camera_facing_reference(**(kwargs|{'relative_position_n_m':[.5,0,0]}),up_n=[0,0,1])


@pytest.mark.parametrize('angle', [np.pi, np.pi-1e-8, -np.pi+1e-8, 2.1, -.4])
def test_back_facing_target_has_nonzero_bounded_corrective_torque(angle):
    current = rotation([1.,2.,3.],angle)
    torque=bounded_pointing_torque(target_rotation_n_from_body=current,
        reference_rotation_n_from_body=np.eye(3),target_angular_rate_body_rad_s=[0,0,0],
        reference_angular_rate_n_rad_s=[0,0,0])
    assert 0 < np.linalg.norm(torque) <= .02+1e-12
    # Independently step the current orientation in the commanded direction.
    after=current@rotation(torque,1e-4)
    assert np.trace(after) > np.trace(current)


def status(**overrides):
    return pointing_status(**(dict(now_s=10.,input_stamps_s=dict(navigation=9.9,
        service_attitude=10.,target_telemetry=9.9,camera_kinematics=10.),
        target_rotation_n_from_body=np.eye(3),reference_rotation_n_from_body=np.eye(3),
        target_angular_rate_body_rad_s=[0,0,0],reference_angular_rate_n_rad_s=[0,0,0],
        navigation_position_sigma_m=.03)|overrides))


def test_attitude_and_gnss_are_separate_inputs_and_stale_data_revokes_alignment():
    assert status().ready
    for name in ['navigation','service_attitude','target_telemetry','camera_kinematics']:
        stamps=dict(navigation=10.,service_attitude=10.,target_telemetry=10.,camera_kinematics=10.)
        stamps[name]=9.
        assert status(input_stamps_s=stamps).reason=='input_stale'
    assert not status(input_stamps_s=dict(navigation=10.)).ready
    assert not status(navigation_position_sigma_m=.06).ready
    assert not status(inputs_valid=False).ready
    assert not status(target_rotation_n_from_body=rotation([0,0,1],np.deg2rad(2))).ready
    assert not status(target_angular_rate_body_rad_s=[.002,0,0]).ready


def test_tracking_orbit_rate_is_not_required_to_be_inertially_zero():
    current=rotation([1,0,0],.4)
    omega_n=np.array([0,0,.0012])
    assert status(target_rotation_n_from_body=current,reference_rotation_n_from_body=current,
        reference_angular_rate_n_rad_s=omega_n,target_angular_rate_body_rad_s=current.T@omega_n).ready
