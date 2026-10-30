"""Pre-vision cooperative target pointing from estimated/telemetered states.

All world components use J2000 axes. ``relative_position_n_m`` is TARGET body
origin minus SERVICE body origin, as in near_field.RelativeEKF.x[:3]. Rotations
map body vectors into J2000. The target's printed face normal is body +Y and
body +Z defines its printed up direction. No renderer, images or truth reader
is imported here. Integration with the mission controller is a separate step.
"""
from dataclasses import dataclass

import numpy as np


def _vector(value, name):
    value = np.asarray(value, dtype=float)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError(f'{name} must be a finite three-vector')
    return value


def _rotation(value):
    value = np.asarray(value, dtype=float)
    if (value.shape != (3, 3) or not np.isfinite(value).all()
            or not np.allclose(value.T @ value, np.eye(3), atol=1e-7, rtol=0)
            or not np.isclose(np.linalg.det(value), 1., atol=1e-7, rtol=0)):
        raise ValueError('rotation must belong to SO(3)')
    return value


def _unit(value):
    norm = np.linalg.norm(value)
    if norm < 1e-10:
        raise ValueError('pointing direction is undefined')
    return value / norm


def camera_facing_reference(*, relative_position_n_m, service_rotation_n_from_body,
                            camera_midpoint_service_m, marker_center_target_m, up_n):
    """Return target body-to-J2000 reference; handle both installation offsets.

    ``camera_midpoint_service_m`` must come from the arm kinematics at the
    navigation prediction epoch. It is not a constant for a moving arm. ``up_n``
    is an explicit roll convention (e.g. an agreed orbital direction), not the
    relative velocity at a stationary hold. This pre-alignment helper rejects
    separations below 1 m; it does not define close-range/contact coordination.
    """
    relative = _vector(relative_position_n_m, 'relative position')
    service = _rotation(service_rotation_n_from_body)
    camera = _vector(camera_midpoint_service_m, 'camera installation')
    marker = _vector(marker_center_target_m, 'marker reference point')
    up = _unit(_vector(up_n, 'roll reference'))
    target_to_camera_origin = -relative + service @ camera
    if np.linalg.norm(target_to_camera_origin) < 1.:
        raise ValueError('pre-alignment requires at least 1 m origin-to-camera separation')

    def frame(normal):
        normal = _unit(normal)
        vertical = up - normal * (up @ normal)
        if np.linalg.norm(vertical) < 1e-6:
            raise ValueError('roll reference is parallel to line of sight')
        vertical = _unit(vertical)
        return np.column_stack((np.cross(normal, vertical), normal, vertical))

    reference = frame(target_to_camera_origin)
    for _ in range(40):
        # The marker reference point itself moves when the target rotates.
        sight = target_to_camera_origin - reference @ marker
        updated = frame(sight)
        if np.linalg.norm(updated - reference) < 1e-11:
            return updated
        reference = updated
    raise ValueError('installation-offset pointing solution did not converge')


def _rotation_vector(rotation):
    angle = float(np.arccos(np.clip((np.trace(rotation) - 1.) / 2., -1., 1.)))
    antisymmetric = np.array([rotation[2, 1] - rotation[1, 2],
                             rotation[0, 2] - rotation[2, 0],
                             rotation[1, 0] - rotation[0, 1]])
    if angle < 1e-7:
        return antisymmetric / 2.
    if np.pi - angle < 1e-6:
        # The usual sin(angle) expression would vanish for a back-facing target.
        _, vectors = np.linalg.eigh((rotation + rotation.T + 2*np.eye(3)) / 4.)
        axis = vectors[:, -1]
        if np.linalg.norm(antisymmetric) > 1e-12:
            axis *= 1 if axis @ antisymmetric >= 0 else -1
        elif axis[np.argmax(np.abs(axis))] < 0:
            axis = -axis
        return angle * axis
    return angle / (2*np.sin(angle)) * antisymmetric


def bounded_pointing_torque(*, target_rotation_n_from_body, reference_rotation_n_from_body,
                            target_angular_rate_body_rad_s, reference_angular_rate_n_rad_s,
                            kp_nm_per_rad=.02, kd_nm_s_per_rad=.12, torque_limit_nm=.02):
    """Bounded equivalent body torque, matching the mainline model's abstraction.

    This is not a reaction-wheel/thruster allocation model. Apply the output to
    the authoritative dynamics; never overwrite the target quaternion.
    """
    current = _rotation(target_rotation_n_from_body)
    reference = _rotation(reference_rotation_n_from_body)
    omega = _vector(target_angular_rate_body_rad_s, 'target angular rate')
    omega_ref = _vector(reference_angular_rate_n_rad_s, 'reference angular rate')
    gains = np.array([kp_nm_per_rad, kd_nm_s_per_rad, torque_limit_nm], dtype=float)
    if not np.isfinite(gains).all() or np.any(gains <= 0):
        raise ValueError('torque gains and limit must be positive')
    error = _rotation_vector(current.T @ reference)
    torque = kp_nm_per_rad * error - kd_nm_s_per_rad * (omega - current.T @ omega_ref)
    return torque * min(1., torque_limit_nm / max(np.linalg.norm(torque), 1e-15))


@dataclass(frozen=True)
class PointingStatus:
    ready: bool
    reason: str
    attitude_error_deg: float | None = None
    rate_error_rad_s: float | None = None


def pointing_status(*, now_s, input_stamps_s, target_rotation_n_from_body,
                    reference_rotation_n_from_body, target_angular_rate_body_rad_s,
                    reference_angular_rate_n_rad_s, navigation_position_sigma_m,
                    inputs_valid=True, max_age_s=.35, max_angle_deg=1.,
                    max_rate_rad_s=.001, max_navigation_sigma_m=.05):
    """Instantaneous pre-alignment assessment, NOT a mission release permit.

    All four sources (navigation, service attitude, target telemetry and camera
    kinematics) need explicit physical timestamps. The mission still owns dwell,
    camera FOV, visual readiness, corridor, braking and contact checks.
    """
    if not inputs_valid:
        return PointingStatus(False, 'input_invalid')
    required = {'navigation', 'service_attitude', 'target_telemetry', 'camera_kinematics'}
    if not np.isfinite(now_s) or not isinstance(input_stamps_s, dict) or set(input_stamps_s) != required:
        return PointingStatus(False, 'input_timestamps_missing')
    stamps = np.asarray(list(input_stamps_s.values()), dtype=float)
    if not np.isfinite(stamps).all() or np.any(stamps > now_s + 1e-9):
        return PointingStatus(False, 'input_timestamp_invalid')
    if np.any(now_s - stamps > max_age_s + 1e-9):
        return PointingStatus(False, 'input_stale')
    if (not np.isfinite(navigation_position_sigma_m)
            or not 0 <= navigation_position_sigma_m < max_navigation_sigma_m):
        return PointingStatus(False, 'navigation_uncertainty')
    current = _rotation(target_rotation_n_from_body)
    reference = _rotation(reference_rotation_n_from_body)
    angle = float(np.rad2deg(np.linalg.norm(_rotation_vector(current.T @ reference))))
    rate = float(np.linalg.norm(_vector(target_angular_rate_body_rad_s, 'target angular rate')
                 - current.T @ _vector(reference_angular_rate_n_rad_s, 'reference angular rate')))
    if angle >= max_angle_deg:
        return PointingStatus(False, 'attitude_not_aligned', angle, rate)
    if rate >= max_rate_rad_s:
        return PointingStatus(False, 'angular_rate_not_settled', angle, rate)
    return PointingStatus(True, 'instantaneously_aligned_not_mission_release', angle, rate)
