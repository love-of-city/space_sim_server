"""Known-target line-of-sight tracking from noisy public spacecraft telemetry."""
from __future__ import annotations

import numpy as np

from space_arm_platform.benchmark_attitude import WheelTorqueAction
from simulation.benchmark_attitude_methods import NoWheelTorque, UnsafeSpin, quaternion_product


def line_of_sight(goal, state, ephemeris):
    position, velocity = np.asarray(state[:3]), np.asarray(state[3:])
    if goal.kind == "inertial_direction":
        return np.asarray(goal.direction_j2000_unit), np.zeros(3)
    if goal.kind == "earth_center":
        relative, relative_rate = -position, -velocity
    elif goal.kind == "sun":
        relative = np.asarray(ephemeris.sun_geocentric_j2000[:3])-position
        relative_rate = np.asarray(ephemeris.sun_geocentric_j2000[3:])-velocity
    else:
        fixed = 6378136.6*np.asarray(goal.ground_direction_iau_earth_unit)
        c = np.asarray(ephemeris.earth_j2000_to_fixed)
        cdot = np.asarray(ephemeris.earth_j2000_to_fixed_rate_s)
        target = c.T@fixed
        target_rate = -c.T@cdot@target
        relative, relative_rate = target-position, target_rate-velocity
    distance = np.linalg.norm(relative)
    direction = relative/distance
    return direction, (relative_rate-direction*(direction@relative_rate))/distance


def reference_frame(direction, derivative):
    """+Z along LOS; roll uses projected inertial north with a polar fallback."""
    guide = np.array([0., 0., 1.]) if abs(direction[2]) < .95 else np.array([1., 0., 0.])
    raw_x = np.cross(guide, direction)
    x = raw_x/np.linalg.norm(raw_x)
    raw_dx = np.cross(guide, derivative)
    dx = (raw_dx-x*(x@raw_dx))/np.linalg.norm(raw_x)
    y, dy = np.cross(direction, x), np.cross(derivative, x)+np.cross(direction, dx)
    frame = np.column_stack((x, y, direction))
    angular_velocity = .5*(np.cross(x, dx)+np.cross(y, dy)+np.cross(direction, derivative))
    return frame, angular_velocity


def matrix_to_quaternion(matrix):
    # Use the long-standing xyzw API, including the project's SciPy 1.11 floor.
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(matrix).as_quat()[[3, 0, 1, 2]]


class PointingWheelPD:
    frozen_reference = False

    def reset(self, context):
        self.context = context
        self.initial_reference = None

    def act(self, observation):
        if not observation.gyro_valid:
            torque = np.zeros(3)
        else:
            direction, derivative = line_of_sight(self.context.task.objective, observation.chaser_gnss_j2000, observation.ephemeris)
            frame, reference_rate = reference_frame(direction, derivative)
            target = matrix_to_quaternion(frame)
            if self.frozen_reference:
                if self.initial_reference is None:
                    self.initial_reference = target.copy()
                target, reference_rate = self.initial_reference, np.zeros(3)
            measured = np.asarray(observation.orientation_body_to_j2000_wxyz)
            error = quaternion_product(target*[1., -1., -1., -1.], measured)
            if error[0] < 0:
                error = -error
            sigma = error[1:]/(1.+error[0])
            from scipy.spatial.transform import Rotation
            body_to_inertial = Rotation.from_quat(measured[[1, 2, 3, 0]]).as_matrix()
            feedforward_body_rate = body_to_inertial.T@reference_rate
            desired_rate = feedforward_body_rate-(8./12.)*sigma
            limit_rate = min(.08, self.context.task.safety.maximum_body_rate_rad_s/2)
            desired_rate *= min(1., limit_rate/max(np.linalg.norm(desired_rate), 1e-12))
            omega = np.asarray(observation.angular_velocity_body_rad_s)
            momentum = (np.asarray(self.context.inertia_com_body_kg_m2)@omega
                        + np.asarray(self.context.wheel_spin_inertia_kg_m2)*observation.wheel_speed_rad_s)
            body_torque = -12.*(omega-desired_rate)+np.cross(omega, momentum)
            limit = self.context.task.safety.maximum_motor_torque_nm
            torque = np.clip(-body_torque, -limit, limit)
        return WheelTorqueAction(episode_id=observation.episode_id, observation_sequence=observation.sequence,
                                 motor_torque_xyz_nm=tuple(torque))

    def close(self):
        pass


class FrozenPointingPD(PointingWheelPD):
    frozen_reference = True


METHODS = {"ephemeris-pointing-pd-v1": PointingWheelPD, "frozen-pointing-pd-v1": FrozenPointingPD,
           "no-pointing-torque-v1": NoWheelTorque, "unsafe-pointing-spin-v1": UnsafeSpin}
