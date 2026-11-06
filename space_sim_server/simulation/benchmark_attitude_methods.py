"""Body-rate/MRP feedback baseline with the team's attitude-control gains."""
from __future__ import annotations

import numpy as np

from space_arm_platform.benchmark_attitude import AttitudeContext, AttitudeObservation, WheelTorqueAction


def quaternion_product(left, right):
    a, b = np.asarray(left), np.asarray(right)
    return np.r_[a[0]*b[0] - a[1:] @ b[1:], a[0]*b[1:] + b[0]*a[1:] + np.cross(a[1:], b[1:])]


class AttitudeWheelPD:
    rate_limited = False

    def reset(self, context: AttitudeContext):
        self.context = context

    def act(self, observation: AttitudeObservation):
        if not observation.gyro_valid:
            torque = np.zeros(3)
        else:
            target = np.asarray(self.context.task.objective.orientation_body_to_j2000_wxyz)
            error = quaternion_product(target * [1., -1., -1., -1.], observation.orientation_body_to_j2000_wxyz)
            if error[0] < 0:
                error = -error
            sigma = error[1:] / (1 + error[0])
            omega = np.asarray(observation.angular_velocity_body_rad_s)
            momentum = (np.asarray(self.context.inertia_com_body_kg_m2) @ omega
                        + np.asarray(self.context.wheel_spin_inertia_kg_m2) * observation.wheel_speed_rad_s)
            # Existing team gains: 8 N m per MRP and 12 N m s/rad. Opposite
            # motor sign: positive wheel torque gives negative bus reaction.
            reference_rate = -(8. / 12.) * sigma
            if self.rate_limited:
                rate_limit = min(.08, self.context.task.safety.maximum_body_rate_rad_s / 2)
                reference_rate *= min(1., rate_limit / max(np.linalg.norm(reference_rate), 1e-12))
                body_torque = -12. * (omega - reference_rate) + np.cross(omega, momentum)
            else:
                # Preserve the established baseline arithmetic exactly.
                body_torque = -8. * sigma - 12. * omega + np.cross(omega, momentum)
            limit = self.context.task.safety.maximum_motor_torque_nm
            torque = np.clip(-body_torque, -limit, limit)
        return WheelTorqueAction(episode_id=observation.episode_id, observation_sequence=observation.sequence,
                                 motor_torque_xyz_nm=tuple(torque))

    def close(self):
        pass


class NoWheelTorque(AttitudeWheelPD):
    def act(self, observation):
        return WheelTorqueAction(episode_id=observation.episode_id, observation_sequence=observation.sequence,
                                 motor_torque_xyz_nm=(0., 0., 0.))


class UnsafeSpin(AttitudeWheelPD):
    def act(self, observation):
        limit = self.context.task.safety.maximum_motor_torque_nm
        return WheelTorqueAction(episode_id=observation.episode_id, observation_sequence=observation.sequence,
                                 motor_torque_xyz_nm=(limit, limit, limit))


class RateLimitedAttitudePD(AttitudeWheelPD):
    rate_limited = True


METHODS = {"attitude-wheel-pd-v1": AttitudeWheelPD, "no-wheel-torque-v1": NoWheelTorque,
           "unsafe-spin-v1": UnsafeSpin, "attitude-rate-limited-pd-v1": RateLimitedAttitudePD}
