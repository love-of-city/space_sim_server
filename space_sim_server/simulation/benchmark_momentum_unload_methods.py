"""Existing hysteretic unload policy with wheel reaction-torque feedforward."""
import numpy as np

from simulation.benchmark_attitude_methods import AttitudeWheelPD
from simulation.wheel_unloading import WheelUnloading
from space_arm_platform.benchmark_momentum_unload import MomentumUnloadAction


class MomentumUnloadPD(AttitudeWheelPD):
    def reset(self, context):
        super().reset(context)
        self.unloader = WheelUnloading()
        self.unloader.max_torque_nm = context.task.safety.maximum_external_torque_norm_nm

    def command(self, observation):
        feedback = np.asarray(super().act(observation).motor_torque_xyz_nm)
        external = self.unloader.update(observation.wheel_speed_rad_s,
            self.context.wheel_spin_inertia_kg_m2, self.context.hardware_speed_limits_rad_s)
        # Positive wheel torque reacts negatively on the bus. Adding the same
        # external torque to the wheel command unloads rotors while preserving
        # the PD controller's requested net bus torque.
        return feedback+external, external

    def act(self, observation):
        wheel, external = self.command(observation)
        if not observation.gyro_valid:
            wheel, external = np.zeros(3), np.zeros(3)
        limit = self.context.task.safety.maximum_motor_torque_nm
        bound = self.context.task.safety.maximum_external_torque_norm_nm
        external = external*min(1., bound/max(np.linalg.norm(external), 1e-12))
        return MomentumUnloadAction(episode_id=observation.episode_id, observation_sequence=observation.sequence,
            motor_torque_xyz_nm=tuple(np.clip(wheel, -limit, limit)), external_torque_body_nm=tuple(external))


class AttitudeOnlyUnload(MomentumUnloadPD):
    def command(self, observation):
        return np.asarray(AttitudeWheelPD.act(self, observation).motor_torque_xyz_nm), np.zeros(3)


class WheelsOnlyUnload(MomentumUnloadPD):
    def command(self, observation):
        # Braking wheels without an external couple transfers their stored
        # angular momentum into bus rotation. This is an explicit negative control.
        limit = self.context.task.safety.maximum_motor_torque_nm
        return -limit*np.sign(observation.wheel_speed_rad_s), np.zeros(3)


class UnsafeUnloadSpin(MomentumUnloadPD):
    def command(self, observation):
        return np.full(3, self.context.task.safety.maximum_motor_torque_nm), np.zeros(3)


METHODS = {'momentum-unload-pd-v1': MomentumUnloadPD, 'attitude-only-unload-v1': AttitudeOnlyUnload,
    'wheels-only-unload-v1': WheelsOnlyUnload, 'unsafe-unload-spin-v1': UnsafeUnloadSpin}
