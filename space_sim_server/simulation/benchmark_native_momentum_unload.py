"""Native wheel unloading: actuator messages, with no post-reset state writes."""
from Basilisk.architecture import messaging

from simulation.benchmark_momentum_unload_model import prepare_momentum_unload_model
from simulation.benchmark_native_attitude import NativeAttitudeEnvironment
from space_arm_platform.benchmark_attitude import WheelTorqueAction
from space_arm_platform.benchmark_momentum_unload import MomentumUnloadContext, MomentumUnloadTruthSample
from space_arm_platform.benchmark_momentum_unload_score import MomentumUnloadScorer


class NativeMomentumUnloadEnvironment(NativeAttitudeEnvironment):
    context_type = MomentumUnloadContext
    scorer_type = MomentumUnloadScorer
    model_builder = staticmethod(prepare_momentum_unload_model)

    def _setup_additional_observation(self, ctx, seed):
        self._external_messages = [messaging.SingleActuatorMsg().write(messaging.SingleActuatorMsgPayload(input=0.)) for _ in range(3)]
        names = [f'momentum_dump_{axis}' for axis in range(3)]
        for name, message in zip(names, self._external_messages, strict=True):
            ctx.ports.drive_actuator(name, message, owner='benchmark_momentum_unload_method')
        self._external_indices = [self._stepper.actuator_names.index(name) for name in names]
        self._requested_external_torque = (0., 0., 0.)

    def _truth_sample(self, nanos):
        return MomentumUnloadTruthSample(**super()._truth_sample(nanos).model_dump(),
            requested_external_torque_body_nm=self._requested_external_torque,
            applied_external_torque_body_nm=tuple(float(self._stepper.actuator_force[i]) for i in self._external_indices))

    def step(self, action):
        if self._simulation is None or self._observation is None:
            raise RuntimeError('reset must precede step')
        if self.evaluation()['outcome'] != 'incomplete':
            raise RuntimeError('episode has terminated; reset before another step')
        action.validate_for(self._observation, self.task)
        self._requested_external_torque = action.external_torque_body_nm
        for message, torque in zip(self._external_messages, action.external_torque_body_nm, strict=True):
            message.write(messaging.SingleActuatorMsgPayload(input=torque), self._time_ns)
        wheel = WheelTorqueAction(**action.model_dump(exclude={'schema_version', 'external_torque_body_nm'}))
        return super().step(wheel)

    def provenance(self):
        result = super().provenance()
        result.update(actuation_model='original protected wheel joint motors plus three explicitly declared ideal pure rotational site motors; no translation command',
            external_torque_model='body axes, vector norm <= 0.02 N m; no thruster geometry, minimum impulse bit, fuel, plume or electrical energy model')
        return result
