"""Image-only approach using actual body-axis motors, without truth guidance."""
import numpy as np
from Basilisk.architecture import messaging

from space_arm_platform.benchmark_vision_approach import (
    VisionApproachContext,VisionApproachEstimateEvidence,VisionApproachObservation,
)
from space_arm_platform.benchmark_vision_approach_score import VisionApproachScorer
from simulation.benchmark_native_flight import _InertialForceActuator
from simulation.benchmark_native_vision import NativeVisionPoseEnvironment,public_context


class _BodyForceActuator(_InertialForceActuator):
    def __init__(self,ports):
        super().__init__(ports)
        self.ModelTag='benchmarkBodyAxisForce'

    def UpdateState(self,nanos):
        # Inputs already refer to physical body axes. Do not rotate the
        # method's command using private attitude or an inertial guidance law.
        for message,value in zip(self.messages,self.command,strict=True):
            message.write(messaging.SingleActuatorMsgPayload(input=float(value)),nanos)


def approach_context(task,chaser_mass_kg):
    return public_context(task,chaser_mass_kg,context_type=VisionApproachContext)


class NativeVisionApproachEnvironment(NativeVisionPoseEnvironment):
    context_type=staticmethod(approach_context)
    scorer_type=VisionApproachScorer
    actuator_type=_BodyForceActuator

    def _measure(self,sequence):
        observation=super()._measure(sequence)
        return VisionApproachObservation(**observation.model_dump(exclude={'schema_version'}))

    def step(self,action):
        if self._simulation is None or self._observation is None:
            raise RuntimeError('reset must precede visual control')
        if self.evaluation()['outcome']!='incomplete':
            raise RuntimeError('episode has terminated; reset before visual control')
        action.validate_for(self._observation,self.task)
        record=VisionApproachEstimateEvidence(sequence=action.observation_sequence,estimate_time_ns=action.estimate_time_ns,
            reference_frame=action.reference_frame,image_sha256=tuple(image.png_sha256 for image in self._observation.images),
            pose=action.pose,force_frame=action.force_frame,force_body_n=action.force_body_n)
        self._scorer.add(record)
        self._truth_stream.write(record.model_dump_json()+'\n')
        self._actuator.command=np.asarray(action.force_body_n)
        self._time_ns+=self.task.decision_period_ns
        self._simulation.ConfigureStopTime(self._time_ns)
        self._simulation.ExecuteSimulation()
        self._observation=self._measure(self._observation.sequence+1)
        return self._observation

    def provenance(self):
        value=super().provenance()
        value.update(actuation_model='bounded body-axis COM force held over each image interval; original native motors; passive wheels',
            control_authority='one chaser only; no target telemetry or actuation; no truth-derived steering',
            terminal_condition='continuous physical optical-frame position and speed dwell evaluated at image decision boundaries')
        return value
