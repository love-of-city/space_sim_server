"""Original capture servos with causal native encoders and private readback.

The environment owns initialization, all other actuators, scheduling and close.
This module never assigns physical state or provides target truth to a method.
"""
from __future__ import annotations

from dataclasses import asdict
import hashlib
import math
from pathlib import Path

from simulation.benchmark_capture_arm_control import PERIOD_NS, CaptureArmReferenceController
from simulation.benchmark_capture_model import verify_capture_model
from simulation.capture_arm_target import INITIAL_JOINTS
from space_arm_platform.benchmark_capture_arm import CaptureEncoders


class NativeCaptureEncoderReader:
    def __init__(self, ports, *, episode_id, epoch_ns=0):
        if type(epoch_ns) is not int or epoch_ns < 0:
            raise ValueError('capture encoder epoch must be a nonnegative integer')
        if not isinstance(episode_id, str) or not 1 <= len(episode_id) <= 128:
            raise ValueError('capture encoders require an episode identity')
        self.stepper = ports.stepper
        if self.stepper is None:
            raise ValueError('capture encoders require the authoritative local stepper')
        self.episode_id, self.epoch_ns = episode_id, epoch_ns
        self.positions = tuple(ports.joint_state(joint).addSubscriber() for _, joint, _ in INITIAL_JOINTS)
        self.velocities = tuple(ports.joint_rate(joint).addSubscriber() for _, joint, _ in INITIAL_JOINTS)

    def physical_time(self, sample_time_ns):
        if not self.stepper._handle:
            raise ValueError('native capture runtime is closed')
        if type(sample_time_ns) is not int or sample_time_ns < self.epoch_ns:
            raise ValueError('capture sampling requires an integer physical epoch')
        local = sample_time_ns-self.epoch_ns
        if (local % PERIOD_NS or self.stepper._last_nanos != local
                or self.stepper.step_count != local//PERIOD_NS):
            raise ValueError('capture sampling requires the current contiguous 400 Hz native publication')
        if self.stepper.external_writes != 0:
            raise ValueError('capture runtime received a post-initialization physical state write')
        return local

    def sample(self, sample_time_ns):
        local = self.physical_time(sample_time_ns)
        values = []
        for readers in (self.positions, self.velocities):
            if any(not reader.isWritten() or int(reader.timeWritten()) != local for reader in readers):
                raise ValueError('capture joint publication is missing or stale')
            values.append(tuple(float(reader().state) for reader in readers))
        q, v = values
        return CaptureEncoders(episode_id=self.episode_id, sample_time_ns=sample_time_ns,
            arm_position_rad=q[:6], arm_velocity_rad_s=v[:6], axial_position_m=q[6], axial_velocity_m_s=v[6],
            cam_angle_rad=q[7], cam_velocity_rad_s=v[7], jaw_position_m=q[8:], jaw_velocity_m_s=v[8:])


def _publisher(binding):
    # Lazy import keeps the read-port integrity checks available in basic CI.
    from Basilisk.architecture import messaging, sysModel

    class Publisher(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.ModelTag = 'benchmarkCaptureArmReferences'
            self.positionOutMsgs = [messaging.ScalarJointStateMsg() for _ in INITIAL_JOINTS]
            self.velocityOutMsgs = [messaging.ScalarJointStateMsg() for _ in INITIAL_JOINTS]
            self.writers = [[message.addAuthor() for message in messages]
                            for messages in (self.positionOutMsgs, self.velocityOutMsgs)]
            self.payload = messaging.ScalarJointStateMsgPayload()

        def Reset(self, nanos):
            binding.check_initial_reset(nanos)
            self.UpdateState(nanos)

        def UpdateState(self, nanos):
            position, velocity = binding.reference_at(nanos)
            for writers, values in zip(self.writers, (position, velocity), strict=True):
                for writer, value in zip(writers, values, strict=True):
                    self.payload.state = value
                    writer(self.payload, self.moduleID, nanos)

    return Publisher()


class NativeCaptureArm:
    """Install before physics; schedule ``publisher`` in COMMAND at 400 Hz.

    Rebuild on reset. Only ``encoders.sample`` is a method observation;
    ``evidence`` contains private requested/applied forces and provenance.
    """
    def __init__(self, ports, model, *, repository, episode_id, epoch_ns=0, initial_arm_rad=None):
        model = Path(model)
        core = ports.stepper
        if core is None or not core._handle or core._last_nanos is not None or core.servos_finalized:
            raise ValueError('capture servos must be installed before native initialization')
        manifest = verify_capture_model(repository, model.parent)
        digest = hashlib.sha256(model.read_bytes()).hexdigest()
        if digest != manifest['generated_sha256'] or digest != core.model_xml_sha256:
            raise ValueError('capture servos must bind the verified, actually loaded full model')
        self.controller = CaptureArmReferenceController(model, episode_id=episode_id,
            epoch_ns=epoch_ns, initial_arm_rad=initial_arm_rad)
        self.channels = tuple(dict(row) for row in manifest['capture_joints'])
        names = tuple(row['actuator'] for row in self.channels)
        connected = {core.actuator_names[index] for index in core.servo_index}
        if any(name not in core.actuator_names or name in ports._actuator_owners
                or name in ports._servo_owners or name in connected for name in names):
            raise ValueError('all original capture actuators must be present and exclusively unclaimed')
        self.encoders = NativeCaptureEncoderReader(ports, episode_id=episode_id, epoch_ns=epoch_ns)
        self.model_sha256, self.episode_id, self.epoch_ns = digest, episode_id, epoch_ns
        self.core = core
        self.last_reference = None
        self.failure = None
        self.publisher = _publisher(self)
        for index, row in enumerate(self.channels):
            ports.drive_servo(row['actuator'], self.publisher.positionOutMsgs[index],
                self.publisher.velocityOutMsgs[index], owner='benchmark_capture_arm', joint=row['joint'],
                kp=row['kp'], kd=row['kd'], limit=row['effort_limit'])
        self.effort_readers = tuple(tuple(message.addSubscriber() for message in ports.servo_effort(name))
                                   for name in names)

    def check_initial_reset(self, nanos):
        if (type(nanos) is not int or nanos != 0 or not self.core._handle or self.core.step_count
                or self.controller.last_reference_time_ns != self.epoch_ns
                or self.controller.last_command_time_ns is not None or self.failure is not None):
            raise ValueError('capture reset requires a fresh native core and arm binding')

    def submit(self, command, sample_time_ns):
        if self.failure is not None:
            raise ValueError('capture publisher failed; rebuild this episode')
        self.encoders.physical_time(sample_time_ns)
        self.controller.submit(command, sample_time_ns)

    def reference_at(self, physics_time_ns):
        if self.failure is not None:
            raise ValueError('capture publisher failed; rebuild this episode')
        try:
            if type(physics_time_ns) is not int or physics_time_ns < 0:
                raise ValueError('capture references require integer native time')
            if physics_time_ns == 0:
                self.check_initial_reset(physics_time_ns)
                return self.controller.initial_reference
            sample = self.encoders.sample(self.epoch_ns+physics_time_ns-PERIOD_NS)
            reference = self.controller.advance(self.epoch_ns+physics_time_ns, sample)
            self.last_reference = reference
            return reference.position_si, reference.velocity_si_s
        except Exception as error:
            self.failure = str(error)
            raise

    def evidence(self, sample_time_ns):
        """Private read-only receipt after PHYSICS; no force is invented at t=0."""
        if self.failure is not None:
            raise ValueError('capture publisher failed; no valid actuation receipt')
        sample = self.encoders.sample(sample_time_ns)
        local = sample_time_ns-self.epoch_ns
        effort = None
        reference = None
        if local:
            if self.last_reference is None or self.last_reference.control_time_ns != sample_time_ns:
                raise ValueError('capture actuation lacks its matching causal reference')
            reference = asdict(self.last_reference)
            effort = []
            for row, readers in zip(self.channels, self.effort_readers, strict=True):
                if any(not reader.isWritten() or int(reader.timeWritten()) != local for reader in readers):
                    raise ValueError('capture servo effort publication is missing or stale')
                requested, applied = (float(reader().input) for reader in readers)
                if not math.isfinite(requested) or not math.isfinite(applied):
                    raise ValueError('capture servo effort must be finite')
                index = self.core.actuator_names.index(row['actuator'])
                if applied != float(self.core.actuator_force[index]):
                    raise ValueError('capture servo effort does not match its actual native actuator')
                effort.append(dict(actuator=row['actuator'], unit=row['effort_unit'],
                    requested_post_step_pid=requested, applied_native=applied))
        return dict(schema_version='spacesim-autonomy-capture-arm-actuation/1',
            episode_id=self.episode_id, model_xml_sha256=self.model_sha256,
            epoch_ns=self.epoch_ns, sample_time_ns=sample_time_ns, physics_time_ns=local,
            native_step_count=self.core.step_count, external_state_writes=self.core.external_writes,
            encoders=sample.model_dump(mode='json'), reference=reference, effort=effort,
            initial_reference_si=list(self.controller.initial_reference[0]) if local == 0 else None,
            effort_semantics='post-step requested PID diagnostic and actual native solver force; mixed units explicit per actuator')
