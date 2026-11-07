"""Causal bounded references for the original capture arm; no native writes.

The caller must connect these references to the twelve original implicit
servos, apply only legal initial states, and provide the previous 400 Hz native
encoder publication. This controller alone is not a physical environment.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from pydantic import TypeAdapter

from simulation.capture_arm_target import CaptureArmTarget
from space_arm_platform.benchmark_capture_arm import Arm6, CaptureArmCommand, CaptureEncoders

PERIOD_NS = 2_500_000
COMMAND_LIFETIME_NS = 200_000_000


@dataclass(frozen=True)
class CaptureJointReference:
    """Requested servo references, not measured joint state or applied effort."""
    control_time_ns: int
    encoder_sample_time_ns: int
    command_time_ns: int | None
    command_fresh: bool
    position_si: tuple[float, ...]
    velocity_si_s: tuple[float, ...]
    status: str


class CaptureArmReferenceController:
    def __init__(self, model, *, episode_id, epoch_ns=0, initial_arm_rad=None):
        if type(epoch_ns) is not int or epoch_ns < 0:
            raise ValueError('capture controller requires a nonnegative integer epoch')
        if not isinstance(episode_id, str) or not 1 <= len(episode_id) <= 128:
            raise ValueError('capture controller requires an episode identity')
        self._target = CaptureArmTarget(model)
        if initial_arm_rad is not None:
            initial = np.asarray(TypeAdapter(Arm6).validate_python(initial_arm_rad), dtype=float)
            if (initial.shape != (6,) or not np.isfinite(initial).all()
                    or np.any(initial < self._target.lower[:6]) or np.any(initial > self._target.upper[:6])):
                raise ValueError('capture initial arm pose is outside its authored joint limits')
            self._target.position[:6] = initial
        self.episode_id, self.epoch_ns = episode_id, epoch_ns
        self.last_reference_time_ns = epoch_ns
        self.last_command_time_ns = None
        self._command = CaptureArmCommand()

    @property
    def initial_reference(self):
        if self.last_reference_time_ns != self.epoch_ns:
            raise ValueError('initial reference is only available before physical stepping')
        return tuple(float(value) for value in self._target.position), (0.,)*12

    def submit(self, command, sample_time_ns):
        command = CaptureArmCommand.model_validate(command)
        if (type(sample_time_ns) is not int or sample_time_ns != self.last_reference_time_ns
                or self.last_command_time_ns is not None and sample_time_ns <= self.last_command_time_ns):
            raise ValueError('capture command must be new at the current published control boundary')
        self._command, self.last_command_time_ns = command, sample_time_ns

    def advance(self, control_time_ns, encoders):
        """COMMAND precedes PHYSICS: consume the immediately preceding sample."""
        encoders = CaptureEncoders.model_validate(encoders)
        if (type(control_time_ns) is not int or control_time_ns != self.last_reference_time_ns+PERIOD_NS
                or encoders.sample_time_ns != control_time_ns-PERIOD_NS
                or encoders.episode_id != self.episode_id):
            raise ValueError('capture reference requires contiguous 400 Hz time and matching previous encoders')
        fresh = (self.last_command_time_ns is not None
                 and control_time_ns-self.last_command_time_ns < COMMAND_LIFETIME_NS)
        position, velocity = self._target.step(PERIOD_NS*1e-9, self._command.legacy_action(),
                                             not fresh, np.asarray(encoders.positions()))
        self.last_reference_time_ns = control_time_ns
        return CaptureJointReference(control_time_ns, encoders.sample_time_ns,
            self.last_command_time_ns, fresh, tuple(float(value) for value in position),
            tuple(float(value) for value in velocity), self._target.status)
