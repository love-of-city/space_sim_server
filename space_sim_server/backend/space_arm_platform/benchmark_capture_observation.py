"""Public capture observations: current encoders and causally delivered pixels.

The native environment supplies actual exposures and separately records private
mount/physics receipts. No renderer transform, target state or file path enters
these contracts. This component does not register a complete capture task.
"""
from __future__ import annotations

from collections import deque
from typing import Annotated, Literal

from pydantic import Field, model_validator

from .benchmark_articulated_camera import MountedCameraImage, MountedPinholeCamera
from .benchmark_capture_arm import CaptureArmCommand, CaptureEncoders
from .benchmark_contract import Contract, Count

PERIOD_NS = 100_000_000
DELIVERY_DELAY_NS = 200_000_000
EpisodeId = Annotated[str, Field(strict=True, min_length=1, max_length=128)]


class CapturedStereoFrame(Contract):
    schema_version: Literal['spacesim-autonomy-captured-stereo/1'] = 'spacesim-autonomy-captured-stereo/1'
    episode_id: EpisodeId
    sample_time_ns: Count
    delivery_time_ns: Count
    capture_encoders: CaptureEncoders
    images: tuple[MountedCameraImage, MountedCameraImage]

    @model_validator(mode='after')
    def one_exposure(self):
        first, second = self.images
        if (self.delivery_time_ns != self.sample_time_ns+DELIVERY_DELAY_NS
                or self.capture_encoders.episode_id != self.episode_id
                or self.capture_encoders.sample_time_ns != self.sample_time_ns
                or first.camera_id == second.camera_id
                or first.sample_time_ns != self.sample_time_ns or second.sample_time_ns != self.sample_time_ns
                or first.renderer_session_id != second.renderer_session_id
                or first.source_frame_id != second.source_frame_id):
            raise ValueError('capture stereo requires simultaneous pixels/encoders and fixed 200 ms delivery')
        return self


class CaptureObservation(Contract):
    schema_version: Literal['spacesim-autonomy-capture-observation/1'] = 'spacesim-autonomy-capture-observation/1'
    episode_id: EpisodeId
    epoch_ns: Count
    sequence: Count
    decision_time_ns: Count
    current_encoders: CaptureEncoders
    vision_status: Literal['warming_up', 'delivered']
    vision: CapturedStereoFrame | None

    @model_validator(mode='after')
    def causal_decision(self):
        if (self.decision_time_ns != self.epoch_ns+self.sequence*PERIOD_NS
                or self.current_encoders.episode_id != self.episode_id
                or self.current_encoders.sample_time_ns != self.decision_time_ns):
            raise ValueError('capture decisions require current encoders on the declared 10 Hz grid')
        warming_up = self.sequence < DELIVERY_DELAY_NS//PERIOD_NS
        if warming_up:
            if self.vision_status != 'warming_up' or self.vision is not None:
                raise ValueError('capture warmup cannot expose images before their delivery')
        elif (self.vision_status != 'delivered' or self.vision is None
                or self.vision.episode_id != self.episode_id
                or self.vision.delivery_time_ns != self.decision_time_ns):
            raise ValueError('capture observation requires exactly the due stereo frame')
        return self


class CaptureArmAction(Contract):
    schema_version: Literal['spacesim-autonomy-capture-action/1'] = 'spacesim-autonomy-capture-action/1'
    episode_id: EpisodeId
    observation_sequence: Count
    decision_time_ns: Count
    command: CaptureArmCommand

    def validate_for(self, observation):
        observation = CaptureObservation.model_validate(observation)
        if (self.episode_id != observation.episode_id or self.observation_sequence != observation.sequence
                or self.decision_time_ns != observation.decision_time_ns):
            raise ValueError('capture action must refer to this episode and current decision observation')
        if observation.vision_status == 'warming_up' and self.command.enabled:
            raise ValueError('capture visual control must hold during initial image delivery warmup')


class CaptureObservationBuffer:
    """One exposure then one decision per tick; reset by creating a new buffer."""
    def __init__(self, cameras, *, episode_id, epoch_ns=0):
        if type(epoch_ns) is not int or epoch_ns < 0 or not isinstance(episode_id, str) or not 1 <= len(episode_id) <= 128:
            raise ValueError('capture buffer requires a valid episode and integer epoch')
        self.cameras = tuple(MountedPinholeCamera.model_validate(camera) for camera in cameras)
        if (len(self.cameras) != 2 or self.cameras[0].camera_id == self.cameras[1].camera_id
                or any(camera.sample_rate_hz != 10 for camera in self.cameras)):
            raise ValueError('capture buffer requires two distinct 10 Hz cameras')
        self.episode_id, self.epoch_ns = episode_id, epoch_ns
        self.sequence = 0
        self._pending = deque()
        self._current_encoders = None
        self._session = self._last_frame = None
        self.failed = False

    @property
    def pending_frames(self):
        return len(self._pending)

    def capture(self, images, encoders):
        if self.failed:
            raise ValueError('capture buffer failed; reset the episode')
        try:
            encoders = CaptureEncoders.model_validate(encoders)
            now = self.epoch_ns+self.sequence*PERIOD_NS
            frame = CapturedStereoFrame(episode_id=self.episode_id, sample_time_ns=now,
                delivery_time_ns=now+DELIVERY_DELAY_NS, capture_encoders=encoders, images=tuple(images))
            first = frame.images[0]
            if (self._current_encoders is not None or len(self._pending) >= 3
                    or any((image.camera_id, image.width, image.height) != (camera.camera_id, camera.width, camera.height)
                           for image, camera in zip(frame.images, self.cameras, strict=True))
                    or self._session is not None and first.renderer_session_id != self._session
                    or self._last_frame is not None and first.source_frame_id <= self._last_frame):
                raise ValueError('capture frame is duplicated, out of order, or differs from the frozen cameras/session')
            self._pending.append(frame)
            self._current_encoders = encoders
            self._session, self._last_frame = first.renderer_session_id, first.source_frame_id
        except Exception:
            self.failed = True
            raise

    def observe(self, decision_time_ns, current_encoders):
        if self.failed:
            raise ValueError('capture buffer failed; reset the episode')
        try:
            current_encoders = CaptureEncoders.model_validate(current_encoders)
            if type(decision_time_ns) is not int or decision_time_ns != self.epoch_ns+self.sequence*PERIOD_NS:
                raise ValueError('capture decisions require consecutive integer 10 Hz timestamps')
            if self._current_encoders is None or current_encoders != self._current_encoders:
                raise ValueError('capture decision requires this tick\'s completed exposure and current encoder publication')
            frame = self._pending[0] if self._pending and self._pending[0].delivery_time_ns <= decision_time_ns else None
            result = CaptureObservation(episode_id=self.episode_id, epoch_ns=self.epoch_ns, sequence=self.sequence,
                decision_time_ns=decision_time_ns, current_encoders=current_encoders,
                vision_status='warming_up' if frame is None else 'delivered', vision=frame)
            if frame is not None:
                self._pending.popleft()
            self.sequence += 1
            self._current_encoders = None
            return result
        except Exception:
            self.failed = True
            raise
