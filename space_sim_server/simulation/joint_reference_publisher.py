"""Publish held references at control-task rate, not at every RK stage."""

from collections.abc import Callable

import numpy as np
from Basilisk.architecture import messaging, sysModel


class HeldJointReferencePublisher(sysModel.SysModel):
    def __init__(self, reference: Callable[[float], tuple[np.ndarray, np.ndarray]], joint_count: int) -> None:
        super().__init__()
        self.ModelTag = "heldJointReferencePublisher"
        self.clock = None
        self.stride = 1
        self.reference = reference
        self.joint_count = joint_count
        self.positionOutMsgs = [messaging.ScalarJointStateMsg() for _ in range(joint_count)]
        self.velocityOutMsgs = [messaging.ScalarJointStateMsg() for _ in range(joint_count)]
        # Msg.write() creates a new author per call; keep one per message.
        self._position_writers = [message.addAuthor() for message in self.positionOutMsgs]
        self._velocity_writers = [message.addAuthor() for message in self.velocityOutMsgs]
        self._payload = messaging.ScalarJointStateMsgPayload()

    def Reset(self, current_sim_nanos: int) -> None:
        self.UpdateState(current_sim_nanos)

    def UpdateState(self, current_sim_nanos: int) -> None:
        if self.clock is not None and self.clock.step_index % self.stride:
            return
        position, velocity = self.reference(current_sim_nanos * 1e-9)
        if len(position) != self.joint_count or len(velocity) != self.joint_count:
            raise ValueError("joint reference dimensions must match the actuator chain")
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(velocity)):
            raise ValueError("joint references must be finite")
        payload = self._payload
        for index in range(self.joint_count):
            payload.state = float(position[index])
            self._position_writers[index](payload, self.moduleID, current_sim_nanos)
            payload.state = float(velocity[index])
            self._velocity_writers[index](payload, self.moduleID, current_sim_nanos)
