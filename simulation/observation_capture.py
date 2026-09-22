"""Freeze telemetry immediately after the render bridge samples native state."""
from __future__ import annotations

from collections import deque
from typing import Callable

from Basilisk.architecture import sysModel


class AuthoritativeObservationModel(sysModel.SysModel):
    """Same task/instant as the bridge; network delivery remains outside physics.

    Physics uses an absolute 240 Hz grid; every eighth step publishes a render
    frame. Snapshot immediately after that frame, not later in the outer loop.
    """
    def __init__(self, bridge, snapshot: Callable[[int, int], dict]):
        super().__init__()
        self.ModelTag = "authoritativeObservationSnapshot"
        self.bridge = bridge
        self.snapshot = snapshot
        self.pending = deque()
        self.last_key = None

    def UpdateState(self, CurrentSimNanos: int) -> None:
        stamp = int(self.bridge.last_published_sim_time_ns)
        frame = int(self.bridge.last_published_frame_id)
        if stamp != int(CurrentSimNanos) or frame < 0:
            return
        key = (self.bridge.session_id, frame)
        if key == self.last_key:
            return
        if len(self.pending) >= 128:
            raise RuntimeError("authoritative observation queue overflow")
        self.pending.append(self.snapshot(frame, stamp))
        self.last_key = key

    def drain(self) -> list[dict]:
        result = list(self.pending)
        self.pending.clear()
        return result
