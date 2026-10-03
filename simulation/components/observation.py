"""Observation snapshot: sample the authoritative state on each render frame."""
from __future__ import annotations

from typing import Any, Callable

from simulation.assembly import AssemblyContext, Slot


class ObservationComponent:
    """Snapshot the state immediately after the render frame that carries it."""

    name = "observation"

    def __init__(self, bridge: Any, snapshot: Callable[[int, int], dict[str, Any]]) -> None:
        self.bridge = bridge
        self.snapshot = snapshot
        self.model: Any = None

    def install(self, ctx: AssemblyContext) -> None:
        from simulation.observation_capture import AuthoritativeObservationModel

        model = AuthoritativeObservationModel(self.bridge, self.snapshot)
        self.model = model
        # Same OUTPUT segment, registered after the render bridge: the frame is
        # already published, so the snapshot matches it exactly.
        ctx.add(model, Slot.OUTPUT, every=1)
        ctx.keep_alive(model)
