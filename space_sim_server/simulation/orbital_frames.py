"""Local-frame sampling and the force-provider contract; no BSK gravity law here.

The frame translates with O and its axes remain parallel to the inertial axes.
The physics core depends on this protocol, not on a particular gravity model.
"""
from __future__ import annotations

from typing import Protocol

import numpy as np


class OrbitalForceProvider(Protocol):
    """An installed provider owns orbital differential forces, not integration.

Sources are configured before stepping. begin_interval snapshots environment
inputs; forces is evaluated at each substep START on fresh authoritative COMs.
Return a finite (nbody, 3) array of COM forces in inertial/local axes [N], with
world/massless entries zero. Never include user wrenches (drag/thrust) here.
"""

    def reset(self) -> None: ...
    def begin_interval(self) -> None: ...
    def forces(self, positions_local: np.ndarray, masses: np.ndarray,
               origin_position: np.ndarray, nanos: int) -> np.ndarray: ...
    def telemetry(self) -> dict: ...


def interpolate_origin(r0, v0, r1, v1, dt: float, fraction: float):
    """Cubic Hermite position inside an accepted orbit interval, not a solver."""
    u = float(fraction)
    if not np.isfinite(dt) or dt <= 0 or not np.isfinite(u) or not 0 <= u <= 1:
        raise ValueError("origin sampling needs dt > 0 and fraction in [0, 1]")
    return ((2*u**3 - 3*u**2 + 1)*r0 + (u**3 - 2*u**2 + u)*dt*v0
            + (-2*u**3 + 3*u**2)*r1 + (u**3 - u**2)*dt*v1)
