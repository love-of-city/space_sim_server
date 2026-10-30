"""Directional joint-reference protection, independent of IK and the simulator."""

from dataclasses import dataclass

import numpy as np

from simulation.reference_recovery import ReferenceRecovery


@dataclass(frozen=True)
class ReferenceGovernorSettings:
    soft_error_rad: tuple[float, ...] = (0.04, 0.04, 0.04, 0.025, 0.025, 0.025)
    hard_error_rad: tuple[float, ...] = (0.08, 0.08, 0.08, 0.06, 0.06, 0.05)
    saturation_threshold: float = 0.98
    saturation_hold_s: float = 0.15

    def __post_init__(self) -> None:
        soft = np.asarray(self.soft_error_rad)
        hard = np.asarray(self.hard_error_rad)
        if soft.shape != (6,) or hard.shape != (6,):
            raise ValueError("reference error limits must contain six values")
        if not np.all(np.isfinite(soft)) or not np.all(np.isfinite(hard)):
            raise ValueError("reference error limits must be finite")
        if np.any(soft <= 0) or np.any(hard <= soft):
            raise ValueError("reference limits must satisfy 0 < soft < hard")
        if not 0 < self.saturation_threshold <= 1:
            raise ValueError("saturation threshold must be in (0, 1]")
        if not np.isfinite(self.saturation_hold_s) or self.saturation_hold_s <= 0:
            raise ValueError("saturation hold duration must be finite and positive")


@dataclass(frozen=True)
class GovernedReference:
    velocity: np.ndarray
    scale: float
    state: str
    limited_joints: tuple[int, ...]
    reference_rebased: bool = False


class JointReferenceGovernor:
    """Bound outward reference lead while allowing reversal and automatic recovery."""

    def __init__(self, settings: ReferenceGovernorSettings | None = None) -> None:
        self.settings = settings or ReferenceGovernorSettings()
        self.soft = np.asarray(self.settings.soft_error_rad)
        self.hard = np.asarray(self.settings.hard_error_rad)
        self.reset()

    def reset(self) -> None:
        self.saturation_seconds = np.zeros(6)
        self.blocked = np.zeros(6, dtype=bool)
        self.recovery = ReferenceRecovery()

    def apply(
        self, reference: np.ndarray, measured: np.ndarray, requested_velocity: np.ndarray,
        dt: float, *, enabled: bool, effort_ratio: np.ndarray | None = None,
        allow_recovery: bool = False,
    ) -> GovernedReference:
        vectors = [np.asarray(value, dtype=float) for value in (reference, measured, requested_velocity)]
        if any(value.shape != (6,) or not np.all(np.isfinite(value)) for value in vectors):
            raise ValueError("governor requires six finite joint values")
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("governor interval must be finite and positive")
        reference, measured, requested_velocity = vectors
        effort = np.zeros(6) if effort_ratio is None else np.asarray(effort_ratio, dtype=float)
        if effort.shape != (6,) or not np.all(np.isfinite(effort)) or np.any(effort < 0):
            raise ValueError("effort ratios must contain six finite non-negative values")
        error = reference - measured
        outward = error * requested_velocity > 0
        recovering = error * requested_velocity < 0
        self.blocked[(np.abs(error) <= self.soft) | recovering] = False
        saturated_outward = (
            enabled & outward & (np.abs(error) > self.soft)
            & (effort >= self.settings.saturation_threshold)
        )
        self.saturation_seconds = np.where(saturated_outward, self.saturation_seconds + dt, 0.0)
        self.blocked |= self.saturation_seconds >= self.settings.saturation_hold_s
        recovery_velocity = self.recovery.update(
            reference, measured, self.soft, dt, enabled=enabled,
            permitted=allow_recovery and not np.any(requested_velocity),
        )
        if recovery_velocity is not None:
            affected = tuple(int(index + 1) for index in np.flatnonzero(self.recovery.selected))
            return GovernedReference(recovery_velocity, 0.0, self.recovery.state, affected, True)
        if not enabled:
            return GovernedReference(np.zeros(6), 0.0, "holding", ())

        scales = np.ones(6)
        taper = np.clip((self.hard - np.abs(error)) / (self.hard - self.soft), 0.0, 1.0)
        scales[outward] = taper[outward]
        moving = np.abs(requested_velocity) > 1e-12
        available = self.hard - np.sign(requested_velocity) * error
        scales[moving] = np.minimum(
            scales[moving], np.clip(available[moving] / (np.abs(requested_velocity[moving]) * dt), 0.0, 1.0)
        )
        blocked_outward = self.blocked & outward
        scales[blocked_outward] = 0.0
        scale = float(np.min(scales))
        limited = tuple(int(index + 1) for index in np.flatnonzero(scales < 1.0 - 1e-9))
        state = "saturation_limited" if np.any(blocked_outward) else "tracking_limited" if limited else "active"
        return GovernedReference(requested_velocity * scale, scale, state, limited)
