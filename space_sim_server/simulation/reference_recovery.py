"""Bounded, explicitly permitted unloading of stale joint-reference error."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ReferenceRecoverySettings:
    neutral_delay_s: float = 0.20
    max_speed_rad_s: float = 0.08
    max_acceleration_rad_s2: float = 0.40
    max_correction_rad: float = 0.12
    max_duration_s: float = 2.0
    residual_fraction: float = 0.25

    def __post_init__(self) -> None:
        values = (self.neutral_delay_s, self.max_speed_rad_s, self.max_acceleration_rad_s2,
                  self.max_correction_rad, self.max_duration_s, self.residual_fraction)
        if not all(np.isfinite(value) and value > 0 for value in values):
            raise ValueError("recovery settings must be finite and positive")
        if self.residual_fraction >= 1:
            raise ValueError("recovery residual fraction must be below one")


class ReferenceRecovery:
    """One finite reference retreat per release; never changes physical joint state."""

    def __init__(self, settings: ReferenceRecoverySettings | None = None) -> None:
        self.settings = settings or ReferenceRecoverySettings()
        self.reset()

    def reset(self) -> None:
        self.neutral_seconds = 0.0
        self.elapsed_seconds = 0.0
        self.destination: np.ndarray | None = None
        self.velocity = np.zeros(6)
        self.selected = np.zeros(6, dtype=bool)
        self.consumed = False
        self.state = "holding"

    def update(
        self, reference: np.ndarray, measured: np.ndarray, soft_limits: np.ndarray,
        dt: float, *, enabled: bool, permitted: bool,
    ) -> np.ndarray | None:
        if enabled:
            self.reset()
            return None
        if not permitted:
            self.neutral_seconds = 0.0
            self.destination = None
            self.velocity.fill(0.0)
            self.state = "holding"
            return None
        if self.consumed and self.destination is None:
            return None
        self.neutral_seconds += dt
        error = reference - measured
        if self.destination is None:
            if self.neutral_seconds < self.settings.neutral_delay_s:
                return None
            self.selected = np.abs(error) > soft_limits
            if not np.any(self.selected):
                return None
            correction = np.sign(error) * np.minimum(
                np.maximum(np.abs(error) - soft_limits * self.settings.residual_fraction, 0.0),
                self.settings.max_correction_rad,
            )
            self.destination = reference - np.where(self.selected, correction, 0.0)
            self.consumed = True
        self.elapsed_seconds += dt
        remaining = self.destination - reference
        toward_actual = measured - reference
        allowed = self.selected & (remaining * toward_actual > 0) & (
            np.abs(error) > soft_limits * self.settings.residual_fraction
        )
        if self.elapsed_seconds >= self.settings.max_duration_s or not np.any(allowed):
            self.destination = None
            self.velocity.fill(0.0)
            self.state = "recovery_complete"
            return self.velocity.copy()
        desired = np.clip(remaining / dt, -self.settings.max_speed_rad_s, self.settings.max_speed_rad_s)
        change = self.settings.max_acceleration_rad_s2 * dt
        velocity = self.velocity + np.clip(desired - self.velocity, -change, change)
        max_step = np.minimum(np.abs(remaining), np.abs(toward_actual))
        self.velocity = np.where(
            allowed, np.sign(remaining) * np.minimum(np.abs(velocity), max_step / dt), 0.0,
        )
        self.state = "reference_recovery"
        return self.velocity.copy()
