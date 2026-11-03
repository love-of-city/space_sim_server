"""Passive body-rate sensor; truth enters only at the measurement boundary.

Units are rad/s. White noise is per-sample standard deviation, while bias
random walk is rad/s/sqrt(s). These are configurable engineering assumptions,
not a calibrated hardware model or an attitude estimator.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass
import math

import numpy as np


GYRO_HZ = 50
GYRO_PERIOD_NS = 1_000_000_000 // GYRO_HZ


@dataclass(frozen=True)
class GyroConfig:
    enabled: bool = True
    white_noise_std_rad_s: float = 1e-4
    initial_bias_std_rad_s: float = 1e-5
    bias_random_walk_rad_s_sqrt_s: float = 1e-6
    range_rad_s: float = 0.5

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError("gyro enabled must be boolean")
        for key, value in asdict(self).items():
            if key == "enabled":
                continue
            if (isinstance(value, bool) or not isinstance(value, (float, int))
                    or not math.isfinite(value) or value < 0 or value > 1e3):
                raise ValueError(f"gyro {key} must be finite and in [0, 1000]")
        if self.range_rad_s <= 0:
            raise ValueError("gyro range_rad_s must be positive")

    @classmethod
    def from_mapping(cls, value):
        if not isinstance(value, dict):
            raise ValueError("service_gyro must be an object")
        try:
            return cls(**value)
        except TypeError as error:
            raise ValueError(f"unsupported service_gyro configuration: {error}") from error


class Gyroscope:
    """One measurement per physical 50 Hz tick, independent of telemetry reads."""

    def __init__(self, config: GyroConfig | None = None, *, seed: int = 0):
        if isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed < 2**64:
            raise ValueError("gyro seed must be an unsigned 64-bit integer")
        self.config = config or GyroConfig()
        self.seed = seed
        self.reset()

    def reset(self):
        # A private stream never changes scene/GNSS/vision randomness. Separate
        # bias and white streams also keep bias evolution independent of reads.
        sequence = np.random.SeedSequence([self.seed, 0x4759524F])
        bias_seed, noise_seed = sequence.spawn(2)
        self._bias_rng = np.random.default_rng(bias_seed)
        self._noise_rng = np.random.default_rng(noise_seed)
        self._bias = self._bias_rng.normal(0, self.config.initial_bias_std_rad_s, 3)
        self._last = None

    @property
    def latest(self):
        return deepcopy(self._last)

    def sample(self, sample_time_ns: int, angular_velocity_body_rad_s):
        if (isinstance(sample_time_ns, bool) or not isinstance(sample_time_ns, int)
                or sample_time_ns < 0 or sample_time_ns % GYRO_PERIOD_NS):
            raise ValueError("gyro timestamp must be on the nonnegative 50 Hz physical grid")
        truth = np.asarray(angular_velocity_body_rad_s, dtype=float)
        if truth.shape != (3,) or not np.isfinite(truth).all():
            raise ValueError("gyro body rate must be a finite three-vector")
        if not self.config.enabled:
            return None
        if self._last is not None:
            previous = int(self._last["sample_time_ns"])
            if sample_time_ns == previous:
                return self.latest
            if sample_time_ns - previous != GYRO_PERIOD_NS:
                raise ValueError("gyro sampling must be contiguous; reset explicitly for a new scene")
            self._bias += self._bias_rng.normal(
                0, self.config.bias_random_walk_rad_s_sqrt_s / math.sqrt(GYRO_HZ), 3)
        elif sample_time_ns != 0:
            raise ValueError("a new gyro stream must start at physical time zero")
        measured = truth + self._bias + self._noise_rng.normal(
            0, self.config.white_noise_std_rad_s, 3)
        saturated = np.abs(measured) > self.config.range_rad_s
        # Prior error variance, including unknown accumulated bias. Saturation
        # invalidates this Gaussian approximation and the reading for control.
        variance = (self.config.white_noise_std_rad_s**2
                    + self.config.initial_bias_std_rad_s**2
                    + self.config.bias_random_walk_rad_s_sqrt_s**2 * sample_time_ns * 1e-9)
        self._last = {
            "sensor_name": "service_gyro", "body_name": "cubesat_bus",
            "frame": "spacecraft_body", "reference_frame": "J2000",
            "measurement": "simulated_body_angular_rate", "sample_rate_hz": GYRO_HZ,
            "sample_time_ns": str(sample_time_ns),
            "angular_velocity_body_rad_s": np.clip(
                measured, -self.config.range_rad_s, self.config.range_rad_s).tolist(),
            "error_variance_rad2_s2": [variance] * 3,
            "saturated_axes": saturated.tolist(), "valid": not bool(saturated.any()),
        }
        return self.latest
