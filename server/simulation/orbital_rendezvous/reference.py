"""Smooth bounded-thrust reference; never assigned to propagated states."""

from dataclasses import dataclass

import numpy as np

MU = 3.986004418e14
EARTH_RADIUS = 6378136.6
_QUADRATURE_X, _QUADRATURE_W = np.polynomial.legendre.leggauss(32)


def gravity(position):
    r = np.asarray(position)
    return -MU * r / np.linalg.norm(r) ** 3


def blend(time, duration):
    u = np.clip(time / duration, 0.0, 1.0)
    return (
        10 * u**3 - 15 * u**4 + 6 * u**5,
        (30 * u**2 - 60 * u**3 + 30 * u**4) / duration,
        (60 * u - 180 * u**2 + 120 * u**3) / duration**2,
    )


@dataclass(frozen=True)
class FlightProfile:
    scenario: str = "in-orbit"
    separation_m: float = 7000.0
    target_altitude_m: float = 500000.0
    initial_altitude_m: float = 300000.0
    raise_duration_s: float = 6000.0
    approach_duration_s: float = 5000.0
    hold_duration_s: float = 120.0
    # Match SpaceSim's default orbit, rather than starting on Earth's night side.
    initial_true_anomaly_deg: float = 180.0
    # Keep a 0.5 m nominal margin outside the 100 m near-field boundary.
    handoff_distance_m: float = 100.5

    def __post_init__(self):
        if self.scenario not in ("in-orbit", "orbit-raise"):
            raise ValueError("Unknown flight scenario")
        for name, value in vars(self).items():
            if name == "initial_true_anomaly_deg":
                if not np.isfinite(value):
                    raise ValueError("initial_true_anomaly_deg must be finite")
            elif name != "scenario" and (not np.isfinite(value) or value <= 0):
                raise ValueError(f"{name} must be positive and finite")
        if (
            self.separation_m <= self.handoff_distance_m
            or self.initial_altitude_m >= self.target_altitude_m
        ):
            raise ValueError("Invalid flight altitude or separation")

    @property
    def start_approach_s(self):
        return self.raise_duration_s if self.scenario == "orbit-raise" else 0.0

    @property
    def duration_s(self):
        return self.start_approach_s + self.approach_duration_s + self.hold_duration_s

    def radius(self, t):
        r1, r2 = (
            EARTH_RADIUS + self.initial_altitude_m,
            EARTH_RADIUS + self.target_altitude_m,
        )
        if self.scenario == "in-orbit":
            return r2, 0.0, 0.0
        b, db, ddb = blend(t, self.raise_duration_s)
        return r1 + (r2 - r1) * b, (r2 - r1) * db, (r2 - r1) * ddb

    def target(self, t):
        r = EARTH_RADIUS + self.target_altitude_m
        n = np.sqrt(MU / r**3)
        return self.polar(r, 0.0, 0.0, np.deg2rad(self.initial_true_anomaly_deg) + n * t, n, 0.0)

    @staticmethod
    def polar(r, dr, ddr, theta, dtheta, ddtheta):
        # Shared inclination of 51.6 degrees, matching the platform default.
        c, s = np.cos(np.deg2rad(51.6)), np.sin(np.deg2rad(51.6))
        radial = np.array([np.cos(theta), np.sin(theta) * c, np.sin(theta) * s])
        tangent = np.array([-np.sin(theta), np.cos(theta) * c, np.cos(theta) * s])
        return (
            r * radial,
            dr * radial + r * dtheta * tangent,
            (ddr - r * dtheta**2) * radial + (2 * dr * dtheta + r * ddtheta) * tangent,
        )

    def chaser(self, t):
        r2 = EARTH_RADIUS + self.target_altitude_m
        n2 = np.sqrt(MU / r2**3)
        if t < self.start_approach_s:
            r, dr, ddr = self.radius(t)
            # Set initial phase to meet the target's orbit 7 km behind it.
            half = 0.5 * (self.raise_duration_s - t)
            nodes = t + half * (1 + _QUADRATURE_X)
            remaining = half * np.dot(
                _QUADRATURE_W, np.sqrt(MU / self.radius(nodes)[0] ** 3)
            )
            theta = np.deg2rad(self.initial_true_anomaly_deg) + n2 * self.raise_duration_s - self.separation_m / r2 - remaining
            n = np.sqrt(MU / r**3)
            return self.polar(r, dr, ddr, theta, n, -1.5 * n * dr / r)
        b, db, ddb = blend(t - self.start_approach_s, self.approach_duration_s)
        delta = self.handoff_distance_m - self.separation_m
        distance, rate, acc = self.separation_m + delta * b, delta * db, delta * ddb
        return self.polar(
            r2, 0.0, 0.0, np.deg2rad(self.initial_true_anomaly_deg) + n2 * t - distance / r2, n2 - rate / r2, -acc / r2
        )

    def phase(self, t):
        if t < self.start_approach_s:
            return "Orbit raising"
        if t < self.start_approach_s + self.approach_duration_s:
            return "Team approach"
        return "100 m handoff hold"
