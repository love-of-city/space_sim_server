"""Smooth reference timing with cruise, terminal zones and derivative limits.

Quintic velocity blends make path acceleration continuous. A C2 joint spline
removes the velocity jumps at piecewise-linear IK knots. Only actuator references
are generated; physical state and measured sensor samples are never filtered.
"""

from dataclasses import dataclass
import math
import numpy as np
from .joint_path import JointPath


@dataclass(frozen=True)
class PathTiming:
    path: JointPath
    progress: np.ndarray
    velocity: np.ndarray
    times: np.ndarray

    @property
    def duration(self):
        return float(self.times[-1])

    def state(self, seconds):
        """Progress and its first three time derivatives."""
        if seconds <= 0:
            return 0.0, 0.0, 0.0, 0.0
        if seconds >= self.duration:
            return 1.0, 0.0, 0.0, 0.0
        i = int(np.searchsorted(self.times, seconds, side="right") - 1)
        duration = self.times[i + 1] - self.times[i]
        x = (seconds - self.times[i]) / duration
        v = self.velocity[i]
        delta = self.velocity[i + 1] - v
        u = self.progress[i] + duration * (
            v * x + delta * (2.5 * x**4 - 3 * x**5 + x**6)
        )
        speed = v + delta * (10 * x**3 - 15 * x**4 + 6 * x**5)
        acceleration = delta / duration * (30 * x**2 - 60 * x**3 + 30 * x**4)
        jerk = delta / duration**2 * (60 * x - 180 * x**2 + 120 * x**3)
        return float(u), float(speed), float(acceleration), float(jerk)

    def limit_derivatives(self, bounds, speed, acceleration, jerk):
        """Dilate time to bound another curve driven by this same progress law."""
        m = np.asarray(bounds, dtype=float)
        if m.shape != (3,) or not np.isfinite(m).all() or np.any(m < 0):
            raise ValueError("Expected three finite nonnegative spatial derivative bounds")
        if any(not math.isfinite(x) or x <= 0 for x in (speed, acceleration, jerk)):
            raise ValueError("Derivative limits must be finite and positive")
        durations = np.diff(self.times)
        deltas = np.abs(np.diff(self.velocity))
        v = float(self.velocity.max())
        a = float(np.max(1.875*deltas/durations))
        j = float(np.max((10/math.sqrt(3))*deltas/durations**2))
        scale = max(1.0, m[0]*v/speed,
                    math.sqrt((m[1]*v*v + m[0]*a)/acceleration),
                    ((m[2]*v**3 + 3*m[1]*v*a + m[0]*j)/jerk)**(1/3))
        return PathTiming(self.path, self.progress, self.velocity/scale, self.times*scale)

    def at(self, seconds):
        return self.state(seconds)[0]

    def joint_state(self, seconds):
        u, v, a, j = self.state(seconds)
        q, q1, q2, q3 = self.path.state(u)
        return q, q1 * v, q2 * v**2 + q1 * a, q3 * v**3 + 3 * q2 * v * a + q1 * j


def time_joint_path(
    path,
    distance,
    angle,
    *,
    joint_speed,
    linear_speed,
    angular_speed,
    linear_acceleration,
    angular_acceleration,
    joint_acceleration,
    linear_jerk=0.8,
    angular_jerk=1.6,
    joint_jerk=8.0,
    slow_distance=0.0,
    slow_speed=None,
    minimum_duration=1.0,
    slow_fraction=0.0,
    slow_joint_speed=None,
):
    """C2 joint motion with bounded speed, acceleration and piecewise jerk.

    Smooth velocity transitions finish before the terminal slow zone. Global
    spline derivative bounds include path curvature; uniform time dilation
    conservatively enforces joint limits without changing the geometric path.
    These are continuous reference bounds, not bounds on contact-induced motion
    or on an emergency tracking/force stop.
    """
    positive = (
        joint_speed,
        linear_speed,
        angular_speed,
        linear_acceleration,
        angular_acceleration,
        joint_acceleration,
        linear_jerk,
        angular_jerk,
        joint_jerk,
        minimum_duration,
    )
    if any(not math.isfinite(x) or x <= 0 for x in positive):
        raise ValueError("Motion timing limits must be finite and positive")
    if any(not math.isfinite(x) or x < 0 for x in (distance, angle, slow_distance)) or (
        slow_speed is not None and (not math.isfinite(slow_speed) or slow_speed <= 0)
    ):
        raise ValueError("Invalid motion path or terminal zone")
    if (
        not math.isfinite(slow_fraction)
        or not 0 <= slow_fraction <= 1
        or (
            slow_joint_speed is not None
            and (not math.isfinite(slow_joint_speed) or slow_joint_speed <= 0)
        )
        or (slow_fraction > 0 and slow_joint_speed is None)
        or (slow_fraction > 0 and slow_distance > 0)
    ):
        raise ValueError("Invalid joint terminal zone")
    curve = JointPath.from_waypoints(path)
    m1, m2, m3 = curve.derivative_bounds()
    slope = max(float(m1.max()), 1e-12)
    cap = joint_speed / slope
    acceleration = joint_acceleration / slope
    jerk = joint_jerk / slope
    if distance > 1e-12:
        cap = min(cap, linear_speed / distance)
        acceleration = min(acceleration, linear_acceleration / distance)
        jerk = min(jerk, linear_jerk / distance)
    if angle > 1e-12:
        cap = min(cap, angular_speed / angle)
        acceleration = min(acceleration, angular_acceleration / angle)
        jerk = min(jerk, angular_jerk / angle)

    def transition(v0, v1):
        delta = abs(v1 - v0)
        return max(
            1.875 * delta / acceleration, math.sqrt((10 / math.sqrt(3)) * delta / jerk)
        )

    def transition_distance(v0, v1):
        return (v0 + v1) * transition(v0, v1) / 2

    def reachable(length, limit):
        low, high = 0.0, limit
        for _ in range(64):
            mid = (low + high) / 2
            if transition_distance(0, mid) <= length:
                low = mid
            else:
                high = mid
        return low

    progress, velocity, times = [0.0], [0.0], [0.0]

    def append(v1, duration):
        if duration <= 1e-15:
            return
        progress.append(progress[-1] + duration * (velocity[-1] + v1) / 2)
        velocity.append(v1)
        times.append(times[-1] + duration)

    def zone(length, end_speed, limit):
        start_speed = velocity[-1]
        low, high = max(start_speed, end_speed), limit

        def travel(peak):
            return transition_distance(start_speed, peak) + transition_distance(
                peak, end_speed
            )

        for _ in range(64):
            peak = (low + high) / 2
            if travel(peak) <= length:
                low = peak
            else:
                high = peak
        peak = low
        append(peak, transition(start_speed, peak))
        append(peak, max(0.0, length - travel(peak)) / peak)
        append(end_speed, transition(peak, end_speed))

    terminal = (
        min(1.0, slow_distance / distance)
        if distance > 1e-12 and slow_speed is not None
        else 0.0
    )
    slow_cap = min(cap, slow_speed / distance) if terminal > 0 else cap
    if slow_fraction > 0:
        terminal = slow_fraction
        slow_cap = min(cap, slow_joint_speed / slope)
    if terminal >= 1 - 1e-12:
        zone(1.0, 0.0, slow_cap)
    elif terminal > 1e-12:
        boundary_speed = min(
            reachable(1 - terminal, slow_cap), reachable(terminal, slow_cap)
        )
        zone(1 - terminal, boundary_speed, cap)
        zone(terminal, 0.0, slow_cap)
    else:
        zone(1.0, 0.0, cap)
    progress, velocity, times = map(np.asarray, (progress, velocity, times))
    # Exact smoothstep extrema and analytic spatial derivative bounds. Joint
    # acceleration/jerk include curvature, not just the scalar time law.
    durations = np.diff(times)
    deltas = np.abs(np.diff(velocity))
    vmax = float(velocity.max())
    amax = float(np.max(1.875 * deltas / durations))
    jmax = float(np.max((10 / math.sqrt(3)) * deltas / durations**2))
    scale = max(
        1.0,
        minimum_duration / times[-1],
        float(np.max(m1 * vmax)) / joint_speed,
        math.sqrt(float(np.max(m2 * vmax**2 + m1 * amax)) / joint_acceleration),
        (float(np.max(m3 * vmax**3 + 3 * m2 * vmax * amax + m1 * jmax)) / joint_jerk)
        ** (1 / 3),
    )
    progress[-1] = 1.0
    return PathTiming(curve, progress, velocity / scale, times * scale)
