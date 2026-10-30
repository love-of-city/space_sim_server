"""C2 joint interpolation and analytic bounds, using only NumPy.

The not-a-knot cubic interpolant preserves every IK waypoint. Position limits
must be checked on the interpolant too, since interpolation can overshoot.
"""

from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class JointPath:
    coefficients: np.ndarray  # constant, linear, quadratic, cubic; interval; joint
    width: float

    @classmethod
    def from_waypoints(cls, waypoints):
        q = np.asarray(waypoints, dtype=float)
        if q.ndim != 2 or len(q) < 2 or q.shape[1] == 0 or not np.isfinite(q).all():
            raise ValueError("Joint path requires at least two finite joint vectors")
        n = len(q)
        h = 1.0 / (n - 1)
        if n == 2:
            second = np.zeros_like(q)
        elif n == 3:
            second = np.broadcast_to((q[2] - 2 * q[1] + q[0]) / h**2, q.shape)
        else:
            matrix = np.zeros((n, n))
            matrix[0, :3] = (1, -2, 1)
            matrix[-1, -3:] = (1, -2, 1)
            for i in range(1, n - 1):
                matrix[i, i - 1 : i + 2] = (1, 4, 1)
            rhs = np.zeros_like(q)
            rhs[1:-1] = 6 * np.diff(q, n=2, axis=0) / h**2
            second = np.linalg.solve(matrix, rhs)
        coefficients = np.array(
            [
                q[:-1],
                np.diff(q, axis=0) / h - h * (2 * second[:-1] + second[1:]) / 6,
                second[:-1] / 2,
                np.diff(second, axis=0) / (6 * h),
            ]
        )
        return cls(coefficients, h)

    def state(self, progress):
        u = float(np.clip(progress, 0, 1))
        i = min(int(u / self.width), self.coefficients.shape[1] - 1)
        x = u - i * self.width
        a, b, c, d = self.coefficients[:, i]
        return (
            a + x * (b + x * (c + x * d)),
            b + x * (2 * c + x * 3 * d),
            2 * c + 6 * d * x,
            6 * d,
        )

    def at(self, progress):
        return self.state(progress)[0]

    def derivative_bounds(self):
        """Per-joint absolute maxima of dq/du, d²q/du² and d³q/du³."""
        _, b, c, d = self.coefficients
        h = self.width
        first = np.maximum(np.abs(b), np.abs(b + h * (2 * c + 3 * d * h)))
        roots = np.divide(-c, 3 * d, out=np.zeros_like(c), where=d != 0)
        inside = (roots > 0) & (roots < h)
        first = np.maximum(
            first, np.where(inside, np.abs(b + roots * (2 * c + 3 * d * roots)), 0)
        )
        second = np.maximum(np.abs(2 * c), np.abs(2 * c + 6 * d * h))
        return np.array(
            [first.max(axis=0), second.max(axis=0), np.abs(6 * d).max(axis=0)]
        )

    def position_bounds(self):
        a, b, c, d = self.coefficients
        h = self.width
        end = a + h * (b + h * (c + h * d))
        low = np.minimum(a, end)
        high = np.maximum(a, end)
        # Solve q'(u)=0 inside each interval; waypoint-only checks are insufficient.
        for i in range(a.shape[0]):
            for j in range(a.shape[1]):
                roots = np.polynomial.polynomial.polyroots(
                    [b[i, j], 2 * c[i, j], 3 * d[i, j]]
                )
                for root in roots:
                    if np.isreal(root) and 0 < root.real < h:
                        x = root.real
                        value = a[i, j] + x * (b[i, j] + x * (c[i, j] + x * d[i, j]))
                        low[i, j] = min(low[i, j], value)
                        high[i, j] = max(high[i, j], value)
        return low.min(axis=0), high.max(axis=0)

    def validate_limits(self, lower, upper):
        low, high = self.position_bounds()
        if np.any(low < np.asarray(lower) - 1e-10) or np.any(
            high > np.asarray(upper) + 1e-10
        ):
            raise ValueError("Smooth joint path exceeds a joint position limit")
