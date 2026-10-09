"""Coordinate and attitude conversion for the BSK render protocol."""

from __future__ import annotations

from typing import Iterable

import numpy as np


def vector(values: Iterable[float], size: int, name: str) -> np.ndarray:
    """Return a finite vector of the requested size."""

    result = np.asarray(list(values), dtype=np.float64)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain {size} finite values")
    return result


def mrp_to_dcm_b_from_n(sigma_bn: Iterable[float]) -> np.ndarray:
    """Return Basilisk's passive :math:`C_{BN}` for an MRP vector."""

    q1, q2, q3 = vector(sigma_bn, 3, "sigma_BN")
    d1 = q1 * q1 + q2 * q2 + q3 * q3
    s = 1.0 - d1
    denominator = (1.0 + d1) ** 2
    return np.array(
        [
            [4.0 * (2.0 * q1 * q1 - d1) + s * s, 8.0 * q1 * q2 + 4.0 * q3 * s, 8.0 * q1 * q3 - 4.0 * q2 * s],
            [8.0 * q2 * q1 - 4.0 * q3 * s, 4.0 * (2.0 * q2 * q2 - d1) + s * s, 8.0 * q2 * q3 + 4.0 * q1 * s],
            [8.0 * q3 * q1 + 4.0 * q2 * s, 8.0 * q3 * q2 - 4.0 * q1 * s, 4.0 * (2.0 * q3 * q3 - d1) + s * s],
        ],
        dtype=np.float64,
    ) / denominator


_ROTATION_IDENTITY = np.eye(3)
_ROTATION_DIAGONAL_TOLERANCE = 1e-9 + 1e-5
_ROTATION_GRAM_TOLERANCE = 1e-9 + 1e-5 * _ROTATION_IDENTITY
_ROTATION_IDENTITY.flags.writeable = False
_ROTATION_GRAM_TOLERANCE.flags.writeable = False


def dcm_to_quaternion_wxyz(matrix: Iterable[Iterable[float]]) -> np.ndarray:
    """Convert a proper active rotation matrix to a normalized quaternion."""

    r = np.asarray(matrix, dtype=np.float64)
    if r.shape != (3, 3) or not np.all(np.isfinite(r)):
        raise ValueError("rotation matrix must be 3x3 and finite")
    # Fixed-shape equivalent of allclose(..., atol=1e-9, rtol=1e-5).
    # Keep BOTH orthogonality and determinant checks; avoid the general-purpose
    # broadcasting/isclose setup for every rendered body's 3x3 matrix.
    if (not np.all(np.abs(r @ r.T - _ROTATION_IDENTITY) <= _ROTATION_GRAM_TOLERANCE)
            or not abs(float(np.linalg.det(r)) - 1.0) <= _ROTATION_DIAGONAL_TOLERANCE):
        raise ValueError("rotation matrix must be proper orthonormal")
    trace = float(np.trace(r))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        q = np.array([0.25 * scale, (r[2, 1] - r[1, 2]) / scale, (r[0, 2] - r[2, 0]) / scale, (r[1, 0] - r[0, 1]) / scale])
    else:
        axis = int(np.argmax(np.diag(r)))
        if axis == 0:
            scale = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
            q = np.array([(r[2, 1] - r[1, 2]) / scale, 0.25 * scale, (r[0, 1] + r[1, 0]) / scale, (r[0, 2] + r[2, 0]) / scale])
        elif axis == 1:
            scale = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
            q = np.array([(r[0, 2] - r[2, 0]) / scale, (r[0, 1] + r[1, 0]) / scale, 0.25 * scale, (r[1, 2] + r[2, 1]) / scale])
        else:
            scale = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
            q = np.array([(r[1, 0] - r[0, 1]) / scale, (r[0, 2] + r[2, 0]) / scale, (r[1, 2] + r[2, 1]) / scale, 0.25 * scale])
    q /= np.linalg.norm(q)
    if q[0] < 0.0:
        q *= -1.0
    return q


class FrameConverter:
    """Convert inertial BSK states into a floating-origin render frame."""

    def __init__(self, c_l_n: Iterable[Iterable[float]] | None = None) -> None:
        matrix = np.identity(3) if c_l_n is None else np.asarray(c_l_n, dtype=np.float64)
        if matrix.shape != (3, 3) or not np.allclose(matrix @ matrix.T, np.identity(3), atol=1e-9):
            raise ValueError("C_LN must be a 3x3 orthonormal matrix")
        if not np.isclose(np.linalg.det(matrix), 1.0, atol=1e-9):
            raise ValueError("C_LN must have determinant +1")
        self.c_l_n = matrix

    def position(self, position_n_m: Iterable[float], origin_n_m: Iterable[float]) -> np.ndarray:
        """Convert an inertial position to local floating-origin metres."""

        return self.c_l_n @ (vector(position_n_m, 3, "position_N_m") - vector(origin_n_m, 3, "origin_N_m"))

    def velocity(self, velocity_n_mps: Iterable[float]) -> np.ndarray:
        """Convert an inertial velocity to local components."""

        return self.c_l_n @ vector(velocity_n_mps, 3, "velocity_N_mps")

    def body_orientation(self, sigma_bn: Iterable[float]) -> np.ndarray:
        """Return the active body-to-local quaternion."""

        return dcm_to_quaternion_wxyz(self.c_l_n @ mrp_to_dcm_b_from_n(sigma_bn).T)

    def fixed_orientation(self, c_p_n: Iterable[Iterable[float]]) -> np.ndarray:
        """Return the active planet-fixed-to-local quaternion from passive N-to-P DCM."""

        return dcm_to_quaternion_wxyz(self.c_l_n @ np.asarray(c_p_n, dtype=np.float64).T)
