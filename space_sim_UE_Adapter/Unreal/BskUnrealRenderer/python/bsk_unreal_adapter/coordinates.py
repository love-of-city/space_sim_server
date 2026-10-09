"""Small reference implementation of the UE half of the coordinate contract."""

from __future__ import annotations

from collections.abc import Iterable


def _values(items: Iterable[float], expected: int) -> list[float]:
    result = [float(value) for value in items]
    if len(result) != expected:
        raise ValueError(f"expected {expected} values")
    return result


def wire_position_to_unreal_cm(position_m: Iterable[float]) -> list[float]:
    """Map right-handed local meters to UE centimeters (X, -Y, Z)."""

    x, y, z = _values(position_m, 3)
    return [100.0 * x, -100.0 * y, 100.0 * z]


def wire_quat_wxyz_to_ue_xyzw(quaternion_wxyz: Iterable[float]) -> list[float]:
    """Map an active wire quaternion into UE's FQuat component order."""

    w, x, y, z = _values(quaternion_wxyz, 4)
    magnitude = (w * w + x * x + y * y + z * z) ** 0.5
    if magnitude <= 1e-15:
        raise ValueError("quaternion cannot be zero")
    return [-x / magnitude, y / magnitude, -z / magnitude, w / magnitude]
