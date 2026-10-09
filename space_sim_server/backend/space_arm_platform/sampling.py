"""Shared integer-time contract for dynamics, IK and RGB dataset sampling.

All timestamps are rounded absolute tick times, never accumulated rounded periods.
The renderer's BskLeRobotSampling.h must use the same dataset grid.
"""
from __future__ import annotations

NANOSECONDS_PER_SECOND = 1_000_000_000
DYNAMICS_HZ = 240
DEFAULT_IK_HZ = 120
RENDER_HZ = 30
DEFAULT_CAPTURE_HZ = 30
SUPPORTED_FPS = (1, 2, 5, 10, 30)


def tick_time_ns(index: int, rate_hz: int) -> int:
    """Nearest integer nanosecond to index / rate_hz (ties round up)."""
    if index < 0 or rate_hz <= 0:
        raise ValueError("tick index must be nonnegative and rate positive")
    return (index * NANOSECONDS_PER_SECOND + rate_hz // 2) // rate_hz


def sample_tick(sim_time_ns: int, fps: int) -> int | None:
    """Accept only an exact rounded timestamp on the advertised dataset grid."""
    if sim_time_ns < 0 or fps not in SUPPORTED_FPS:
        return None
    tick = (sim_time_ns * fps + NANOSECONDS_PER_SECOND // 2) // NANOSECONDS_PER_SECOND
    return tick if sim_time_ns == tick_time_ns(tick, fps) else None


def ik_step_stride(rate_hz: float) -> int:
    """IK executes on physics steps, never on an independently rounded clock."""
    if not 0 < rate_hz <= DYNAMICS_HZ or rate_hz != int(rate_hz) or DYNAMICS_HZ % int(rate_hz):
        raise ValueError(f"IK rate must be an integer divisor of {DYNAMICS_HZ} Hz; default is {DEFAULT_IK_HZ} Hz")
    return DYNAMICS_HZ // int(rate_hz)
