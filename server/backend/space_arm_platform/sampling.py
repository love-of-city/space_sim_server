"""Shared integer-time contract for dynamics, IK and RGB dataset sampling.

All timestamps are rounded absolute tick times, never accumulated rounded periods.
The renderer's BskLeRobotSampling.h must use the same dataset grid.
"""
from __future__ import annotations

import math

NANOSECONDS_PER_SECOND = 1_000_000_000
DYNAMICS_HZ = 400
DEFAULT_IK_HZ = 100
RENDER_HZ = 50
DEFAULT_CAPTURE_HZ = 50
WRENCH_HZ = 200
SCRIPTED_TASK_HZ = 200
SUPPORTED_FPS = (1, 2, 5, 10, 25, 30, 50)
# 30 Hz remains readable for historical episodes, but is not a live 50 Hz divisor.
LIVE_CAPTURE_FPS = tuple(fps for fps in SUPPORTED_FPS if RENDER_HZ % fps == 0)


def tick_time_ns(index: int, rate_hz: int) -> int:
    """Nearest integer nanosecond to index / rate_hz (ties round up)."""
    if index < 0 or rate_hz <= 0:
        raise ValueError("tick index must be nonnegative and rate positive")
    return (index * NANOSECONDS_PER_SECOND + rate_hz // 2) // rate_hz


def capture_preroll_ns(seconds: float) -> int:
    """End pre-roll on an observable frame, at or after the requested time."""
    if not math.isfinite(seconds) or seconds < 0:
        raise ValueError("capture pre-roll must be finite and nonnegative")
    minimum_ns = round(seconds * NANOSECONDS_PER_SECOND)
    tick = (minimum_ns * RENDER_HZ + NANOSECONDS_PER_SECOND - 1) // NANOSECONDS_PER_SECOND
    return tick_time_ns(tick, RENDER_HZ)


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
