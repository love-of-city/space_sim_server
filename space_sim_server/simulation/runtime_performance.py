"""Low-overhead wall-clock diagnostics; never alter the simulation time grid."""
from __future__ import annotations

import math
import time


class RuntimePerformanceMonitor:
    """Report compute/transport time separately from pacing and video FPS.

    The execute duration includes native dynamics, Python control, and render
    snapshot publication (including publisher backpressure). It is deliberately
    not labelled pure physics CPU time. Windows use wall time so a stalled
    transport is observable even when very little simulation time advances.
    """

    def __init__(self, target_rate: float, *, wall_start: float, interval_s: float = 5.0):
        if not math.isfinite(target_rate) or target_rate <= 0:
            raise ValueError("target_rate must be positive and finite")
        if not math.isfinite(interval_s) or interval_s <= 0:
            raise ValueError("interval_s must be positive and finite")
        self.target_rate = target_rate
        self.wall_start = self.last_wall = wall_start
        self.interval_s = interval_s
        self.last_sim_ns = 0
        self.execute_s = self.send_s = 0.0
        self.frames = 0
        self.max_execute_s = 0.0

    def record(self, sim_time_ns: int, execute_s: float, send_s: float, *, now: float | None = None):
        self.execute_s += execute_s
        self.send_s += send_s
        self.frames += 1
        self.max_execute_s = max(self.max_execute_s, execute_s)
        now = time.monotonic() if now is None else now
        wall_s = now - self.last_wall
        if wall_s < self.interval_s:
            return None
        simulated_s = (sim_time_ns - self.last_sim_ns) * 1e-9
        processing_s = self.execute_s + self.send_s
        result = {
            "type": "runtime_performance",
            "sim_time_ns": str(sim_time_ns),
            "target_simulation_rate": self.target_rate,
            "window_wall_s": wall_s,
            "window_simulated_s": simulated_s,
            "wall_real_time_factor": simulated_s / wall_s,
            "processing_real_time_factor": simulated_s / processing_s if processing_s > 0 else None,
            "simulation_execute_s": self.execute_s,
            "observation_send_s": self.send_s,
            "max_execute_frame_ms": self.max_execute_s * 1000,
            "outer_frames": self.frames,
            "schedule_lag_s": max(0.0, now - self.wall_start - sim_time_ns * 1e-9 / self.target_rate),
        }
        self.last_wall, self.last_sim_ns = now, sim_time_ns
        self.execute_s = self.send_s = self.max_execute_s = 0.0
        self.frames = 0
        return result


def render_transport_status(publisher) -> dict:
    """Read optional counters, without consuming or modifying frame queues."""
    stats = getattr(publisher, "stats", None)
    if stats is None:
        return {}
    queued, sent, dropped = stats.frames_queued, stats.frames_sent, stats.frames_dropped
    return {
        "render_frames_queued": queued,
        "render_frames_sent": sent,
        "render_frames_dropped": dropped,
        "render_backlog_frames": max(0, queued - sent - dropped),
        "render_transport_error": stats.last_error,
    }
