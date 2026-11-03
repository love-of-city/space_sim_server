"""One private collision model per worker/layout, with fresh query state.

The live physics model is never borrowed. The worker executes episodes and
planning queries serially; only the most recent content-identified model stays
resident. Each task hashes its inputs once, and every borrow restores the model
initial state before the caller supplies the measured scene.
"""
import atexit
from pathlib import Path
import time

_cached = None


def clear_cache():
    global _cached
    if _cached is not None:
        _cached[1].close()
        _cached = None


atexit.register(clear_cache)


class ProbeLease:
    def __init__(self, probe):
        self._probe = probe

    def __getattr__(self, name):
        return getattr(self._probe, name)

    def close(self):
        # Lifetime belongs to the bounded worker cache, not the individual query.
        pass


class PlanningContext:
    def __init__(self, model):
        self.model = Path(model)
        self.key = None
        self.probe = None
        self.stats = dict(model_builds=0, model_reuses=0, queries=0,
                          fingerprint_seconds=0., model_load_seconds=0.)

    def acquire(self):
        global _cached
        if self.probe is None:
            from simulation.compiled_model_cache import configure_asset_cache, model_cache_key
            from simulation.mujoco_plugins import ensure_model_plugins
            from simulation.mujoco_runtime import basilisk_mujoco_runtime
            from .native_geometry_probe import NativePoseProbe
            started = time.perf_counter()
            ensure_model_plugins(self.model)
            configure_asset_cache()
            self.key = model_cache_key(self.model, basilisk_mujoco_runtime())
            self.stats['fingerprint_seconds'] = time.perf_counter() - started
            if self.key is None or _cached is None or _cached[0] != self.key:
                clear_cache()
                started = time.perf_counter()
                probe = NativePoseProbe(self.model, clearance_m=.002, cache_key=self.key)
                self.stats['model_load_seconds'] = time.perf_counter() - started
                self.stats['model_builds'] += 1
                _cached = (self.key, probe, probe.home.copy())
            else:
                self.stats['model_reuses'] += 1
            self.probe = _cached[1]
            self.initial_qpos = _cached[2].copy()
        self.stats['queries'] += 1
        self.probe.home[:] = self.initial_qpos
        self.probe.q[:] = self.initial_qpos
        self.probe.v.fill(0.)
        self.probe.passive_position_offsets.clear()
        return ProbeLease(self.probe)


def acquire_probe(task, *, clearance_m=.002):
    context = getattr(task, '_planning_context', None)
    if context is not None:
        return context.acquire()
    # Explicit diagnostics and small isolated tests own their original probe.
    from .native_geometry_probe import NativePoseProbe
    return NativePoseProbe(task.model_path, clearance_m=clearance_m)
