"""Opt-in offline MJScene thread-pool experiment, not enabled by live startup.

Windows x64 / Basilisk 2.11.1 / MuJoCo 3.7.0 only. SWIG omits the public native
getMujocoData/getMujocoModel accessors, so invoke their exported x64 ABI symbols.
No guessed structure offsets and no import of the Python mujoco package.
Header-compiled instrumentation is version checked before touching mjData.
"""
from __future__ import annotations

import ctypes
from pathlib import Path
import sys

from simulation.native_acceleration import load_native_library

KEYS = ("ncon", "nefc", "nisland", "nq", "nv", "ngeom", "arena_bytes",
        "stack_bytes", "stack_base", "threadpool_address")


class MJSceneThreadPoolProbe:
    def __init__(self, scene, workers: int):
        if isinstance(workers, bool) or not isinstance(workers, int) or not 0 <= workers <= 8:
            raise ValueError("experimental MJScene pool workers must be an integer in [0, 8]")
        if getattr(scene, "isDynamicsSynced", False):
            # Local dynamics backend: MJScene's own mjData is a publish-only,
            # contact-free copy that never runs dynamics. Refuse to probe it.
            raise RuntimeError("MJScene does not integrate under the local dynamics backend; "
                               "its internal mjData is not the authoritative dynamics state.")
        if sys.platform != "win32" or ctypes.sizeof(ctypes.c_void_p) != 8:
            raise RuntimeError("The experimental MJScene adapter requires Windows x64.")
        self._closed = False
        import Basilisk
        if Basilisk.__version__ != "2.11.1":
            raise RuntimeError("Experimental native accessor ABI is pinned to Basilisk 2.11.1.")
        python_mj = sys.modules.get("mujoco")
        if python_mj is not None and hasattr(python_mj, "MjModel"):
            raise RuntimeError("Do not load a second MuJoCo runtime into this process.")
        root = Path(Basilisk.__file__).resolve().parent
        # Basilisk already loaded these exact libraries before scene creation.
        self._mj = ctypes.CDLL(str(root / "mujoco.dll"))
        self._mj.mj_version.restype = ctypes.c_int
        self._mj.mj_versionString.restype = ctypes.c_char_p
        if self._mj.mj_version() != 3007000 or self._mj.mj_versionString() != b"3.7.0":
            raise RuntimeError("Experimental pool adapter requires Basilisk's MuJoCo 3.7.0.")
        self._probe = load_native_library("mjscene_probe")
        self._probe.probe_header_version.restype = ctypes.c_int
        self._probe.probe_abi.restype = ctypes.c_int
        if self._probe.probe_header_version() != 3007000 or self._probe.probe_abi() != 1:
            raise RuntimeError("Thread-pool probe was compiled against incompatible headers.")
        native = ctypes.CDLL(str(root / "mujocodynamicslib.dll"))
        data = getattr(native, "?getMujocoData@MJScene@@QEAAPEAUmjData_@@XZ")
        model = getattr(native, "?getMujocoModel@MJScene@@QEAAPEAUmjModel_@@XZ")
        for function in (data, model):
            function.argtypes, function.restype = [ctypes.c_void_p], ctypes.c_void_p
        self._native = native
        self._scene = scene  # Keep the native object alive until after detach/destroy.
        self._data, self._model = data(int(scene.this)), model(int(scene.this))
        if not self._data or not self._model:
            raise RuntimeError("MJScene must be initialized before binding a thread pool.")
        self._probe.probe_stats.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(ctypes.c_int64)]
        self._probe.probe_stats.restype = ctypes.c_int
        self._probe.probe_detach_finished.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
        self._probe.probe_detach_finished.restype = ctypes.c_int
        for name, args, result in (
            ("mju_threadPoolCreate", [ctypes.c_size_t], ctypes.c_void_p),
            ("mju_bindThreadPool", [ctypes.c_void_p, ctypes.c_void_p], None),
            ("mju_threadPoolNumberOfThreads", [ctypes.c_void_p], ctypes.c_size_t),
            ("mju_threadPoolDestroy", [ctypes.c_void_p], None),
        ):
            function = getattr(self._mj, name)
            function.argtypes, function.restype = args, result
        before = self.snapshot()
        if before["threadpool_address"] or before["stack_bytes"] or before["stack_base"]:
            raise RuntimeError("MJScene already has a pool or an active stack; refusing to rebind.")
        if not (0 < before["nq"] < 100000 and 0 < before["nv"] < 100000):
            raise RuntimeError("Unexpected MJScene model layout.")
        self.workers, self._pool = workers, None
        if workers:
            # Stack is sharded among workers + caller. Do not alter model memory,
            # solver, integrator, or contacts to make the thread experiment pass.
            if before["arena_bytes"] // (2 * (workers + 1)) < 65536:
                raise RuntimeError("Existing MuJoCo arena is too small for this pool experiment.")
            self._pool = self._mj.mju_threadPoolCreate(workers)
            if not self._pool:
                raise RuntimeError("MuJoCo failed to create the thread pool.")
            self._mj.mju_bindThreadPool(self._data, self._pool)
            if self.snapshot()["threadpool_address"] != self._pool:
                raise RuntimeError("Pool was not bound to the running MJScene data.")
            if self._mj.mju_threadPoolNumberOfThreads(self._pool) != workers:
                raise RuntimeError("MuJoCo thread count differs from the requested value.")
        self.initial = self.snapshot()

    def snapshot(self):
        if self._closed:
            raise RuntimeError("Cannot read an MJScene probe after close.")
        values = (ctypes.c_int64 * len(KEYS))()
        if self._probe.probe_stats(self._data, self._model, values):
            raise RuntimeError("MJScene probe failed.")
        return dict(zip(KEYS, values, strict=True))

    def close(self):
        """Only call after the benchmark is fully stopped; retain scene until then."""
        if self._closed:
            return
        if self._pool:
            if self._probe.probe_detach_finished(self._data, self._pool):
                raise RuntimeError("Cannot safely detach the experimental thread pool.")
            self._mj.mju_threadPoolDestroy(self._pool)
            self._pool = None
        self._closed = True
        self._scene = None
