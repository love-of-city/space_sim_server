"""Content key for one process-local, immutable compiled physics-model cache.

Only standalone local MJCFs and the known SARM SDF plugin are cacheable. Every
episode still gets its own model copy and fresh mjData. No poses, velocities,
forces, controllers or random generators are cached.
"""
import hashlib
import ctypes
from pathlib import Path
import xml.etree.ElementTree as ET


def configure_asset_cache() -> dict[str, int]:
    """Keep the current layout's decoded meshes across MJScene recompiles.

    MuJoCo's default 500 MiB cache is smaller than this platform's ~713 MiB
    working set, causing eviction during initialization itself. One GiB is a
    capacity, not a reservation; normal MuJoCo resource invalidation still applies.
    """
    from simulation.mujoco_runtime import basilisk_mujoco_runtime
    lib = ctypes.CDLL(str(basilisk_mujoco_runtime()))
    lib.mj_getCache.restype = ctypes.c_void_p
    lib.mj_setCacheCapacity.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
    lib.mj_setCacheCapacity.restype = ctypes.c_size_t
    for name in ('mj_getCacheSize', 'mj_getCacheCapacity'):
        function = getattr(lib, name)
        function.argtypes = [ctypes.c_void_p]
        function.restype = ctypes.c_size_t
    cache = lib.mj_getCache()
    capacity = lib.mj_getCacheCapacity(cache)
    if capacity < 1024 ** 3:
        capacity = lib.mj_setCacheCapacity(cache, 1024 ** 3)
    return dict(capacity_bytes=capacity, used_bytes=lib.mj_getCacheSize(cache))


def model_cache_key(model_path: Path, runtime_path: Path) -> str | None:
    path = Path(model_path).resolve()
    data = path.read_bytes()
    root = ET.fromstring(data)
    if root.find(".//include") is not None:
        return None
    if any(p.get("plugin") not in {None, "sarm.sdf.two_start_thread"}
           for p in root.iter("plugin")):
        return None
    digest = hashlib.sha256(b"compiled-physics-model/1\0" + data)
    compiler = root.find("compiler")
    directories = compiler.attrib if compiler is not None else {}
    if directories.get("strippath", "false") == "true":
        return None
    files = {Path(runtime_path).resolve()}
    if root.find("./extension/plugin") is not None:
        base = Path(__file__).resolve().parents[1]
        # ensure_model_plugins validates this manifest and its library first.
        files.add(base / "run/native_acceleration/sarm_thread.json")
    for element in root.iter():
        for key, value in element.attrib.items():
            if key not in {"file", "fileup", "filedown", "fileleft", "fileright", "filefront", "fileback"}:
                continue
            if "://" in value:
                return None
            directory = directories.get("assetdir", "")
            if element.tag in {"mesh", "flexcomp", "skin"}:
                directory = directories.get("meshdir", directory)
            elif element.tag == "texture":
                directory = directories.get("texturedir", directory)
            files.add((path.parent / directory / value).resolve())
    for asset in sorted(files):
        digest.update(str(asset).encode())
        digest.update(b"\0")
        # Content is authoritative: rapid same-size rewrites can share all stat
        # timestamps on some filesystems. Never let an edited asset hit stale physics.
        digest.update(hashlib.sha256(asset.read_bytes()).digest())
    return digest.hexdigest()
