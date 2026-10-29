"""Register SARM plugins in the platform runtime, without importing Python MuJoCo."""
from pathlib import Path
import ctypes
import hashlib
import json
import sys

_LOADED = {}
DELIVERED_SOURCE = 'model/SARM_mjcf_20261012/thread_sdf.cpp'
PLATFORM_SOURCE = 'native/sarm_thread_sdf.cpp'

def ensure_model_plugins(model_path: Path):
    import xml.etree.ElementTree as ET
    root = ET.parse(model_path).getroot()
    if root.find('./extension/plugin[@plugin="sarm.sdf.two_start_thread"]') is None:
        return
    from simulation.local_mujoco_stepper import _basilisk_mujoco_runtime
    base = Path(__file__).resolve().parents[1]
    manifest = base / 'run/native_acceleration/sarm_thread.json'
    if not manifest.is_file():
        raise FileNotFoundError('SARM thread plugin is missing; run tools/assets/build_sarm_plugin.py with the platform Python')
    record = json.loads(manifest.read_text(encoding='utf-8-sig'))
    return ensure_sarm_plugin(_basilisk_mujoco_runtime(), base / record['library'], manifest)

def validate_plugin_manifest(plugin_path: Path, manifest_path: Path, runtime_version: int) -> dict:
    plugin_path, manifest_path = Path(plugin_path), Path(manifest_path)
    if not plugin_path.is_file():
        raise FileNotFoundError(f'SARM plugin missing: {plugin_path}; build the native plugin first')
    record = json.loads(manifest_path.read_text(encoding='utf-8-sig'))
    if record['runtime_version'] != runtime_version:
        raise ValueError(f'SARM plugin runtime version mismatch: {record["runtime_version"]} != {runtime_version}')
    if hashlib.sha256(plugin_path.read_bytes()).hexdigest() != record['plugin_sha256']:
        raise ValueError(f'SARM plugin hash mismatch: {plugin_path}')
    return record

def ensure_sarm_plugin(runtime_path: Path, plugin_path: Path, manifest_path: Path) -> dict:
    runtime_path, plugin_path = Path(runtime_path).resolve(), Path(plugin_path).resolve()
    if sys.platform.startswith('linux'):
        from bsk_render_adapter.native_plugins import loaded_mujoco_library
        runtime = loaded_mujoco_library(global_symbols=True)
        if Path(runtime._name).resolve() != runtime_path:
            raise RuntimeError('Plugin runtime differs from the already loaded MuJoCo runtime')
    else:
        runtime = ctypes.CDLL(str(runtime_path))
    runtime.mj_version.restype = ctypes.c_int
    record = validate_plugin_manifest(plugin_path, manifest_path, runtime.mj_version())
    base = Path(__file__).resolve().parents[1]
    # The DLL is built from the platform source, which the build proved
    # equivalent to this exact delivered source; either changing needs a rebuild.
    for key, source in (('source_sha256', base / PLATFORM_SOURCE), ('delivered_source_sha256', base / DELIVERED_SOURCE)):
        if hashlib.sha256(source.read_bytes()).hexdigest() != record.get(key):
            raise ValueError(f'SARM plugin source hash changed ({source.name}); rebuild with tools/assets/build_sarm_plugin.py')
    key = (str(runtime_path), str(plugin_path))
    if key not in _LOADED:
        runtime.mj_loadPluginLibrary.argtypes = [ctypes.c_char_p]
        runtime.mj_loadPluginLibrary.restype = None
        runtime.mj_loadPluginLibrary(str(plugin_path).encode('utf-8'))
        runtime.mjp_getPlugin.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_int)]
        runtime.mjp_getPlugin.restype = ctypes.c_void_p
        slot = ctypes.c_int(-1)
        if not runtime.mjp_getPlugin(b'sarm.sdf.two_start_thread', ctypes.byref(slot)):
            raise RuntimeError(f'SARM plugin not registered in {runtime_path}')
        _LOADED[key] = runtime
    return record
