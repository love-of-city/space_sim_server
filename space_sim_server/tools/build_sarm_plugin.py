"""Build the SARM SDF plugin for the platform MuJoCo ABI (Windows/MSVC)."""
from pathlib import Path
import ctypes, hashlib, json, subprocess, sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from build_native_acceleration import compiler_environment, VENDORED_MUJOCO_INCLUDE, BUILD_FLAGS
ROOT=Path(__file__).resolve().parents[1]

def build():
    import Basilisk
    runtime=Path(Basilisk.__file__).parent/'mujoco.dll'
    lib=ctypes.CDLL(str(runtime));lib.mj_version.restype=ctypes.c_int
    version=lib.mj_version()
    if version != 3007000: raise RuntimeError(f'Vendored headers require MuJoCo 3.7.0; actual version {version}')
    source=ROOT/'model/SARM_mjcf_20261012/thread_sdf.cpp'
    sha=hashlib.sha256(source.read_bytes()).hexdigest()
    out=ROOT/'run/native_acceleration'/('sarm_thread_'+sha[:16]);out.mkdir(parents=True,exist_ok=True)
    manifest=ROOT/'run/native_acceleration/sarm_thread.json'
    if manifest.is_file():
        cached=json.loads(manifest.read_text())
        dll=ROOT/cached['library']
        if cached.get('source_sha256')==sha and cached.get('runtime_version')==version and dll.is_file() and hashlib.sha256(dll.read_bytes()).hexdigest()==cached.get('plugin_sha256'):
            print(json.dumps(cached)); return
    env=compiler_environment();bin=Path(env['VCToolsInstallDir'])/'bin/Hostx64/x64'
    definition=out/'mujoco.def'
    definition.write_text('LIBRARY mujoco.dll\nEXPORTS\nmj_getPluginConfig\nmjp_defaultPlugin\nmjp_registerPlugin\n',encoding='ascii')
    subprocess.run([str(bin/'lib.exe'),'/nologo',f'/def:{definition}',f'/out:{out/"mujoco.lib"}','/machine:x64'],env=env,cwd=out,check=True)
    dll=out/'sarm_thread.dll'
    subprocess.run([str(bin/'cl.exe'),*BUILD_FLAGS,str(source),f'/I{VENDORED_MUJOCO_INCLUDE}',f'/Fo:{out/"sarm_thread.obj"}',f'/Fe:{dll}','/link',str(out/'mujoco.lib')],env=env,cwd=out,check=True)
    record=dict(runtime_version=version,source_sha256=sha,plugin_sha256=hashlib.sha256(dll.read_bytes()).hexdigest(),library=str(dll.relative_to(ROOT)))
    manifest=ROOT/'run/native_acceleration/sarm_thread.json';manifest.write_text(json.dumps(record,indent=2))
    print(json.dumps(record))
if __name__=='__main__':build()
