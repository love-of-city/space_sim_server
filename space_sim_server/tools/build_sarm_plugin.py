"""Build the SARM SDF plugin for the platform MuJoCo ABI (Windows or Linux).

The plugin is compiled from native/sarm_thread_sdf.cpp, a faster platform copy of
the delivered model/SARM_mjcf_20261012/thread_sdf.cpp. Both sources are compiled
into native/sarm_thread_sdf_check.cpp first; the plugin is installed only if its
distances and gradients match the delivered code.
"""
from pathlib import Path
import ctypes, hashlib, json, shutil, subprocess, sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from build_native_acceleration import compiler_environment, VENDORED_MUJOCO_INCLUDE, BUILD_FLAGS, LINUX_BUILD_FLAGS
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from simulation.mujoco_runtime import basilisk_mujoco_runtime
DELIVERED=ROOT/'model/SARM_mjcf_20261012/thread_sdf.cpp'
SOURCE=ROOT/'native/sarm_thread_sdf.cpp'
CHECKER=ROOT/'native/sarm_thread_sdf_check.cpp'

def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()

def check_equivalence(env,bin,out):
    exe=out/('sarm_thread_sdf_check.exe' if sys.platform == 'win32' else 'sarm_thread_sdf_check')
    if sys.platform == 'win32':
        command=[str(bin/'cl.exe'),'/nologo','/O2','/fp:strict','/EHsc','/std:c++17',str(CHECKER),
                    f'/I{VENDORED_MUJOCO_INCLUDE}',f'/DDELIVERED_SOURCE="{DELIVERED.as_posix()}"',
                    f'/DPLATFORM_SOURCE="{SOURCE.as_posix()}"',f'/Fo:{out/"sarm_thread_sdf_check.obj"}',f'/Fe:{exe}']
    else:
        command=[str(bin),'-O2','-fno-fast-math','-ffp-contract=off','-std=c++17',str(CHECKER),
                 '-I',str(VENDORED_MUJOCO_INCLUDE),f'-DDELIVERED_SOURCE="{DELIVERED.as_posix()}"',
                 f'-DPLATFORM_SOURCE="{SOURCE.as_posix()}"','-o',str(exe)]
    subprocess.run(command,env=env,cwd=out,check=True)
    result=subprocess.run([str(exe)],cwd=out,capture_output=True,text=True)
    report=json.loads(result.stdout)
    if result.returncode!=0 or not report.get('pass'):
        raise RuntimeError(f'Platform thread SDF differs from the delivered source: {report}')
    return report

def build():
    runtime=basilisk_mujoco_runtime()
    lib=ctypes.CDLL(str(runtime));lib.mj_version.restype=ctypes.c_int
    version=lib.mj_version()
    if version != 3007000: raise RuntimeError(f'Vendored headers require MuJoCo 3.7.0; actual version {version}')
    sha,delivered_sha,checker_sha=digest(SOURCE),digest(DELIVERED),digest(CHECKER)
    out=ROOT/'run/native_acceleration'/('sarm_thread_'+sha[:16]);out.mkdir(parents=True,exist_ok=True)
    manifest=ROOT/'run/native_acceleration/sarm_thread.json'
    if manifest.is_file():
        cached=json.loads(manifest.read_text())
        dll=ROOT/cached['library']
        if (cached.get('source_sha256')==sha and cached.get('delivered_source_sha256')==delivered_sha
                and cached.get('checker_sha256')==checker_sha and cached.get('runtime_version')==version
                and cached.get('platform')==sys.platform and cached.get('runtime_path')==str(runtime)
                and dll.is_file() and digest(dll)==cached.get('plugin_sha256')):
            print(json.dumps(cached)); return
    env=compiler_environment()
    if sys.platform == 'win32':
        bin=Path(env['VCToolsInstallDir'])/'bin/Hostx64/x64'
    else:
        compiler=shutil.which(env.get('CXX','g++'))
        if not compiler:
            raise RuntimeError('Install g++/clang++ or set CXX to its executable path')
        bin=Path(compiler)
    equivalence=check_equivalence(env,bin,out)
    if sys.platform == 'win32':
        definition=out/'mujoco.def'
        definition.write_text('LIBRARY mujoco.dll\nEXPORTS\nmj_getPluginConfig\nmjp_defaultPlugin\nmjp_registerPlugin\nmjCOLLISIONFUNC DATA\n',encoding='ascii')
        subprocess.run([str(bin/'lib.exe'),'/nologo',f'/def:{definition}',f'/out:{out/"mujoco.lib"}','/machine:x64'],env=env,cwd=out,check=True)
        dll=out/'sarm_thread.dll'
        command=[str(bin/'cl.exe'),*BUILD_FLAGS,str(SOURCE),f'/I{VENDORED_MUJOCO_INCLUDE}',f'/Fo:{out/"sarm_thread.obj"}',f'/Fe:{dll}','/link',str(out/'mujoco.lib')]
    else:
        dll=out/'libsarm_thread.so'
        # Resolve MuJoCo symbols in the host at load time. A DT_NEEDED on
        # Basilisk's hashed SONAME would pull a second runtime into offline
        # Python MuJoCo asset tools. Both callers explicitly expose their one
        # already-loaded runtime before loading this plugin.
        command=[str(bin),*LINUX_BUILD_FLAGS,str(SOURCE),'-I',str(VENDORED_MUJOCO_INCLUDE),
                 '-o',str(dll)]
    subprocess.run(command,env=env,cwd=out,check=True)
    record=dict(runtime_version=version,source_sha256=sha,delivered_source_sha256=delivered_sha,checker_sha256=checker_sha,
                equivalence=equivalence,plugin_sha256=digest(dll),library=str(dll.relative_to(ROOT)),
                platform=sys.platform,runtime_path=str(runtime))
    manifest.write_text(json.dumps(record,indent=2))
    print(json.dumps(record))
if __name__=='__main__':build()
