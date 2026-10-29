"""Owned-process lifecycle and explicit preparation for the capture regression."""
import json
import os
from pathlib import Path
import socket
import subprocess
import time
import urllib.request
import psutil


def ensure_ports_free(ports):
    sockets=[]
    try:
        for port in ports:
            s=socket.socket();sockets.append(s)
            try:s.bind(('127.0.0.1',port))
            except OSError as error:raise RuntimeError(f'port {port} is occupied; no process was stopped') from error
    finally:
        for s in sockets:s.close()


class ManagedProcesses:
    def __init__(self,output):
        self.output=Path(output);self.jobs=[];self.owned={};self.files=[]

    def start(self,name,argv,*,env=None,cwd=None):
        stdout=(self.output/f'{name}.out.log').open('wb')
        stderr=(self.output/f'{name}.err.log').open('wb')
        self.files.extend([stdout,stderr])
        process=subprocess.Popen([str(x) for x in argv],cwd=cwd,env=env,stdout=stdout,stderr=stderr,
            creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        self.jobs.append(process)
        self.owned[process.pid]=psutil.Process(process.pid).create_time()
        self.observe()
        return process

    def observe(self):
        for job in self.jobs:
            try:
                parent=psutil.Process(job.pid)
                if parent.create_time()!=self.owned[job.pid]:continue
                for child in parent.children(recursive=True):self.owned[child.pid]=child.create_time()
            except psutil.NoSuchProcess:pass

    def close(self):
        self.observe();live=[]
        for pid,created in reversed(list(self.owned.items())):
            try:
                p=psutil.Process(pid)
                if p.create_time()==created:p.terminate();live.append(p)
            except psutil.NoSuchProcess:pass
        _,remaining=psutil.wait_procs(live,timeout=5)
        for p in remaining:
            try:
                if p.create_time()==self.owned[p.pid]:p.kill()
            except psutil.NoSuchProcess:pass
        for job in self.jobs:
            try:job.wait(timeout=10)
            except subprocess.TimeoutExpired:raise RuntimeError(f'owned process did not stop: {job.pid}')
        for file in self.files:file.close()


def wait_ready(process,port,*,api=False,timeout=240):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if process.poll() is not None:raise RuntimeError(f'process exited before port {port} became ready: {process.returncode}')
        try:
            if api:
                with urllib.request.urlopen(f'http://127.0.0.1:{port}/api/health',timeout=1) as reply:
                    if json.load(reply).get('ok'):return
            else:
                with socket.create_connection(('127.0.0.1',port),timeout=1):return
        except OSError:pass
        time.sleep(.25)
    raise RuntimeError(f'startup timed out at port {port}')


def prepare_native_assets(root,adapter,python,powershell,unreal,vision_python,output,env):
    """Use tracked UE assets as-is; only derive machine-local catalogs."""
    def run(name,argv):
        with (output/f'{name}.log').open('wb') as log:
            subprocess.run([str(x) for x in argv],cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,
                check=True,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    run('native-build',[python,root/'tools/build_native_acceleration.py','--component','all'])
    run('thread-build',[python,root/'tools/build_sarm_plugin.py'])
    run('ue-build',[powershell,'-NoProfile','-File',adapter/'Unreal/BskUnrealRenderer/scripts/build.ps1',
        '-UnrealRoot',unreal])
    project=adapter/'Unreal/BskUnrealRenderer'
    saved=project/'Saved/AssetImport';saved.mkdir(parents=True,exist_ok=True)
    profile=json.loads((root/'config/scene_render_profiles.json').read_text('utf-8'))['sarm-dual-capture26']
    manifest=json.loads((root/'model/SARM/platform/sarm_dual_capture26.manifest.json').read_text('utf-8'))
    plugin=json.loads((root/'run/native_acceleration/sarm_thread.json').read_text())['library']
    imports=[(manifest['base_model'],'Sarm20261012'),
        ('model/dual_arm_capture26/import_assets.xml','Capture26'),
        ('model/dual_arm_capture26/import_trimmed_base.xml','Capture26SideV1')]
    for index,((model,destination),catalog) in enumerate(zip(imports,profile['source_catalogs'],strict=True)):
        catalog_path=saved/catalog
        run(f'catalog-{index}',[python,project/'scripts/generate_mjcf_asset_catalog.py',
            '--mjcf',root/model,'--destination',f'/Game/BSK/Generated/{destination}',
            '--catalog',catalog_path,'--import-settings',saved/f'{index}.import.json',
            '--mesh-cache',saved/f'{index}_mesh_cache','--normal-mode','preserve',
            '--build-scale','100','--component-scale','1','--mujoco-plugin',root/plugin])
        catalog_data=json.loads(catalog_path.read_text('utf-8-sig'))
        assets=[v['asset_path'] for v in catalog_data['assets'].values()]
        assets+=list(catalog_data.get('textures',{}).values())
        for asset in assets:
            path=project/'Content'/(asset.split('.')[0].removeprefix('/Game/')+'.uasset')
            if not path.is_file() or path.stat().st_size<256:
                raise RuntimeError(f'tracked UE asset missing/LFS pointer: {path}; run git lfs pull')
    run('catalog-validation',[python,root/'tools/verify_capture26_import.py','--adapter-root',adapter])


def trial_environment(root,adapter,python,vision_python):
    env=os.environ.copy()
    env.update(PYTHONPATH=os.pathsep.join(map(str,[root/'backend',root,adapter/'Adapters'])),
        SPACE_SIM_PYTHON=str(python),SPACE_SIM_VISION_PYTHON=str(vision_python),
        SPACE_SIM_POSTURE_PYTHON=str(vision_python),SPACE_SIM_DYNAMICS_BACKEND='local',
        SPACE_SIM_ORBITAL_MODE='bsk',SPACE_SIM_LOCAL_SUBSTEPS='1',SPACE_SIM_IK_MODE='ik_pose',
        PYTHONUNBUFFERED='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
    return env
