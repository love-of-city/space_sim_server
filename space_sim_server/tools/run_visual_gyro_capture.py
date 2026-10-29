"""One portable native UE-image/gyro/force capture qualification; Windows/UE5.6."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'backend'),str(ROOT)]
from space_arm_platform.capture_regression import prepare_trial,temporary_vision_runtime,physical_fingerprint
from space_arm_platform.capture_regression_runtime import ManagedProcesses,ensure_ports_free,wait_ready,prepare_native_assets,trial_environment


def last_row(path):
    if not path.exists():return None
    with path.open('rb') as f:
        f.seek(0,2);f.seek(max(0,f.tell()-65536));lines=f.read().splitlines()
    for line in reversed(lines):
        try:return json.loads(line)
        except (ValueError,UnicodeError):pass
    return None


def write(path,data):path.write_text(json.dumps(data,indent=2,ensure_ascii=False),encoding='utf-8')


def monitor(process,prepared,managed):
    start=time.monotonic();hold=None;previous=None
    path=prepared.output/'capture/visual_control.jsonl'
    while time.monotonic()-start<prepared.recipe['max_wall_s']:
        managed.observe()
        row=last_row(path)
        if row:
            t=row['t'];status=row['status']
            if 'abort' in status:raise RuntimeError(f'capture aborted: {status} at {t}s')
            if status=='predictive_hold_force':
                if hold is None:hold=t
            else:hold=None
            progress=dict(stage=status,t=t,hold_s=t-hold if hold is not None else 0,
                blind_path_mm=(row.get('prediction') or {}).get('blind_path_mm'),wall_s=time.monotonic()-start)
            write(prepared.output/'progress.json',progress)
            if status!=previous:print(json.dumps(progress),flush=True);previous=status
            if hold is not None and t-hold>=prepared.recipe['stop_after_hold_s']:return
            if t>=prepared.recipe['max_sim_s']:raise RuntimeError('simulation time budget exhausted before stable hold')
        if process.poll() is not None:raise RuntimeError(f'simulation exited before hold; code={process.returncode}')
        time.sleep(1)
    raise RuntimeError('wall time budget exhausted before stable hold')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True,help='New directory; existing results are never overwritten')
    parser.add_argument('--vision-python',type=Path,required=True,help='Separate OpenCV + MuJoCo3.7 worker interpreter')
    parser.add_argument('--unreal-root',type=Path,required=True,help='UE5.6 directory containing Engine')
    parser.add_argument('--powershell',type=Path,default=Path(shutil.which('pwsh') or 'pwsh'))
    parser.add_argument('--port-base',type=int,default=19100,help='Uses base API, +1 control, +2 capture, +3 rendezvous, +4 UE')
    args=parser.parse_args()
    adapter=ROOT.parent/'space_sim_UE_Adapter';unreal=args.unreal_root.resolve()
    vision=args.vision_python.resolve();python=Path(sys.executable).resolve()
    ports=list(range(args.port_base,args.port_base+5))
    ensure_ports_free(ports)
    env=trial_environment(ROOT,adapter,python,vision)
    # Explicitly prove all Python modules resolve to this checkout despite shared editable environments.
    probe='import pathlib,space_arm_platform,bsk_render_adapter,simulation.own_motion; print(space_arm_platform.__file__);print(bsk_render_adapter.__file__);print(simulation.own_motion.__file__)'
    paths=subprocess.check_output([str(python),'-c',probe],env=env,cwd=ROOT,text=True).splitlines()
    if not all(Path(p).resolve().is_relative_to(ROOT.parent) for p in paths):raise RuntimeError('mixed checkout Python imports')
    subprocess.run([str(vision),'-c','import cv2,mujoco;assert hasattr(cv2,"aruco") and mujoco.__version__=="3.7.0"'],check=True)
    import psutil
    for p in psutil.process_iter(['pid','cmdline']):
        command=' '.join(p.info['cmdline'] or [])
        if p.pid!=os.getpid() and any(x in command for x in ('space_arm_platform.main','teleop_grasp_unreal.py')):
            try:in_checkout=Path(p.cwd()).resolve()==ROOT or str(ROOT).lower() in command.lower()
            except (psutil.NoSuchProcess,psutil.AccessDenied):continue
            if in_checkout:raise RuntimeError(f'this checkout already has a backend/simulation PID{p.pid}; leave it intact and use a clean checkout')
    prepared=prepare_trial(args.output,vision,root=ROOT)
    output=prepared.output
    frozen=physical_fingerprint(ROOT);write(output/'physical-before.json',frozen)
    source_files=[p for base in ('simulation','backend/space_arm_platform','native','config/experiments/visual_gyro_capture')
        for p in (ROOT/base).rglob('*') if p.is_file() and p.suffix in ('.py','.cpp','.json')]
    write(output/'source-manifest.json',dict(commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        source_roots=paths,files={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in source_files}))
    managed=ManagedProcesses(output)
    error=None
    try:
        with temporary_vision_runtime(ROOT,prepared.vision,cleanup=lambda:managed.close()):
            write(output/'progress.json',dict(stage='preparing_native_assets'))
            prepare_native_assets(ROOT,adapter,python,args.powershell,unreal,vision,output,env)
            backend=managed.start('backend',[python,'-m','space_arm_platform.main','--port',ports[0],
                '--simulation-port',ports[1],'--capture-port',ports[2],'--rendezvous-port',ports[3],
                '--data-root',output/'episodes','--auth-database',output/'auth.sqlite3',
                '--no-access-log','--log-level','warning'],env=env,cwd=ROOT)
            wait_ready(backend,ports[0],api=True)
            project=adapter/'Unreal/BskUnrealRenderer'
            renderer=managed.start('renderer',[unreal/'Engine/Binaries/Win64/UnrealEditor.exe',
                project/'BskUnrealRenderer.uproject','-game','-RenderOffscreen','-unattended','-nosplash','-nosound',
                '-ResX=1280','-ResY=720',f'-BskPort={ports[4]}','-BskCaptureHost=127.0.0.1',
                f'-BskCapturePort={ports[2]}','-BskCaptureProducts=rgb','-BskOfflineCapture',
                '-DDC=InstalledDerivedDataBackendGraph',f'-LocalDataCachePath={os.environ.get("LOCALAPPDATA",str(output))}/BskDDC/capture-regression',
                f'-abslog={output}/ue.log'],env=env,cwd=ROOT)
            wait_ready(renderer,ports[4])
            simulation=managed.start('simulation',[python,ROOT/'simulation/teleop_grasp_unreal.py',
                '--adapter-root',adapter,'--model-root',ROOT/'model/SARM/platform',
                '--catalog',project/'Saved/AssetImport/sarm_dual_capture26.catalog.json',
                '--control-port',ports[1],'--render-port',ports[4],'--scene-instance',prepared.scene,
                '--dynamics-backend','local','--orbital-mode','bsk'],env=env,cwd=ROOT)
            monitor(simulation,prepared,managed)
            # Flush log writers and detach from the actual grasp before the counterfactual.
            managed.close();managed=ManagedProcesses(output)
            audit=ROOT/'run/capture_contact_audit.jsonl'
            shutil.copyfile(audit,output/'capture/contact_audit.jsonl')
            shutil.copyfile(ROOT/'run/wrist_vision_config.json',output/'calibration.json')
            write(output/'progress.json',dict(stage='independent_free_reference'))
            backend=managed.start('reference-backend',[python,'-m','space_arm_platform.main','--port',ports[0],
                '--simulation-port',ports[1],'--capture-port',ports[2],'--rendezvous-port',ports[3],
                '--data-root',output/'reference-episodes','--auth-database',output/'reference-auth.sqlite3',
                '--no-access-log','--log-level','warning'],env=env,cwd=ROOT)
            wait_ready(backend,ports[0],api=True)
            reference=managed.start('free-reference',[python,ROOT/'tools/capture_free_reference.py',
                '--scene',prepared.scene,'--output',output,'--adapter-root',adapter,'--control-port',ports[1],
                '--duration',prepared.recipe['max_sim_s']],env=env,cwd=ROOT)
            reference.wait(timeout=prepared.recipe['max_wall_s'])
            if reference.returncode:raise RuntimeError('free-flight reference failed; inspect free-reference.err.log')
            managed.close();managed=ManagedProcesses(output)
    except Exception as failure:
        error=str(failure)
    finally:
        managed.close()
        after=physical_fingerprint(ROOT);write(output/'physical-after.json',after)
        if after!=frozen:error=(error+'; ' if error else '')+'physical model changed'
    if error:
        write(output/'result.json',dict(success=False,error=error));raise RuntimeError(error)
    completed=subprocess.run([str(vision),str(ROOT/'tools/score_visual_gyro_capture.py'),
        '--output',str(output)],env=env,cwd=ROOT,check=False)
    if completed.returncode:raise RuntimeError(f'grasp qualification failed; inspect {output}/result.json')
    print(f'PASS: {output}/result.json',flush=True)


if __name__=='__main__':main()
