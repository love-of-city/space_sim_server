"""Test candidate stationary capture poses, then freeze successful initial states.

This is input preparation, not dataset recording. Every candidate runs through
collection/run_scripted_task.py and retains geometry, force and failure evidence.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'backend')]
from collection.capture_initial_pool import source_scene


def sample_candidates(config):
    """Draw reproducible continuous poses, each requiring a full physical trial."""
    from simulation.scripted_tasks.capture_initialization import CaptureInitialState
    from dataclasses import asdict
    if config.get('schema') != 'capture-initial-sampling/1':
        raise ValueError('unsupported capture sampling configuration')
    count = config['sample_count']
    low, high = config['gap_range_m']
    bearing, tilt = config['bearing_cone_deg'], config['tilt_cone_deg']
    roll_low, roll_high = config['roll_range_deg']
    if (type(count) is not int or not 1 <= count <= 10000 or
            not .025 <= low <= high <= .80 or not 0 <= bearing <= 30 or
            not 0 <= tilt <= 30 or not -30 <= roll_low <= roll_high <= 30):
        raise ValueError('invalid capture sampling range')
    rng = np.random.Generator(np.random.PCG64(config['seed']))
    arm_bounds = config.get('arm_joint_offset_range_deg')
    arm_baseline = config.get('arm_joint_baseline_rad')
    if (arm_bounds is None) != (arm_baseline is None):
        raise ValueError('capture arm randomization needs baseline and six ranges')
    if arm_bounds is not None:
        arm_bounds, arm_baseline = np.asarray(arm_bounds), np.asarray(arm_baseline)
        if (arm_bounds.shape != (6,2) or arm_baseline.shape != (6,)
                or not np.isfinite(arm_bounds).all() or not np.isfinite(arm_baseline).all()
                or np.any(arm_bounds[:,0] > arm_bounds[:,1])):
            raise ValueError('invalid capture arm randomization bounds')
        arm_rng = np.random.Generator(np.random.PCG64(int(config['seed']) ^ 0x43415054))
    states = []
    for _ in range(count):
        theta = np.arccos(rng.uniform(np.cos(np.deg2rad(bearing)), 1.))
        phi = rng.uniform(-np.pi, np.pi)
        bx, by = np.rad2deg(np.arctan(np.tan(theta)*np.array([np.cos(phi), np.sin(phi)])))
        theta = np.arccos(rng.uniform(np.cos(np.deg2rad(tilt)), 1.))
        phi = rng.uniform(-np.pi, np.pi)
        ty = np.arcsin(np.sin(theta)*np.cos(phi))
        tx = np.arctan2(-np.sin(theta)*np.sin(phi), np.cos(theta))
        states.append(asdict(CaptureInitialState(gap_m=float(rng.uniform(low, high)),
            bearing_x_deg=float(bx), bearing_y_deg=float(by),
            tilt_x_deg=float(np.rad2deg(tx)), tilt_y_deg=float(np.rad2deg(ty)),
            roll_deg=float(rng.uniform(roll_low, roll_high)),
            arm_joint_position_rad=(None if arm_bounds is None else tuple(arm_baseline +
                np.deg2rad(arm_rng.uniform(arm_bounds[:,0],arm_bounds[:,1])))))))
    return states


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    inputs=parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument('--candidates',type=Path,help='JSON list of diagnostic initial states')
    inputs.add_argument('--sampling-config',type=Path,help='Continuous random sampling configuration')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--catalog',type=Path,required=True)
    parser.add_argument('--model',type=Path,default=ROOT/'model/SARM/platform/sarm_dual_capture26.xml')
    parser.add_argument('--workers',type=int,default=2,choices=(1,2,3,4))
    parser.add_argument('--trial-wall-seconds',type=float,default=7200.,
                        help='Infrastructure deadline, separate from the simulation task timeout')
    args=parser.parse_args()
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    if args.sampling_config:
        config=json.loads(args.sampling_config.read_text())
        (output/'sampling-config.json').write_text(json.dumps(config,indent=2))
        candidates=sample_candidates(config)
    else:
        candidates=json.loads(args.candidates.read_text())
    (output/'candidates.json').write_text(json.dumps(candidates,indent=2))
    progress=dict(schema='capture-feasibility/1',status='running',total=len(candidates),trials={})
    def save():
        temporary=output/'progress.tmp';temporary.write_text(json.dumps(progress,indent=2));temporary.replace(output/'progress.json')
    save()
    def trial(index,state):
        folder=output/f'trial-{index:03d}';folder.mkdir()
        scene=source_scene(index);scene['capture_initial_state'].update(state)
        from dataclasses import asdict
        from simulation.scripted_tasks.capture_initialization import CaptureInitialState
        scene['capture_initial_state']=asdict(CaptureInitialState(**scene['capture_initial_state']))
        (folder/'scene.json').write_text(json.dumps(scene,indent=2))
        command=[sys.executable,str(ROOT/'collection/run_scripted_task.py'),'--task','capture_target_grasp',
            '--model',str(args.model.resolve()),
            '--scene',str(folder/'scene.json'),'--output',str(folder/'dynamics'),'--max-sim-seconds','1200','--substeps','1']
        start=time.monotonic()
        with (folder/'simulation.log').open('w') as log:
            try:
                result=subprocess.run(command,cwd=ROOT,env={**os.environ,'OMP_NUM_THREADS':'1','OPENBLAS_NUM_THREADS':'1',
                    'PYTHONFAULTHANDLER':'1'},stdout=log,stderr=subprocess.STDOUT,timeout=args.trial_wall_seconds)
                code=result.returncode
            except subprocess.TimeoutExpired:code=124
        path=folder/'dynamics/summary.json'
        summary=json.loads(path.read_text()) if path.exists() else {}
        success=code==0 and summary.get('result')=='success' and summary.get('preflight',{}).get('accepted') is True
        return index,dict(state=scene['capture_initial_state'],accepted=success,returncode=code,
            result=summary.get('result'),reason=summary.get('reason'),physical_report=str(path),
            wall_seconds=time.monotonic()-start)
    with ThreadPoolExecutor(max_workers=args.workers) as workers:
        for future in as_completed([workers.submit(trial,i,state) for i,state in enumerate(candidates)]):
            index,result=future.result();progress['trials'][str(index)]=result;save()
            print(json.dumps(dict(index=index,**result)),flush=True)
    accepted=[]
    for key,result in sorted(progress['trials'].items(),key=lambda item:int(item[0])):
        if result['accepted']:
            path=Path(result['physical_report'])
            accepted.append(dict(id=f'capture-{int(key):03d}',initial_state=result['state'],physical_report=str(path),
                process_returncode=result['returncode'],
                report_sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    pool=dict(schema='capture-initial-pool/1',model=str(args.model.resolve()),
        catalog=str(args.catalog.resolve()),layouts=accepted,
        scope='discrete physically tested initial states; no certification between sampled points')
    (output/'pool.json').write_text(json.dumps(pool,indent=2))
    progress.update(status='complete',accepted=len(accepted));save()
    return 0 if accepted else 1


if __name__=='__main__':raise SystemExit(main())
