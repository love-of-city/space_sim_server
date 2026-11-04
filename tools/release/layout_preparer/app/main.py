"""Portable serial random-layout preparation. No environment installation needed."""
from pathlib import Path
import argparse
import datetime
import fcntl
import hashlib
import json
import math
import os
import signal
import subprocess
import sys
import time
import uuid

BASE = Path(__file__).resolve().parents[1]
ROOT = BASE/'repo/server'
ADAPTER = BASE/'repo/ue-adapter'
SIM = BASE/'runtime/simenv/bin/python'
GEO = BASE/'runtime/geometryenv/bin/python'
UE = BASE/'runtime/UE_5.6'

def atomic(path, value):
    temp = path.with_suffix(path.suffix+'.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
    temp.replace(path)

def fingerprints():
    files = [ROOT/'model/SARM/platform/sarm_20261012.xml']
    for folder in ('simulation', 'tools', 'backend/space_arm_platform'):
        files.extend(sorted((ROOT/folder).rglob('*.py')))
    files.extend(sorted((BASE/'app').glob('*.py')))
    return {str(p.relative_to(BASE)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}

class StageFailure(RuntimeError):
    pass

class Job:
    def __init__(self, args):
        self.args = args
        self.out = args.output.expanduser().resolve()
        self.out.mkdir(parents=True, exist_ok=True)
        self.lock = (self.out/'.job.lock').open('a')
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.pins = fingerprints()
        settings = dict(count=args.count, seed=args.seed, yaw_range_deg=args.yaw_range_deg,
                        bundle=str(BASE), output=str(self.out), source_sha256=self.pins)
        config = self.out/'settings.json'
        if config.exists():
            old = json.loads(config.read_text())
            if old != settings:
                raise ValueError('Resume requires unchanged count, seed, yaw, source and paths. Use a new output directory.')
        else:
            if args.command == 'certify':
                raise ValueError('Run prepare before certify.')
            if any(p.name != '.job.lock' for p in self.out.iterdir()):
                raise ValueError('New job requires an empty output directory.')
            atomic(config, settings)
        path = self.out/'progress.json'
        self.state = json.loads(path.read_text()) if path.exists() else dict(
            schema='layout-preparer/1', target=args.count, prepared=0, certified=0,
            failed=0, base_seed=args.seed, yaw_range_deg=args.yaw_range_deg,
            namespace=uuid.uuid4().hex[:16], pool_published=False)
        path = self.out/'prepared-assets.json'
        self.items = json.loads(path.read_text())['layouts'] if path.exists() else []
        self.state.update(status='running', pid=os.getpid(), last_error=None, parallel_workers=1)
        if args.resume:
            (self.out/'cancel.request').unlink(missing_ok=True)
        elif (self.out/'cancel.request').exists():
            raise ValueError('Job was stopped; pass --resume to continue.')
        self.save()

    def save(self):
        self.state['updated_at'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
        atomic(self.out/'prepared-assets.json', dict(schema='prepared-layout-assets/1',not_a_certified_pool=True,layouts=self.items))
        atomic(self.out/'progress.json', self.state)

    def run(self, command, folder, stage, allowed=(0,)):
        if fingerprints() != self.pins:
            raise RuntimeError('Source files changed during preparation.')
        if (self.out/'cancel.request').exists():
            raise KeyboardInterrupt('Stop requested')
        self.state.update(current_stage=stage, current_directory=str(folder))
        self.save()
        print(json.dumps(dict(stage=stage,directory=str(folder))), flush=True)
        start = time.monotonic()
        with (folder/(stage+'.log')).open('w') as log:
            proc = subprocess.Popen(list(map(str, command)), cwd=ROOT, stdout=log,
                                    stderr=subprocess.STDOUT, start_new_session=True)
            try:
                while proc.poll() is None:
                    if (self.out/'cancel.request').exists():
                        raise KeyboardInterrupt('Stop requested')
                    if time.monotonic()-start > self.args.stage_timeout:
                        raise TimeoutError(stage)
                    time.sleep(.5)
            except BaseException:
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    proc.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()
                raise
        with (self.out/'timings.jsonl').open('a') as f:
            f.write(json.dumps(dict(stage=stage, directory=str(folder),seconds=time.monotonic()-start,
                                   returncode=proc.returncode))+'\n')
        if proc.returncode not in allowed:
            raise StageFailure(f'{stage} exit={proc.returncode}; see {folder/(stage+".log")}')

    def prepare(self):
        from simulation.collection_assets import prepare_base_catalog
        from simulation.capture_layout import validated_capture_layout
        base = prepare_base_catalog(self.out/'base.catalog.json', adapter=ADAPTER)
        self.state['phase'] = 'assets'
        for index in range(self.args.count):
            seed = self.args.seed+index
            folder = self.out/f'layout-{index:03d}-seed-{seed}'
            model = folder/'model/layout.xml'
            native = folder/'native-geometry.json'
            old = next((item for item in self.items if item['id']==f'seed-{seed}'),None)
            if old:
                validated_capture_layout(Path(old['model']), Path(old['catalog']))
                continue
            folder.mkdir(exist_ok=True)
            marker = folder/'build.complete.json'
            if marker.exists():
                hashes = json.loads(marker.read_text())
                if any(not Path(p).is_file() or hashlib.sha256(Path(p).read_bytes()).hexdigest()!=sha for p,sha in hashes.items()):
                    raise ValueError('Generated geometry changed; use a new output directory.')
            else:
                # Preserve an interrupted generation, then rebuild in the intended location.
                if model.parent.exists():
                    model.parent.rename(folder/('model.interrupted-'+str(time.time_ns())))
                self.run([GEO,ROOT/'tools/assets/build_independent_task_box_layout.py','--source',ROOT/'model/SARM/platform/sarm_20261012.xml',
                          '--output',model,'--seed',seed,'--target-mounts','inset','--pickup-mounts','inset',
                          '--yaw-range-deg',self.args.yaw_range_deg],folder,'build')
                atomic(marker,{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in model.parent.rglob('*') if p.is_file()})
            for script,report,stage in [('check_independent_task_box_layout.py',native,'native'),
                                        ('check_inset_target_geometry.py',folder/'inset-geometry.json','geometry')]:
                cmd = [GEO,ROOT/'tools/diagnostics'/script,'--layout',model,'--report',report]
                if stage=='native':
                    cmd.extend(['--source',ROOT/'model/SARM/platform/sarm_20261012.xml'])
                self.run(cmd,folder,stage)
                if json.loads(report.read_text()).get('passed') is not True:
                    raise StageFailure(f'Geometry check rejected {report}')
            assets = folder/'ue-assets'
            if assets.exists():
                assets.rename(folder/('ue-assets.interrupted-'+str(time.time_ns())))
            destination = '/Game/BSK/Generated/Portable_'+self.state['namespace']+'_'+uuid.uuid4().hex[:12]
            self.run([SIM,ROOT/'tools/assets/prepare_independent_layout_catalog.py','--layout',model,'--base-catalog',base,
                      '--adapter',ADAPTER,'--unreal',UE,'--output',assets,'--destination',destination],folder,'ue')
            self.items.append(dict(id=f'seed-{seed}',model=str(model),catalog=str(assets/'layout.catalog.json'),
                                   native_report=str(native),trajectory_status='pending'))
            self.state['prepared']=len(self.items)
            self.save()

    def certify(self):
        from space_arm_platform.collection_layouts import LayoutPool, pool_entry
        from space_arm_platform.layout_trajectories import TASKS, INITIAL_JOINTS_DEG, bind_trajectory_acceptance
        from tools.assets.prepare_collection_layout_pool import validation_scene
        if not self.items:
            raise ValueError('No prepared assets to certify.')
        scene = self.out/'validation-scene.json'
        if scene.exists():
            from space_arm_platform.layout_trajectories import validate_scene
            validate_scene(scene)
        else:
            scene = validation_scene(None,self.out,self.args.seed)
        accepted = []
        self.state['phase']='trajectory_validation'
        for item in self.items:
            model=Path(item['model']);folder=model.parent.parent;native=Path(item['native_report'])
            try:
                if item.get('trajectory_status') != 'accepted':
                    trajectories={}
                    for task in TASKS:
                        trajectories[task]=folder/('trajectory-'+task)
                        if trajectories[task].exists():
                            trajectories[task].rename(folder/('trajectory-'+task+'.previous-'+str(time.time_ns())))
                        self.run([SIM,ROOT/'tools/diagnostics/validate_task_trajectory_contacts.py','--model',model,'--scene',scene,'--task',task,
                                  '--output',trajectories[task],'--native-geometry','--ignore-free-bolt',
                                  '--initial-joints-deg',*INITIAL_JOINTS_DEG],folder,'trajectory-'+task,allowed=(0,1))
                    bind_trajectory_acceptance(model,native,scene,trajectories,folder/'trajectory-acceptance.json')
                accepted.append(pool_entry(item['id'],model,Path(item['catalog']),native))
                item['trajectory_status']='accepted'
            except (StageFailure,ValueError,KeyError) as e:
                item.update(trajectory_status='rejected',reason=str(e))
                atomic(folder/'trajectory-rejection.json',dict(reason=str(e),pool_published=False))
            self.state['certified']=len(accepted)
            self.save()
        if len(accepted)>=2:
            temporary=self.out/'pool.tmp'
            atomic(temporary,dict(schema='collection-layout-pool/1',layouts=accepted))
            LayoutPool(temporary)
            temporary.replace(self.out/'pool.json')
            self.state['pool_published']=True
        elif (self.out/'pool.json').exists():
            (self.out/'pool.json').rename(self.out/('pool.invalidated-'+str(time.time_ns())+'.json'))
            self.state['pool_published']=False
        self.state['note']='Only the original five-task trajectory policy is checked; six production tasks and arbitrary joint initialization are not certified.'

    def execute(self):
        from space_arm_platform.collection_processes import ManagerLease
        path=ADAPTER/'Unreal/BskUnrealRenderer/Saved/collection-layout-import.lock'
        path.parent.mkdir(parents=True,exist_ok=True)
        lease=None
        try:
            lease=ManagerLease(path)
            if self.args.command=='prepare':
                self.prepare()
            if self.args.command=='certify' or not self.args.assets_only:
                self.certify()
            self.state.update(status='finished',current_stage=None)
            self.save()
            print(json.dumps(self.state,ensure_ascii=False,indent=2))
        except KeyboardInterrupt:
            self.state.update(status='cancelled',last_error='Stopped; completed layouts retained')
            self.save()
            raise SystemExit(130)
        except BaseException as e:
            self.state.update(status='failed',failed=self.state.get('failed',0)+1,last_error=f'{type(e).__name__}: {e}')
            self.save()
            raise
        finally:
            if lease:
                lease.close()
            self.lock.close()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    sub.add_parser('check',help='Check bundled Python, native plugins, assets and UE dependencies')
    for name in ('prepare','certify'):
        p=sub.add_parser(name,help='Prepare layouts serially' if name=='prepare' else 'Validate prepared trajectories and publish eligible pool')
        p.add_argument('--output',type=Path,required=True)
        p.add_argument('--count',type=int,default=150)
        p.add_argument('--seed',type=int,default=202610280100)
        p.add_argument('--yaw-range-deg',type=float,default=0)
        p.add_argument('--assets-only',action='store_true',help='Prepare geometry and UE assets without publishing a certified pool')
        p.add_argument('--resume',action='store_true',help='Clear a previous stop request and continue completed stages')
        p.add_argument('--stage-timeout',type=float,default=14400)
    for name in ('status','stop'):
        p=sub.add_parser(name)
        p.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.command=='check':
        subprocess.run([str(SIM),str(BASE/'app/check.py')],check=True)
    elif args.command=='status':
        print((args.output/'progress.json').read_text())
    elif args.command=='stop':
        if not (args.output/'settings.json').is_file():
            parser.error('Not a layout-preparer job directory')
        (args.output/'cancel.request').touch()
        print('Stop requested; completed layouts are retained.')
    else:
        if args.count<1 or args.seed<0 or not math.isfinite(args.yaw_range_deg) or not 0<=args.yaw_range_deg<=180:
            parser.error('Require count>=1, seed>=0 and yaw-range-deg in [0,180]')
        if not math.isfinite(args.stage_timeout) or args.stage_timeout<=0:
            parser.error('stage-timeout must be finite and positive')
        Job(args).execute()

if __name__=='__main__':
    main()
