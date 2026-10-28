"""Freeze one demonstration, then evaluate exactly N identical native replays.

No UE/camera policy is evaluated here. Every attempt is retained, including
native process errors. Existing success-targeted collectors are not invoked.
"""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import platform
import psutil

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'backend')]
from simulation.evaluation.fixed_trajectory import FrozenTrajectory,TASKS,freeze_trace,sha256
from simulation.evaluation.scoring import PhysicalScorer,aggregate


def write_json(path,value):
    path=Path(path);temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value,indent=2,allow_nan=False),encoding='utf-8');temporary.replace(path)


def execute(task,model,output,*,trajectory=None,max_sim_seconds=360.,wall_timeout=3600.):
    command=[sys.executable,str(ROOT/'tools/run_scripted_task.py'),'--task',task,'--model',str(model),'--output',str(output),
             '--max-sim-seconds',str(max_sim_seconds)]
    if trajectory:command+=['--replay-trajectory',str(trajectory),'--no-save-trace']
    with output.parent.joinpath(output.name+'.log').open('x',encoding='utf-8') as log:
        process=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
        try:
            returncode=process.wait(timeout=wall_timeout)
            return dict(command=command,returncode=returncode,timed_out=False)
        except subprocess.TimeoutExpired:
            # Windows virtual-environment launchers may spawn the real interpreter.
            try:
                for child in psutil.Process(process.pid).children(recursive=True):
                    try:child.kill()
                    except psutil.NoSuchProcess:pass
            except psutil.NoSuchProcess:pass
            process.kill();process.wait()
            return dict(command=command,returncode=None,timed_out=True)


def score_recording(directory):
    directory=Path(directory)
    spec=json.loads((directory/'scorer_spec.json').read_text(encoding='utf-8'))
    scorer=PhysicalScorer(**spec)
    with (directory/'evaluation_rows.jsonl').open(encoding='utf-8') as stream:
        for line in stream:scorer.update(json.loads(line))
    return scorer.report()


def canonical(args):
    args.output.mkdir(parents=True,exist_ok=False)
    tasks=TASKS if args.task=='all' else [args.task]
    report={}
    for task in tasks:
        directory=args.output/task;directory.mkdir()
        process=execute(task,args.model,directory/'donor',max_sim_seconds=args.max_sim_seconds,wall_timeout=args.wall_timeout)
        donor=directory/'donor/summary.json';trace=directory/'donor/trace.jsonl'
        result=dict(process=process,canonical_prepared=False)
        if donor.is_file():result['donor']=json.loads(donor.read_text(encoding='utf-8'))
        try:
            if process['timed_out']:raise ValueError('Donor timed out; incomplete trajectory is not frozen')
            frozen=freeze_trace(trace,args.model,task,directory/'trajectory.json',ROOT)
            result.update(canonical_prepared=True,trajectory_sha256=sha256(directory/'trajectory.json'),duration_s=frozen['samples'][-1]['time_s'])
        except (OSError,ValueError,KeyError) as exc:result['error']=str(exc)
        write_json(directory/'canonical.json',result);report[task]=result
        write_json(args.output/'canonical_summary.json',report)
        print(task,'prepared' if result['canonical_prepared'] else 'not prepared', 'donor:',result.get('donor',{}).get('result'),flush=True)
    return 0 if all(x['canonical_prepared'] for x in report.values()) else 1


def evaluate(args):
    args.output.mkdir(parents=True,exist_ok=False)
    tasks=TASKS if args.task=='all' else [args.task]
    suite={}
    protocol=dict(schema='space-arm-repeatability/1',protocol='fixed-condition-fixed-reference',trials_per_task=args.trials,
                  model=str(args.model),model_sha256=sha256(args.model),randomization='disabled',
                  observer='native physical state; no visual policy',python=sys.version,platform=platform.platform(),
                  terminal_hold_s=2.5,
                  environment={k:v for k,v in os.environ.items() if k.startswith(('SPACE_SIM_ORBITAL_','SPACE_SIM_PLUG_RETENTION','SPACE_SIM_LOCAL_SUBSTEPS'))},
                  sources={p:sha256(ROOT/p) for p in ['tools/evaluate_fixed_trajectories.py','tools/run_scripted_task.py','simulation/evaluation/fixed_trajectory.py','simulation/evaluation/scoring.py','simulation/evaluation/replay.py']})
    for relative in protocol['sources']:
        destination=args.output/'sources'/relative;destination.parent.mkdir(parents=True,exist_ok=True)
        destination.write_bytes((ROOT/relative).read_bytes())
    write_json(args.output/'protocol.json',protocol)
    for task in tasks:
        artifact=(args.canonical_dir/task/'trajectory.json').resolve()
        results=[];donor_result=None
        for i in range(args.trials):
            directory=args.output/task/f'trial-{i:04d}';directory.mkdir(parents=True)
            result=dict(task=task,trial=i,status='infrastructure_error',reason='not_started')
            try:
                if any(sha256(ROOT/p)!=expected for p,expected in protocol['sources'].items()):
                    raise ValueError('Evaluation source changed during the fixed-count suite')
                frozen=FrozenTrajectory(json.loads(artifact.read_text(encoding='utf-8')))
                frozen.validate_runtime(args.model,frozen.document['initial_joints'],ROOT)
                if frozen.document['task']!=task:raise ValueError('Trajectory task mismatch')
                donor_result=frozen.document.get('donor_result')
                result['trajectory_sha256']=sha256(artifact)
            except (OSError,ValueError,KeyError) as exc:
                result.update(status='initialization_rejected',reason=str(exc));results.append(result)
                write_json(directory/'result.json',result)
                suite[task]=dict(**aggregate(results),trials=results);write_json(args.output/'summary.json',suite)
                continue
            try:
                process=execute(task,args.model,directory/'native',trajectory=artifact,
                                max_sim_seconds=frozen.duration+3.5,wall_timeout=args.wall_timeout)
                result['process']=process
                native=directory/'native/summary.json'
                if process['timed_out']:raise RuntimeError('Native replay exceeded wall-time limit')
                if not native.is_file():raise RuntimeError('Native process did not produce a report; see native.log')
                report=json.loads(native.read_text(encoding='utf-8'));result['native_reason']=report.get('reason')
                if report.get('runtime_error'):raise RuntimeError(report['reason'])
                if 'independent_evaluation' not in report:raise RuntimeError('Native runtime failed before independent scoring')
                score=score_recording(directory/'native')
                result['evidence_sha256']={name:sha256(directory/'native'/name) for name in ['evaluation_rows.jsonl','scorer_spec.json']}
                if score!=report['independent_evaluation']:raise RuntimeError('Online/offline physical scores differ')
                result.update(score)
                if report.get('simulated_seconds',-1)<frozen.duration+2.5-1e-8 and result['status']!='unscorable':
                    result.update(status='task_failure',success=False,reason='premature_stop: '+str(report.get('reason')))
                if process['returncode'] not in (0,1):raise RuntimeError('Unexpected native exit code')
            except (OSError,ValueError,KeyError,RuntimeError) as exc:
                result.update(status='infrastructure_error',success=False,reason=str(exc))
            results.append(result);write_json(directory/'result.json',result)
            suite[task]=dict(**aggregate(results),trials=results)
            write_json(args.output/'summary.json',suite)
            print(task,i,result['status'],result['reason'],flush=True)
        # A trajectory donor failure remains explicit; it is never replaced by a better donor.
        suite[task]['donor_result']=donor_result
    write_json(args.output/'summary.json',suite)
    rows=[r for t in suite.values() for r in t['trials']]
    write_json(args.output/'aggregate.json',dict(**aggregate(rows),tasks={t:{k:v for k,v in a.items() if k!='trials'} for t,a in suite.items()}))
    return 0 if all(x['status'] in ('success','task_failure') for x in rows) else 2


def main():
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='command',required=True)
    for name in ['canonical','evaluate']:
        p=sub.add_parser(name);p.add_argument('--task',choices=['all',*TASKS],default='all')
        p.add_argument('--model',type=Path,default=ROOT/'model/SARM/platform/sarm_20261012.xml')
        p.add_argument('--output',type=Path,required=True);p.add_argument('--wall-timeout',type=float,default=3600.)
        if name=='canonical':p.add_argument('--max-sim-seconds',type=float,default=360.)
        else:p.add_argument('--canonical-dir',type=Path,required=True);p.add_argument('--trials',type=int,default=10)
    p=sub.add_parser('rescore');p.add_argument('directory',type=Path)
    args=parser.parse_args()
    if args.command=='rescore':print(json.dumps(score_recording(args.directory),indent=2));return 0
    if args.wall_timeout<=0 or not __import__('math').isfinite(args.wall_timeout):parser.error('wall-timeout must be finite and positive')
    if args.command=='evaluate' and args.trials<=0:parser.error('trials must be positive')
    if args.command=='canonical' and (args.max_sim_seconds<=0 or not __import__('math').isfinite(args.max_sim_seconds)):parser.error('max-sim-seconds must be finite and positive')
    return canonical(args) if args.command=='canonical' else evaluate(args)


if __name__=='__main__':raise SystemExit(main())
