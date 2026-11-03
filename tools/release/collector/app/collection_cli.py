"""Self-contained SpaceSim scripted data collection (Linux x86_64)."""
from pathlib import Path
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import time

BASE=Path(__file__).resolve().parents[1]
ROOT=BASE/'repo/server'
ADAPTER=BASE/'repo/ue-adapter'
UE=BASE/'runtime/UE_5.6'

def collector_tasks():
    """Load this product's supported subset without requiring server on import."""
    from space_arm_platform.task_catalog import COLLECTOR_TASK_CHOICES, TASK_LABELS

    return {name: TASK_LABELS[name] for name in COLLECTOR_TASK_CHOICES}

def atomic(path,doc):
    tmp=path.with_suffix('.tmp')
    tmp.write_text(json.dumps(doc,ensure_ascii=False,indent=2)+'\n')
    tmp.replace(path)

def make_scene(args,output):
    if args.scene:
        doc=json.loads(args.scene.expanduser().resolve().read_text())
        # An explicit collection setting should never be silently disabled by a saved diagnostic scene.
        if args.range_deg and doc.get('initialization_enabled') is False:
            raise ValueError('Input scene disables joint initialization; use --range-deg 0 or an enabled scene.')
    else:
        from space_arm_platform.models import SceneInstanceCreate
        from space_arm_platform.scene_runtime import _sample_instance
        from space_arm_platform.collection_randomization import CollectionRandomization
        request=SceneInstanceCreate(template_id='sarm-task-box-20261012',
            randomization_profile='teleop-zero-prepare-v1',operating_arm_joint_position_deg=[90,-60,60,0,-90,0],
            randomize_orbit_phase=args.randomize_orbit_phase,randomize_overview_camera=args.randomize_overview_camera)
        doc=_sample_instance(request,args.seed)
        doc.update(initialization_enabled=True,arm_preparation_required=False,
                   collection_randomization=CollectionRandomization().model_dump(mode='json'))
    if doc.get('created_by') is None:
        doc.pop('created_by',None)
    elif not isinstance(doc['created_by'],dict):
        raise ValueError('scene.created_by must be an object or null')
    doc['randomize_orbit_phase']=args.randomize_orbit_phase
    doc['randomize_overview_camera']=args.randomize_overview_camera
    doc['overview_camera_sampling']='presets'
    path=output/'scene.json'
    atomic(path,doc)
    return path

def collection_command(args,output,scene):
    command=[sys.executable,str(ROOT/'collection/collect_scripted_tasks.py'),
             '--task',args.task,'--scene',str(scene),'--output',str(output/'batch'),
             '--episodes-root',str(output/'episodes'),'--cancel-file',str(output/'cancel.request'),
             '--adapter',str(ADAPTER),'--unreal',str(UE),'--recording-config',str(output/'recording.json'),
             '--count',str(args.count),'--max-attempts',str(args.max_attempts),
             '--seed',str(args.seed),'--range-deg',str(args.range_deg),
             '--offline','--gpu-index',str(args.gpu_index),'--reuse-renderer','--reuse-dynamics',
             '--dynamics-max-episodes','4','--dynamics-max-rss-mb','8192',
             '--simulation-startup-timeout-seconds','600','--warmup-seconds','5','--reuse-warmup-seconds','5',
             '--overview-camera-sampling','presets']
    command.append('--randomize-overview-camera' if args.randomize_overview_camera else '--no-randomize-overview-camera')
    command.append('--randomize-orbit-phase' if args.randomize_orbit_phase else '--no-randomize-orbit-phase')
    if args.layout_pool:
        command+=['--layout-pool',str(args.layout_pool.expanduser().resolve())]
    else:
        command+=['--model',str(args.model.expanduser().resolve()),'--catalog',str(args.catalog.expanduser().resolve())]
    for offset,name in enumerate(('control','capture','render','signalling')):
        command += [f'--{name}-port',str(args.port_base+offset)]
    return command

def run(args):
    from space_arm_platform.collection_processes import ManagerLease
    from space_arm_platform.collection_runtime import check_port_available
    from space_arm_platform.recording_config import load_recording_config, DEFAULT_CAMERA_IDS
    if os.geteuid()==0:
        raise ValueError('UE must run as a regular Linux user, not root.')
    if args.layout_pool:
        from space_arm_platform.collection_layouts import LayoutPool
        LayoutPool(args.layout_pool.expanduser().resolve())
    else:
        from simulation.capture_layout import validated_capture_layout
        validated_capture_layout(args.model.expanduser().resolve(),args.catalog.expanduser().resolve())
    recording_path=(args.recording_config or ROOT/'config/recording-veryfast.json').expanduser().resolve()
    if not recording_path.is_file():
        raise FileNotFoundError(recording_path)
    recording=load_recording_config(recording_path)
    # The production validator requires these capture semantics.
    if (recording.capture_rate_hz!=50 or not recording.dataset_capture or not recording.video.streaming_encoding
            or recording.camera_ids!=list(DEFAULT_CAMERA_IDS) or recording.capture_products!=["rgb"]):
        raise ValueError('Collection requires both production RGB cameras, 50 Hz capture and streaming encoding.')
    out=args.output.expanduser().resolve()
    out.parent.mkdir(parents=True,exist_ok=True)
    out.mkdir(exist_ok=False)
    state=dict(schema='portable-collection/1',status='starting',task=args.task,requested_successes=args.count,
               bundle=str(BASE),output=str(out),pid=os.getpid(),accepted_episodes=[],
               randomization=dict(joint_half_range_deg=args.range_deg,overview_presets=args.randomize_overview_camera,
                                  orbit_phase=args.randomize_orbit_phase,layout_pool=str(args.layout_pool) if args.layout_pool else None))
    atomic(out/'launch.json',state)
    leases=[];process=None;old_handlers={}
    stopped_at=None
    def request_stop(*_):
        nonlocal stopped_at
        (out/'cancel.request').touch()
        if stopped_at is None:
            stopped_at=time.monotonic()
    started=time.monotonic()
    try:
        locks=Path(tempfile.gettempdir())/f'spacesim-collection-ports-{os.getuid()}'
        locks.mkdir(exist_ok=True)
        for port in range(args.port_base,args.port_base+4):
            leases.append(ManagerLease(locks/f'{port}.lock'))
            check_port_available(port)
        scene=make_scene(args,out)
        atomic(out/'recording.json',recording.model_dump(mode='json'))
        command=collection_command(args,out,scene)
        state.update(command=command,status='running')
        atomic(out/'launch.json',state)
        for sig in (signal.SIGINT,signal.SIGTERM):
            old_handlers[sig]=signal.signal(sig,request_stop)
        print(json.dumps(dict(status='running',task=args.task,output=str(out),target=args.count),ensure_ascii=False),flush=True)
        with (out/'collector.log').open('w') as log:
            process=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            while process.poll() is None:
                if (out/'cancel.request').exists() and stopped_at is None:
                    stopped_at=time.monotonic()
                if time.monotonic()-started>args.timeout_seconds:
                    state['error']='Batch wall-clock timeout'
                    request_stop()
                if stopped_at is not None and time.monotonic()-stopped_at>60:
                    os.killpg(process.pid,signal.SIGTERM)
                    try:process.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid,signal.SIGKILL);process.wait()
                time.sleep(.5)
        summary_path=out/'batch/summary.json'
        summary=json.loads(summary_path.read_text()) if summary_path.exists() else {}
        accepted=summary.get('accepted_episodes',[])
        success=process.returncode==0 and summary.get('status')=='complete' and len(accepted)==args.count
        state.update(status='complete' if success else ('cancelled' if stopped_at else 'failed'),
                     returncode=process.returncode,accepted_episodes=accepted,wall_seconds=time.monotonic()-started,
                     summary=str(summary_path))
        atomic(out/'launch.json',state)
        print(json.dumps(state,ensure_ascii=False,indent=2))
        return 0 if success else 1
    except BaseException as error:
        if process and process.poll() is None:
            request_stop()
            try:process.wait(timeout=60)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM)
                try:process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL);process.wait()
        state.update(status='failed',error=f'{type(error).__name__}: {error}',wall_seconds=time.monotonic()-started)
        atomic(out/'launch.json',state)
        raise
    finally:
        for sig,handler in old_handlers.items():signal.signal(sig,handler)
        for lease in leases:lease.close()

def check():
    from collection.recording_validation import validate_recording

    subprocess.run([sys.executable,str(BASE/'app/check.py')],check=True)
    import av
    import numpy as np
    import pyarrow
    from space_arm_platform.streaming_video import StrictStreamingVideoEncoder
    with tempfile.TemporaryDirectory(prefix='spacesim-codec-') as temp:
        path=Path(temp)/'test.mp4'
        with av.open(str(path),'w') as container:
            stream=container.add_stream('libx264',rate=50)
            stream.width=64;stream.height=64;stream.pix_fmt='yuv420p'
            for i in range(3):
                frame=av.VideoFrame.from_ndarray(np.full((64,64,3),i*40,dtype=np.uint8),format='rgb24')
                for packet in stream.encode(frame):container.mux(packet)
            for packet in stream.encode():container.mux(packet)
        with av.open(str(path)) as container:assert len(list(container.decode(video=0)))==3
    subprocess.run([str(BASE/'runtime/system-bin/ffprobe'),'-version'],stdout=subprocess.DEVNULL,check=True)
    print(json.dumps(dict(passed=True,h264_encode_decode=True,streaming_writer=True,parquet=True,
                         gpu_driver='Requires a Vulkan-capable installed driver; verified by real collection, not this dependency check')))
    return 0

def main(argv=None):
    tasks=collector_tasks()
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    sub.add_parser('tasks',help='List tasks supported by this collector product')
    sub.add_parser('check',help='Check bundled dependencies and H.264 encoding')
    p=sub.add_parser('run',help='Collect and validate episodes sequentially')
    p.add_argument('--task',choices=tasks,required=True)
    p.add_argument('--count',type=int,default=2)
    p.add_argument('--max-attempts',type=int)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--layout-pool',type=Path,help='Certified random layout pool (recommended)')
    p.add_argument('--model',type=Path,help='Explicit fixed layout for a controlled experiment')
    p.add_argument('--catalog',type=Path,help='UE asset catalog paired with --model')
    p.add_argument('--scene',type=Path,help='Optional scene; default creates the current SARM template')
    p.add_argument('--seed',type=int,default=2026102800)
    p.add_argument('--range-deg',type=float,default=30.0)
    p.add_argument('--randomize-overview-camera',action=argparse.BooleanOptionalAction,default=True)
    p.add_argument('--randomize-orbit-phase',action=argparse.BooleanOptionalAction,default=True)
    p.add_argument('--gpu-index',type=int,default=0)
    p.add_argument('--port-base',type=int,default=28500)
    p.add_argument('--recording-config',type=Path)
    p.add_argument('--timeout-seconds',type=float,default=86400)
    for name in ('status','stop'):
        p=sub.add_parser(name);p.add_argument('--output',type=Path,required=True)
    p=sub.add_parser('validate');p.add_argument('--episode',type=Path,required=True);p.add_argument('--task',choices=tasks,required=True)
    p=sub.add_parser('prepare-layouts',help='Run the bundled serial layout preparation/trajectory certification CLI')
    p.add_argument('arguments',nargs=argparse.REMAINDER)
    args=parser.parse_args(argv)
    if args.command=='tasks':
        print(json.dumps(tasks,ensure_ascii=False,indent=2));return 0
    if args.command=='check':return check()
    if args.command=='prepare-layouts':
        return subprocess.call([sys.executable,str(BASE/'app/main.py'),*args.arguments])
    if args.command=='validate':
        from collection.recording_validation import validate_recording
        print(json.dumps(validate_recording(args.episode.expanduser().resolve(),args.task),indent=2));return 0
    if args.command in ('status','stop'):
        out=args.output.expanduser().resolve()
        state=json.loads((out/'launch.json').read_text())
        if state.get('schema')!='portable-collection/1':parser.error('Not a portable collector output directory')
        if args.command=='stop':
            (out/'cancel.request').touch();print('Stop requested; completed data will be retained.')
        else:
            path=out/'batch/summary.json'
            if path.exists():state['batch']=json.loads(path.read_text())
            print(json.dumps(state,ensure_ascii=False,indent=2))
        return 0
    args.max_attempts=args.max_attempts if args.max_attempts is not None else min(1000,max(args.count*4,args.count))
    if not 1<=args.count<=args.max_attempts<=1000 or args.seed<0:parser.error('Require 1 <= count <= max-attempts <= 1000 and seed >= 0')
    if not math.isfinite(args.range_deg) or not 0<=args.range_deg<=50:parser.error('range-deg must be in [0,50]')
    if not math.isfinite(args.timeout_seconds) or args.timeout_seconds<=0:parser.error('timeout-seconds must be positive and finite')
    if not 1024<=args.port_base<=65532 or not 0<=args.gpu_index<=255:parser.error('Invalid port-base or gpu-index')
    if bool(args.model)!=bool(args.catalog) or bool(args.layout_pool)==bool(args.model):
        parser.error('Supply --layout-pool, or both --model and --catalog. Random layouts never fall back to a fixed model.')
    return run(args)

if __name__=='__main__':
    raise SystemExit(main())
