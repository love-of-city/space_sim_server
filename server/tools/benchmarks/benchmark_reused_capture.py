"""Short sequential recording probes in one UE and one dynamics process.

These are diagnostic captures, never successful training episodes. Only this
benchmark's worker treats the requested total_simulation_timeout as a normal
probe boundary so the next probe can exercise production session cleanup/reuse.
Production workers still retire on task failures.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'backend')]


def worker(mailbox, parent_pid, parent_created, count):
    from tqdm import tqdm
    tqdm.monitor_interval = 0
    from space_arm_platform.dynamics_worker import serve
    from collection.run_scripted_task import main as run_episode

    def run(argv):
        code = run_episode(argv=argv)
        output = Path(argv[argv.index('--output') + 1])
        result = json.loads((output / 'summary.json').read_text())
        if code == 1 and result['reason'] == 'total_simulation_timeout':
            return 0  # Expected diagnostic boundary; source summary remains failed.
        raise RuntimeError(f'Unexpected short probe result: {code}, {result.get("reason")}')

    serve(mailbox, dict(pid=parent_pid, created=parent_created), run,
          max_episodes=count + 1, max_rss_mb=8192)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path)
    parser.add_argument('--model', type=Path)
    parser.add_argument('--catalog', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--unreal', type=Path)
    parser.add_argument('--task', default='cylindrical_plug_insert')
    parser.add_argument('--seconds', type=float, default=8.)
    parser.add_argument('--count', type=int, default=2)
    parser.add_argument('--port-base', type=int, default=29400)
    parser.add_argument('--worker-mailbox', type=Path, help=argparse.SUPPRESS)
    parser.add_argument('--parent-pid', type=int, help=argparse.SUPPRESS)
    parser.add_argument('--parent-created', type=float, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.worker_mailbox:
        return worker(args.worker_mailbox, args.parent_pid, args.parent_created, args.count)
    if any(getattr(args, field) is None for field in ('scene', 'model', 'catalog', 'output', 'unreal')):
        parser.error('scene/model/catalog/output/unreal are required')
    if not 1 <= args.seconds <= 60 or not 2 <= args.count <= 10:
        parser.error('seconds must be [1,60], count [2,10]')
    import psutil
    from space_arm_platform.collection_runtime import PersistentRenderer
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    mailbox = out / 'dynamics-worker'
    mailbox.mkdir()
    env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
               MKL_NUM_THREADS='1', NUMEXPR_NUM_THREADS='1')
    os.environ.update({key: env[key] for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS',
                                                'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS')})
    renderer_args = SimpleNamespace(adapter=ROOT.parent/'ue-adapter', unreal=args.unreal,
        render_port=args.port_base+2, capture_port=args.port_base+1, offline=True, gpu_index=0)
    sources = ('native/local_mujoco_stepper.cpp', 'simulation/local_mujoco_stepper.py',
               'simulation/publication_scene.py', 'simulation/compiled_model_cache.py',
               'collection/run_scripted_task.py', 'collection/record_scripted_task.py',
               'tools/benchmarks/benchmark_reused_capture.py')
    manifest = dict(diagnostic_only=True, requested_seconds=args.seconds,
        sources_sha256={s: hashlib.sha256((ROOT/s).read_bytes()).hexdigest() for s in sources},
        input_sha256={str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest()
                      for p in (args.scene, args.model, args.catalog)},
        native_builds={name: json.loads((ROOT/'run/native_acceleration'/f'{name}.json').read_text())
                       for name in ('local_mujoco_stepper', 'posture', 'sarm_thread')}, commands=[])
    renderer = dynamics = None
    log = (out/'worker.log').open('w')
    measurements = []
    previous_finish = time.monotonic()
    try:
        renderer = PersistentRenderer(renderer_args, out)
        parent = psutil.Process()
        dynamics = subprocess.Popen([sys.executable, str(Path(__file__).resolve()),
            '--worker-mailbox', str(mailbox), '--parent-pid', str(parent.pid),
            '--parent-created', str(parent.create_time()), '--count', str(args.count)],
            env=env, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 120
        while not (mailbox/'ready.json').exists():
            if dynamics.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('Probe worker failed to start')
            time.sleep(.05)
        for index in range(args.count):
            directory = out / f'episode-{index+1}'
            command = [sys.executable, '-u', str(ROOT/'collection/record_scripted_task.py'),
                '--task', args.task, '--scene', str(args.scene.resolve()), '--model', str(args.model.resolve()),
                '--catalog', str(args.catalog.resolve()), '--output', str(directory),
                '--episodes-root', str(out/'diagnostic-episodes'), '--unreal', str(args.unreal),
                '--offline', '--gpu-index', '0', '--external-renderer', '--dynamics-worker', str(mailbox),
                '--max-sim-seconds', str(args.seconds), '--warmup-sim-seconds', '0.08',
                '--warmup-seconds', '5', '--recording-config', str(ROOT/'config/recording-veryfast.json')]
            for offset, name in enumerate(('control', 'capture', 'render', 'signalling')):
                command += [f'--{name}-port', str(args.port_base + offset)]
            manifest['commands'].append(command)
            (out/'manifest.json').write_text(json.dumps(manifest, indent=2))
            with (out/f'episode-{index+1}.log').open('w') as episode_log:
                process = subprocess.run(command, cwd=ROOT, env=env, stdout=episode_log, stderr=subprocess.STDOUT)
            finished = time.monotonic()
            renderer.check()
            result = json.loads((directory/'summary.json').read_text())
            task = json.loads((directory/'dynamics/summary.json').read_text())
            if process.returncode not in (0, 1) or result['task'].get('reason') != 'total_simulation_timeout':
                raise RuntimeError(f'Unexpected probe result; see {directory}')
            measurement = {key: result[key] for key in ('startup_wall_seconds', 'recording_wall_seconds',
                'recorded_sim_seconds', 'recording_realtime_factor', 'finalization_wall_seconds',
                'end_to_end_wall_seconds', 'episode_path')}
            measurement.update(index=index+1, completion_interval_seconds=finished-previous_finish,
                initialization_seconds=task['initialization_seconds'], model_compilation=task['model_compilation'],
                renderer_pid=renderer.process.pid, dynamics_worker=result['dynamics_worker'],
                frames_per_camera=result['episode']['dataset_frame_count'],
                publication_scene=task['publication_scene'])
            measurements.append(measurement)
            previous_finish = finished
            (out/'timing.json').write_text(json.dumps(dict(diagnostic_only=True, episodes=measurements), indent=2))
            print(json.dumps(measurement), flush=True)
            if (mailbox/'retired').exists():
                raise RuntimeError('Probe worker retired; reuse validation failed')
    finally:
        if dynamics is not None:
            (mailbox/'shutdown').touch()
            try:
                dynamics.wait(10)
            except subprocess.TimeoutExpired:
                dynamics.terminate()
                dynamics.wait(10)
        log.close()
        if renderer is not None:
            renderer.close()


if __name__ == '__main__':
    main()
