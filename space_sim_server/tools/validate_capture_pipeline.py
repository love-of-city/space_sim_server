"""Isolated real-GPU soak; never starts/stops a production scene or backend.

Replays an offline native-state bundle into the real renderer and recorder.
This measures capture throughput, NOT dynamics throughput. Outputs only under
the explicitly supplied fresh diagnostic directory. Run with the BSK runtime.
"""
from __future__ import annotations

import argparse
import copy
import io
import json
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'backend')]


def run(args):
    import numpy as np
    from PIL import Image
    from space_arm_platform.capture_receiver import CaptureReceiver
    from space_arm_platform.models import EpisodeStart, EpisodeStop, SimulationObservation
    from space_arm_platform.recorder import EpisodeRecorder
    sys.path.insert(0, str(args.adapter_root / 'Adapters'))
    from bsk_render_adapter.protocol import RenderPublisher

    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    bundle = json.loads(args.bundle.read_text(encoding='utf-8'))
    camera_ids = [c['camera_id'] for c in bundle['manifest']['cameras']]
    assert len(camera_ids) == 2, camera_ids
    count = round(args.seconds * 30)
    recorder = EpisodeRecorder(out / 'episodes') if args.write_dataset else None
    rows, errors, samples = [], [], {}
    lock = threading.Lock()
    def receive(meta, products):
        if recorder:
            recorder.record_authoritative_capture(meta, products)
        with lock:
            if meta.get('type') == 'capture_error':
                errors.append(str(meta))
                return
            rows.append(dict(metadata=meta, received_wall_ns=time.time_ns(), bytes=len(products.get('rgb', b''))))
            samples.setdefault(meta['camera_id'], products.get('rgb', b''))
    captures = CaptureReceiver('127.0.0.1', 0, receive, errors.append)
    captures.start()
    process = publisher = None
    result = dict(scope='Isolated saved-state replay: real scene assets, UE GPU, dual camera; NOT a dynamics benchmark.',
                  requested_simulated_seconds=args.seconds, requested_fps=30, write_dataset=args.write_dataset,
                  synchronous_baseline=args.synchronous, bounded_flow=args.flow_control,
                  frame_count=count, camera_ids=camera_ids)
    try:
        while not captures._listener: time.sleep(.01)
        with socket.socket() as s:
            s.bind(('127.0.0.1', 0))
            port = s.getsockname()[1]
        capture_port = captures._listener.getsockname()[1]
        editor = args.ue_root / 'Engine/Binaries/Win64/UnrealEditor-Cmd.exe'
        command = [str(editor), str(args.adapter_root / 'Unreal/BskUnrealRenderer/BskUnrealRenderer.uproject'),
            '-game', '-unattended', '-nop4', '-nosplash', '-RenderOffscreen', '-ForceRes', '-ResX=640', '-ResY=360',
            '-BskListen=127.0.0.1', f'-BskPort={port}', '-BskCaptureProducts=rgb', '-BskCaptureRate=30',
            '-BskCaptureHost=127.0.0.1', f'-BskCapturePort={capture_port}', '-ExecCmds=t.MaxFPS 60,r.VSync 0',
            f'-AbsLog={out / "ue.log"}']
        if args.synchronous: command.append('-BskSynchronousCapture')
        startup = subprocess.STARTUPINFO()
        startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        startup.wShowWindow = 0
        process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            startupinfo=startup, creationflags=subprocess.CREATE_NO_WINDOW)
        result.update(renderer_pid=process.pid, command=command)
        deadline = time.monotonic() + 120
        while True:
            assert process.poll() is None, 'isolated renderer exited during startup'
            try:
                with socket.create_connection(('127.0.0.1', port), timeout=.2): break
            except OSError:
                assert time.monotonic() < deadline, 'UE startup timeout'
                time.sleep(.1)
        publisher = RenderPublisher(port=port)
        publisher.retain_hello(bundle['hello'])
        publisher.retain_manifest(bundle['manifest'])
        def render(i, episode):
            return {**copy.deepcopy(bundle['frame']), 'frame_id': str(i),
                'sim_time_ns': str((i * 1_000_000_000 + 15) // 30), 'wall_time_ns': str(time.time_ns()),
                'capture_episode_id': episode, 'capture_request_id': 'diagnostic'}
        for i in range(120):
            publisher.publish_frame(render(i, ''))
            time.sleep(1 / 30)
        assert not rows, 'idle emitted dataset images'
        if recorder:
            info = recorder.start(EpisodeStart(camera_ids=camera_ids, max_frames=max(count, 18000),
                tags=['diagnostic', 'not-demonstration'], instruction='Isolated saved-state capture replay'),
                capture_on_demand=True)
            episode = info['episode_id']
        else: episode = 'isolated-capture-soak'
        start = time.perf_counter()
        progress_at, last_complete, peak_inflight = start, 0, 0
        for i in range(count):
            while True:
                if recorder:
                    status = recorder.sync_status()
                    assert not status['dataset_error'], status
                    complete = status['dataset_frame_count']
                else: complete = len(rows) // 2
                if complete != last_complete:
                    progress_at, last_complete = time.perf_counter(), complete
                assert process.poll() is None, 'renderer exited during capture'
                assert time.perf_counter() - progress_at < 30, 'no capture progress for 30s'
                if publisher.has_frame_capacity() and (not args.flow_control or i - complete < 14): break
                time.sleep(.005)
            peak_inflight = max(peak_inflight, i + 1 - complete)
            frame = render(i + 120, episode)
            if recorder:
                obs = {**copy.deepcopy(bundle['observation']), 'step_id': str(i + 121),
                       'render_frame_id': frame['frame_id'], 'sim_time_ns': frame['sim_time_ns'],
                       'wall_time_ns': frame['wall_time_ns'], 'render_session_id': frame['session_id'],
                       'capture_episode_id': episode, 'capture_request_id': 'diagnostic'}
                recorder.record_observation(SimulationObservation.model_validate(obs), None)
            publisher.publish_frame(frame)
            if (i + 1) % 300 == 0:
                print(json.dumps(dict(published=i + 1, completed=complete, images=len(rows),
                    elapsed_s=round(time.perf_counter() - start, 2), pipeline=captures.status()['pipeline'])), flush=True)
            time.sleep(max(0., start + (i + 1) / 30 - time.perf_counter()))
        result['publish_wall_s'] = time.perf_counter() - start
        if recorder: recorder.freeze_observations()
        publisher.publish_frame(render(count + 120, ''))
        deadline = time.monotonic() + 90
        while len(rows) < count * 2:
            assert process.poll() is None and time.monotonic() < deadline, ('drain timeout', len(rows))
            assert not errors, errors
            time.sleep(.02)
        result['capture_wall_s'] = time.perf_counter() - start
        time.sleep(.25)
        assert len(rows) == count * 2 and not errors
        for camera in camera_ids:
            cr = [r for r in rows if r['metadata']['camera_id'] == camera]
            assert [int(r['metadata']['source_frame_id']) for r in cr] == list(range(120, 120 + count))
            assert all(r['metadata']['capture_episode_id'] == episode for r in cr)
            assert all(int(r['metadata']['sim_time_ns']) == (int(r['metadata']['source_frame_id']) * 1_000_000_000 + 15)//30 for r in cr)
            assert Image.open(io.BytesIO(samples[camera])).size == (640, 360)
        if recorder:
            finish_start = time.perf_counter()
            closed = recorder.stop(EpisodeStop(outcome='success', note='Isolated capture pipeline validation, saved-state replay.'))
            result['dataset_finalize_wall_s'] = time.perf_counter() - finish_start
            result['episode_result'] = closed
            assert closed['dataset_status'] == 'complete', closed
            assert closed['dataset_frame_count'] == closed['step_count'] == count
            assert closed['capture_count'] == count * 2
        lag = [(int(r['metadata']['capture_wall_time_ns']) - int(r['metadata']['source_wall_time_ns']))/1e6 for r in rows]
        result.update(status='complete', received_images=len(rows), peak_inflight=peak_inflight,
            source_to_capture_ms_p50_p95_max=np.percentile(lag, [50, 95, 100]).tolist(),
            source_to_capture_first_last_ms=[lag[0], lag[-1]], pipeline=captures.status()['pipeline'])
    except BaseException as error:
        result.update(status='failed', error=f'{type(error).__name__}: {error}', received_images=len(rows))
        raise
    finally:
        if publisher: publisher.close()
        result['renderer_exit_before_cleanup'] = process.poll() if process else None
        if process and process.poll() is None:
            process.terminate()
            try: process.wait(15)
            except subprocess.TimeoutExpired: process.kill(); process.wait(10)
        captures.close()
        for index, camera in enumerate(camera_ids):
            if samples.get(camera):
                (out / f'first-camera-{index}.jpg').write_bytes(samples[camera])
        if recorder:
            if recorder.episode_id: recorder.fail('diagnostic aborted')
            recorder.close()
        (out / 'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        (out / 'capture-timing.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in rows), encoding='utf-8')
        print(json.dumps({k:v for k,v in result.items() if k not in ('command', 'episode_result')}, indent=2), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--write-dataset', action='store_true')
    parser.add_argument('--flow-control', action='store_true')
    parser.add_argument('--synchronous', action='store_true')
    parser.add_argument('--adapter-root', type=Path, default=ROOT.parent/'space_sim_UE_Adapter')
    parser.add_argument('--ue-root', type=Path, default=Path(r'C:\Program Files\Epic Games\UE_5.6'))
    args = parser.parse_args()
    if not 0 < args.seconds <= 3600: parser.error('seconds must be in (0, 3600]')
    run(args)
