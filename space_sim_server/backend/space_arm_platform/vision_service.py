"""Bounded, image-only stereo consumer, independent of the dataset recorder.

Only RGB bytes enter the estimator. Raw renderer extrinsics are deliberately
not part of its interface. The static configuration supplies measured/nominal
calibration and known marker geometry, never a simulated target pose.
"""
from __future__ import annotations

import base64
import contextlib
from collections import OrderedDict
import json
import os
from pathlib import Path
import queue
import subprocess
import struct
import sys
import threading
import time

from .visual_navigation import calibration_id


class _Worker:
    module = 'space_arm_platform.vision_worker'
    log_name = 'vision_worker.log'
    binary_input = True
    def __init__(self, root: Path, config_path: Path):
        runtime_path = root / 'run' / 'vision_runtime.json'
        runtime = json.loads(runtime_path.read_text('utf-8-sig')) if runtime_path.exists() else {}
        self.python = os.environ.get('SPACE_SIM_VISION_PYTHON') or runtime.get('python_executable') or sys.executable
        self.config_path = config_path
        self.root = root
        self.process = None
        self.replies = queue.Queue()
        self.timeout_s = 15.
        self.failed = ''

    def _start(self):
        env = os.environ.copy()
        env['PYTHONPATH'] = str(self.root / 'backend')
        env['PYTHONIOENCODING'] = 'utf-8'
        # Avoid oversubscribing the render/physics processes with tiny vision
        # operations dispatched to every CPU core.
        for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
            env[key] = '1'
        log = self.root / 'run' / self.log_name
        log.parent.mkdir(parents=True, exist_ok=True)
        with log.open('ab') as stderr:
            command = [self.python, '-m', self.module, '--config', str(self.config_path)]
            if self.binary_input:
                command.append('--binary-input')
            self.process = subprocess.Popen(
                command,
                cwd=self.root, env=env, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=stderr, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        self.replies = queue.Queue()
        process, replies = self.process, self.replies

        def read_results():
            for line in process.stdout:
                try:
                    replies.put(json.loads(line))
                except (ValueError, UnicodeError):
                    replies.put({'status': 'error', 'reason': 'invalid worker response'})
            replies.put({'status': 'error', 'reason': f'vision worker exited ({process.wait()}); see run/vision_worker.log'})
        threading.Thread(target=read_results, daemon=True, name='stereo-worker-output').start()

    def __call__(self, left: bytes, right: bytes):
        if self.binary_input:
            return self._exchange(struct.pack('!II', len(left), len(right))+left+right)
        request = {'left_png': base64.b64encode(left).decode('ascii'),
                   'right_png': base64.b64encode(right).decode('ascii')}
        return self._exchange(request)

    def _exchange(self, request):
        if self.failed:
            raise RuntimeError(self.failed)
        if self.process is not None and self.process.poll() is not None:
            self.failed = 'vision worker exited'
            raise RuntimeError(self.failed)
        if self.process is None:
            self._start()
        try:
            data = request if isinstance(request, bytes) else json.dumps(request, separators=(',', ':')).encode('ascii') + b'\n'
            self.process.stdin.write(data)
            self.process.stdin.flush()
            result = self.replies.get(timeout=self.timeout_s)
            if result.get('status') == 'error' and 'vision worker exited' in result.get('reason', ''):
                self.failed = result['reason']
                raise RuntimeError(self.failed)
            if result.get('reason') == 'invalid worker response':
                self.failed = 'invalid worker response'
                raise RuntimeError(self.failed)
            return result
        except (OSError, queue.Empty) as error:
            self.failed = 'vision worker unavailable; see run/vision_worker.log'
            self.close()
            raise RuntimeError(self.failed) from error

    def close(self):
        if self.process is not None:
            if self.process.poll() is None:
                # Closing stdin lets the Python venv launcher AND its native
                # child exit, rather than orphaning a worker on Windows.
                try:
                    self.process.stdin.close()
                except OSError:
                    pass
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    import psutil
                    try:
                        parent = psutil.Process(self.process.pid)
                        children = parent.children(recursive=True)
                        for child in reversed(children):
                            with contextlib.suppress(psutil.NoSuchProcess):
                                child.kill()
                        parent.kill()
                    except psutil.NoSuchProcess:
                        pass
                    self.process.wait(timeout=2)
            self.process = None


class StereoVisionService:
    def __init__(self, root: Path, *, config=None, estimator=None, on_result=None, config_path=None):
        self.root = Path(root)
        self.config_path = Path(config_path) if config_path else self.root / 'config' / 'vision' / 'capture26_stereo.json'
        self.config = config
        self._static_test_config = config is not None
        self._estimator = estimator
        self._lock = threading.RLock()
        self._queue = queue.Queue(maxsize=16)
        self._ordered_delivery = False
        self._pending = OrderedDict()
        self._images = {}
        self._result_images = {}
        self._result_metadata = {}
        self._last_metadata = {}
        self._session = ''
        self._mission_id = ''
        self._bound_session = None
        self._retired_sessions = set()
        self._completed_frames = set()
        self._last_epoch = None
        self._on_result = on_result
        self._last_received = None
        self._last_result = None
        self._last_error = None
        self._stop = threading.Event()
        self._thread = None
        self._counts = dict(received_images=0, matched_pairs=0, processed_pairs=0,
                            rejected_pairs=0, dropped_pairs=0, capture_errors=0)
        runtime = self.root / 'run' / 'vision_runtime.json'
        settings = json.loads(runtime.read_text('utf-8-sig')) if runtime.exists() else {}
        self.evidence_dir = Path(settings['evidence_directory']) if settings.get('evidence_directory') else None
        self.evidence_all = False
        self._saved_evidence = set()

    def bind_task(self, mission_id, session_id, *, config_path=None, on_result=None):
        """Explicit task routing prevents late frames switching the active task."""
        with self._lock:
            self._mission_id, self._bound_session = str(mission_id), str(session_id)
            self._ordered_delivery = True
            task_path = self.root/'config/missions/capture26-v2.json'
            self._task_timeout_s = (float(json.loads(task_path.read_text('utf-8-sig'))['mission']['vision_task_timeout_wall_s'])
                                    if task_path.is_file() else 15.)
            if isinstance(self._estimator, _Worker):
                self._estimator.timeout_s = self._task_timeout_s
            if on_result is not None:
                self._on_result = on_result
            if config_path and Path(config_path) != self.config_path:
                self.config_path, self.config = Path(config_path), None
                if isinstance(self._estimator, _Worker):
                    self._estimator.close()
                    self._estimator = None

    def unbind_task(self):
        """Return the shared service to operation-camera configuration."""
        with self._lock:
            if self._bound_session:
                self._retired_sessions.add(self._bound_session)
            self._bound_session, self._mission_id, self._on_result = None, '', None
            self._ordered_delivery = False
            default = self.root/'config/vision/capture26_stereo.json'
            if self.config_path != default:
                self.config_path, self.config = default, None
                if isinstance(self._estimator, _Worker):
                    self._estimator.close()
                    self._estimator = None

    def _invalid(self, session, frame, timestamp, reason, *, process_healthy=True):
        result = dict(status='rejected', reason=str(reason), ambiguous=False,
                      mission_id=self._mission_id, session_id=session, source_frame_id=frame,
                      sim_time_ns=str(timestamp), calibration_id=calibration_id(self.config),
                      pose_target_to_left=None, pose_covariance=None, process_healthy=process_healthy)
        self._last_result, self._last_error = result, str(reason)
        if self._on_result is not None:
            self._on_result(dict(result))

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._work, daemon=True, name='stereo-image-estimator')
        self._thread.start()

    def warmup(self):
        """Load the isolated solver before starting the mission physical clock."""
        with self._lock:
            self._load()
            if isinstance(self._estimator, _Worker):
                response = self._estimator(b'', b'')
                if response.get('status') != 'ready':
                    raise RuntimeError('vision solver warmup failed: '+str(response.get('reason', 'invalid response')))

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1)
        if isinstance(self._estimator, _Worker):
            self._estimator.close()
        if self._thread:
            self._thread.join(timeout=2)

    def _load(self, force=False):
        if self.config is None or (force and not self._static_test_config):
            new_config = json.loads(self.config_path.read_text('utf-8-sig'))
            if self.config is not None and new_config != self.config and isinstance(self._estimator, _Worker):
                self._estimator.close()
                self._estimator = None
            self.config = new_config
        if self._estimator is None:
            self._estimator = _Worker(self.root, self.config_path)
            self._estimator.timeout_s = getattr(self, '_task_timeout_s', 15.)

    def on_capture(self, metadata, products):
        with self._lock:
            session = str(metadata.get('session_id', ''))
            if (self._bound_session is not None and session != self._bound_session) or session in self._retired_sessions:
                return
            self._load(force=bool(session and session != self._session))
            cameras = self.config['camera_ids']
            eye = next((key for key in ('left', 'right') if cameras[key] == metadata.get('camera_id')), None)
            if eye is None:
                return
            frame = str(metadata.get('source_frame_id', ''))
            if not session or not frame:
                raise ValueError('vision capture requires session and source frame')
            sim_time = int(metadata['sim_time_ns'])
            if session != self._session:
                if self._session:
                    self._retired_sessions.add(self._session)
                self._pending.clear()
                self._images.clear()
                self._result_images.clear()
                self._result_metadata.clear()
                self._last_metadata.clear()
                self._last_result = None
                self._last_epoch = None
                self._completed_frames.clear()
                self._session = session
                while True:
                    try:
                        self._queue.get_nowait()
                        self._queue.task_done()
                    except queue.Empty:
                        break
            self._last_received = time.monotonic()
            key = (session, frame)
            if metadata.get('type') == 'capture_error' or not products.get('rgb'):
                self._pending.pop(key, None)
                self._counts['capture_errors'] += 1
                self._last_error = str(metadata.get('error', 'vision frame has no RGB'))
                self._invalid(session, frame, sim_time, self._last_error)
                return
            if frame in self._completed_frames:
                self._counts['rejected_pairs'] += 1
                self._invalid(session, frame, sim_time, 'duplicate source frame')
                return
            self._counts['received_images'] += 1
            self._images[eye] = products['rgb']
            # Retain raw metadata for independent diagnostics only. Never pass it to the solver.
            self._last_metadata[eye] = dict(metadata)
            pair = self._pending.setdefault(key, {})
            pair[eye] = (sim_time, products['rgb'], dict(metadata))
            while len(self._pending) > 16:
                (old_session, old_frame), old_pair = self._pending.popitem(last=False)
                self._counts['dropped_pairs'] += 1
                if self._ordered_delivery:
                    self._invalid(old_session, old_frame, next(iter(old_pair.values()))[0], 'unmatched authoritative stereo pair queue exhausted')
            if len(pair) != 2:
                return
            self._pending.pop(key, None)
            skew = abs(pair['left'][0] - pair['right'][0])
            if self._ordered_delivery and skew != 0 or skew > int(self.config.get('max_skew_ns', 100000)):
                self._counts['rejected_pairs'] += 1
                self._last_error = f'stereo timestamp skew {skew} ns violates exposure binding'
                self._invalid(session, frame, pair['left'][0], self._last_error)
                return
            epoch = pair['left'][0]
            if self._last_epoch is not None and epoch <= self._last_epoch:
                self._counts['rejected_pairs'] += 1
                self._invalid(session, frame, epoch, 'out-of-order exposure epoch')
                return
            self._last_epoch = epoch
            self._completed_frames.add(frame)
            if len(self._completed_frames) > 1024:
                self._completed_frames = {frame}
            self._counts['matched_pairs'] += 1
            item = (session, frame, pair['left'][0], skew, pair['left'][1], pair['right'][1],
                    {'left': pair['left'][2], 'right': pair['right'][2]},
                    self._mission_id, calibration_id(self.config), time.perf_counter())
            try:
                self._queue.put_nowait(item)
            except queue.Full:
                if self._ordered_delivery:
                    self._counts['dropped_pairs'] += 1
                    self._invalid(session, frame, epoch, 'authoritative image queue exhausted')
                    return
                try:
                    self._queue.get_nowait()
                    self._queue.task_done()
                    self._counts['dropped_pairs'] += 1
                except queue.Empty:
                    pass  # Consumer admitted it between Full and get_nowait.
                self._queue.put_nowait(item)

    def _work(self):
        while not self._stop.is_set():
            try:
                session, frame, timestamp, skew, left, right, diagnostic_metadata, mission, calibration, queued = self._queue.get(timeout=.1)
            except queue.Empty:
                continue
            started = time.perf_counter()
            try:
                result = self._estimator(left, right)
                result.update(session_id=session, source_frame_id=frame, sim_time_ns=str(timestamp),
                              stereo_skew_ns=skew, processing_ms=(time.perf_counter()-started)*1000,
                              queue_ms=(started-queued)*1000, mission_id=mission, calibration_id=calibration,
                              input_contract='RGB only + static calibration + known marker corners')
                # Timing diagnostics are allowlisted separately from raw UE
                # metadata. Camera/target truth never enters the pose solver or
                # the result channel used by navigation.
                timing_fields = ('source_wall_time_ns', 'readback_completed_wall_time_ns',
                                 'capture_wall_time_ns', 'received_wall_time_ns',
                                 'capture_pipeline_ms', 'capture_pending_jobs',
                                 'render_queue_frames', 'image_send_queue_packets')
                result['capture_timing'] = {
                    eye: {key: value for key, value in diagnostic_metadata[eye].items() if key in timing_fields}
                    for eye in ('left', 'right')}
                result['solved_wall_time_ns'] = str(time.time_ns())
                with self._lock:
                    if session == self._session and mission == self._mission_id and calibration == calibration_id(self.config):
                        self._last_result = result
                        self._result_images = {'left': left, 'right': right}
                        self._result_metadata = diagnostic_metadata
                        self._last_error = result.get('reason') if result.get('status') == 'error' else None
                        self._counts['processed_pairs'] += 1
                        if self._on_result is not None:
                            self._on_result(dict(result))
                self._save_evidence(result, left, right, diagnostic_metadata)
            except Exception as error:
                with self._lock:
                    if session == self._session and mission == self._mission_id:
                        self._invalid(session, frame, timestamp, 'vision process failure: '+str(error), process_healthy=False)
            finally:
                self._queue.task_done()

    def _save_evidence(self, result, left, right, metadata):
        if self.evidence_dir is None:
            return
        labels = ['first_pair']
        if result.get('status') == 'ok':
            labels.append('first_tracking')
        if self.evidence_all:
            labels.append(f"exposure-{int(result['sim_time_ns']):020d}")
        for label in labels:
            key = (result['session_id'], label)
            if key in self._saved_evidence:
                continue
            directory = self.evidence_dir / result['session_id'] / label
            directory.mkdir(parents=True, exist_ok=True)
            (directory/'left.png').write_bytes(left)
            (directory/'right.png').write_bytes(right)
            for name, value in [('result.json',result),('capture_metadata.json',metadata),
                                ('configuration.json',self.config)]:
                (directory/name).write_text(json.dumps(value,indent=2)+'\n',encoding='utf-8')
            self._saved_evidence.add(key)

    def wait_idle(self, timeout=3):
        deadline = time.monotonic() + timeout
        while self._queue.unfinished_tasks and time.monotonic() < deadline:
            time.sleep(.01)
        return self._queue.unfinished_tasks == 0

    def latest_image(self, eye):
        with self._lock:
            return (self._result_images or self._images).get(eye)

    def status(self):
        with self._lock:
            age = None if self._last_received is None else time.monotonic()-self._last_received
            return {**self._counts, 'session_id': self._session, 'last_result': self._last_result,
                    'last_error': self._last_error, 'pending_pairs': len(self._pending),
                    'queued_pairs': self._queue.qsize(), 'age_s': age,
                    'stale': age is None or age > 2,
                    'configuration': self.config,
                    'algorithm_uses_simulator_truth': False}

    def diagnostics(self):
        """Raw capture metadata is separate from image-only solver input."""
        with self._lock:
            return dict(self._result_metadata or self._last_metadata)
