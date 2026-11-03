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
import sys
import threading
import time


class _Worker:
    module = 'space_arm_platform.vision_worker'
    log_name = 'vision_worker.log'
    def __init__(self, root: Path, config_path: Path):
        runtime_path = root / 'run' / 'vision_runtime.json'
        runtime = json.loads(runtime_path.read_text('utf-8-sig')) if runtime_path.exists() else {}
        self.python = os.environ.get('SPACE_SIM_VISION_PYTHON') or runtime.get('python_executable') or sys.executable
        self.config_path = config_path
        self.root = root
        self.process = None
        self.replies = queue.Queue()

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
            self.process = subprocess.Popen(
                [self.python, '-m', self.module, '--config', str(self.config_path)],
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
        request = {'left_png': base64.b64encode(left).decode('ascii'),
                   'right_png': base64.b64encode(right).decode('ascii')}
        return self._exchange(request)

    def _exchange(self, request):
        if self.process is None or self.process.poll() is not None:
            self._start()
        try:
            self.process.stdin.write(json.dumps(request, separators=(',', ':')).encode('ascii') + b'\n')
            self.process.stdin.flush()
            return self.replies.get(timeout=15)
        except (OSError, queue.Empty) as error:
            self.close()
            raise RuntimeError('vision worker unavailable; see run/vision_worker.log') from error

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
    def __init__(self, root: Path, *, config=None, estimator=None):
        self.root = Path(root)
        self.config_path = self.root / 'config' / 'vision' / 'capture26_stereo.json'
        self.config = config
        self._static_test_config = config is not None
        self._estimator = estimator
        self._lock = threading.RLock()
        self._queue = queue.Queue(maxsize=2)
        self._pending = OrderedDict()
        self._images = {}
        self._result_images = {}
        self._result_metadata = {}
        self._last_metadata = {}
        self._session = ''
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
        self._saved_evidence = set()

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._work, daemon=True, name='stereo-image-estimator')
        self._thread.start()

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

    def on_capture(self, metadata, products):
        with self._lock:
            session = str(metadata.get('session_id', ''))
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
                self._pending.clear()
                self._images.clear()
                self._result_images.clear()
                self._result_metadata.clear()
                self._last_metadata.clear()
                self._last_result = None
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
                return
            self._counts['received_images'] += 1
            self._images[eye] = products['rgb']
            # Retain raw metadata for independent diagnostics only. Never pass it to the solver.
            self._last_metadata[eye] = dict(metadata)
            pair = self._pending.setdefault(key, {})
            pair[eye] = (sim_time, products['rgb'], dict(metadata))
            while len(self._pending) > 16:
                self._pending.popitem(last=False)
                self._counts['dropped_pairs'] += 1
            if len(pair) != 2:
                return
            self._pending.pop(key, None)
            skew = abs(pair['left'][0] - pair['right'][0])
            if skew > int(self.config.get('max_skew_ns', 100000)):
                self._counts['rejected_pairs'] += 1
                self._last_error = f'stereo timestamp skew {skew} ns exceeds configured tolerance'
                return
            self._counts['matched_pairs'] += 1
            item = (session, frame, pair['left'][0], skew, pair['left'][1], pair['right'][1],
                    {'left': pair['left'][2], 'right': pair['right'][2]})
            try:
                self._queue.put_nowait(item)
            except queue.Full:
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
                session, frame, timestamp, skew, left, right, diagnostic_metadata = self._queue.get(timeout=.1)
            except queue.Empty:
                continue
            started = time.perf_counter()
            try:
                result = self._estimator(left, right)
                result.update(session_id=session, source_frame_id=frame, sim_time_ns=str(timestamp),
                              stereo_skew_ns=skew, processing_ms=(time.perf_counter()-started)*1000,
                              input_contract='RGB only + static calibration + known marker corners')
                with self._lock:
                    if session == self._session:
                        self._last_result = result
                        self._result_images = {'left': left, 'right': right}
                        self._result_metadata = diagnostic_metadata
                        self._last_error = result.get('reason') if result.get('status') == 'error' else None
                        self._counts['processed_pairs'] += 1
                self._save_evidence(result, left, right, diagnostic_metadata)
            except Exception as error:
                with self._lock:
                    self._last_error = str(error)
            finally:
                self._queue.task_done()

    def _save_evidence(self, result, left, right, metadata):
        if self.evidence_dir is None:
            return
        labels = ['first_pair']
        if result.get('status') == 'ok':
            labels.append('first_tracking')
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
