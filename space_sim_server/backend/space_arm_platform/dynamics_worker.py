"""Sequential local episode worker. Only imports/libraries survive an episode.

Each request creates a fresh BSK graph, MuJoCo state and noise generators. The
reply is published only after cleanup; a failure retires the entire worker.
The private mailbox uses unique request IDs and PID creation times so old replies
and recycled PIDs cannot be mistaken for the current episode/process.
"""
from pathlib import Path
import gc
import json
import os
import subprocess
import sys
import threading
import time
import traceback
import uuid

import psutil


def atomic_json(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2), encoding='utf-8')
    temporary.replace(path)


def live_process(identity):
    try:
        process = psutil.Process(identity['pid'])
        if process.create_time() == identity['created'] and process.is_running() and process.status() != psutil.STATUS_ZOMBIE:
            return process
    except psutil.NoSuchProcess:
        pass
    return None


class EpisodeProcess:
    """Popen-like handle for one request, never for another episode."""
    def __init__(self, directory, argv, stop_file):
        self.directory = Path(directory)
        self.identity = json.loads((self.directory / 'ready.json').read_text())
        if live_process(self.identity) is None:
            raise RuntimeError('Dynamics worker is not alive')
        if (self.directory / 'retired').exists() or (self.directory / 'shutdown').exists():
            raise RuntimeError('Dynamics worker is retiring')
        self.pid = self.identity['pid']
        self.request_id = uuid.uuid4().hex
        self.reply = self.directory / (self.request_id + '.result.json')
        self.returncode = None
        self.stop_file = Path(stop_file)
        self.result = None
        # Single outstanding episode per worker; concurrent submit is an error.
        with (self.directory / 'busy').open('x') as lock:
            lock.write(self.request_id)
        try:
            atomic_json(self.directory / 'request.json', dict(
                request_id=self.request_id, token=self.identity['token'], argv=list(argv)))
        except BaseException:
            (self.directory / 'busy').unlink()
            raise

    def poll(self):
        if self.returncode is not None:
            return self.returncode
        alive = live_process(self.identity) is not None
        # Check after liveness: the final reply may have been atomically written
        # between the first filesystem check and a worker's normal exit.
        if self.reply.exists():
            result = json.loads(self.reply.read_text())
            if result['request_id'] != self.request_id or result['token'] != self.identity['token']:
                raise RuntimeError('Stale dynamics worker result')
            self.result = result
            self.returncode = result['returncode']
        elif not alive:
            self.returncode = 2
        return self.returncode

    def wait(self, timeout=None):
        deadline = time.monotonic() + timeout if timeout is not None else float('inf')
        while self.poll() is None:
            if time.monotonic() >= deadline:
                raise subprocess.TimeoutExpired('dynamics episode', timeout)
            time.sleep(.05)
        return self.returncode

    def terminate(self):
        self.stop_file.write_text('recording cancelled', encoding='utf-8')

    def kill(self):
        # Used only after cooperative stop has timed out. PID identity is checked.
        process = live_process(self.identity)
        if process is not None:
            process.kill()


class PersistentDynamics:
    def __init__(self, server, output, *, max_episodes=4, max_rss_mb=8192):
        self.directory = Path(output) / ('dynamics-worker-' + uuid.uuid4().hex)
        self.directory.mkdir()
        self.log = (self.directory / 'worker.log').open('w', encoding='utf-8')
        parent = psutil.Process()
        command = [sys.executable, str(Path(server) / 'tools/dynamics_worker.py'),
                   '--mailbox', str(self.directory), '--parent-pid', str(parent.pid),
                   '--parent-created', str(parent.create_time()),
                   '--max-episodes', str(max_episodes), '--max-rss-mb', str(max_rss_mb)]
        self.process = subprocess.Popen(command, cwd=server, stdout=self.log, stderr=subprocess.STDOUT,
                                        env={**os.environ, 'PYTHONIOENCODING': 'utf-8'},
                                        creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        self.identity = None
        try:
            deadline = time.monotonic() + 120
            while not (self.directory / 'ready.json').exists():
                if self.process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('Dynamics worker failed to start; see ' + str(self.directory))
                time.sleep(.05)
            self.identity = json.loads((self.directory / 'ready.json').read_text())
            self.pid = self.identity['pid']
        except BaseException:
            self.close()
            raise

    def reusable(self):
        return (self.process.poll() is None and not (self.directory / 'retired').exists()
                and not (self.directory / 'busy').exists())

    def close(self):
        try:
            (self.directory / 'shutdown').touch()
            try:
                self.process.wait(10)
            except subprocess.TimeoutExpired:
                # Windows venv launchers may have a different PID from Python.
                process = live_process(self.identity) if self.identity else None
                (process or self.process).terminate()
                try:
                    self.process.wait(5)
                except subprocess.TimeoutExpired:
                    process = live_process(self.identity) if self.identity else None
                    (process or self.process).kill()
                    self.process.wait(5)
        finally:
            self.log.close()


def serve(directory, parent, runner, *, max_episodes=4, max_rss_mb=8192):
    if max_episodes <= 0 or max_rss_mb <= 0:
        raise ValueError('Dynamics worker recycling limits must be positive')
    directory = Path(directory)
    process = psutil.Process()
    identity = dict(pid=process.pid, created=process.create_time(), token=uuid.uuid4().hex)
    watcher_stop = threading.Event()

    def watch_parent():
        while not watcher_stop.wait(1):
            if live_process(parent) is None:
                # The owning collector vanished while the native runner may be
                # blocked. Exit this worker instead of leaving orphaned physics.
                os._exit(2)

    threading.Thread(target=watch_parent, name='dynamics-parent-watch', daemon=True).start()
    atomic_json(directory / 'ready.json', identity)
    completed = 0
    while live_process(parent) is not None and not (directory / 'shutdown').exists():
        request_path = directory / 'request.json'
        if not request_path.exists():
            time.sleep(.05)
            continue
        request = json.loads(request_path.read_text())
        request_path.unlink()
        request_id = request['request_id']
        if request['token'] != identity['token'] or not request_id.isalnum():
            raise RuntimeError('Invalid dynamics request identity')
        baseline_threads = set(threading.enumerate())
        started = time.monotonic()
        result = dict(request_id=request_id, token=identity['token'], pid=os.getpid())
        try:
            code = int(runner(request['argv']))
            if code not in (0, 1):
                raise RuntimeError('Unexpected episode return code: ' + str(code))
            result['returncode'] = code
        except BaseException:
            result.update(returncode=2, error=traceback.format_exc())
            # Do not retain a traceback with a partially initialized physics graph.
        gc.collect()
        extra = [t for t in threading.enumerate() if t not in baseline_threads]
        for thread in extra:
            thread.join(timeout=1)
        leaked = [t.name for t in extra if t.is_alive()]
        frames = sys._current_frames()
        thread_details = [dict(name=t.name, type=type(t).__module__ + '.' + type(t).__qualname__,
                               stack=''.join(traceback.format_stack(frames[t.ident]))
                               if t.ident in frames else '') for t in extra if t.is_alive()]
        del frames
        completed += 1
        result.update(wall_seconds=time.monotonic()-started, completed_episodes=completed,
                      rss_bytes=process.memory_info().rss, threads=process.num_threads(),
                      extra_threads=leaked, extra_thread_details=thread_details)
        # Native allocators/graphs may retain memory despite Python GC. Recycle
        # only BETWEEN episodes, never reset or kill a valid sample mid-flight.
        reasons = []
        if result['returncode'] != 0:
            reasons.append('episode_failed')
        if leaked:
            reasons.append('cleanup_incomplete')
        if completed >= max_episodes:
            reasons.append('episode_limit')
        if result['rss_bytes'] >= max_rss_mb * 1024 * 1024:
            reasons.append('rss_limit')
        retire = bool(reasons)
        result.update(retired=retire, retirement_reasons=reasons)
        if retire:
            (directory / 'retired').write_text(','.join(reasons))
        (directory / 'busy').unlink()
        atomic_json(directory / (request_id + '.result.json'), result)
        if retire:
            break
    watcher_stop.set()
