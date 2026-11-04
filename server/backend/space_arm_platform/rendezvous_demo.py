"""Wall-clock scheduled, synchronized UE and dashboard mission playback."""
from __future__ import annotations

import copy
import gzip
import json
import socket
import struct
import threading
import time
import uuid
from pathlib import Path


def packet(message):
    data = json.dumps(message, separators=(',', ':'), allow_nan=False).encode('utf-8')
    return struct.pack('!I', len(data)) + data


class RendezvousDemo:
    def __init__(self, path: Path, state_port: int, render_port: int = 5558):
        with gzip.open(path, 'rt', encoding='utf-8') as stream:
            self.data = json.load(stream)
        if self.data.get('schema') != 'rendezvous-demo/1' or not self.data['source_report']['basic_nominal_passed']:
            raise ValueError('Invalid demonstration package')
        self.source = Path(self.data['source_dir'])
        self.state_port, self.render_port = state_port, render_port
        self.run_id = ''
        self.elapsed = 0.0
        self.error = None
        self.completed = False
        self.started_at = None
        self.finished_at = None
        self._thread = None
        self._stop = threading.Event()

    @property
    def running(self):
        return self._thread is not None and self._thread.is_alive()

    def start(self):
        if self.running:
            raise RuntimeError('演示已经在播放')
        # Fail before starting the clock if either receiver is unavailable.
        render = socket.create_connection(('127.0.0.1', self.render_port), timeout=2)
        try:
            state = socket.create_connection(('127.0.0.1', self.state_port), timeout=2)
        except Exception:
            render.close()
            raise
        self.run_id = uuid.uuid4().hex
        self.elapsed, self.error, self.completed = 0.0, None, False
        self.started_at = self.finished_at = None
        self._stop.clear()
        self._thread = threading.Thread(target=self._play, args=(render, state), daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=5)

    def _play(self, render, state):
        try:
            with render, state:
                for key in ('hello', 'manifest'):
                    message = copy.deepcopy(self.data[key])
                    message['session_id'] = self.run_id
                    render.sendall(packet(message))
                # Let the receiver apply the scene before timed motion starts.
                if self._stop.wait(.2):
                    return
                start = time.perf_counter()
                self.started_at = time.time()
                for sample in self.data['samples']:
                    if self._stop.wait(max(0, start + sample['elapsed_s'] - time.perf_counter())):
                        return
                    frame = {**sample['frame'], 'session_id': self.run_id,
                             'wall_time_ns': str(time.time_ns())}
                    t = int(frame['sim_time_ns']) / 1e9
                    capture = self.data['source_report']['capture']['latch_time_s']
                    bodies = [{
                        'body_id': body['object_id'], 'position_m': body['position_m'],
                        'velocity_m_s': body['velocity_mps'], 'attitude_wxyz': body['orientation_wxyz'],
                        'angular_velocity_rad_s': body['angular_velocity_B_radps'],
                    } for body in frame['objects'] if body['object_id'] in ('rendezvous/target', 'rendezvous/chaser')]
                    dashboard = {'protocol': 'space-sim-state/1', 'type': 'state',
                        'sim_time_s': t, 'bodies': bodies, 'metadata': {
                            'run_id': self.run_id, 'phase': sample['phase'], 'mode': 'demo',
                            'reference_frame': 'recorded local non-rotating display frame',
                            'capture_time_s': capture if t >= capture else None,
                            'playback_elapsed_s': sample['elapsed_s'], 'playback_duration_s': 15,
                            'source_run': self.source.name,
                        }}
                    render.sendall(packet(frame))
                    state.sendall(packet(dashboard))
                    self.elapsed = sample['elapsed_s']
                self.completed = True
                self.finished_at = time.time()
        except Exception as error:
            self.error = str(error)
