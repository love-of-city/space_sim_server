"""Route the shared image ingress to one explicitly registered flight session.

The local flight process publishes its result port before starting physical
time. Warmup is asynchronous to HTTP, and no simulation tick waits for images.
"""
from __future__ import annotations

import contextlib
import json
from pathlib import Path
import threading

from .vision_channel import VisionResultSender
from .visual_navigation import calibration_id


class MissionVisionRouter:
    def __init__(self, root, runtime, vision, scenes=None):
        self.root, self.runtime, self.vision = Path(root), runtime, vision
        self.scenes = scenes
        self._stop = threading.Event()
        self._lock = threading.RLock()
        self._session = None
        self._sender = None
        self._thread = None

    def accepts(self, metadata):
        with self._lock:
            return self._session is not None and metadata.get('session_id') == self._session

    def start(self):
        self._stop.clear()
        self._thread = threading.Thread(target=self._work, name='mission-vision-route', daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=20)
        self._clear()

    def _clear(self):
        with self._lock:
            if self._session is None:
                return
            self._session = None
            self.vision.unbind_task()
            if self._sender:
                self._sender.close()
                self._sender = None

    def _folder(self):
        folder = self.runtime.output_dir
        if folder is not None and self.runtime.status().get('running'):
            return Path(folder)
        if self.scenes is not None:
            scene = self.scenes.status()
            instance = scene.get('instance') or {}
            folder = instance.get('capture26_output_directory')
            if scene.get('active') and folder:
                base = (self.root/'run/capture26-operations').resolve()
                expected = (base/instance['instance_id']).resolve()
                if expected.parent != base or Path(folder).resolve() != expected:
                    raise ValueError('capture26 operations vision route is outside its instance directory')
                return expected
        return None

    def update(self):
        folder = self._folder()
        if folder is None:
            self._clear()
            return
        route_file = folder/'vision-route.json'
        if not route_file.is_file():
            return
        route = json.loads(route_file.read_text('utf-8'))
        config_path = self.root/'config/vision/capture26_rendezvous_stereo.json'
        config = json.loads(config_path.read_text('utf-8-sig'))
        identity = tuple(route.get(k) for k in ('mission_id', 'session_id', 'calibration_id'))
        port = route.get('result_port')
        if (route.get('schema') != 'space-sim-vision-route/1'
                or any(not isinstance(v, str) or not v for v in identity)
                or identity[2] != calibration_id(config)
                or type(port) is not int or not 1024 <= port <= 65535):
            raise ValueError('invalid capture26 flight vision route')
        with self._lock:
            if identity[1] != self._session:
                self._clear()
                sender = VisionResultSender(port)
                self.vision.bind_task(identity[0], identity[1], config_path=config_path, on_result=sender)
                try:
                    self.vision.warmup()
                except Exception:
                    sender.close()
                    self.vision.unbind_task()
                    raise
                self._sender, self._session = sender, identity[1]
                (folder/'vision-ready.json').write_text(json.dumps(dict(
                    mission_id=identity[0], session_id=identity[1], calibration_id=identity[2])), encoding='utf-8')
        temporary = folder/'vision-status.tmp'
        temporary.write_text(json.dumps(self.vision.status()), encoding='utf-8')
        temporary.replace(folder/'vision-status.json')

    def _work(self):
        while not self._stop.wait(.1):
            try:
                self.update()
            except (OSError, ValueError, RuntimeError) as exc:
                # A malformed scene path must not kill the routing worker or
                # redirect its error report outside the registered task.
                with contextlib.suppress(OSError, ValueError, RuntimeError):
                    folder = self._folder()
                    if folder:
                        (Path(folder)/'vision-route-error.json').write_text(
                            json.dumps(dict(error=str(exc))), encoding='utf-8')
