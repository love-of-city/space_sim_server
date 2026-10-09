"""Isolate official LeRobot imports, image decoding and finalization from the API.

A single long-lived spawn process is warmed before recording becomes available.
Only the recorder's serial I/O thread calls append; RPCs never hold its state lock.
"""
from __future__ import annotations

import multiprocessing as mp
import threading
import time
from pathlib import Path
from typing import Any


def _serve(connection) -> None:
    writer = None
    try:
        from .lerobot_capture import LiveLeRobotWriter
        from .models import EpisodeStart, SimulationObservation
        # Import explicitly before advertising readiness (not on the first sample).
        from lerobot.datasets.lerobot_dataset import CODEBASE_VERSION
        if CODEBASE_VERSION != "v3.0":
            raise RuntimeError(f"Expected LeRobot v3.0, found {CODEBASE_VERSION}")
        connection.send((True, {"ready": True}))
        while True:
            command, payload = connection.recv()
            try:
                if command == "close":
                    if writer is not None:
                        writer.close_failed()
                    connection.send((True, {}))
                    break
                if command == "begin":
                    if writer is not None:
                        raise RuntimeError("dataset writer already has an episode")
                    writer = LiveLeRobotWriter(Path(payload[0]), EpisodeStart.model_validate(payload[1]))
                elif command == "append":
                    if writer is None:
                        raise RuntimeError("dataset writer has no episode")
                    writer.append(SimulationObservation.model_validate(payload[0]), payload[1])
                elif command in {"finish", "fail"}:
                    if writer is not None:
                        try:
                            if command == "finish":
                                writer.finish(*payload)
                            else:
                                writer.close_failed()
                        finally:
                            writer = None
                else:
                    raise RuntimeError(f"unknown dataset worker command: {command}")
                connection.send((True, {"frames": writer.frames if writer else 0}))
            except Exception as error:
                connection.send((False, f"{type(error).__name__}: {error}"))
    except (EOFError, BrokenPipeError):
        pass
    except BaseException as error:
        try:
            connection.send((False, f"{type(error).__name__}: {error}"))
        except (OSError, EOFError):
            pass
    finally:
        connection.close()


class DatasetWriterService:
    def __init__(self, *, startup_timeout_s: float = 120., rpc_timeout_s: float = 120.) -> None:
        self.startup_timeout_s = startup_timeout_s
        self.rpc_timeout_s = rpc_timeout_s
        self._lock = threading.Lock()
        self._process = None
        self._connection = None

    @property
    def ready(self) -> bool:
        return self._process is not None and self._process.is_alive() and self._connection is not None

    def _receive(self, timeout_s: float):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._connection.poll(.1):
                ok, result = self._connection.recv()
                if not ok:
                    raise RuntimeError(f"LeRobot writer: {result}")
                return result
            if not self._process.is_alive():
                raise RuntimeError(f"LeRobot writer exited (code {self._process.exitcode})")
        # A timed-out RPC must never let the next command consume its late reply.
        self.abort()
        raise RuntimeError("LeRobot writer timed out; recording cannot be marked complete")

    def prepare(self) -> None:
        with self._lock:
            if self.ready:
                return
            if self._process is not None or self._connection is not None:
                self.abort()
            context = mp.get_context("spawn")
            parent, child = context.Pipe()
            self._connection = parent
            self._process = context.Process(target=_serve, args=(child,), name="lerobot-dataset-writer")
            self._process.start()
            child.close()
            try:
                self._receive(self.startup_timeout_s)
            except BaseException:
                self.abort()
                raise

    def call(self, command: str, payload: Any = None):
        with self._lock:
            if not self.ready:
                raise RuntimeError("LeRobot writer is not ready")
            self._connection.send((command, payload))
            return self._receive(600. if command == "finish" else self.rpc_timeout_s)

    def writer(self, root: Path, request):
        self.call("begin", (str(root), request.model_dump(mode="json")))
        return ProcessLeRobotWriter(self, request)

    def abort(self) -> None:
        # Called by the owning RPC thread, including on a timeout.
        if self._process is not None:
            if self._process.is_alive():
                self._process.terminate()
            self._process.join(timeout=5.)
        if self._connection is not None:
            self._connection.close()
        self._connection = None
        self._process = None

    def close(self) -> None:
        try:
            if self.ready:
                self.call("close")
        finally:
            self.abort()


class ProcessLeRobotWriter:
    def __init__(self, service, request):
        self.service = service
        self.fps = request.fps
        self.cameras = tuple(request.camera_ids)
        self.frames = 0
        self.last_tick = None

    def append(self, observation, captures):
        result = self.service.call("append", (observation.model_dump(mode="json"), captures))
        self.frames = result["frames"]

    def finish(self, outcome="unknown", note=""):
        self.service.call("finish", (outcome, note))

    def close_failed(self):
        self.service.call("fail")
