"""Versioned JSONL adapter for trusted research methods in an owned process.

This is an observation boundary and process-lifetime boundary, not an OS
sandbox. No simulation object, private score or run-directory path is sent.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
from typing import Annotated, Literal

from pydantic import Field, model_validator

from .benchmark_contract import Contract, Count, Number
from .collection_processes import launch_collector

PROTOCOL = "spacesim-autonomy-method-jsonl/1"
MAX_MESSAGE_BYTES = 1024 * 1024
MAX_STDERR_BYTES = 256 * 1024


class PythonMethodSpec(Contract):
    schema_version: Literal["spacesim-autonomy-python-method/1"]
    method_id: str = Field(pattern=r"^[a-z0-9]+(?:-[a-z0-9]+)*$")
    task_schemas: tuple[Literal["spacesim-autonomy-waypoint/1", "spacesim-autonomy-attitude/1"], ...] = Field(min_length=1)
    entrypoint: str = Field(min_length=1)
    arguments: tuple[str, ...] = ()
    files: tuple[str, ...] = Field(min_length=1)
    initialization_timeout_s: Annotated[Number, Field(gt=0, le=300)] = 60.
    response_timeout_s: Annotated[Number, Field(gt=0, le=300)] = 5.

    @model_validator(mode="after")
    def complete_identity(self):
        if self.entrypoint not in self.files or not self.entrypoint.endswith(".py"):
            raise ValueError("Python entrypoint must be included in the declared implementation files")
        if len(set(self.files)) != len(self.files) or len(set(self.task_schemas)) != len(self.task_schemas):
            raise ValueError("implementation files and supported task schemas must be unique")
        return self


class MethodRequest(Contract):
    protocol: Literal["spacesim-autonomy-method-jsonl/1"] = PROTOCOL
    request_id: Count
    operation: Literal["reset", "act", "close"]
    payload: dict


class MethodResponse(Contract):
    protocol: Literal["spacesim-autonomy-method-jsonl/1"] = PROTOCOL
    request_id: Count
    status: Literal["ok", "error"]
    payload: dict | None = None
    error: str | None = None

    @model_validator(mode="after")
    def consistent_status(self):
        if self.status == "error" and (not self.error or self.payload is not None):
            raise ValueError("method error must contain a message and no action")
        if self.status == "ok" and self.error is not None:
            raise ValueError("successful reply must not contain an error")
        return self


def decode_message(raw: bytes) -> dict:
    if len(raw) > MAX_MESSAGE_BYTES or not raw.endswith(b"\n"):
        raise ValueError("method message exceeds size limit or lacks newline")

    def pairs(items):
        value = {}
        for key, item in items:
            if key in value:
                raise ValueError("duplicate JSON key")
            value[key] = item
        return value

    def invalid_constant(_):
        raise ValueError("non-finite JSON number")

    value = json.loads(raw.decode("utf-8"), object_pairs_hook=pairs, parse_constant=invalid_constant)
    if not isinstance(value, dict):
        raise ValueError("method message must be a JSON object")
    return value


class ExternalMethodDefinition:
    def __init__(self, path: Path, task):
        self.path = path.resolve()
        self.spec = PythonMethodSpec.model_validate_json(self.path.read_text(encoding="utf-8"))
        if task.schema_version not in self.spec.task_schemas:
            raise ValueError("external method does not support this task schema")
        self.root = self.path.parent
        self.identity = self.current_identity()

    def file(self, relative):
        path = (self.root / relative).resolve()
        if Path(relative).is_absolute() or not path.is_relative_to(self.root) or not path.is_file():
            raise ValueError("method files must exist within the specification directory")
        return path

    def current_identity(self):
        # Include the interpreter and all declared implementation/model files.
        # Repository-imported adapters are covered by the run's source manifest.
        hashes = {}
        for name in self.spec.files:
            with self.file(name).open("rb") as stream:
                hashes[name] = hashlib.file_digest(stream, "sha256").hexdigest()
        return {"specification": self.spec.model_dump(mode="json"),
                "specification_sha256": self.spec.fingerprint(),
                "files_sha256": hashes,
                "python": sys.version,
                "interpreter_sha256": hashlib.sha256(Path(sys.executable).read_bytes()).hexdigest()}

    def verify_unchanged(self):
        current_spec = PythonMethodSpec.model_validate_json(self.path.read_text(encoding="utf-8"))
        if current_spec != self.spec or self.current_identity() != self.identity:
            raise ValueError("external method changed after its identity was frozen")


class ExternalMethod:
    def __init__(self, definition: ExternalMethodDefinition, output: Path):
        self.definition, self.output = definition, output
        self.process = self.owner = self._stderr = self._calls = None
        self._io_thread = self._stderr_thread = None
        self._requests, self._responses = queue.Queue(maxsize=1), queue.Queue(maxsize=1)
        self._request_id, self._failed, self._ready = 0, False, False
        self.call_count, self.total_wall_s, self.maximum_wall_s = 0, 0., 0.
        self.stderr_bytes, self.stderr_stored = 0, 0

    def reset(self, context):
        if self.process is not None:
            raise RuntimeError("method reset requires a fresh process")
        self.definition.verify_unchanged()
        self._stderr = (self.output / "method_stderr.log").open("wb")
        self._calls = (self.output / "method_calls.jsonl").open("w", encoding="utf-8", newline="\n")
        spec = self.definition.spec
        command = [sys.executable, "-X", "utf8", "-u", str(self.definition.file(spec.entrypoint)), *spec.arguments]
        self.process, self.owner = launch_collector(command, cwd=self.definition.root,
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        self._io_thread = threading.Thread(target=self._io, daemon=True)
        self._stderr_thread = threading.Thread(target=self._drain_stderr, daemon=True)
        self._io_thread.start()
        self._stderr_thread.start()
        response = self._exchange("reset", {"context": context.model_dump(mode="json")}, spec.initialization_timeout_s)
        if response.payload is not None:
            self._failed = True
            raise ValueError("reset reply must not contain an action")
        self._ready = True

    def _io(self):
        while True:
            request = self._requests.get()
            if request is None:
                return
            try:
                view = memoryview(request)
                while view:
                    written = self.process.stdin.write(view)
                    if not written:
                        raise BrokenPipeError("method input pipe closed")
                    view = view[written:]
                self.process.stdin.flush()
                # Bounded even when a faulty client never sends a newline.
                reply = self.process.stdout.readline(MAX_MESSAGE_BYTES + 1)
                if not reply:
                    raise RuntimeError("method process closed its output")
                response = MethodResponse.model_validate(decode_message(reply))
            except Exception as error:
                self._responses.put(error)
                return
            self._responses.put(response)

    def _drain_stderr(self):
        while data := self.process.stderr.read(4096):
            self.stderr_bytes += len(data)
            keep = data[:max(0, MAX_STDERR_BYTES - self.stderr_stored)]
            self._stderr.write(keep)
            self.stderr_stored += len(keep)

    def _exchange(self, operation, payload, timeout):
        if self._failed:
            raise RuntimeError("method process has already failed")
        request = MethodRequest(request_id=self._request_id, operation=operation, payload=payload)
        encoded = (request.model_dump_json() + "\n").encode("utf-8")
        if len(encoded) > MAX_MESSAGE_BYTES:
            raise ValueError("method request exceeds message size limit")
        started = time.monotonic()
        status = "ok"
        try:
            self._requests.put_nowait(encoded)
            try:
                response = self._responses.get(timeout=timeout)
            except queue.Empty as error:
                raise TimeoutError(f"method {operation} exceeded {timeout:g} s wall time") from error
            if isinstance(response, Exception):
                raise RuntimeError(f"invalid method response: {response}") from response
            if response.request_id != request.request_id:
                raise ValueError("method response has a stale or mismatched request identity")
            if response.status == "error":
                raise RuntimeError(f"method reported error: {response.error}")
            return response
        except BaseException:
            status, self._failed = "failed", True
            raise
        finally:
            elapsed = time.monotonic() - started
            self.call_count += 1
            self.total_wall_s += elapsed
            self.maximum_wall_s = max(self.maximum_wall_s, elapsed)
            self._calls.write(json.dumps({"request_id": self._request_id, "operation": operation,
                "wall_time_s": elapsed, "status": status}, allow_nan=False) + "\n")
            self._request_id += 1

    def act(self, observation):
        if not self._ready:
            raise RuntimeError("method must acknowledge reset before acting")
        response = self._exchange("act", {"observation": observation.model_dump(mode="json")},
                                  self.definition.spec.response_timeout_s)
        if response.payload is None:
            self._failed = True
            raise ValueError("act reply must contain an action")
        return response.payload

    def telemetry(self):
        return {"protocol": PROTOCOL, "call_count": self.call_count, "total_wall_time_s": self.total_wall_s,
                "maximum_call_wall_time_s": self.maximum_wall_s, "stderr_bytes": self.stderr_bytes,
                "stderr_stored_bytes": self.stderr_stored,
                "timing_scope": "host roundtrip including serialization/IPC and method work; simulation waits for each reply"}

    def close(self):
        try:
            if self.process is not None and self._ready and not self._failed:
                response = self._exchange("close", {}, 5.)
                if response.payload is not None:
                    raise ValueError("close reply must not contain an action")
                if self.process.wait(timeout=5.) != 0:
                    raise RuntimeError("method process exited unsuccessfully")
            self.definition.verify_unchanged()
        finally:
            if self.owner is not None:
                self.owner.close()
                self.owner = None
            if self.process is not None:
                self.process.wait(timeout=5.)
                try:
                    self._requests.put_nowait(None)
                except queue.Full:
                    # A request may not have been consumed before its timeout.
                    self._requests.get_nowait()
                    self._requests.put_nowait(None)
                for thread in (self._io_thread, self._stderr_thread):
                    if thread is not None:
                        thread.join(timeout=2.)
                for stream in (self.process.stdin, self.process.stdout, self.process.stderr):
                    stream.close()
                self.process = None
            for stream in (self._stderr, self._calls):
                if stream is not None and not stream.closed:
                    stream.close()
            self._ready = False


def serve_method(method, context_type, observation_type):
    """Small Python SDK: one reset, ordered observations, one close per process."""
    ready, expected_id = False, 0
    for raw in iter(lambda: sys.stdin.buffer.readline(MAX_MESSAGE_BYTES + 1), b""):
        request = MethodRequest.model_validate(decode_message(raw))
        try:
            if request.request_id != expected_id:
                raise ValueError("method request sequence is not continuous")
            if request.operation == "reset" and not ready and expected_id == 0:
                if set(request.payload) != {"context"}:
                    raise ValueError("reset requires only context")
                method.reset(context_type.model_validate(request.payload["context"]))
                ready, payload = True, None
            elif request.operation == "act" and ready:
                if set(request.payload) != {"observation"}:
                    raise ValueError("act requires only observation")
                action = method.act(observation_type.model_validate(request.payload["observation"]))
                payload = action.model_dump(mode="json") if isinstance(action, Contract) else action
            elif request.operation == "close" and ready and not request.payload:
                method.close()
                payload = None
            else:
                raise ValueError("invalid method lifecycle request")
            response = MethodResponse(request_id=request.request_id, status="ok", payload=payload)
        except Exception as error:
            response = MethodResponse(request_id=request.request_id, status="error", error=f"{type(error).__name__}: {error}")
        sys.stdout.write(response.model_dump_json() + "\n")
        sys.stdout.flush()
        if response.status == "error" or request.operation == "close":
            return 1 if response.status == "error" else 0
        expected_id += 1
    return 1
