"""Linux process ownership shared by platform and scene supervisors."""
from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import subprocess
import time

import psutil


def write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f"{path.name}.tmp.{os.getpid()}")
    temporary.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def identity(pid: int) -> dict:
    process = psutil.Process(pid)
    return {"pid": pid, "created": process.create_time(), "uid": process.uids().real}


def owned_process(record: dict | None):
    if not record or not record.get("created") or record.get("uid") != os.getuid():
        return None
    try:
        process = psutil.Process(int(record["pid"]))
        if (process.uids().real == os.getuid()
                and process.create_time() == record["created"]
                and process.status() != psutil.STATUS_ZOMBIE):
            return process
    except (psutil.Error, KeyError, ValueError):
        pass
    return None


def terminate(record: dict | None, timeout: float = 30) -> None:
    process = owned_process(record)
    if process is None:
        return
    # Supervisors get a chance to finalize datasets and stop their children first.
    process.send_signal(signal.SIGTERM)
    deadline = time.monotonic() + timeout
    while owned_process(record) is not None and time.monotonic() < deadline:
        time.sleep(.1)
    if owned_process(record) is not None:
        raise RuntimeError(f"PID {process.pid} did not stop within {timeout}s; inspect logs")


class Children:
    def __init__(self):
        self.processes = {}
        self.records = {}
        self.logs = []

    def start(self, name, command, *, cwd, log, env=None):
        stream = Path(log).open("ab")
        self.logs.append(stream)
        process = subprocess.Popen(command, cwd=cwd, env=env, stdout=stream,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        try:
            record = identity(process.pid)
        except psutil.NoSuchProcess:
            process.wait()
            raise RuntimeError(f"{name} exited before startup; inspect its log") from None
        self.processes[name] = process
        self.records[name] = record
        return process

    def check(self):
        for name, process in self.processes.items():
            if process.poll() is not None:
                raise RuntimeError(f"{name} exited with code {process.returncode}; inspect its log")

    def close(self, order, timeout=45):
        errors = []
        for name in order:
            if name not in self.processes:
                continue
            process = self.processes[name]
            leader = owned_process(self.records[name])
            descendants = []
            if leader is not None:
                try:
                    descendants = [identity(child.pid) for child in leader.children(recursive=True)]
                except psutil.Error:
                    pass
            try:
                terminate(self.records[name], timeout)
            except (RuntimeError, psutil.Error) as error:
                errors.append(str(error))
                # Only the session this supervisor created is eligible for escalation.
                if owned_process(self.records[name]) is not None and os.getpgid(process.pid) == process.pid:
                    os.killpg(process.pid, signal.SIGKILL)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                errors.append(f"{name} could not be reaped")
            for record in reversed(descendants):
                descendant = owned_process(record)
                if descendant is None:
                    continue
                try:
                    terminate(record, 10)
                except (RuntimeError, psutil.Error):
                    if owned_process(record) is not None:
                        descendant.kill()
        for stream in self.logs:
            stream.close()
        if errors:
            raise RuntimeError("; ".join(errors))
