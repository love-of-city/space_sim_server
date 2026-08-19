"""Persistent task definitions separated from transient UE and WebRTC sessions."""

from __future__ import annotations

import json
import os
import threading
import time
import uuid
from pathlib import Path
from typing import Any

from .models import TaskCreate


class TaskStore:
    def __init__(self, root: Path) -> None:
        self.root = root.resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._tasks: dict[str, dict[str, Any]] = {}
        for path in self.root.glob("*.json"):
            try:
                task = json.loads(path.read_text(encoding="utf-8"))
                self._tasks[task["task_id"]] = task
            except (OSError, ValueError, KeyError):
                continue

    def create(self, request: TaskCreate) -> dict[str, Any]:
        now = str(time.time_ns())
        task_id = f"task-{uuid.uuid4().hex[:12]}"
        task = {
            "schema": "space-arm-task/1",
            "task_id": task_id,
            "status": "queued",
            "instruction": request.instruction,
            "scene_id": request.scene_id,
            "seed": request.seed,
            "tags": request.tags,
            "episode_id": None,
            "outcome": None,
            "created_wall_time_ns": now,
            "updated_wall_time_ns": now,
        }
        with self._lock:
            self._tasks[task_id] = task
            self._persist(task)
        return dict(task)

    def get(self, task_id: str) -> dict[str, Any] | None:
        with self._lock:
            task = self._tasks.get(task_id)
            return dict(task) if task else None

    def list(self) -> list[dict[str, Any]]:
        with self._lock:
            return [dict(task) for task in sorted(self._tasks.values(), key=lambda item: item["created_wall_time_ns"], reverse=True)]

    def transition(self, task_id: str, expected: set[str], status: str, **changes: Any) -> dict[str, Any]:
        with self._lock:
            task = self._tasks.get(task_id)
            if task is None:
                raise KeyError(task_id)
            if task["status"] not in expected:
                raise RuntimeError(f"task {task_id} is {task['status']}, expected one of {sorted(expected)}")
            task.update(changes, status=status, updated_wall_time_ns=str(time.time_ns()))
            self._persist(task)
            return dict(task)

    def _persist(self, task: dict[str, Any]) -> None:
        path = self.root / f"{task['task_id']}.json"
        temporary = path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(task, ensure_ascii=False, indent=2), encoding="utf-8")
        os.replace(temporary, path)
