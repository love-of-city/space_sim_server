"""Persistent bounded background jobs for sealing immutable episode archives."""

from __future__ import annotations

import gzip
import hashlib
import json
import os
import tarfile
import threading
import time
import uuid
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any


class JobManager:
    def __init__(self, data_root: Path, workers: int = 2) -> None:
        self.data_root = data_root.resolve()
        self.episode_root = self.data_root / "episodes"
        self.archive_root = self.data_root / "archives"
        self.job_root = self.data_root / "jobs"
        self.archive_root.mkdir(parents=True, exist_ok=True)
        self.job_root.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._executor = ThreadPoolExecutor(max_workers=max(1, min(workers, 4)), thread_name_prefix="space-arm-job")
        self._jobs: dict[str, dict[str, Any]] = {}
        self._idempotency: dict[str, str] = {}
        for path in self.job_root.glob("*.json"):
            try:
                job = json.loads(path.read_text(encoding="utf-8"))
                if job.get("status") in {"queued", "running"}:
                    job["status"] = "interrupted"
                    job["error"] = "backend restarted while job was active"
                    self._atomic_json(path, job)
                self._jobs[job["job_id"]] = job
                if job.get("idempotency_key"):
                    self._idempotency[job["idempotency_key"]] = job["job_id"]
            except (OSError, ValueError, KeyError):
                continue

    def submit_archive(self, episode_id: str) -> dict[str, Any]:
        key = f"archive:{episode_id}"
        with self._lock:
            previous_id = self._idempotency.get(key)
            if previous_id and self._jobs[previous_id]["status"] in {"queued", "running", "completed"}:
                return dict(self._jobs[previous_id])
            now = str(time.time_ns())
            job_id = f"job-{uuid.uuid4().hex[:12]}"
            job = {
                "schema": "space-arm-job/1",
                "job_id": job_id,
                "kind": "archive_episode",
                "status": "queued",
                "episode_id": episode_id,
                "idempotency_key": key,
                "created_wall_time_ns": now,
                "updated_wall_time_ns": now,
                "attempt": 0,
                "result": None,
                "error": None,
            }
            self._jobs[job_id] = job
            self._idempotency[key] = job_id
            self._persist(job)
            self._executor.submit(self._run_archive, job_id)
            return dict(job)

    def list(self) -> list[dict[str, Any]]:
        with self._lock:
            return [dict(job) for job in sorted(self._jobs.values(), key=lambda item: item["created_wall_time_ns"], reverse=True)]

    def get(self, job_id: str) -> dict[str, Any] | None:
        with self._lock:
            job = self._jobs.get(job_id)
            return dict(job) if job else None

    def close(self) -> None:
        self._executor.shutdown(wait=True, cancel_futures=False)

    def _run_archive(self, job_id: str) -> None:
        self._update(job_id, status="running", attempt=self._jobs[job_id]["attempt"] + 1, error=None)
        try:
            episode_id = self._jobs[job_id]["episode_id"]
            result = self._seal_episode(episode_id)
            self._update(job_id, status="completed", result=result)
        except Exception as error:  # the persisted job carries the actionable failure
            self._update(job_id, status="failed", error=f"{type(error).__name__}: {error}")

    def _seal_episode(self, episode_id: str) -> dict[str, Any]:
        episode_dir = (self.episode_root / episode_id).resolve()
        if episode_dir.parent != self.episode_root or not episode_dir.is_dir():
            raise ValueError(f"episode does not exist: {episode_id}")
        metadata = json.loads((episode_dir / "metadata.json").read_text(encoding="utf-8"))
        if metadata.get("status") != "complete":
            raise ValueError("only completed episodes can be archived")

        entries: list[dict[str, Any]] = []
        for path in sorted(item for item in episode_dir.rglob("*") if item.is_file() and item.name != "artifact_manifest.json"):
            relative = path.relative_to(episode_dir).as_posix()
            digest = hashlib.sha256()
            with path.open("rb") as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                    digest.update(chunk)
            entries.append({"path": relative, "size": path.stat().st_size, "sha256": digest.hexdigest()})
        manifest = {
            "schema": "space-arm-artifact-manifest/1",
            "episode_id": episode_id,
            "sealed_wall_time_ns": str(time.time_ns()),
            "files": entries,
        }
        self._atomic_json(episode_dir / "artifact_manifest.json", manifest)

        archive = self.archive_root / f"{episode_id}.tar.gz"
        temporary = archive.with_suffix(".tar.gz.partial")
        with temporary.open("wb") as raw:
            with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as compressed:
                with tarfile.open(fileobj=compressed, mode="w") as bundle:
                    for path in sorted(item for item in episode_dir.rglob("*") if item.is_file()):
                        info = bundle.gettarinfo(str(path), arcname=f"{episode_id}/{path.relative_to(episode_dir).as_posix()}")
                        info.uid = info.gid = 0
                        info.uname = info.gname = ""
                        info.mtime = 0
                        with path.open("rb") as stream:
                            bundle.addfile(info, stream)
        os.replace(temporary, archive)
        archive_digest = hashlib.sha256()
        with archive.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                archive_digest.update(chunk)
        archive_sha = archive_digest.hexdigest()
        return {
            "archive_path": str(archive),
            "archive_size": archive.stat().st_size,
            "archive_sha256": archive_sha,
            "file_count": len(entries) + 1,
        }

    def _update(self, job_id: str, **changes: Any) -> None:
        with self._lock:
            job = self._jobs[job_id]
            job.update(changes)
            job["updated_wall_time_ns"] = str(time.time_ns())
            self._persist(job)

    def _persist(self, job: dict[str, Any]) -> None:
        self._atomic_json(self.job_root / f"{job['job_id']}.json", job)

    @staticmethod
    def _atomic_json(path: Path, value: dict[str, Any]) -> None:
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")
        os.replace(temporary, path)
