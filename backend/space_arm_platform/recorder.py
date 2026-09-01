"""Episode-oriented synchronized action, state and camera recording."""

from __future__ import annotations

import json
import threading
import time
import uuid
from pathlib import Path
from typing import Any

from .models import AppliedAction, EpisodeStart, EpisodeStop, SimulationObservation


class EpisodeRecorder:
    """Write append-only JSONL plus camera products for one active episode."""

    def __init__(self, root: str | Path) -> None:
        self.root = Path(root).resolve()
        self._lock = threading.RLock()
        self._write_condition = threading.Condition(self._lock)
        self._episode_id: str | None = None
        self._episode_dir: Path | None = None
        self._step_index = 0
        self._capture_index = 0
        self._steps_by_frame: dict[str, tuple[str, str]] = {}
        self._pending_captures: dict[str, list[tuple[dict[str, Any], dict[str, bytes]]]] = {}
        self._pending_capture_count = 0
        self._rejected_capture_count = 0
        self._inflight_writes = 0

    @property
    def episode_id(self) -> str | None:
        with self._lock:
            return self._episode_id

    @property
    def episode_directory(self) -> Path | None:
        with self._lock:
            return self._episode_dir

    def start(self, request: EpisodeStart) -> dict[str, Any]:
        with self._lock:
            if self._episode_id is not None:
                raise RuntimeError(f"episode {self._episode_id} is already active")
            now = time.time_ns()
            episode_id = f"episode-{time.strftime('%Y%m%d-%H%M%S')}-{uuid.uuid4().hex[:8]}"
            directory = self.root / episode_id
            (directory / "cameras").mkdir(parents=True, exist_ok=False)
            metadata = {
                "schema": "space-arm-episode/1",
                "episode_id": episode_id,
                "status": "recording",
                "created_wall_time_ns": str(now),
                "task": request.task,
                "task_id": request.task_id,
                "instruction": request.instruction,
                "operator": request.operator,
                "operator_user_id": request.operator_user_id,
                "operator_username": request.operator_username,
                "operator_role": request.operator_role,
                "seed": request.seed,
                "tags": request.tags,
                "scene_instance": request.scene_instance,
                "control_protocol": "space-arm-control/1",
                "render_protocol": "bsk-render/2",
                "capture_protocol": "bsk-capture/1",
            }
            self._write_json(directory / "metadata.json", metadata)
            self._episode_id = episode_id
            self._episode_dir = directory
            self._step_index = 0
            self._capture_index = 0
            self._steps_by_frame.clear()
            self._pending_captures.clear()
            self._pending_capture_count = 0
            self._rejected_capture_count = 0
            self._inflight_writes = 0
            return metadata

    def stop(self, request: EpisodeStop) -> dict[str, Any]:
        with self._lock:
            if self._episode_id is None or self._episode_dir is None:
                raise RuntimeError("no episode is active")
            self._write_condition.wait_for(lambda: self._inflight_writes == 0, timeout=10.0)
            metadata_path = self._episode_dir / "metadata.json"
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            metadata.update(
                {
                    "status": "complete",
                    "outcome": request.outcome,
                    "note": request.note,
                    "closed_wall_time_ns": str(time.time_ns()),
                    "step_count": self._step_index,
                    "capture_count": self._capture_index,
                    "unmatched_capture_count": self._pending_capture_count,
                    "rejected_capture_count": self._rejected_capture_count,
                    "capture_sync": "complete" if self._pending_capture_count == 0 else "incomplete",
                }
            )
            self._write_json(metadata_path, metadata)
            self._episode_id = None
            self._episode_dir = None
            self._steps_by_frame.clear()
            self._pending_captures.clear()
            self._pending_capture_count = 0
            return metadata

    def sync_status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "matched_capture_count": self._capture_index,
                "pending_capture_count": self._pending_capture_count,
                "rejected_capture_count": self._rejected_capture_count,
                "inflight_write_count": self._inflight_writes,
            }

    def record_action(self, action: AppliedAction) -> None:
        with self._lock:
            if self._episode_dir is None:
                return
            self._append_jsonl(
                self._episode_dir / "actions.jsonl",
                {"record_wall_time_ns": str(time.time_ns()), **action.model_dump(mode="json")},
            )

    def record_observation(
        self,
        observation: SimulationObservation,
        action: AppliedAction | None,
    ) -> None:
        pending: list[tuple[dict[str, Any], dict[str, bytes]]] = []
        with self._lock:
            if self._episode_dir is None:
                return
            self._step_index += 1
            self._steps_by_frame[observation.render_frame_id] = (
                observation.step_id,
                observation.sim_time_ns,
            )
            row: dict[str, Any] = {
                "schema": "space-arm-step/1",
                "episode_id": self._episode_id,
                "step_id": observation.step_id,
                "render_frame_id": observation.render_frame_id,
                "record_wall_time_ns": str(time.time_ns()),
                "observation": observation.model_dump(mode="json"),
                "applied_action": action.model_dump(mode="json") if action else None,
            }
            self._append_jsonl(self._episode_dir / "steps.jsonl", row)
            pending = self._pending_captures.pop(observation.render_frame_id, [])
            self._pending_capture_count -= len(pending)
        for metadata, products in pending:
            self._write_matched_capture(metadata, products, observation.step_id, observation.sim_time_ns)

    def record_authoritative_capture(self, metadata: dict[str, Any], products: dict[str, bytes]) -> None:
        matched_step: tuple[str, str] | None = None
        with self._lock:
            if self._episode_dir is None:
                return
            if metadata.get("stream_kind") != "authoritative" or metadata.get("state_kind") != "authoritative":
                self._rejected_capture_count += 1
                raise ValueError("episode recorder accepts authoritative captures only")
            frame_id = str(metadata.get("source_frame_id", ""))
            sim_time_ns = str(metadata.get("sim_time_ns", ""))
            if not frame_id.isdecimal() or not sim_time_ns.isdecimal():
                self._rejected_capture_count += 1
                raise ValueError("authoritative capture is missing decimal source_frame_id/sim_time_ns")
            step = self._steps_by_frame.get(frame_id)
            if step is not None:
                matched_step = step
            elif self._pending_capture_count >= 128:
                self._rejected_capture_count += 1
                raise ValueError("authoritative capture pairing buffer overflow")
            else:
                self._pending_captures.setdefault(frame_id, []).append((metadata, products))
                self._pending_capture_count += 1
        if matched_step is not None:
            self._write_matched_capture(metadata, products, *matched_step)

    # Compatibility name for callers outside this repository. It remains strict.
    record_capture = record_authoritative_capture

    def _write_matched_capture(
        self,
        metadata: dict[str, Any],
        products: dict[str, bytes],
        step_id: str,
        observation_sim_time_ns: str,
    ) -> None:
        with self._lock:
            if self._episode_dir is None:
                return
            capture_sim_time_ns = str(metadata.get("sim_time_ns", ""))
            if capture_sim_time_ns != observation_sim_time_ns:
                self._rejected_capture_count += 1
                raise ValueError(
                    "authoritative capture sim_time_ns does not match its simulation observation"
                )
            self._capture_index += 1
            capture_id = self._capture_index
            episode_id = self._episode_id
            episode_dir = self._episode_dir
            self._inflight_writes += 1
        try:
            camera_id = self._safe_name(str(metadata.get("camera_id", "camera")))
            sequence = self._safe_name(str(metadata.get("capture_sequence", capture_id)))
            camera_dir = episode_dir / "cameras" / camera_id
            camera_dir.mkdir(parents=True, exist_ok=True)
            saved: dict[str, str] = {}
            product_metadata = {
                str(item.get("name")): item for item in metadata.get("products", []) if isinstance(item, dict)
            }
            for name, blob in products.items():
                item = product_metadata.get(name, {})
                supplied_name = Path(str(item.get("file_name", f"{sequence}_{name}.bin"))).name
                file_name = f"{sequence}_{supplied_name}"
                path = camera_dir / file_name
                path.write_bytes(blob)
                saved[name] = str(path.relative_to(episode_dir)).replace("\\", "/")
            row = {
                "schema": "space-arm-authoritative-capture/2",
                "episode_id": episode_id,
                "capture_id": str(capture_id),
                "step_id": step_id,
                "source_frame_id": str(metadata["source_frame_id"]),
                "sim_time_ns": capture_sim_time_ns,
                "authoritative_state": True,
                "record_wall_time_ns": str(time.time_ns()),
                "metadata": metadata,
                "files": saved,
            }
            with self._lock:
                self._append_jsonl(episode_dir / "captures.jsonl", row)
        except Exception:
            with self._lock:
                self._rejected_capture_count += 1
            raise
        finally:
            with self._lock:
                self._inflight_writes -= 1
                self._write_condition.notify_all()

    @staticmethod
    def _append_jsonl(path: Path, value: dict[str, Any]) -> None:
        with path.open("a", encoding="utf-8", newline="\n") as stream:
            stream.write(json.dumps(value, ensure_ascii=False, separators=(",", ":")) + "\n")

    @staticmethod
    def _write_json(path: Path, value: dict[str, Any]) -> None:
        path.write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding="utf-8")

    @staticmethod
    def _safe_name(value: str) -> str:
        cleaned = "".join("_" if char in '/\\:*?\"<>|' else char for char in value).strip(" .")
        return cleaned or "unnamed"
