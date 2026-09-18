"""Episode-oriented synchronized action, state and camera recording."""

from __future__ import annotations

import json
import threading
import time
import uuid
from pathlib import Path
from typing import Any

from .lerobot_capture import LiveLeRobotWriter, sample_tick
from .models import AppliedAction, EpisodeStart, EpisodeStop, SimulationObservation


class EpisodeRecorder:
    """Record live LeRobot v3 datasets plus lossless platform audit sidecars."""

    def __init__(self, root: str | Path, *, drain_timeout_s: float = 10.0) -> None:
        self.root = Path(root).resolve()
        self._drain_timeout_s = drain_timeout_s
        self._dataset: LiveLeRobotWriter | None = None
        self._dataset_observations: dict[str, SimulationObservation] = {}
        self._dataset_captures: dict[str, dict[str, tuple[dict, dict]]] = {}
        self._dataset_error: str | None = None
        self._dataset_session: str | None = None
        self._stopping = False
        self._sealing = False
        self._actions: dict[str, AppliedAction] = {}
        self._lock = threading.RLock()
        self._write_condition = threading.Condition(self._lock)
        self._episode_id: str | None = None
        self._episode_dir: Path | None = None
        self._step_index = 0
        self._capture_index = 0
        self._steps_by_frame: dict[str, tuple[str, str, str]] = {}
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
            if self._inflight_writes:
                raise RuntimeError("previous episode disk writes are still draining")
            now = time.time_ns()
            episode_id = f"episode-{time.strftime('%Y%m%d-%H%M%S')}-{uuid.uuid4().hex[:8]}"
            directory = self.root / episode_id
            # Fail before opening an episode if the official writer is unavailable.
            try:
                dataset = LiveLeRobotWriter(directory / "lerobot", request)
            except ImportError as error:
                raise RuntimeError("LeRobot recording dependencies are missing; install the project dependencies") from error
            (directory / "cameras").mkdir(parents=True, exist_ok=False)
            metadata = {
                "schema": "space-arm-episode/2",
                "dataset_format": "lerobot-v3",
                "dataset_path": "lerobot",
                "dataset_fps": request.fps,
                "dataset_camera_ids": request.camera_ids,
                "dataset_capture_products": request.capture_products,
                "dataset_status": "recording",
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
            self._dataset = dataset
            self._dataset_observations.clear()
            self._dataset_captures.clear()
            self._dataset_error = None
            self._dataset_session = None
            self._stopping = False
            self._sealing = False
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
            if self._stopping:
                raise RuntimeError("episode is already finalizing")
            # Freeze the observation cutoff; capture callbacks can still drain it.
            self._stopping = True
            drained = self._write_condition.wait_for(
                lambda: self._inflight_writes == 0 and (
                    not self._dataset_observations or self._dataset_error is not None),
                timeout=self._drain_timeout_s,
            )
            if not drained:
                self._dataset_error = "timed out waiting for synchronized camera samples / disk writes"
            self._sealing = True
            dataset = self._dataset
            assert dataset is not None
            metadata_path = self._episode_dir / "metadata.json"
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        # Encoding/finalization must not hold the recorder lock or the API event
        # loop: controllers and network receivers remain responsive during stop.
        dataset_status = "empty"
        try:
            if self._dataset_error:
                dataset.close_failed()
                dataset_status = "failed"
            elif dataset.frames:
                dataset.finish(request.outcome, request.note)
                dataset_status = "complete"
        except Exception as error:
            self._dataset_error = f"{type(error).__name__}: {error}"
            dataset_status = "failed"
        with self._lock:
            metadata.update({
                "status": dataset_status,
                "dataset_status": dataset_status,
                "dataset_error": self._dataset_error,
                "dataset_frame_count": dataset.frames,
                "incomplete_sample_count": len(self._dataset_observations),
                "outcome": request.outcome, "note": request.note,
                "closed_wall_time_ns": str(time.time_ns()),
                "step_count": self._step_index, "capture_count": self._capture_index,
                "unmatched_capture_count": self._pending_capture_count,
                "rejected_capture_count": self._rejected_capture_count,
                "capture_sync": "complete" if not self._dataset_error else "incomplete",
            })
            self._write_json(metadata_path, metadata)
            self._episode_id = None
            self._episode_dir = None
            self._dataset = None
            self._dataset_observations.clear()
            self._dataset_captures.clear()
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
                "dataset_format": "lerobot-v3",
                "dataset_frame_count": self._dataset.frames if self._dataset else 0,
                "pending_dataset_samples": len(self._dataset_observations),
                "dataset_error": self._dataset_error,
                "finalizing": self._stopping and self._episode_id is not None,
            }

    def record_action(self, action: AppliedAction) -> None:
        with self._lock:
            self._actions[action.server_sequence] = action
            while len(self._actions) > 4096:
                self._actions.pop(next(iter(self._actions)))
            if self._episode_dir is None or self._stopping:
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
            if self._episode_dir is None or self._stopping:
                return
            recording_id = self._episode_id
            self._step_index += 1
            self._steps_by_frame[observation.render_frame_id] = (
                observation.step_id,
                observation.sim_time_ns,
                observation.render_session_id,
            )
            if action is None or action.server_sequence != observation.applied_action_sequence:
                action = self._actions.get(observation.applied_action_sequence)
            if action is not None and action.reset_generation != observation.reset_generation:
                action = None
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
            self._accept_dataset_observation(observation)
            # Bounded transport pairing window, separate from the selected samples.
            while len(self._steps_by_frame) > 512:
                self._steps_by_frame.pop(next(iter(self._steps_by_frame)))
            pending = self._pending_captures.pop(observation.render_frame_id, [])
            self._pending_capture_count -= len(pending)
        for metadata, products in pending:
            self._write_matched_capture(metadata, products, observation.step_id, observation.sim_time_ns,
                                        observation.render_session_id, recording_id)

    def record_authoritative_capture(self, metadata: dict[str, Any], products: dict[str, bytes]) -> None:
        matched_step: tuple[str, str, str] | None = None
        with self._lock:
            if self._episode_dir is None or self._sealing:
                return
            recording_id = self._episode_id
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
            elif not self._stopping:
                self._pending_captures.setdefault(frame_id, []).append((metadata, products))
                self._pending_capture_count += 1
        if matched_step is not None:
            self._write_matched_capture(metadata, products, *matched_step, recording_id)

    # Compatibility name for callers outside this repository. It remains strict.
    record_capture = record_authoritative_capture

    def _write_matched_capture(
        self,
        metadata: dict[str, Any],
        products: dict[str, bytes],
        step_id: str,
        observation_sim_time_ns: str,
        render_session_id: str = "",
        recording_id: str | None = None,
    ) -> None:
        with self._lock:
            if self._episode_dir is None or self._sealing or recording_id != self._episode_id:
                return
            if render_session_id and metadata.get("session_id") != render_session_id:
                # Also enforce at the locked write boundary: a capture callback
                # can already be in flight when the API completes a reset.
                self._rejected_capture_count += 1
                return
            capture_sim_time_ns = str(metadata.get("sim_time_ns", ""))
            if capture_sim_time_ns != observation_sim_time_ns:
                self._rejected_capture_count += 1
                raise ValueError(
                    "authoritative capture sim_time_ns does not match its simulation observation"
                )
            self._accept_dataset_capture(metadata, products)
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

    def _accept_dataset_observation(self, observation: SimulationObservation) -> None:
        if self._dataset is None or self._dataset_error:
            return
        try:
            tick = sample_tick(int(observation.sim_time_ns), self._dataset.fps)
            if tick is None:
                return
            if self._dataset_session is None:
                self._dataset_session = observation.render_session_id
            elif observation.render_session_id != self._dataset_session:
                raise ValueError("render session changed during recording")
            if self._dataset.last_tick is not None and tick <= self._dataset.last_tick:
                raise ValueError("non-monotonic/duplicate dataset observation")
            if observation.render_frame_id in self._dataset_observations:
                raise ValueError("duplicate dataset frame ID")
            if len(self._dataset_observations) >= 128:
                raise ValueError("dataset synchronization buffer overflow (128 samples)")
            self._dataset_observations[observation.render_frame_id] = observation
            self._flush_dataset_samples()
        except Exception as error:
            self._dataset_error = f"{type(error).__name__}: {error}"
            self._write_condition.notify_all()

    def _accept_dataset_capture(self, metadata: dict, products: dict) -> None:
        if self._dataset is None or self._dataset_error:
            return
        frame_id, camera = str(metadata["source_frame_id"]), str(metadata.get("camera_id", ""))
        if frame_id not in self._dataset_observations or camera not in self._dataset.cameras:
            return
        try:
            cameras = self._dataset_captures.setdefault(frame_id, {})
            if camera in cameras:
                raise ValueError("duplicate camera sample")
            cameras[camera] = (metadata, products)
            self._flush_dataset_samples()
        except Exception as error:
            self._dataset_error = f"{type(error).__name__}: {error}"
        self._write_condition.notify_all()

    def _flush_dataset_samples(self) -> None:
        assert self._dataset is not None
        while self._dataset_observations:
            frame_id = next(iter(self._dataset_observations))
            cameras = self._dataset_captures.get(frame_id, {})
            if any(camera not in cameras for camera in self._dataset.cameras):
                break
            self._dataset.append(self._dataset_observations[frame_id], cameras)
            del self._dataset_observations[frame_id]
            self._dataset_captures.pop(frame_id, None)
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
