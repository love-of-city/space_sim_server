"""Bounded authoritative pairing with serial I/O outside the control-plane lock."""
from __future__ import annotations

from collections import deque
import json
import threading
import time
import uuid
from pathlib import Path
from typing import Any

from .dataset_worker import DatasetWriterService
from .sampling import sample_tick
from .models import AppliedAction, EpisodeStart, EpisodeStop, SimulationObservation


class EpisodeRecorder:
    MAX_PENDING_SAMPLES = 128
    MAX_WRITE_JOBS = 512
    MAX_WRITE_BYTES = 128 * 1024 * 1024

    def __init__(self, root: str | Path, *, drain_timeout_s: float = 10., writer_factory=None) -> None:
        self.root = Path(root).resolve()
        self._drain_timeout_s = drain_timeout_s
        # Injection supports focused tests; production always uses the isolated process.
        self._writer_factory = writer_factory
        self._service = DatasetWriterService() if writer_factory is None else None
        self._lock = threading.RLock()
        self._write_condition = threading.Condition(self._lock)
        self._prepare_lock = threading.Lock()
        self._episode_id = None
        self._episode_dir = None
        self._dataset = None
        self._starting = False
        self._stopping = False
        self._sealing = False
        self._ready = False
        self._preparation_error = None
        self._actions = {}
        self._worker = None
        self._reset_buffers()

    def _reset_buffers(self):
        self._dataset_observations = {}
        self._dataset_captures = {}
        self._dataset_error = None
        self._dataset_session = None
        self._last_observation_tick = None
        self._first_observation_frame_id = None
        self._steps_by_frame = {}
        self._pending_captures = {}
        self._pending_capture_count = 0
        self._seen_cameras = {}
        self._step_index = self._capture_index = 0
        self._rejected_capture_count = self._stale_capture_count = 0
        self._completed_frames = 0
        self._jobs = deque()
        self._queued_bytes = 0
        self._inflight_writes = 0
        self._io_progress = 0
        self._worker_stop = False
        self._capture_on_demand = False
        self._capture_cutoff = False

    @property
    def episode_id(self):
        with self._lock:
            return self._episode_id

    @property
    def episode_directory(self):
        with self._lock:
            return self._episode_dir

    def prepare(self):
        # Never acquire the recorder state lock while importing LeRobot/spawning.
        with self._prepare_lock:
            try:
                if self._service:
                    self._service.prepare()
                with self._lock:
                    self._ready = True
                    self._preparation_error = None
            except Exception as error:
                with self._lock:
                    self._ready = False
                    self._preparation_error = str(error)
                raise

    def start(self, request: EpisodeStart, *, capture_on_demand: bool = False):
        with self._lock:
            if self._starting or self._episode_id is not None:
                raise RuntimeError("an episode is already preparing, recording or finalizing")
            if self._worker and self._worker.is_alive():
                raise RuntimeError("previous episode writes are still draining")
            self._starting = True
        dataset = None
        try:
            self.prepare()
            episode_id = f"episode-{time.strftime('%Y%m%d-%H%M%S')}-{uuid.uuid4().hex[:8]}"
            directory = self.root / episode_id
            dataset = (self._service.writer(directory / "lerobot", request) if self._service
                       else self._writer_factory(directory / "lerobot", request))
            (directory / "cameras").mkdir(parents=True, exist_ok=False)
            metadata = {
                "schema": "space-arm-episode/2", "dataset_format": "lerobot-v3", "dataset_path": "lerobot",
                "dataset_fps": request.fps, "dataset_camera_ids": request.camera_ids,
                "capture_on_demand": capture_on_demand,
                "dataset_capture_products": request.capture_products, "dataset_status": "recording",
                "episode_id": episode_id, "status": "recording", "created_wall_time_ns": str(time.time_ns()),
                **{k: getattr(request, k) for k in ("task", "task_id", "instruction", "operator",
                    "operator_user_id", "operator_username", "operator_role", "seed", "tags", "scene_instance")},
                "control_protocol": "space-arm-control/1", "render_protocol": "bsk-render/2", "capture_protocol": "bsk-capture/1",
            }
            self._write_json(directory / "metadata.json", metadata)
            with self._lock:
                self._reset_buffers()
                self._capture_on_demand = capture_on_demand
                self._dataset = dataset
                self._metadata = metadata
                self._stopping = self._sealing = False
                self._episode_id, self._episode_dir = episode_id, directory
                self._worker = threading.Thread(target=self._write_loop, name="episode-serial-io", daemon=True)
                self._worker.start()
                self._write_condition.notify_all()
            return metadata.copy()
        except Exception:
            # A failed directory/metadata write must not strand the persistent
            # child in an active episode and poison every subsequent start.
            if dataset is not None:
                try:
                    dataset.close_failed()
                except Exception:
                    if self._service:
                        self._service.abort()
            with self._lock:
                self._episode_id = self._episode_dir = self._dataset = None
            raise
        finally:
            with self._lock:
                self._starting = False

    def _fail_locked(self, error):
        if not self._dataset_error:
            self._dataset_error = str(error)
        self._write_condition.notify_all()

    def fail(self, error):
        with self._lock:
            if self._episode_id and not self._sealing:
                self._fail_locked(error)

    def _enqueue_locked(self, kind, payload, size=0):
        # Producer calls run on receiver threads, never the asyncio control loop.
        recording_id = self._episode_id
        ready = self._write_condition.wait_for(
            lambda: self._sealing or self._episode_id != recording_id
            or (len(self._jobs) < self.MAX_WRITE_JOBS and self._queued_bytes + size <= self.MAX_WRITE_BYTES),
            timeout=self._drain_timeout_s)
        if self._sealing or self._episode_id != recording_id:
            return False
        if not ready:
            self._fail_locked("episode I/O backpressure timed out; writer/disk is not keeping up (no valid dataset will be published)")
            return False
        self._jobs.append((kind, payload, size))
        self._queued_bytes += size
        self._write_condition.notify_all()
        return True

    def _write_loop(self):
        while True:
            with self._lock:
                self._write_condition.wait_for(lambda: self._jobs or self._worker_stop)
                if not self._jobs:
                    return
                kind, payload, size = self._jobs.popleft()
                self._inflight_writes += 1
                skip_sample = bool(self._dataset_error)
            try:
                if kind == "jsonl":
                    self._append_jsonl(*payload)
                elif kind == "capture":
                    directory, row, products = payload
                    for name, blob in products.items():
                        path = directory / row["files"][name]
                        path.parent.mkdir(parents=True, exist_ok=True)
                        path.write_bytes(blob)
                    self._append_jsonl(directory / "captures.jsonl", row)
                elif kind == "sample" and not skip_sample:
                    self._dataset.append(*payload)
                    with self._lock:
                        self._completed_frames = self._dataset.frames
            except Exception as error:
                with self._lock:
                    self._fail_locked(f"{type(error).__name__}: {error}")
            finally:
                with self._lock:
                    self._queued_bytes -= size
                    self._inflight_writes -= 1
                    self._io_progress += 1
                    if not self._sealing:
                        self._flush_dataset_samples()
                    self._write_condition.notify_all()

    def wait_for_writes(self, timeout_s=30.):
        with self._lock:
            return self._write_condition.wait_for(lambda: not self._jobs and not self._inflight_writes, timeout_s)

    def freeze_observations(self):
        """Freeze the stop boundary before disabling capture; keep accepting its RGB."""
        with self._lock:
            if self._episode_id is None:
                raise RuntimeError("no episode is active")
            self._capture_cutoff = True
            # Captures without an accepted observation are outside the frozen
            # boundary. Already paired/inflight samples are deliberately retained.
            self._stale_capture_count += self._pending_capture_count
            self._pending_captures.clear()
            self._pending_capture_count = 0
            self._write_condition.notify_all()

    def stop(self, request: EpisodeStop):
        with self._lock:
            if self._episode_id is None:
                raise RuntimeError("no episode is active")
            if self._stopping:
                raise RuntimeError("episode is already finalizing")
            self._stopping = True  # Freeze cutoff BEFORE waiting for any capture/IO queue.
            self._write_condition.notify_all()
            idle_deadline = time.monotonic() + self._drain_timeout_s
            total_deadline = time.monotonic() + max(60., self._drain_timeout_s * 6)
            progress = self._io_progress
            while self._dataset_observations and not self._dataset_error:
                now = time.monotonic()
                busy = bool(self._jobs or self._inflight_writes)
                if now >= total_deadline or (not busy and now >= idle_deadline):
                    self._fail_locked("timed out waiting for synchronized camera samples / disk writes")
                    break
                self._write_condition.wait(timeout=.02)
                if busy or progress != self._io_progress:
                    progress = self._io_progress
                    idle_deadline = time.monotonic() + self._drain_timeout_s
            self._sealing = True
            self._worker_stop = True
            self._write_condition.notify_all()
            dataset, directory, metadata = self._dataset, self._episode_dir, self._metadata.copy()
        # The worker drains every accepted job, then exits. No lock during encoding.
        self._worker.join(timeout=120.)
        if self._worker.is_alive():
            if self._service:
                self._service.abort()
            self._worker.join(timeout=5.)
            with self._lock:
                self._fail_locked("episode I/O worker did not drain before shutdown")
        status = "empty"
        try:
            if self._dataset_error:
                dataset.close_failed()
                status = "failed"
            elif self._completed_frames:
                dataset.finish(request.outcome, request.note)
                status = "complete"
            else:
                dataset.close_failed()
        except Exception as error:
            with self._lock:
                self._fail_locked(f"{type(error).__name__}: {error}")
            status = "failed"
        with self._lock:
            metadata.update(status=status, dataset_status=status, dataset_error=self._dataset_error,
                dataset_frame_count=self._completed_frames, incomplete_sample_count=len(self._dataset_observations),
                outcome=request.outcome, note=request.note, closed_wall_time_ns=str(time.time_ns()),
                step_count=self._step_index, capture_count=self._capture_index,
                unmatched_capture_count=self._pending_capture_count, rejected_capture_count=self._rejected_capture_count,
                stale_capture_count=self._stale_capture_count,
                capture_sync="complete" if not self._dataset_error else "incomplete")
        self._write_json(directory / "metadata.json", metadata)
        with self._lock:
            self._episode_id = self._episode_dir = self._dataset = None
            self._dataset_observations.clear()
            self._dataset_captures.clear()
            self._steps_by_frame.clear()
            self._pending_captures.clear()
            self._pending_capture_count = 0
            self._write_condition.notify_all()
        return metadata

    def close(self):
        if self.episode_id:
            self.stop(EpisodeStop(outcome="aborted", note="recorder closed"))
        if self._service:
            self._service.close()
        with self._lock:
            self._ready = False

    def sync_status(self):
        with self._lock:
            return {"matched_capture_count": self._capture_index, "pending_capture_count": self._pending_capture_count,
                "rejected_capture_count": self._rejected_capture_count, "stale_capture_count": self._stale_capture_count,
                "inflight_write_count": self._inflight_writes, "pending_write_jobs": len(self._jobs),
                "pending_write_bytes": self._queued_bytes, "dataset_format": "lerobot-v3",
                "dataset_frame_count": self._completed_frames if self._dataset else 0,
                "pending_dataset_samples": len(self._dataset_observations), "dataset_error": self._dataset_error,
                "writer_ready": self._ready and (self._service is None or self._service.ready),
                "writer_preparation_error": self._preparation_error, "preparing": self._starting,
                "capture_on_demand": self._capture_on_demand,
                "capture_boundary_frozen": self._capture_cutoff,
                "finalizing": (self._stopping or self._capture_cutoff) and self._episode_id is not None}

    def record_action(self, action: AppliedAction):
        with self._lock:
            self._actions[action.server_sequence] = action
            while len(self._actions) > 4096:
                self._actions.pop(next(iter(self._actions)))
            if self._episode_dir is not None and not self._stopping and not self._capture_cutoff:
                self._enqueue_locked("jsonl", (self._episode_dir / "actions.jsonl",
                    {"record_wall_time_ns": str(time.time_ns()), **action.model_dump(mode="json")}))

    def _discard_stale_captures_locked(self, minimum_frame_id):
        for frame_id in list(self._pending_captures):
            if int(frame_id) < minimum_frame_id:
                count = len(self._pending_captures.pop(frame_id))
                self._pending_capture_count -= count
                self._stale_capture_count += count

    def record_observation(self, observation: SimulationObservation, action):
        with self._lock:
            if self._episode_dir is None or self._stopping or self._capture_cutoff:
                return
            if self._capture_on_demand and observation.capture_episode_id != self._episode_id:
                return
            recording_id = self._episode_id
            ready = self._write_condition.wait_for(lambda: self._episode_id != recording_id or self._stopping or self._capture_cutoff
                or self._dataset_error or len(self._dataset_observations) < self.MAX_PENDING_SAMPLES,
                timeout=self._drain_timeout_s)
            if self._episode_id != recording_id or self._stopping or self._capture_cutoff:
                return
            if not ready:
                first = next(iter(self._dataset_observations), "unknown")
                missing = [c for c in self._dataset.cameras if c not in self._dataset_captures.get(first, {})]
                self._fail_locked(f"timed out waiting for RGB capture backpressure to clear (128 samples); first_frame={first}, missing_cameras={missing}")
            frame_id = observation.render_frame_id
            if self._first_observation_frame_id is None:
                self._first_observation_frame_id = int(frame_id)
                self._discard_stale_captures_locked(int(frame_id))
            self._steps_by_frame[frame_id] = (observation.step_id, observation.sim_time_ns, observation.render_session_id)
            self._step_index += 1
            if action is None or action.server_sequence != observation.applied_action_sequence:
                action = self._actions.get(observation.applied_action_sequence)
            if action is not None and action.reset_generation != observation.reset_generation:
                action = None
            self._accept_dataset_observation(observation)
            self._enqueue_locked("jsonl", (self._episode_dir / "steps.jsonl", {
                "schema": "space-arm-step/1", "episode_id": self._episode_id, "step_id": observation.step_id,
                "render_frame_id": frame_id, "record_wall_time_ns": str(time.time_ns()),
                "observation": observation.model_dump(mode="json"), "applied_action": action.model_dump(mode="json") if action else None}))
            pending = self._pending_captures.pop(frame_id, [])
            self._pending_capture_count -= len(pending)
            for metadata, products in pending:
                self._write_matched_capture(metadata, products, *self._steps_by_frame[frame_id])
            while len(self._steps_by_frame) > 512:
                expired = next(iter(self._steps_by_frame))
                del self._steps_by_frame[expired]
                self._seen_cameras.pop(expired, None)
            if self._steps_by_frame:
                self._discard_stale_captures_locked(min(int(f) for f in self._steps_by_frame))
            self._write_condition.notify_all()

    def record_authoritative_capture(self, metadata, products):
        with self._lock:
            if self._episode_dir is None or self._sealing:
                return
            if self._capture_on_demand and metadata.get("capture_episode_id") != self._episode_id:
                self._stale_capture_count += 1
                return
            if metadata.get("stream_kind") != "authoritative" or metadata.get("state_kind") != "authoritative":
                self._rejected_capture_count += 1
                raise ValueError("episode recorder accepts authoritative captures only")
            advertised = {str(p.get("name")) for p in metadata.get("products", []) if isinstance(p, dict)}
            if (set(products) | advertised) - {"rgb"}:
                self._rejected_capture_count += 1
                self._fail_locked("RGB-only capture received unsupported products; restart the UE scene")
                raise ValueError(self._dataset_error)
            frame_id = str(metadata.get("source_frame_id", ""))
            if not frame_id.isdecimal() or not str(metadata.get("sim_time_ns", "")).isdecimal():
                self._rejected_capture_count += 1
                raise ValueError("authoritative capture is missing decimal source_frame_id/sim_time_ns")
            if self._dataset_session and metadata.get("session_id") != self._dataset_session:
                self._rejected_capture_count += 1
                return
            boundary = min((int(f) for f in self._steps_by_frame), default=self._first_observation_frame_id)
            if boundary is not None and int(frame_id) < boundary:
                self._stale_capture_count += 1
                return
            if frame_id in self._steps_by_frame:
                self._write_matched_capture(metadata, products, *self._steps_by_frame[frame_id])
                return
            if self._stopping or self._capture_cutoff or self._dataset_error:
                return
            recording_id = self._episode_id
            ready = self._write_condition.wait_for(lambda: self._episode_id != recording_id or self._stopping or self._capture_cutoff
                or self._dataset_error or frame_id in self._steps_by_frame or self._pending_capture_count < 128,
                timeout=self._drain_timeout_s)
            if self._episode_id != recording_id or self._stopping or self._capture_cutoff or self._dataset_error:
                return
            if not ready:
                self._fail_locked(f"authoritative capture pairing stalled waiting for state frame={frame_id}; no packet was silently dropped")
                raise ValueError(self._dataset_error)
            # Recheck after waiting: observations may have caught up or moved boundary.
            if frame_id in self._steps_by_frame:
                self._write_matched_capture(metadata, products, *self._steps_by_frame[frame_id])
            elif self._steps_by_frame and int(frame_id) < min(int(f) for f in self._steps_by_frame):
                self._stale_capture_count += 1
            else:
                self._pending_captures.setdefault(frame_id, []).append((metadata, products))
                self._pending_capture_count += 1

    record_capture = record_authoritative_capture

    def _write_matched_capture(self, metadata, products, step_id, stamp, session):
        if session and metadata.get("session_id") != session:
            self._rejected_capture_count += 1
            return
        if str(metadata.get("sim_time_ns")) != stamp:
            self._rejected_capture_count += 1
            self._fail_locked("authoritative capture sim_time_ns does not match its simulation observation")
            raise ValueError(self._dataset_error)
        frame_id, camera = str(metadata["source_frame_id"]), str(metadata.get("camera_id", "camera"))
        seen = self._seen_cameras.setdefault(frame_id, set())
        if camera in seen:
            self._rejected_capture_count += 1
            self._fail_locked(f"duplicate camera sample: frame={frame_id}, camera={camera}")
            return
        seen.add(camera)
        self._capture_index += 1
        sequence = self._safe_name(str(metadata.get("capture_sequence", self._capture_index)))
        descriptors = {str(p.get("name")): p for p in metadata.get("products", []) if isinstance(p, dict)}
        files = {name: "cameras/" + self._safe_name(camera) + "/" + sequence + "_" +
                 Path(str(descriptors.get(name, {}).get("file_name", f"{sequence}_{name}.bin"))).name
                 for name in products}
        row = {"schema": "space-arm-authoritative-capture/2", "episode_id": self._episode_id,
               "capture_id": str(self._capture_index), "step_id": step_id, "source_frame_id": frame_id,
               "sim_time_ns": stamp, "authoritative_state": True, "record_wall_time_ns": str(time.time_ns()),
               "metadata": metadata, "files": files}
        self._enqueue_locked("capture", (self._episode_dir, row, products), sum(len(b) for b in products.values()))
        if not self._dataset_error and frame_id in self._dataset_observations and camera in self._dataset.cameras:
            self._dataset_captures.setdefault(frame_id, {})[camera] = (metadata, products)
            self._flush_dataset_samples()
        self._write_condition.notify_all()

    def _accept_dataset_observation(self, observation):
        if self._dataset_error:
            return
        tick = sample_tick(int(observation.sim_time_ns), self._dataset.fps)
        if tick is None:
            return
        if self._dataset_session is None:
            self._dataset_session = observation.render_session_id
        elif observation.render_session_id != self._dataset_session:
            self._fail_locked("render session changed during recording")
            return
        if self._last_observation_tick is not None and tick != self._last_observation_tick + 1:
            self._fail_locked(f"missing/non-monotonic dataset observation: expected_tick={self._last_observation_tick + 1}, received_tick={tick}, frame={observation.render_frame_id}; refusing to compress simulation time")
            return
        self._last_observation_tick = tick
        self._dataset_observations[observation.render_frame_id] = observation
        self._flush_dataset_samples()

    def _flush_dataset_samples(self):
        while self._dataset_observations and not self._dataset_error:
            frame_id = next(iter(self._dataset_observations))
            cameras = self._dataset_captures.get(frame_id, {})
            if any(c not in cameras for c in self._dataset.cameras):
                break
            obs = self._dataset_observations[frame_id]
            size = sum(len(b) for _, products in cameras.values() for b in products.values())
            # Do not release the pairing lock halfway through dispatching a
            # sample. The I/O worker retries dispatch when capacity is freed.
            if len(self._jobs) >= self.MAX_WRITE_JOBS or self._queued_bytes + size > self.MAX_WRITE_BYTES:
                break
            if not self._enqueue_locked("sample", (obs, cameras), size):
                break
            del self._dataset_observations[frame_id]
            self._dataset_captures.pop(frame_id, None)
        self._write_condition.notify_all()

    @staticmethod
    def _append_jsonl(path, value):
        with path.open("a", encoding="utf-8", newline="\n") as stream:
            stream.write(json.dumps(value, ensure_ascii=False, separators=(",", ":")) + "\n")

    @staticmethod
    def _write_json(path, value):
        path.write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding="utf-8")

    @staticmethod
    def _safe_name(value):
        return "".join("_" if c in '/\\:*?"<>|' else c for c in value).strip(" .") or "unnamed"
