"""FastAPI application joining operators, simulator, captures and episodes."""

from __future__ import annotations

import asyncio
import contextlib
import hmac
import time
import uuid
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import ValidationError

from .capture_receiver import CaptureReceiver
from .jobs import JobManager
from .models import EpisodeStart, EpisodeStop, OperatorAction, SimulationObservation, TaskComplete, TaskCreate
from .recorder import EpisodeRecorder
from .safety import ActionRejected, SafetyController
from .simulation_hub import SimulationHub
from .stream_access import issue_stream_access_token
from .tasks import TaskStore


@dataclass(frozen=True)
class PlatformConfig:
    project_root: Path
    data_root: Path
    simulation_host: str = "127.0.0.1"
    simulation_port: int = 8766
    capture_host: str = "127.0.0.1"
    capture_port: int = 8767
    deadman_timeout_s: float = 0.25
    pixel_streaming_player_port: int = 8080
    pixel_streaming_streamer_id: str = "BskRenderer"
    pixel_streaming_signalling_url: str = ""
    pixel_streaming_camera_streamers: tuple[tuple[str, str], ...] = ()
    stream_access_jwt_secret: str = ""
    stream_access_key: str = ""
    stream_access_token_ttl_seconds: int = 900


class OperatorSessions:
    """Allow many viewers but exactly one command owner."""

    def __init__(self) -> None:
        self.clients: dict[str, WebSocket] = {}
        self.active_operator: str | None = None
        self._lock = asyncio.Lock()

    async def connect(self, websocket: WebSocket) -> tuple[str, bool]:
        await websocket.accept()
        identifier = f"operator-{uuid.uuid4().hex[:8]}"
        async with self._lock:
            self.clients[identifier] = websocket
            granted = self.active_operator is None
            if granted:
                self.active_operator = identifier
        return identifier, granted

    async def disconnect(self, identifier: str) -> tuple[bool, tuple[str, WebSocket] | None]:
        async with self._lock:
            self.clients.pop(identifier, None)
            released = self.active_operator == identifier
            promoted: tuple[str, WebSocket] | None = None
            if released:
                self.active_operator = next(iter(self.clients), None)
                if self.active_operator is not None:
                    promoted = (self.active_operator, self.clients[self.active_operator])
            return released, promoted

    def is_owner(self, identifier: str) -> bool:
        return self.active_operator == identifier


def create_app(config: PlatformConfig | None = None) -> FastAPI:
    project_root = Path(__file__).resolve().parents[2]
    config = config or PlatformConfig(
        project_root=project_root,
        data_root=project_root / "data" / "episodes",
    )
    recorder = EpisodeRecorder(config.data_root)
    jobs = JobManager(config.data_root.parent)
    tasks = TaskStore(config.data_root.parent / "tasks")
    safety = SafetyController(config.deadman_timeout_s)
    hub = SimulationHub()
    sessions = OperatorSessions()
    captures = CaptureReceiver(
        config.capture_host,
        config.capture_port,
        recorder.record_authoritative_capture,
    )
    timeout_task: asyncio.Task[None] | None = None

    async def record_observation(observation: SimulationObservation) -> None:
        recorder.record_observation(observation, safety.last_action)

    hub.on_observation = record_observation

    async def watchdog() -> None:
        while True:
            await asyncio.sleep(0.05)
            action = safety.timeout_action(recorder.episode_id)
            if action:
                recorder.record_action(action)
                await hub.publish_action(action)

    @asynccontextmanager
    async def lifespan(_: FastAPI) -> AsyncIterator[None]:
        nonlocal timeout_task
        await hub.start(config.simulation_host, config.simulation_port)
        captures.start()
        timeout_task = asyncio.create_task(watchdog(), name="deadman-watchdog")
        try:
            yield
        finally:
            if timeout_task:
                timeout_task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await timeout_task
            captures.close()
            await hub.close()
            jobs.close()

    app = FastAPI(title="Space Arm Data Platform", version="0.2.0", lifespan=lifespan)
    app.state.config = config
    app.state.recorder = recorder
    app.state.safety = safety
    app.state.hub = hub
    app.state.captures = captures
    app.state.sessions = sessions
    app.state.jobs = jobs
    app.state.tasks = tasks

    frontend_source = config.project_root / "frontend"
    frontend_dist = frontend_source / "dist"
    frontend = frontend_dist if (frontend_dist / "index.html").is_file() else frontend_source
    app.mount("/static", StaticFiles(directory=frontend), name="static")

    @app.get("/")
    async def index() -> FileResponse:
        return FileResponse(frontend / "index.html")

    @app.get("/api/health")
    async def health() -> dict[str, Any]:
        return {
            "ok": True,
            "server_time_ns": str(time.time_ns()),
            "simulation": hub.status(),
            "active_episode": recorder.episode_id,
            "capture_cameras": captures.camera_ids(),
            "capture_error": captures.last_error,
            "capture_channels": captures.status(),
            "capture_sync": recorder.sync_status(),
            "active_operator": sessions.active_operator,
        }

    def authorize_access_key(candidate: str | None) -> None:
        if config.stream_access_key and not hmac.compare_digest(candidate or "", config.stream_access_key):
            raise HTTPException(status_code=401, detail="a valid stream access key is required")

    @app.get("/api/client-config")
    async def client_config(request: Request) -> dict[str, Any]:
        authorize_access_key(request.headers.get("x-space-arm-access-key") or request.query_params.get("access_key"))
        streamers = [
            {"id": config.pixel_streaming_streamer_id, "label": "主视口"},
            *(
                {"id": streamer_id, "label": label}
                for streamer_id, label in config.pixel_streaming_camera_streamers
            ),
        ]
        response = {
            "preview_transport": "pixel_streaming_2",
            "pixel_streaming_player_port": config.pixel_streaming_player_port,
            "pixel_streaming_streamer_id": config.pixel_streaming_streamer_id,
            "pixel_streaming_signalling_url": config.pixel_streaming_signalling_url,
            "pixel_streaming_streamers": streamers,
        }
        if config.stream_access_jwt_secret:
            response["pixel_streaming_access_token"] = issue_stream_access_token(
                config.stream_access_jwt_secret,
                (item["id"] for item in streamers),
                ttl_seconds=config.stream_access_token_ttl_seconds,
            )
        return response

    @app.get("/api/state")
    async def state() -> dict[str, Any]:
        action = safety.last_action
        return {
            "simulation": hub.status(),
            "active_episode": recorder.episode_id,
            "episode_directory": str(recorder.episode_directory) if recorder.episode_directory else None,
            "latest_action": action.model_dump(mode="json") if action else None,
            "cameras": captures.camera_ids(),
            "capture_channels": captures.status(),
            "capture_sync": recorder.sync_status(),
        }

    @app.post("/api/episodes/start")
    async def start_episode(request: EpisodeStart) -> dict[str, Any]:
        try:
            return recorder.start(request)
        except RuntimeError as error:
            raise HTTPException(status_code=409, detail=str(error)) from error

    @app.post("/api/episodes/stop")
    async def stop_episode(request: EpisodeStop) -> dict[str, Any]:
        neutral = safety.neutral(recorder.episode_id, "episode_stopped")
        recorder.record_action(neutral)
        await hub.publish_action(neutral)
        try:
            result = recorder.stop(request)
            result["archive_job"] = jobs.submit_archive(result["episode_id"])
            return result
        except RuntimeError as error:
            raise HTTPException(status_code=409, detail=str(error)) from error

    @app.get("/api/jobs")
    async def list_jobs() -> dict[str, Any]:
        return {"jobs": jobs.list()}

    @app.get("/api/jobs/{job_id}")
    async def get_job(job_id: str) -> dict[str, Any]:
        job = jobs.get(job_id)
        if job is None:
            raise HTTPException(status_code=404, detail="job not found")
        return job

    @app.post("/api/episodes/{episode_id}/archive")
    async def archive_episode(episode_id: str) -> dict[str, Any]:
        return jobs.submit_archive(episode_id)

    @app.post("/api/tasks")
    async def create_task(request: TaskCreate) -> dict[str, Any]:
        return tasks.create(request)

    @app.get("/api/tasks")
    async def list_tasks() -> dict[str, Any]:
        return {"tasks": tasks.list()}

    @app.post("/api/tasks/{task_id}/start")
    async def start_task(task_id: str) -> dict[str, Any]:
        task = tasks.get(task_id)
        if task is None:
            raise HTTPException(status_code=404, detail="task not found")
        if task["status"] != "queued":
            raise HTTPException(status_code=409, detail=f"task {task_id} is {task['status']}, expected queued")
        try:
            episode = recorder.start(EpisodeStart(
                task_id=task_id,
                task="scheduled space manipulator task",
                instruction=task["instruction"],
                seed=task["seed"],
                tags=task["tags"],
            ))
            return tasks.transition(task_id, {"queued"}, "running", episode_id=episode["episode_id"])
        except RuntimeError as error:
            raise HTTPException(status_code=409, detail=str(error)) from error

    @app.post("/api/tasks/{task_id}/complete")
    async def complete_task(task_id: str, request: TaskComplete) -> dict[str, Any]:
        task = tasks.get(task_id)
        if task is None:
            raise HTTPException(status_code=404, detail="task not found")
        if recorder.episode_id != task.get("episode_id"):
            raise HTTPException(status_code=409, detail="task does not own the active episode")
        neutral = safety.neutral(recorder.episode_id, "task_completed")
        recorder.record_action(neutral)
        await hub.publish_action(neutral)
        try:
            closed = recorder.stop(EpisodeStop(outcome=request.outcome, note=request.note))
            archive_job = jobs.submit_archive(closed["episode_id"])
            return tasks.transition(
                task_id, {"running"}, "completed", outcome=request.outcome, archive_job_id=archive_job["job_id"]
            )
        except RuntimeError as error:
            raise HTTPException(status_code=409, detail=str(error)) from error

    @app.get("/api/cameras")
    async def cameras() -> dict[str, Any]:
        return {"cameras": captures.camera_ids()}

    @app.get("/api/preview/{camera_id:path}")
    async def preview(camera_id: str) -> Response:
        frame = captures.latest(camera_id)
        if frame is None:
            raise HTTPException(status_code=404, detail="camera has not produced an RGB frame")
        return Response(frame.data, media_type=frame.content_type, headers={"Cache-Control": "no-store"})

    @app.get("/api/preview-stream")
    async def preview_stream(camera: str | None = None) -> StreamingResponse:
        async def frames() -> AsyncIterator[bytes]:
            revision = 0
            while True:
                frame = await asyncio.to_thread(captures.wait_for_frame, camera, revision, 2.0)
                if frame is None:
                    await asyncio.sleep(0.02)
                    continue
                revision = frame.revision
                yield (
                    b"--frame\r\n"
                    + f"Content-Type: {frame.content_type}\r\nContent-Length: {len(frame.data)}\r\n\r\n".encode("ascii")
                    + frame.data
                    + b"\r\n"
                )

        return StreamingResponse(
            frames(),
            media_type="multipart/x-mixed-replace; boundary=frame",
            headers={"Cache-Control": "no-store"},
        )

    @app.websocket("/ws/operator")
    async def operator_socket(websocket: WebSocket) -> None:
        if config.stream_access_key and not hmac.compare_digest(
            websocket.query_params.get("access_key", ""), config.stream_access_key
        ):
            await websocket.close(code=4401, reason="invalid access key")
            return
        identifier, granted = await sessions.connect(websocket)
        send_task: asyncio.Task[None] | None = None

        async def send_observations() -> None:
            revision = 0
            while True:
                revision, observation = await hub.wait_for_observation(revision)
                if observation:
                    await websocket.send_json(
                        {"type": "observation", "payload": observation.model_dump(mode="json")}
                    )

        try:
            await websocket.send_json(
                {
                    "type": "session",
                    "operator_id": identifier,
                    "control_granted": granted,
                    "simulation_connected": hub.connected,
                    "active_episode": recorder.episode_id,
                    "control_limits": {
                        "linear_speed_min_m_s": safety.LINEAR_SPEED_MIN_M_S,
                        "linear_speed_default_m_s": safety.LINEAR_SPEED_DEFAULT_M_S,
                        "linear_speed_max_m_s": safety.LINEAR_SPEED_MAX_M_S,
                    },
                }
            )
            send_task = asyncio.create_task(send_observations())
            while True:
                raw = await websocket.receive_json()
                if raw.get("type") == "ping":
                    await websocket.send_json({"type": "pong", "server_time_ns": str(time.time_ns())})
                    continue
                if not sessions.is_owner(identifier):
                    await websocket.send_json({"type": "action_rejected", "reason": "view_only"})
                    continue
                try:
                    request = OperatorAction.model_validate(raw)
                    action = safety.process(identifier, request, recorder.episode_id)
                except (ValidationError, ActionRejected) as error:
                    await websocket.send_json({"type": "action_rejected", "reason": str(error)})
                    continue
                recorder.record_action(action)
                delivered = await hub.publish_action(action)
                await websocket.send_json(
                    {
                        "type": "action_ack",
                        "server_sequence": action.server_sequence,
                        "delivered_to_simulation": delivered,
                        "limited": action.limited,
                    }
                )
        except WebSocketDisconnect:
            pass
        finally:
            if send_task:
                send_task.cancel()
            released, promoted = await sessions.disconnect(identifier)
            if released:
                neutral = safety.neutral(recorder.episode_id, "operator_disconnected")
                recorder.record_action(neutral)
                await hub.publish_action(neutral)
            if promoted:
                promoted_id, promoted_socket = promoted
                with contextlib.suppress(RuntimeError, WebSocketDisconnect):
                    await promoted_socket.send_json(
                        {
                            "type": "control_granted",
                            "operator_id": promoted_id,
                            "control_granted": True,
                            "message": "Previous operator disconnected; control transferred to this session.",
                        }
                    )

    return app
