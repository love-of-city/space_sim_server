"""FastAPI application joining operators, simulator, captures and episodes."""

from __future__ import annotations

import asyncio
import contextlib
import time
import uuid
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import ValidationError

from .capture_receiver import CaptureReceiver
from .models import EpisodeStart, EpisodeStop, OperatorAction, SimulationObservation
from .recorder import EpisodeRecorder
from .safety import ActionRejected, SafetyController
from .simulation_hub import SimulationHub


@dataclass(frozen=True)
class PlatformConfig:
    project_root: Path
    data_root: Path
    simulation_host: str = "127.0.0.1"
    simulation_port: int = 8766
    capture_host: str = "127.0.0.1"
    capture_port: int = 8767
    deadman_timeout_s: float = 0.25


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

    app = FastAPI(title="Space Arm Data Platform", version="0.1.0", lifespan=lifespan)
    app.state.config = config
    app.state.recorder = recorder
    app.state.safety = safety
    app.state.hub = hub
    app.state.captures = captures
    app.state.sessions = sessions

    frontend = config.project_root / "frontend"
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
            return recorder.stop(request)
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
