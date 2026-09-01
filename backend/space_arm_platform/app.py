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
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import ValidationError

from .auth import AuthError, AuthStore, SESSION_COOKIE
from .capture_receiver import CaptureReceiver
from .jobs import JobManager
from .models import (
    EpisodeStart, EpisodeStop, LoginRequest, OperatorAction, OperatorCreate,
    PasswordChange, PasswordReset, SceneInstanceCreate, SimulationObservation, TaskComplete, TaskCreate,
)
from .recorder import EpisodeRecorder
from .safety import ActionRejected, SafetyController
from .scene_runtime import SceneLaunchConfig, SceneRuntimeManager
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
    runtime_adapter_root: Path | None = None
    runtime_model_root: Path | None = None
    runtime_unreal_root: Path | None = None
    runtime_powershell_exe: Path | None = None
    runtime_render_port: int = 5558
    runtime_pixel_streamer_port: int = 8888
    runtime_pixel_streaming_camera_ids: tuple[str, ...] = ()
    runtime_pixel_streaming_camera_width: int = 640
    runtime_pixel_streaming_camera_height: int = 360
    runtime_preview_rate: float = 60.0
    runtime_renderer_ready_timeout: int = 240
    runtime_ik_rate: float = 100.0
    runtime_simulation_rate: float = 1.0
    runtime_capture_rate: float = 10.0
    runtime_default_duration: float = 300.0
    runtime_default_dataset_capture: bool = False
    auth_database: Path | None = None
    bootstrap_admin_username: str = "admin"
    bootstrap_admin_password: str = "ChangeMe123!"


class OperatorSessions:
    """Track authenticated browser tabs; one tab at a time emits commands."""

    def __init__(self) -> None:
        self.clients: dict[str, WebSocket] = {}
        self.users: dict[str, dict[str, Any]] = {}
        self.active_operator: str | None = None
        self._lock = asyncio.Lock()

    async def connect(self, websocket: WebSocket, user: dict[str, Any]) -> tuple[str, bool]:
        await websocket.accept()
        identifier = f"operator-{uuid.uuid4().hex[:8]}"
        async with self._lock:
            self.clients[identifier] = websocket
            self.users[identifier] = user
            granted = self.active_operator is None
            if granted:
                self.active_operator = identifier
        return identifier, granted

    async def activate(self, identifier: str) -> tuple[str, WebSocket] | None:
        async with self._lock:
            if identifier not in self.clients:
                return None
            previous = self.active_operator
            self.active_operator = identifier
            if previous and previous != identifier and previous in self.clients:
                return previous, self.clients[previous]
            return None

    async def disconnect(self, identifier: str) -> bool:
        async with self._lock:
            self.clients.pop(identifier, None)
            self.users.pop(identifier, None)
            released = self.active_operator == identifier
            if released:
                self.active_operator = None
            return released

    def is_owner(self, identifier: str) -> bool:
        return self.active_operator == identifier


def create_app(config: PlatformConfig | None = None) -> FastAPI:
    project_root = Path(__file__).resolve().parents[2]
    config = config or PlatformConfig(
        project_root=project_root,
        data_root=project_root / "data" / "episodes",
    )
    recorder = EpisodeRecorder(config.data_root)
    auth = AuthStore(
        config.auth_database or config.data_root.parent / "auth.sqlite3",
        config.bootstrap_admin_username,
        config.bootstrap_admin_password,
    )
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
    launch_config = None
    if all((config.runtime_adapter_root, config.runtime_model_root, config.runtime_unreal_root, config.runtime_powershell_exe)):
        launch_config = SceneLaunchConfig(
            project_root=config.project_root, adapter_root=config.runtime_adapter_root,
            model_root=config.runtime_model_root, unreal_root=config.runtime_unreal_root,
            powershell_exe=config.runtime_powershell_exe, control_port=config.simulation_port,
            capture_port=config.capture_port, render_port=config.runtime_render_port,
            pixel_streamer_port=config.runtime_pixel_streamer_port,
            pixel_streaming_id=config.pixel_streaming_streamer_id,
            pixel_streaming_camera_ids=config.runtime_pixel_streaming_camera_ids,
            pixel_streaming_camera_width=config.runtime_pixel_streaming_camera_width,
            pixel_streaming_camera_height=config.runtime_pixel_streaming_camera_height,
            preview_rate=config.runtime_preview_rate, renderer_ready_timeout=config.runtime_renderer_ready_timeout,
            ik_rate=config.runtime_ik_rate, simulation_rate=config.runtime_simulation_rate,
            capture_rate=config.runtime_capture_rate, default_duration=config.runtime_default_duration,
            default_dataset_capture=config.runtime_default_dataset_capture,
        )
    scenes = SceneRuntimeManager(launch_config, project_root=config.project_root)
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
            await asyncio.to_thread(scenes.close)
            captures.close()
            await hub.close()
            jobs.close()
            auth.close()

    app = FastAPI(title="Space Arm Data Platform", version="0.2.0", lifespan=lifespan)
    app.state.config = config
    app.state.auth = auth
    app.state.recorder = recorder
    app.state.safety = safety
    app.state.hub = hub
    app.state.captures = captures
    app.state.sessions = sessions
    app.state.jobs = jobs
    app.state.tasks = tasks
    app.state.scenes = scenes

    frontend_source = config.project_root / "frontend"
    frontend_dist = frontend_source / "dist"
    frontend = frontend_dist if (frontend_dist / "index.html").is_file() else frontend_source
    app.mount("/static", StaticFiles(directory=frontend), name="static")

    def current_user(request: Request) -> dict[str, Any]:
        user = getattr(request.state, "user", None)
        if not user:
            raise HTTPException(status_code=401, detail="请先登录")
        return user

    def require_admin(request: Request) -> dict[str, Any]:
        user = current_user(request)
        if user["role"] != "admin":
            raise HTTPException(status_code=403, detail="仅管理员可以管理操作员")
        return user

    def can_manage_scene(user: dict[str, Any], status: dict[str, Any]) -> bool:
        owner = (status.get("instance") or {}).get("created_by") or {}
        return user["role"] == "admin" or owner.get("user_id") == user["user_id"]

    @app.middleware("http")
    async def authenticated_api(request: Request, call_next):
        path = request.url.path
        if path.startswith("/api/") and path not in {"/api/health", "/api/auth/login"}:
            user = auth.session_user(request.cookies.get(SESSION_COOKIE))
            if not user:
                return JSONResponse({"detail": "请先登录"}, status_code=401)
            request.state.user = user
        return await call_next(request)

    @app.post("/api/auth/login")
    async def login(payload: LoginRequest) -> JSONResponse:
        user = await asyncio.to_thread(auth.authenticate, payload.username, payload.password)
        if not user:
            raise HTTPException(status_code=401, detail="用户名或密码错误")
        token, max_age = await asyncio.to_thread(auth.create_session, user["user_id"])
        response = JSONResponse({"user": user})
        response.set_cookie(
            SESSION_COOKIE, token, max_age=max_age, httponly=True, samesite="strict",
            secure=False, path="/",
        )
        return response

    @app.get("/api/auth/me")
    async def auth_me(request: Request) -> dict[str, Any]:
        return {"user": current_user(request)}

    @app.post("/api/auth/logout")
    async def logout(request: Request) -> JSONResponse:
        await asyncio.to_thread(auth.delete_session, request.cookies.get(SESSION_COOKIE))
        response = JSONResponse({"ok": True})
        response.delete_cookie(SESSION_COOKIE, path="/")
        return response

    @app.post("/api/auth/change-password")
    async def change_password(payload: PasswordChange, request: Request) -> JSONResponse:
        user = current_user(request)
        try:
            await asyncio.to_thread(
                auth.change_password, user["user_id"], payload.current_password, payload.new_password
            )
        except AuthError as error:
            raise HTTPException(status_code=422, detail=str(error)) from error
        response = JSONResponse({"ok": True})
        response.delete_cookie(SESSION_COOKIE, path="/")
        return response

    @app.get("/api/users")
    async def list_users(request: Request) -> dict[str, Any]:
        require_admin(request)
        return {"users": await asyncio.to_thread(auth.list_users)}

    @app.post("/api/users/operators")
    async def create_operator(payload: OperatorCreate, request: Request) -> dict[str, Any]:
        require_admin(request)
        try:
            return await asyncio.to_thread(auth.create_operator, payload.username, payload.password)
        except AuthError as error:
            raise HTTPException(status_code=422, detail=str(error)) from error

    @app.delete("/api/users/operators/{user_id}")
    async def delete_operator(user_id: str, request: Request) -> dict[str, Any]:
        require_admin(request)
        try:
            await asyncio.to_thread(auth.delete_operator, user_id)
            return {"ok": True}
        except AuthError as error:
            raise HTTPException(status_code=422, detail=str(error)) from error

    @app.post("/api/users/operators/{user_id}/reset-password")
    async def reset_operator_password(user_id: str, payload: PasswordReset, request: Request) -> dict[str, Any]:
        require_admin(request)
        try:
            await asyncio.to_thread(auth.reset_operator_password, user_id, payload.password)
            return {"ok": True}
        except AuthError as error:
            raise HTTPException(status_code=422, detail=str(error)) from error

    @app.get("/")
    async def index() -> FileResponse:
        return FileResponse(frontend / "index.html")

    @app.get("/api/health")
    async def health() -> dict[str, Any]:
        # Kept public for the process launcher; operational state is available
        # only from authenticated endpoints.
        return {"ok": True, "server_time_ns": str(time.time_ns()), "authentication": "required"}

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
            "scene_runtime": scenes.status(),
        }

    @app.get("/api/scenes/catalog")
    async def scene_catalog() -> dict[str, Any]:
        return scenes.catalog()

    @app.get("/api/scenes/runtime")
    async def scene_runtime() -> dict[str, Any]:
        return scenes.status()

    @app.post("/api/scenes/instances")
    async def create_scene_instance(payload: SceneInstanceCreate, request: Request) -> dict[str, Any]:
        user = current_user(request)
        creator = {"user_id": user["user_id"], "username": user["username"], "role": user["role"]}
        try:
            return await asyncio.to_thread(scenes.create_instance, payload, creator)
        except ValueError as error:
            raise HTTPException(status_code=422, detail=str(error)) from error

    @app.post("/api/scenes/start")
    async def start_scene(payload: SceneInstanceCreate, request: Request) -> dict[str, Any]:
        user = current_user(request)
        if recorder.episode_id:
            raise HTTPException(status_code=409, detail="stop the active episode before starting another scene")
        creator = {"user_id": user["user_id"], "username": user["username"], "role": user["role"]}
        try:
            return await asyncio.to_thread(scenes.start, payload, creator)
        except ValueError as error:
            raise HTTPException(status_code=422, detail=str(error)) from error
        except RuntimeError as error:
            raise HTTPException(status_code=409, detail=str(error)) from error

    @app.post("/api/scenes/stop")
    async def stop_scene(request: Request) -> dict[str, Any]:
        user = current_user(request)
        status = scenes.status()
        if status.get("instance") and not can_manage_scene(user, status):
            raise HTTPException(status_code=403, detail="只能关闭自己创建的场景")
        episode_result = None
        if recorder.episode_id:
            neutral = safety.neutral(recorder.episode_id, "scene_stopped")
            recorder.record_action(neutral)
            await hub.publish_action(neutral)
            episode_result = recorder.stop(EpisodeStop(outcome="aborted", note="scene stopped"))
            episode_result["archive_job"] = jobs.submit_archive(episode_result["episode_id"])
        try:
            result = await asyncio.to_thread(scenes.stop)
            if episode_result:
                result["aborted_episode"] = episode_result
            return result
        except RuntimeError as error:
            raise HTTPException(status_code=409, detail=str(error)) from error

    @app.post("/api/episodes/start")
    async def start_episode(payload: EpisodeStart, request: Request) -> dict[str, Any]:
        user = current_user(request)
        scene_status = scenes.status()
        if scenes.enabled and scene_status.get("phase") != "running":
            raise HTTPException(status_code=409, detail="a running scene is required before data collection")
        if scene_status.get("instance") and not can_manage_scene(user, scene_status):
            raise HTTPException(status_code=403, detail="只能采集自己创建的场景")
        scene_instance = scene_status.get("instance")
        if scene_instance:
            payload = payload.model_copy(
                update={
                    "seed": payload.seed if payload.seed is not None else scene_instance.get("seed"),
                    "scene_instance": scene_instance,
                    "operator": user["username"],
                    "operator_user_id": user["user_id"],
                    "operator_username": user["username"],
                    "operator_role": user["role"],
                }
            )
        try:
            return recorder.start(payload)
        except RuntimeError as error:
            raise HTTPException(status_code=409, detail=str(error)) from error

    @app.post("/api/episodes/stop")
    async def stop_episode(payload: EpisodeStop, request: Request) -> dict[str, Any]:
        user = current_user(request)
        scene_status = scenes.status()
        if scene_status.get("instance") and not can_manage_scene(user, scene_status):
            raise HTTPException(status_code=403, detail="只能结束自己场景的数据采集")
        neutral = safety.neutral(recorder.episode_id, "episode_stopped")
        recorder.record_action(neutral)
        await hub.publish_action(neutral)
        try:
            result = recorder.stop(payload)
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
        session_token = websocket.cookies.get(SESSION_COOKIE)
        user = auth.session_user(session_token)
        if not user:
            await websocket.close(code=4401, reason="login required")
            return
        if config.stream_access_key and not hmac.compare_digest(
            websocket.query_params.get("access_key", ""), config.stream_access_key
        ):
            await websocket.close(code=4401, reason="invalid access key")
            return
        identifier, granted = await sessions.connect(websocket, user)
        send_task: asyncio.Task[None] | None = None
        auth_task: asyncio.Task[None] | None = None

        async def validate_session() -> None:
            while True:
                await asyncio.sleep(2.0)
                if not auth.session_user(session_token):
                    await websocket.close(code=4401, reason="session expired")
                    return

        async def send_observations() -> None:
            revision = 0
            while True:
                revision, observation = await hub.wait_for_observation(revision)
                if observation:
                    await websocket.send_json(
                        {"type": "observation", "payload": observation.model_dump(mode="json")}
                    )

        def user_controls_current_scene() -> bool:
            status = scenes.status()
            return status.get("phase") == "running" and can_manage_scene(user, status)

        try:
            await websocket.send_json(
                {
                    "type": "session",
                    "operator_id": identifier,
                    "control_granted": granted,
                    "user": user,
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
            auth_task = asyncio.create_task(validate_session())
            while True:
                raw = await websocket.receive_json()
                message_type = raw.get("type")
                if message_type == "ping":
                    await websocket.send_json({"type": "pong", "server_time_ns": str(time.time_ns())})
                    continue
                if message_type == "activate_control":
                    if not user_controls_current_scene():
                        await websocket.send_json({"type": "action_rejected", "reason": "not_scene_owner"})
                        continue
                    previous = await sessions.activate(identifier)
                    if previous:
                        neutral = safety.neutral(recorder.episode_id, "control_page_changed")
                        recorder.record_action(neutral)
                        await hub.publish_action(neutral)
                        _, previous_socket = previous
                        with contextlib.suppress(RuntimeError, WebSocketDisconnect):
                            await previous_socket.send_json(
                                {"type": "control_revoked", "reason": "another_page_activated"}
                            )
                    await websocket.send_json(
                        {"type": "control_granted", "operator_id": identifier, "control_granted": True}
                    )
                    continue
                if not sessions.is_owner(identifier):
                    await websocket.send_json({"type": "action_rejected", "reason": "inactive_page"})
                    continue
                if not user_controls_current_scene():
                    await websocket.send_json({"type": "action_rejected", "reason": "not_scene_owner"})
                    continue
                try:
                    action_request = OperatorAction.model_validate(raw)
                    action = safety.process(identifier, action_request, recorder.episode_id)
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
            if auth_task:
                auth_task.cancel()
            released = await sessions.disconnect(identifier)
            if released:
                neutral = safety.neutral(recorder.episode_id, "operator_disconnected")
                recorder.record_action(neutral)
                await hub.publish_action(neutral)

    return app
