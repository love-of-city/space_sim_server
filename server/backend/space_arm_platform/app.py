"""Assemble backend services, transports, lifecycle and business route modules."""
from __future__ import annotations

import asyncio
import contextlib
import logging
import sys
import time
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from functools import partial
from pathlib import Path
from typing import Any

from fastapi import FastAPI
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from .auth import AuthStore
from .auth_api import current_user, install_auth_routes
from .capture_lifecycle import CaptureLifecycle
from .capture_receiver import CaptureReceiver
from .collection_api import install_collection_routes
from .episode_api import install_episode_routes, recording_request
from .jobs import JobManager
from .mission_api import install_mission_routes
from .mission_runtime import MissionRuntime
from .rendezvous_hub import RendezvousHub
from .rendezvous_runtime import RendezvousRuntime
from .vision_api import install_vision_routes
from .vision_service import StereoVisionService
from .wrist_vision_service import WristVisionService
from .media_api import install_media_routes
from .models import EpisodeStop, SimulationObservation
from .operator_api import install_operator_routes
from .operator_sessions import OperatorSessions
from .platform_config import PlatformConfig
from .policy_api import PolicySessions, install_policy_routes
from .policy_observation import PolicyObservationCache
from .recorder import EpisodeRecorder
from .recording_config import load_recording_config
from .safety import SafetyController
from .scene_api import install_scene_routes
from .scene_runtime import SceneLaunchConfig, SceneRuntimeManager
from .simulation_hub import SimulationHub
from .tasks import TaskStore
from .web_security import LoginRateLimiter, validate_origins


def create_app(config: PlatformConfig | None = None) -> FastAPI:
    project_root = Path(__file__).resolve().parents[2]
    config = config or PlatformConfig(
        project_root=project_root,
        data_root=project_root / "data" / "episodes",
    )
    allowed_origins = validate_origins(config.allowed_origins)
    login_limiter = LoginRateLimiter(config.login_attempts_per_minute)
    recording_config = load_recording_config(config.project_root / "config" / "recording.json")
    recorder = EpisodeRecorder(config.data_root, save_raw_camera_images=recording_config.save_raw_camera_images,
                               video_encoding=recording_config.video, storage_profile=recording_config.storage_profile)
    # Explicit file settings take precedence over legacy launcher defaults.
    # Explicit scene requests and saved instances retain their own sampling grid.
    capture_defaults = {
        "capture_rate_hz": (recording_config.capture_rate_hz if "capture_rate_hz" in recording_config.model_fields_set
                            else config.runtime_capture_rate),
        "dataset_capture": (recording_config.dataset_capture if "dataset_capture" in recording_config.model_fields_set
                            else config.runtime_default_dataset_capture),
    }
    auth = AuthStore(
        config.auth_database or config.data_root.parent / "auth.sqlite3",
        config.bootstrap_admin_username,
        config.bootstrap_admin_password,
    )
    if config.secure_cookies:
        # Bootstrap flags do not update an existing database. Fail closed if an
        # administrator still uses the documented local-development password.
        for user in auth.list_users():
            if user["role"] == "admin" and auth.authenticate(user["username"], "ChangeMe123!"):
                auth.close()
                raise ValueError("Change the existing default administrator password before HTTPS deployment")
    jobs = JobManager(config.data_root.parent)
    tasks = TaskStore(config.data_root.parent / "tasks")
    safety = SafetyController(config.deadman_timeout_s)
    hub = SimulationHub()
    rendezvous = RendezvousHub()
    rendezvous_runtime = RendezvousRuntime(
        config.rendezvous_python, config.rendezvous_port, config.project_root / "logs/rendezvous",
        team_repository=config.project_root, render_port=config.runtime_render_port, launch_config=config)
    vision = StereoVisionService(config.project_root)
    wrist_vision = WristVisionService(config.project_root)
    capture_lifecycle = CaptureLifecycle(hub, recorder)
    sessions = OperatorSessions()
    policy_cache = PolicyObservationCache()

    def record_current_capture(metadata: dict[str, Any], products: dict[str, bytes]) -> None:
        # Capture transport can drain old packets after physics has reset.
        # Frame IDs and simulation time both restart, so session identity matters.
        if hub.resetting:
            return
        observation = hub.latest_observation
        if (observation and observation.render_session_id
                and metadata.get("session_id") != observation.render_session_id):
            return
        policy_cache.record_capture(metadata, products)
        recorder.record_authoritative_capture(metadata, products)

    def capture_failed(reason: str) -> None:
        policy_cache.fail(reason)
        recorder.fail(reason)

    def accept_vision_capture(metadata, products):
        observation = hub.latest_observation
        if hub.resetting or not observation or not observation.render_session_id:
            return
        if metadata.get("session_id") != observation.render_session_id:
            return
        # Each consumer has its own estimator queue; errors remain diagnostics.
        for service in (vision, wrist_vision):
            try:
                service.on_capture(metadata, products)
            except Exception:
                logging.getLogger(__name__).exception("Vision capture rejected")

    captures = CaptureReceiver(
        config.capture_host,
        config.capture_port,
        record_current_capture,
        capture_failed,
        on_vision_capture=accept_vision_capture,
    )
    launch_config = None
    if all((config.runtime_adapter_root, config.runtime_model_root, config.runtime_unreal_root)) and (
        sys.platform.startswith("linux") or config.runtime_powershell_exe
    ):
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
            encoder_min_quality=config.runtime_encoder_min_quality,
            ik_rate=config.runtime_ik_rate, simulation_rate=config.runtime_simulation_rate,
            capture_rate=capture_defaults["capture_rate_hz"],
            default_dataset_capture=capture_defaults["dataset_capture"],
            python_exe=Path(sys.executable) if sys.platform.startswith("linux") else None,
        )
    scenes = SceneRuntimeManager(launch_config, project_root=config.project_root)
    missions = MissionRuntime(config.data_root / ".missions", scenes, rendezvous_runtime, rendezvous,
        lambda: bool(recorder.episode_id or capture_lifecycle.busy or policy.active), scene_defaults=capture_defaults)
    timeout_task: asyncio.Task[None] | None = None
    capture_failure_task: asyncio.Task | None = None
    policy_watchdog_task: asyncio.Task | None = None

    async def record_observation(observation: SimulationObservation) -> None:
        policy_cache.record_observation(observation)
        safety.gripper_type = observation.gripper_type
        await asyncio.to_thread(recorder.record_observation, observation, safety.last_action)
        if hub.capture_pair_ack_supported:
            await asyncio.to_thread(recorder.wait_for_capture_pair, observation)
        try:
            await asyncio.to_thread(policy_cache.wait_for_pair, observation)
        except RuntimeError as error:
            policy_cache.fail(str(error))

    hub.on_observation = record_observation
    hub.on_transport_error = capture_failed

    async def watchdog() -> None:
        nonlocal capture_failure_task
        while True:
            await asyncio.sleep(0.05)
            status = recorder.sync_status()
            if (not policy.active and recorder.episode_id and status['dataset_error'] and not status['finalizing']
                    and not capture_lifecycle.busy
                    and (capture_failure_task is None or capture_failure_task.done())):
                # Do not leave a failed recorder asking UE to generate strict
                # images forever. Freeze cutoff, send OFF, drain accepted work.
                # Run independently so the safety/control watchdog never waits
                # for disk finalization. A failed episode is never published as valid.
                async def stop_failed_capture(failed_episode_id=recorder.episode_id):
                    if not failed_episode_id:
                        return
                    try:
                        await capture_lifecycle.stop(EpisodeStop(
                            outcome='aborted', note='capture pipeline failed; simulation preserved'),
                            failed_episode_id=failed_episode_id)
                    except Exception:
                        logging.getLogger(__name__).exception('Failed capture shutdown')
                capture_failure_task = asyncio.create_task(stop_failed_capture())
            if policy.active:
                continue
            action = safety.timeout_action(recorder.episode_id)
            if action:
                await asyncio.to_thread(recorder.record_action, action)
                await hub.publish_action(action)

    async def policy_watchdog():
        while True:
            await asyncio.sleep(1.0)
            try:
                if policy.active and recorder.episode_id and recorder.sync_status()["dataset_error"]:
                    await policy.shutdown()
                else:
                    await policy.expire_idle()
            except RuntimeError:
                logging.getLogger(__name__).exception("Policy session cleanup failed")

    @asynccontextmanager
    async def lifespan(_: FastAPI) -> AsyncIterator[None]:
        nonlocal timeout_task, policy_watchdog_task
        try:
            await asyncio.to_thread(recorder.prepare)
        except Exception:
            logging.getLogger(__name__).exception("Dataset writer preparation failed; recording remains unavailable until preparation succeeds")
        await hub.start(config.simulation_host, config.simulation_port)
        if config.rendezvous_port:
            await rendezvous.start("127.0.0.1", config.rendezvous_port)
        vision.start()
        wrist_vision.start()
        captures.start()
        timeout_task = asyncio.create_task(watchdog(), name="deadman-watchdog")
        policy_watchdog_task = asyncio.create_task(policy_watchdog(), name="policy-watchdog")
        try:
            yield
        finally:
            if policy_watchdog_task:
                policy_watchdog_task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await policy_watchdog_task
            try:
                await policy.shutdown()
            except RuntimeError:
                logging.getLogger(__name__).exception("Policy release unconfirmed during backend shutdown")
            if timeout_task:
                timeout_task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await timeout_task
            if capture_failure_task:
                await capture_failure_task
            if recorder.episode_id and not recorder.sync_status()["finalizing"]:
                neutral = safety.neutral(recorder.episode_id, "backend_shutdown")
                await asyncio.to_thread(recorder.record_action, neutral)
                await hub.publish_action(neutral)
                closed = await capture_lifecycle.stop(EpisodeStop(outcome="aborted", note="backend shutdown"))
                if closed["dataset_status"] == "complete":
                    jobs.submit_archive(closed["episode_id"])
            await asyncio.to_thread(collections.close)
            await missions.close()
            await asyncio.to_thread(rendezvous_runtime.close)
            await rendezvous.close()
            await asyncio.to_thread(scenes.close)
            policy.scene_stopped()
            await asyncio.to_thread(captures.close)
            await asyncio.to_thread(vision.close)
            await asyncio.to_thread(wrist_vision.close)
            await asyncio.to_thread(recorder.close)
            await hub.close()
            jobs.close()
            auth.close()

    app = FastAPI(title="Space Arm Data Platform", version="0.2.0", lifespan=lifespan)
    app.state.config = config
    app.state.auth = auth
    app.state.recorder = recorder
    app.state.capture_lifecycle = capture_lifecycle
    app.state.policy_cache = policy_cache
    app.state.safety = safety
    app.state.hub = hub
    app.state.captures = captures
    app.state.sessions = sessions
    app.state.jobs = jobs
    app.state.tasks = tasks
    app.state.scenes = scenes
    app.state.missions = missions
    app.state.rendezvous = rendezvous
    app.state.rendezvous_runtime = rendezvous_runtime
    app.state.vision = vision
    app.state.wrist_vision = wrist_vision

    frontend_source = config.project_root / "frontend"
    frontend_dist = frontend_source / "dist"
    frontend = frontend_dist if (frontend_dist / "index.html").is_file() else frontend_source
    app.mount("/static", StaticFiles(directory=frontend), name="static")

    collections = install_collection_routes(app, config, scenes, current_user)
    prepare_recording_request = partial(recording_request, recording_config=recording_config,
                                        capture_defaults=capture_defaults, hub=hub)
    policy = PolicySessions(hub=hub, cache=policy_cache, recorder=recorder,
        capture_lifecycle=capture_lifecycle, sessions=sessions, safety=safety, scenes=scenes,
        recording_request=prepare_recording_request, require_stable=missions.require_stable,
        episode_started=missions.add_episode)
    app.state.policy_sessions = policy
    install_auth_routes(app, config=config, auth=auth, allowed_origins=allowed_origins,
                        login_limiter=login_limiter)
    install_policy_routes(app, policy, current_user)
    stop_scene_executor = install_scene_routes(app, scenes=scenes, capture_defaults=capture_defaults, recording_config=recording_config,
                         policy=policy, recorder=recorder, capture_lifecycle=capture_lifecycle,
                         sessions=sessions, hub=hub, safety=safety, jobs=jobs, missions=missions)
    install_episode_routes(app, policy=policy, hub=hub, scenes=scenes, recorder=recorder, safety=safety,
                           capture_lifecycle=capture_lifecycle, jobs=jobs, tasks=tasks,
                           recording_request=prepare_recording_request, missions=missions)
    install_media_routes(app, config=config, safety=safety, hub=hub, recorder=recorder, captures=captures, scenes=scenes, missions=missions, rendezvous=rendezvous)
    install_operator_routes(app, config=config, allowed_origins=allowed_origins, auth=auth, sessions=sessions,
                            policy=policy, hub=hub, scenes=scenes, recorder=recorder, safety=safety, missions=missions)

    install_mission_routes(app, missions=missions, rendezvous=rendezvous,
        rendezvous_runtime=rendezvous_runtime, policy=policy, stop_scene_executor=stop_scene_executor)
    install_vision_routes(app, vision=vision, wrist_vision=wrist_vision, hub=hub, scenes=scenes)

    @app.get("/")
    @app.get("/collection")
    @app.get("/datasets")
    @app.get("/users")
    async def index() -> FileResponse:
        return FileResponse(frontend / "index.html")

    @app.get("/api/health")
    async def health() -> dict[str, Any]:
        # Kept public for the process launcher; operational state is available
        # only from authenticated endpoints.
        from .protocol import RUNTIME_CAPABILITIES
        return {"ok": True, "server_time_ns": str(time.time_ns()), "authentication": "required",
                "runtime_capabilities": list(RUNTIME_CAPABILITIES)}

    return app
