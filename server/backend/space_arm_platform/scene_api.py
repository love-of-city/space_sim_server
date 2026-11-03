"""Scene creation, ownership, reset and controlled shutdown routes."""
from __future__ import annotations

import asyncio
import logging
from typing import Any

from fastapi import HTTPException, Request

from .auth_api import current_user, can_manage_scene
from .models import EpisodeStop, SceneInstanceCreate


def install_scene_routes(app, *, scenes, capture_defaults, recording_config, policy, recorder,
                         capture_lifecycle, sessions, hub, safety, jobs):
    @app.get("/api/scenes/catalog")
    async def scene_catalog() -> dict[str, Any]:
        catalog = scenes.catalog()
        catalog["defaults"].update(capture_defaults)
        return catalog

    @app.get("/api/recording/config")
    async def recording_settings() -> dict[str, Any]:
        return {**recording_config.model_dump(mode="json"), **capture_defaults}

    def scene_recording_defaults(payload: SceneInstanceCreate) -> SceneInstanceCreate:
        return payload.model_copy(update={key: value for key, value in capture_defaults.items()
                                          if key not in payload.model_fields_set})

    @app.get("/api/scenes/runtime")
    async def scene_runtime() -> dict[str, Any]:
        return scenes.status()

    @app.post("/api/scenes/instances")
    async def create_scene_instance(payload: SceneInstanceCreate, request: Request) -> dict[str, Any]:
        user = current_user(request)
        creator = {"user_id": user["user_id"], "username": user["username"], "role": user["role"]}
        try:
            return await asyncio.to_thread(scenes.create_instance, scene_recording_defaults(payload), creator)
        except ValueError as error:
            raise HTTPException(status_code=422, detail=str(error)) from error

    @app.post("/api/scenes/start")
    async def start_scene(payload: SceneInstanceCreate, request: Request) -> dict[str, Any]:
        policy.require_free()
        user = current_user(request)
        if recorder.episode_id or capture_lifecycle.busy:
            raise HTTPException(status_code=409, detail="stop the active episode before starting another scene")
        creator = {"user_id": user["user_id"], "username": user["username"], "role": user["role"]}
        try:
            return await asyncio.to_thread(scenes.start, scene_recording_defaults(payload), creator)
        except ValueError as error:
            raise HTTPException(status_code=422, detail=str(error)) from error
        except RuntimeError as error:
            raise HTTPException(status_code=409, detail=str(error)) from error

    @app.post("/api/scenes/reset")
    async def reset_scene(request: Request) -> dict[str, Any]:
        policy.require_free()
        user = current_user(request)
        status = scenes.status()
        if not can_manage_scene(user, status):
            raise HTTPException(status_code=403, detail="只能重置自己创建的场景")
        if status.get("phase") != "running":
            raise HTTPException(status_code=409, detail="场景尚未运行")
        if recorder.episode_id or capture_lifecycle.busy:
            raise HTTPException(status_code=409, detail="请先结束当前采集，再重置状态")
        try:
            request_id = hub.begin_reset()
        except RuntimeError as error:
            raise HTTPException(status_code=409, detail=str(error)) from error
        safety.neutral(None, "scene_reset")
        await sessions.suspend_control()
        try:
            return await hub.finish_reset(request_id)
        except RuntimeError as error:
            raise HTTPException(status_code=503, detail=str(error)) from error

    @app.post("/api/scenes/stop")
    async def stop_scene(request: Request) -> dict[str, Any]:
        user = current_user(request)
        status = scenes.status()
        if status.get("instance") and not can_manage_scene(user, status):
            raise HTTPException(status_code=403, detail="只能关闭自己创建的场景")
        if capture_lifecycle.busy:
            raise HTTPException(status_code=409, detail="采集正在启动或收尾，请稍后停止场景")
        try:
            await policy.shutdown()
        except RuntimeError:
            # Explicit scene STOP must remain available after an uncertain
            # policy release. Ownership is cleared only after process stop.
            logging.getLogger(__name__).exception("Stopping scene with unconfirmed policy release")
        hub.cancel_reset()
        episode_result = None
        if recorder.episode_id:
            neutral = safety.neutral(recorder.episode_id, "scene_stopped")
            await asyncio.to_thread(recorder.record_action, neutral)
            await hub.publish_action(neutral)
            episode_result = await capture_lifecycle.stop(EpisodeStop(outcome="aborted", note="scene stopped"))
            if episode_result["dataset_status"] == "complete":
                episode_result["archive_job"] = jobs.submit_archive(episode_result["episode_id"])
        try:
            result = await asyncio.to_thread(scenes.stop)
            policy.scene_stopped()
            if episode_result:
                result["aborted_episode"] = episode_result
            return result
        except RuntimeError as error:
            raise HTTPException(status_code=409, detail=str(error)) from error
