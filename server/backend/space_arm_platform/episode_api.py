"""Episode recording and scheduled-task routes using the shared capture lifecycle."""
from __future__ import annotations

import asyncio
from typing import Any

from fastapi import HTTPException, Request
from pydantic import ValidationError

from .auth_api import current_user, can_manage_scene
from .models import EpisodeStart, EpisodeStop, TaskCreate, TaskComplete
from .sampling import DEFAULT_CAPTURE_HZ, LIVE_CAPTURE_FPS


def recording_request(payload: EpisodeStart, instance: dict | None, *, recording_config, capture_defaults, hub) -> EpisodeStart:
    explicit_fps = "fps" in payload.model_fields_set
    defaults = {**recording_config.episode_defaults(), "fps": int(capture_defaults["capture_rate_hz"])}
    payload = EpisodeStart.model_validate({**defaults, **payload.model_dump(exclude_unset=True)})
    if instance is None:
        return payload
    if instance.get("arm_preparation_required"):
        from .preparation_planning import preparation_ready_for_recording
        if not preparation_ready_for_recording(instance, hub.status()):
            raise RuntimeError("机械臂尚未实测稳定到达操作姿态，不能开始任务采集")
    runtime = instance.get("runtime", {})
    if payload.camera_ids and not runtime.get("dataset_capture", False):
        raise RuntimeError("当前场景未启用权威相机采集，请启用数据集采集后重新创建场景")
    fps = runtime.get("capture_rate_hz", DEFAULT_CAPTURE_HZ)
    if fps not in LIVE_CAPTURE_FPS:
        raise RuntimeError(f"LeRobot 采样率必须同时对齐动力学和渲染采样：{LIVE_CAPTURE_FPS}")
    if explicit_fps and payload.fps != fps:
        raise RuntimeError("录制 FPS 必须与场景权威采样率一致")
    return EpisodeStart.model_validate({**payload.model_dump(), "fps": int(fps), "scene_instance": instance})


def install_episode_routes(app, *, policy, hub, scenes, recorder, safety, capture_lifecycle,
                           jobs, tasks, recording_request, missions=None):
    @app.post("/api/episodes/start")
    async def start_episode(payload: EpisodeStart, request: Request) -> dict[str, Any]:
        policy.require_free()
        if hub.resetting:
            raise HTTPException(status_code=409, detail="场景正在重置，暂不能开始采集")
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
            if missions:
                missions.require_stable()
            result = await capture_lifecycle.start(recording_request(payload, scene_instance))
            if missions:
                missions.add_episode(result["episode_id"])
            return result
        except ValidationError as error:
            raise HTTPException(status_code=422, detail=str(error)) from error
        except RuntimeError as error:
            raise HTTPException(status_code=409, detail=str(error)) from error

    @app.post("/api/episodes/stop")
    async def stop_episode(payload: EpisodeStop, request: Request) -> dict[str, Any]:
        policy.require_free()
        user = current_user(request)
        scene_status = scenes.status()
        if scene_status.get("instance") and not can_manage_scene(user, scene_status):
            raise HTTPException(status_code=403, detail="只能结束自己场景的数据采集")
        neutral = safety.neutral(recorder.episode_id, "episode_stopped")
        await asyncio.to_thread(recorder.record_action, neutral)
        await hub.publish_action(neutral)
        try:
            result = await capture_lifecycle.stop(payload)
            if result["dataset_status"] == "complete":
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
        policy.require_free()
        if hub.resetting:
            raise HTTPException(status_code=409, detail="场景正在重置，暂不能开始任务")
        task = tasks.get(task_id)
        if task is None:
            raise HTTPException(status_code=404, detail="task not found")
        if task["status"] != "queued":
            raise HTTPException(status_code=409, detail=f"task {task_id} is {task['status']}, expected queued")
        try:
            if missions:
                missions.require_stable()
            episode = await capture_lifecycle.start(recording_request(EpisodeStart(
                task_id=task_id,
                task="scheduled space manipulator task",
                instruction=task["instruction"],
                seed=task["seed"],
                tags=task["tags"],
                scene_instance=scenes.status().get("instance"),
            ), scenes.status().get("instance")))
            if missions:
                missions.add_episode(episode["episode_id"])
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
        await asyncio.to_thread(recorder.record_action, neutral)
        await hub.publish_action(neutral)
        try:
            closed = await capture_lifecycle.stop(EpisodeStop(outcome=request.outcome, note=request.note))
            if closed["dataset_status"] != "complete":
                return tasks.transition(task_id, {"running"}, "failed", outcome=request.outcome,
                                        dataset_status=closed["dataset_status"], dataset_error=closed["dataset_error"])
            archive_job = jobs.submit_archive(closed["episode_id"])
            return tasks.transition(
                task_id, {"running"}, "completed", outcome=request.outcome, archive_job_id=archive_job["job_id"]
            )
        except RuntimeError as error:
            raise HTTPException(status_code=409, detail=str(error)) from error
