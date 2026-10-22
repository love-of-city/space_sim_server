"""Authenticated, exclusive, synchronous joint-policy sessions for external VLA clients."""
from __future__ import annotations

import asyncio
from collections import OrderedDict
import time
import uuid
from typing import Annotated

from fastapi import HTTPException, Request
from pydantic import BaseModel, ConfigDict, Field, FiniteFloat, field_validator

from .models import EpisodeStart, EpisodeStop
from .policy_observation import POLICY_CAMERAS, PERIOD_NS, observation_identity
from .simulation_hub import PolicyCommandRejected


class PolicyStart(BaseModel):
    model_config = ConfigDict(extra="forbid")
    instruction: str = Field(min_length=1, max_length=4096)
    task_name: str = Field(default="", max_length=128)
    reset: bool = True
    record: bool = False
    max_steps: int = Field(default=10000, ge=1, le=100000)
    idle_timeout_s: float = Field(default=180.0, ge=10, le=3600)


class PolicyIdentity(BaseModel):
    model_config = ConfigDict(extra="forbid")
    render_session_id: str = Field(min_length=1, max_length=128)
    render_frame_id: str = Field(pattern=r"^[0-9]+$")
    sim_time_ns: str = Field(pattern=r"^[0-9]+$")


PolicyAction = Annotated[list[FiniteFloat], Field(min_length=7, max_length=7)]


class PolicyStep(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    request_id: str = Field(min_length=1, max_length=64, pattern=r"^[A-Za-z0-9_-]+$")
    expected_observation: PolicyIdentity
    actions: list[PolicyAction] = Field(min_length=1, max_length=100)

    @field_validator("request_id")
    @classmethod
    def canonical_request_id(cls, value):
        return uuid.UUID(value).hex


class PolicySessions:
    def __init__(self, *, hub, cache, recorder, capture_lifecycle, sessions, safety,
                 scenes, recording_request):
        self.hub, self.cache, self.recorder = hub, cache, recorder
        self.capture_lifecycle, self.sessions = capture_lifecycle, sessions
        self.safety, self.scenes, self.recording_request = safety, scenes, recording_request
        self._session = None
        self._lock = asyncio.Lock()
        self._closed_sessions = OrderedDict()

    @property
    def active(self):
        return self._session is not None

    def status(self):
        session = self._session
        if session is None:
            return {"active": False}
        return {"active": True, "session_id": session["id"], "owner_user_id": session["owner"],
                "action_steps": session["steps"], "failed": session.get("failed"),
                "recording_episode_id": session.get("episode_id"), "mode": "synchronous_joint_position"}

    def require_free(self):
        if self.active:
            raise HTTPException(409, "模型正在独占控制；请先释放策略会话，再开始遥控、录制或更换场景")

    def _owned(self, identifier, user):
        session = self._session
        if session is None or session["id"] != identifier:
            raise HTTPException(404, "policy session not found")
        if user["role"] != "admin" and user["user_id"] != session["owner"]:
            raise HTTPException(403, "只能操作自己的策略会话")
        return session

    async def start(self, payload: PolicyStart, user):
        self.require_free()
        if self._lock.locked() or self.recorder.episode_id or self.capture_lifecycle.busy:
            raise HTTPException(409, "请先结束当前录制或控制切换")
        if not self.hub.connected or not self.hub.joint_policy_supported:
            raise HTTPException(409, "请先启动支持 joint_policy_v1 的仿真场景")
        observation = self.hub.latest_observation
        if observation is None or observation.gripper_opening_normalized is None:
            raise HTTPException(409, "当前场景尚未提供 SARM 6+1 状态")
        if getattr(observation, "scripted_task", None):
            raise HTTPException(409, "脚本任务正在驱动机械臂，无法同时开启模型控制")
        status = self.scenes.status()
        instance = status.get("instance")
        if instance and user["role"] != "admin" and (instance.get("created_by") or {}).get("user_id") != user["user_id"]:
            raise HTTPException(403, "只能控制自己创建的场景")
        if not instance and user["role"] != "admin":
            raise HTTPException(403, "外部启动的仿真只能由管理员接管")
        if instance:
            runtime = instance.get("runtime", {})
            if not runtime.get("dataset_capture", False) or runtime.get("capture_rate_hz") != 50:
                raise HTTPException(409, "模型控制要求场景启用 50 Hz 权威双相机采集")
        session = {"id": uuid.uuid4().hex, "owner": user["user_id"], "user": dict(user),
                   "config": payload, "steps": 0, "seen": set(), "last_activity": time.monotonic(),
                   "instance": instance, "failed": None, "identity": None, "ownership": "unclaimed"}
        self._session = session  # Reserve before the first await (blocks WS and recording races).
        async with self._lock:
            try:
                await self.sessions.suspend_control()
                neutral = self.safety.neutral(self.recorder.episode_id, "policy_control_acquired")
                await self.hub.publish_action(neutral)
                return await self._initialize(session, reset=payload.reset)
            except (Exception, asyncio.CancelledError):
                await self._cleanup(session, "aborted")
                raise

    async def _initialize(self, session, *, reset):
        payload = session["config"]
        session.update(steps=0, failed=None, identity=None)
        if reset:
            await self.hub.finish_reset(self.hub.begin_reset())
        if payload.record:
            user = session["user"]
            request = EpisodeStart(instruction=payload.instruction, task=payload.task_name or "policy execution",
                fps=50, camera_ids=list(POLICY_CAMERAS.values()), max_frames=min(108000, payload.max_steps + 1000),
                operator=user["username"], operator_user_id=user["user_id"], operator_username=user["username"],
                operator_role=user["role"], tags=["enactive", "joint-policy-v1"])
            request = self.recording_request(request, session["instance"])
            metadata = await self.capture_lifecycle.start(request, before_enable=self.cache.begin)
            session["episode_id"] = metadata["episode_id"]
        else:
            self.cache.begin("policy-" + session["id"])
            await self.hub.set_capture_episode("policy-" + session["id"])
        session["ownership"] = "uncertain"
        try:
            receipt = await self.hub.policy_command(session["id"], "acquire", period_ns=PERIOD_NS,
                task_name=payload.task_name, idle_timeout_s=payload.idle_timeout_s)
        except PolicyCommandRejected:
            session["ownership"] = "unclaimed"
            raise
        session["ownership"] = "acquired"
        return await self._response(session, receipt)

    async def _response(self, session, receipt):
        key = observation_identity(receipt)
        observation = await asyncio.to_thread(self.cache.get, key)
        session["identity"] = key
        session["last_activity"] = time.monotonic()
        task = observation.get("policy_task_state") or {}
        # Only simulator-owned physical predicates may assert success. Missing
        # task telemetry is explicit; a model STOP or a time limit is not success.
        available = task.get("task_name") == session["config"].task_name and task.get("success_available") is True
        success = bool(session["steps"] > 0 and available and task.get("success") is True)
        observation.update(instruction=session["config"].instruction, task_success=success,
                           success_available=available)
        session["success"] = success
        return {"session_id": session["id"], "observation": observation, "period_ns": PERIOD_NS,
                "action_format": "joint_position", "action_semantics": "absolute_target", "action_dim": 7,
                "success": success, "success_available": available,
                "done": success or session["steps"] >= session["config"].max_steps,
                "action_steps": session["steps"], "execution": receipt,
                "recording_episode_id": session.get("episode_id")}

    async def step(self, identifier, payload: PolicyStep, user):
        session = self._owned(identifier, user)
        if self._lock.locked():
            raise HTTPException(409, "已有策略指令正在执行")
        if session["failed"]:
            raise HTTPException(409, "策略会话出错；请重置或释放控制：" + session["failed"])
        if observation_identity(payload.expected_observation.model_dump()) != session["identity"]:
            raise HTTPException(409, "策略动作引用了过期的观测帧")
        if payload.request_id in session["seen"]:
            raise HTTPException(409, "重复的策略动作请求，未再次执行")
        if session.get("success") or session["steps"] + len(payload.actions) > session["config"].max_steps:
            raise HTTPException(409, "episode action budget exceeded or task already completed")
        if any(not 0 <= action[6] <= 1 for action in payload.actions):
            raise HTTPException(422, "夹爪开度必须在 [0, 1] 范围内")
        async with self._lock:
            session["seen"].add(payload.request_id)
            session["last_activity"] = time.monotonic()
            try:
                receipt = await self.hub.policy_command(identifier, "step", request_id=payload.request_id,
                    actions=payload.actions, period_ns=PERIOD_NS)
                executed = receipt.get("executed_steps")
                if type(executed) is not int or executed != len(payload.actions):
                    raise RuntimeError("runtime did not confirm every requested action interval")
                if (int(receipt["sim_time_ns"]) - int(session["identity"][2]) != executed * PERIOD_NS
                        or receipt["render_session_id"] != session["identity"][0]):
                    raise RuntimeError("policy execution clock or render session changed unexpectedly")
                receipt["requested_actions"] = payload.actions
                receipt["action_limited"] = receipt.get("applied_actions") != payload.actions
                session["steps"] += executed
                return await self._response(session, receipt)
            except (Exception, asyncio.CancelledError) as error:
                session["failed"] = str(error) or "policy request cancelled"
                raise

    async def reset(self, identifier, user):
        session = self._owned(identifier, user)
        if self._lock.locked():
            raise HTTPException(409, "已有策略指令正在执行，请等待完成后重置")
        async with self._lock:
            try:
                released = await self._release_episode(session, "unknown")
                recording = released.get("recording")
                if isinstance(recording, dict) and recording.get("dataset_status") != "complete":
                    raise RuntimeError("previous policy recording failed finalization; close the session before retrying")
                return await self._initialize(session, reset=True)
            except (Exception, asyncio.CancelledError) as error:
                session["failed"] = str(error) or "reset cancelled"
                raise

    async def _release_episode(self, session, outcome):
        # Keep the state/RGB cache alive through the last receipt; then stop the
        # shared capture channel only after physics can acknowledge its OFF boundary.
        if session.get("ownership") != "unclaimed":
            try:
                if not self.hub.connected:
                    raise RuntimeError("simulation disconnected; policy release is unconfirmed")
                await self.hub.policy_command(session["id"], "release", timeout=15)
                session["ownership"] = "unclaimed"
            except RuntimeError as error:
                session["failed"] = str(error)
                raise
        self.cache.end()
        if session.get("episode_id"):
            result = await self.capture_lifecycle.stop(EpisodeStop(
                outcome="success" if session.get("success") else outcome,
                note="joint policy session ended"))
            session.pop("episode_id", None)
        else:
            result = None
            try:
                await self.hub.set_capture_episode("", timeout=5)
            except RuntimeError as error:
                session["failed"] = str(error)
                raise
        return {"recording": result, "release_error": None}

    async def _cleanup(self, session, outcome):
        result = await self._release_episode(session, outcome)
        self.cache.end()
        if self._session is session:
            self._session = None
        return result

    def _closed_result(self, identifier, user):
        cached = self._closed_sessions.get(identifier)
        if cached is None:
            return None
        owner, result = cached
        if user["role"] != "admin" and user["user_id"] != owner:
            raise HTTPException(403, "只能操作自己的策略会话")
        return result

    async def close(self, identifier, user, outcome="unknown"):
        completed = self._closed_result(identifier, user)
        if completed is not None:
            return completed
        session = self._owned(identifier, user)

        async def finish():
            async with self._lock:
                completed = self._closed_result(identifier, user)
                if completed is not None:
                    return completed
                # Concurrent DELETEs can have captured this session before the
                # first release finished. Never stop a newer session's capture.
                if self._session is not session:
                    raise HTTPException(404, "policy session not found")
                result = {"session_id": identifier, "status": "closed",
                          **await self._cleanup(session, outcome)}
                self._closed_sessions[identifier] = (session["owner"], result)
                if len(self._closed_sessions) > 64:
                    self._closed_sessions.popitem(last=False)
                return result

        cleanup = asyncio.create_task(finish())
        try:
            return await asyncio.shield(cleanup)
        except asyncio.CancelledError:
            # Keep ownership reserved until the runtime confirms release, even
            # when the caller closes its HTTP connection during cleanup.
            await cleanup
            raise

    async def shutdown(self):
        session = self._session
        if session:
            await self.close(session["id"], session["user"], "aborted")

    def scene_stopped(self):
        """Only call after the managed scene process has actually stopped."""
        self.cache.end()
        self._session = None

    async def expire_idle(self):
        session = self._session
        if (session and not self._lock.locked()
                and time.monotonic() - session["last_activity"] > session["config"].idle_timeout_s):
            await self.close(session["id"], session["user"], "aborted")


def install_policy_routes(app, manager, current_user):
    async def call(operation):
        try:
            return await operation
        except (RuntimeError, ValueError, KeyError) as error:
            raise HTTPException(409, str(error)) from error

    @app.get("/api/policy/status")
    async def status(request: Request):
        current_user(request)
        return manager.status()

    @app.post("/api/policy/sessions")
    async def start(payload: PolicyStart, request: Request):
        return await call(manager.start(payload, current_user(request)))

    @app.post("/api/policy/sessions/{identifier}/step")
    async def step(identifier: str, payload: PolicyStep, request: Request):
        return await call(manager.step(identifier, payload, current_user(request)))

    @app.post("/api/policy/sessions/{identifier}/reset")
    async def reset(identifier: str, request: Request):
        return await call(manager.reset(identifier, current_user(request)))

    @app.delete("/api/policy/sessions/{identifier}")
    async def close(identifier: str, request: Request):
        return await call(manager.close(identifier, current_user(request)))
