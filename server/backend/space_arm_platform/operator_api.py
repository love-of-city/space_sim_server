"""Authenticated operator WebSocket and exclusive control delivery."""
from __future__ import annotations

import asyncio
import contextlib
import hmac
import time

from fastapi import WebSocket, WebSocketDisconnect
from pydantic import ValidationError

from .auth import SESSION_COOKIE
from .auth_api import can_manage_scene
from .models import OperatorAction
from .safety import ActionRejected


def install_operator_routes(app, *, config, allowed_origins, auth, sessions, policy, hub,
                            scenes, recorder, safety, missions=None):
    @app.websocket("/ws/operator")
    async def operator_socket(websocket: WebSocket) -> None:
        if allowed_origins and websocket.headers.get("origin") not in allowed_origins:
            await websocket.close(code=4403, reason="origin not allowed")
            return
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
        if policy.active:
            granted = False
            await sessions.suspend_control()
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
                if missions:
                    try:
                        missions.require_stable()
                    except RuntimeError:
                        await websocket.send_json({"type": "action_rejected", "reason": "mission_transition_or_flight"})
                        continue
                if policy.active:
                    await websocket.send_json({"type": "action_rejected", "reason": "policy_control_active"})
                    continue
                if hub.resetting:
                    await websocket.send_json({"type": "action_rejected", "reason": "scene_resetting"})
                    continue
                if message_type == "activate_control":
                    if not user_controls_current_scene():
                        await websocket.send_json({"type": "action_rejected", "reason": "not_scene_owner"})
                        continue
                    previous = await sessions.activate(identifier)
                    if previous:
                        neutral = safety.neutral(recorder.episode_id, "control_page_changed")
                        await asyncio.to_thread(recorder.record_action, neutral)
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
                await asyncio.to_thread(recorder.record_action, action)
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
                await asyncio.to_thread(recorder.record_action, neutral)
                await hub.publish_action(neutral)
