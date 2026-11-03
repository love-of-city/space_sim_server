"""Authenticated browser ownership and control handoff."""
from __future__ import annotations

import asyncio
import contextlib
import uuid
from typing import Any

from fastapi import WebSocket, WebSocketDisconnect


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

    async def suspend_control(self) -> None:
        async with self._lock:
            self.active_operator = None
            clients = list(self.clients.values())
        for websocket in clients:
            with contextlib.suppress(RuntimeError, WebSocketDisconnect, TimeoutError, OSError):
                await asyncio.wait_for(
                    websocket.send_json({"type": "control_revoked", "reason": "scene_reset"}), timeout=2.0
                )

    def is_owner(self, identifier: str) -> bool:
        return self.active_operator == identifier
