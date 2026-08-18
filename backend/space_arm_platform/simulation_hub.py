"""Async duplex gateway between the API backend and BSK/MJScene."""

from __future__ import annotations

import asyncio
import contextlib
from collections.abc import Awaitable, Callable
from typing import Any

from pydantic import ValidationError

from .models import AppliedAction, CONTROL_PROTOCOL, SimulationHello, SimulationObservation
from .protocol import read_async, write_async


ObservationCallback = Callable[[SimulationObservation], Awaitable[None]]


class SimulationHub:
    """Own one authoritative simulator connection and latest observation."""

    def __init__(self) -> None:
        self._server: asyncio.AbstractServer | None = None
        self._writer: asyncio.StreamWriter | None = None
        self._writer_lock = asyncio.Lock()
        self._latest_action: AppliedAction | None = None
        self._latest_observation: SimulationObservation | None = None
        self._simulation_id: str | None = None
        self._revision = 0
        self._condition = asyncio.Condition()
        self.on_observation: ObservationCallback | None = None

    @property
    def connected(self) -> bool:
        return self._writer is not None and not self._writer.is_closing()

    @property
    def bound_port(self) -> int | None:
        if not self._server or not self._server.sockets:
            return None
        return int(self._server.sockets[0].getsockname()[1])

    @property
    def simulation_id(self) -> str | None:
        return self._simulation_id

    @property
    def latest_observation(self) -> SimulationObservation | None:
        return self._latest_observation.model_copy(deep=True) if self._latest_observation else None

    async def start(self, host: str, port: int) -> None:
        self._server = await asyncio.start_server(self._handle_connection, host, port)

    async def close(self) -> None:
        if self._server:
            self._server.close()
            await self._server.wait_closed()
        async with self._writer_lock:
            if self._writer:
                self._writer.close()
                with contextlib.suppress(Exception):
                    await self._writer.wait_closed()
                self._writer = None

    async def publish_action(self, action: AppliedAction) -> bool:
        self._latest_action = action.model_copy(deep=True)
        async with self._writer_lock:
            writer = self._writer
            if writer is None or writer.is_closing():
                return False
            try:
                await write_async(writer, action.model_dump(mode="json"))
                return True
            except (ConnectionError, OSError, asyncio.CancelledError):
                return False

    async def wait_for_observation(self, revision: int) -> tuple[int, SimulationObservation | None]:
        async with self._condition:
            await self._condition.wait_for(lambda: self._revision > revision)
            observation = self.latest_observation
            return self._revision, observation

    async def _handle_connection(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        try:
            hello = SimulationHello.model_validate(await asyncio.wait_for(read_async(reader), timeout=3.0))
        except (asyncio.TimeoutError, asyncio.IncompleteReadError, ValueError, ValidationError):
            writer.close()
            await writer.wait_closed()
            return
        previous: asyncio.StreamWriter | None = None
        async with self._writer_lock:
            previous = self._writer
            self._writer = writer
            self._simulation_id = hello.simulation_id
            if self._latest_action:
                await write_async(writer, self._latest_action.model_dump(mode="json"))
        if previous and previous is not writer:
            previous.close()
        try:
            while True:
                raw = await read_async(reader)
                if raw.get("protocol") != CONTROL_PROTOCOL or raw.get("type") != "observation":
                    continue
                observation = SimulationObservation.model_validate(raw)
                self._latest_observation = observation
                async with self._condition:
                    self._revision += 1
                    self._condition.notify_all()
                if self.on_observation:
                    await self.on_observation(observation)
        except (asyncio.IncompleteReadError, ConnectionError, OSError, ValueError, ValidationError):
            pass
        finally:
            async with self._writer_lock:
                if self._writer is writer:
                    self._writer = None
                    self._simulation_id = None
            writer.close()
            with contextlib.suppress(Exception):
                await writer.wait_closed()

    def status(self) -> dict[str, Any]:
        observation = self.latest_observation
        return {
            "connected": self.connected,
            "simulation_id": self.simulation_id,
            "latest_observation": observation.model_dump(mode="json") if observation else None,
        }
