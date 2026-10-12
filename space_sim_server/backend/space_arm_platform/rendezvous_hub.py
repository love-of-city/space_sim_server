"""Read-only bridge for rendezvous scenes using the platform's generic state protocol."""

from __future__ import annotations

import asyncio
import contextlib
import math
import time
from typing import Literal

from pydantic import BaseModel, Field, FiniteFloat, model_validator

from .protocol import read_async


class RendezvousBody(BaseModel):
    body_id: Literal["rendezvous/target", "rendezvous/chaser"]
    position_m: list[FiniteFloat] = Field(min_length=3, max_length=3)
    velocity_m_s: list[FiniteFloat] = Field(min_length=3, max_length=3)
    attitude_wxyz: list[FiniteFloat] = Field(min_length=4, max_length=4)
    angular_velocity_rad_s: list[FiniteFloat] = Field(min_length=3, max_length=3)


class RendezvousState(BaseModel):
    protocol: Literal["space-sim-state/1"]
    type: Literal["state"]
    sim_time_s: FiniteFloat = Field(ge=0)
    bodies: list[RendezvousBody] = Field(min_length=2, max_length=2)
    metadata: dict[str, object]

    @model_validator(mode="after")
    def check_mission(self):
        if {body.body_id for body in self.bodies} != {
            "rendezvous/target",
            "rendezvous/chaser",
        }:
            raise ValueError("one target and one chaser are required")
        if (
            not isinstance(self.metadata.get("run_id"), str)
            or not self.metadata["run_id"]
        ):
            raise ValueError("a run_id is required")
        if not isinstance(self.metadata.get("phase"), str):
            raise ValueError("a mission phase is required")
        return self


class RendezvousHub:
    """Accept one authoritative state stream without entering the arm control path."""

    def __init__(self):
        self._server: asyncio.AbstractServer | None = None
        self._latest: RendezvousState | None = None
        self._received_at_ns: int | None = None
        self._connected = False
        self._writers: set[asyncio.StreamWriter] = set()
        self._active_writer: asyncio.StreamWriter | None = None

    async def start(self, host: str, port: int):
        self._server = await asyncio.start_server(self._handle, host, port)

    @property
    def bound_port(self) -> int | None:
        if self._server is None or not self._server.sockets:
            return None
        return int(self._server.sockets[0].getsockname()[1])

    def clear_cached_state(self) -> None:
        """Wait for the new mission's first packet instead of showing old telemetry."""
        self._latest = None
        self._received_at_ns = None

    async def close(self):
        if self._server is not None:
            self._server.close()
            writers = list(self._writers)
            for writer in writers:
                writer.close()
            await asyncio.gather(*(writer.wait_closed() for writer in writers), return_exceptions=True)
            await self._server.wait_closed()
            self._server = None

    async def _handle(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        self._writers.add(writer)
        stream_run_id = None
        try:
            while True:
                state = RendezvousState.model_validate(await read_async(reader))
                run_id = state.metadata["run_id"]
                if stream_run_id is not None:
                    if self._active_writer is not writer:
                        break  # A newer validated publisher owns the state.
                    if run_id != stream_run_id:
                        raise ValueError("a state connection must retain its run_id")
                if (
                    self._latest is not None
                    and state.metadata["run_id"] == self._latest.metadata["run_id"]
                    and state.sim_time_s <= self._latest.sim_time_s
                ):
                    raise ValueError(
                        "rendezvous timestamps must increase within one run"
                    )
                # An invalid/empty connection must not evict the active stream.
                # Promote only after validating the first packet, then close
                # the old peer so buffered old states cannot replace new ones.
                if stream_run_id is None:
                    previous = self._active_writer
                    self._active_writer = writer
                    stream_run_id = run_id
                    if previous is not None and previous is not writer:
                        previous.close()
                self._connected = True
                self._latest = state
                self._received_at_ns = time.time_ns()
        except (asyncio.IncompleteReadError, ConnectionError, ValueError):
            pass
        finally:
            self._writers.discard(writer)
            if self._active_writer is writer:
                self._active_writer = None
                self._connected = False
            writer.close()
            with contextlib.suppress(ConnectionError):
                await writer.wait_closed()

    def status(self) -> dict:
        state = self._latest
        if state is None:
            return {
                "connected": self._connected,
                "state": None,
                "port": self.bound_port,
            }
        bodies = {body.body_id: body for body in state.bodies}
        target = bodies["rendezvous/target"]
        chaser = bodies["rendezvous/chaser"]
        separation = math.dist(target.position_m, chaser.position_m)
        return {
            "connected": self._connected,
            "port": self.bound_port,
            "received_at_ns": str(self._received_at_ns),
            "run_id": state.metadata["run_id"],
            "phase": state.metadata["phase"],
            "sim_time_s": state.sim_time_s,
            "separation_m": separation,
            "capture_time_s": state.metadata.get("capture_time_s"),
            "state": state.model_dump(mode="json"),
        }
