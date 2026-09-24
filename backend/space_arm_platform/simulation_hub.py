"""Async duplex gateway between the API backend and BSK/MJScene."""

from __future__ import annotations

import asyncio
import contextlib
import logging
from collections import OrderedDict
import uuid
from collections.abc import Awaitable, Callable
from typing import Any

from pydantic import ValidationError

from .models import AppliedAction, CONTROL_PROTOCOL, SimulationHello, SimulationObservation
from .protocol import RUNTIME_CAPABILITIES, read_async, write_async


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
        self._capabilities: set[str] = set()
        self._generation = ""
        self._reset_request_id: str | None = None
        self._reset_future: asyncio.Future[dict[str, Any]] | None = None
        self._reset_error: str | None = None
        self._reset_sent = False
        self._condition = asyncio.Condition()
        self.on_observation: ObservationCallback | None = None
        self.on_transport_error = None
        self._observation_lock = asyncio.Lock()
        self._stream_acks = OrderedDict()
        self._transport_error = None
        self._duplicates = 0
        self._capture_peer_id: str | None = None
        self._capture_request: dict[str, Any] | None = None
        self._capture_error: str | None = None

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
        self.cancel_reset()
        if self._server:
            self._server.close()
            await self._server.wait_closed()
        async with self._writer_lock:
            if self._writer:
                self._writer.close()
                with contextlib.suppress(Exception):
                    await self._writer.wait_closed()
                self._writer = None

    @property
    def resetting(self) -> bool:
        return self._reset_request_id is not None

    def begin_reset(self) -> str:
        """Reserve the reset before yielding, so API/WS/recording cannot race it."""
        if self.resetting:
            raise RuntimeError("场景正在重置，请等待完成")
        if not self.connected:
            raise RuntimeError("仿真未连接，无法重置")
        if "scene_reset" not in self._capabilities:
            raise RuntimeError("当前仿真进程不支持重置，请使用重置分支启动场景")
        self._reset_request_id = uuid.uuid4().hex
        self._reset_future = asyncio.get_running_loop().create_future()
        self._reset_error = None
        self._reset_sent = False
        self._latest_action = None
        self._latest_observation = None
        return self._reset_request_id

    def _reset_packet(self) -> dict[str, Any]:
        return {"protocol": CONTROL_PROTOCOL, "type": "reset",
                "request_id": self._reset_request_id}

    async def finish_reset(self, request_id: str, timeout: float = 60.0) -> dict[str, Any]:
        future = self._reset_future
        if request_id != self._reset_request_id or future is None:
            raise RuntimeError("无效的重置请求")
        self._reset_sent = True
        try:
            async with self._writer_lock:
                if not self.connected:
                    raise ConnectionError("仿真连接已断开")
                await write_async(self._writer, self._reset_packet())
            # A timed-out HTTP request must NOT enable control while reset may
            # still be running. Reconnect resends the same idempotent request.
            return await asyncio.wait_for(asyncio.shield(future), timeout)
        except (TimeoutError, ConnectionError, OSError) as error:
            self._reset_error = "重置尚未确认，操控保持禁用；请等待恢复连接或停止场景"
            raise RuntimeError(self._reset_error) from error

    def cancel_reset(self) -> None:
        if self._reset_future and not self._reset_future.done():
            self._reset_future.set_result({"status": "cancelled"})
        self._reset_request_id = None
        self._reset_future = None
        self._reset_error = None
        self._reset_sent = False
        self._latest_action = None
        self._latest_observation = None

    @property
    def capture_on_demand_supported(self) -> bool:
        return "capture_on_demand_v1" in self._capabilities

    async def set_capture_episode(self, episode_id: str, timeout: float = 15.0) -> None:
        """Retain/replay the desired mode; acknowledge only at a render-frame boundary."""
        if episode_id and (not self.connected or not self.capture_on_demand_supported):
            raise RuntimeError("仿真未连接或不支持按需采集，请更新并重启场景")
        packet = {"protocol": CONTROL_PROTOCOL, "type": "capture_control",
                  "episode_id": episode_id, "request_id": uuid.uuid4().hex}
        self._capture_request = packet
        self._capture_error = None
        try:
            async with self._writer_lock:
                if not self.connected or not self.capture_on_demand_supported:
                    raise ConnectionError("仿真连接已断开，停止采集指令将在重连后重发")
                await write_async(self._writer, packet)
            async with self._condition:
                await asyncio.wait_for(self._condition.wait_for(lambda:
                    self._latest_observation is not None
                    and self._latest_observation.capture_request_id == packet["request_id"]
                    and self._latest_observation.capture_episode_id == episode_id), timeout)
        except (TimeoutError, ConnectionError, OSError) as error:
            self._capture_error = f"采集模式切换尚未确认: {error}"
            raise RuntimeError(self._capture_error) from error

    async def publish_action(self, action: AppliedAction) -> bool:
        async with self._writer_lock:
            if self.resetting or not self.connected:
                return False
            outgoing = action.model_copy(update={"reset_generation": self._generation}, deep=True)
            self._latest_action = outgoing
            try:
                await write_async(self._writer, outgoing.model_dump(mode="json"))
                return True
            except (ConnectionError, OSError, asyncio.CancelledError):
                return False

    async def wait_for_observation(self, revision: int) -> tuple[int, SimulationObservation | None]:
        async with self._condition:
            await self._condition.wait_for(lambda: self._revision > revision)
            observation = self.latest_observation
            return self._revision, observation

    async def _ack(self, writer, stream_id, sequence, kind="observation_ack"):
        async with self._writer_lock:
            if writer is self._writer:
                await write_async(writer, {"protocol": CONTROL_PROTOCOL, "type": kind,
                    "observation_stream_id": stream_id, "observation_sequence": str(sequence)})

    async def _accept_observation(self, observation):
        if self.resetting:
            if observation.reset_generation != self._reset_request_id or not observation.render_session_id:
                return
            self._generation = observation.reset_generation
            result = {"status": "completed", "request_id": self._reset_request_id,
                      "render_session_id": observation.render_session_id, "sim_time_ns": observation.sim_time_ns}
            self._reset_request_id = None
            self._reset_sent = False
            self._reset_error = None
            if self._reset_future and not self._reset_future.done():
                self._reset_future.set_result(result)
            self._reset_future = None
        elif observation.reset_generation != self._generation:
            return
        self._latest_observation = observation
        if (self._capture_request
                and observation.capture_request_id == self._capture_request["request_id"]
                and observation.capture_episode_id == self._capture_request["episode_id"]):
            self._capture_error = None
        async with self._condition:
            self._revision += 1
            self._condition.notify_all()
        if self.on_observation:
            await self.on_observation(observation)

    async def _handle_connection(self, reader, writer):
        try:
            raw_hello = await asyncio.wait_for(read_async(reader), timeout=3.)
            if raw_hello.get("protocol") == CONTROL_PROTOCOL and raw_hello.get("type") == "capabilities_probe":
                # Do not claim the active writer, reset generation or observation stream.
                try:
                    await write_async(writer, {"protocol": CONTROL_PROTOCOL, "type": "backend_capabilities",
                                              "capabilities": list(RUNTIME_CAPABILITIES)})
                finally:
                    writer.close()
                    with contextlib.suppress(Exception):
                        await writer.wait_closed()
                return
            hello = SimulationHello.model_validate(raw_hello)
            reliable = "reliable_observations_v1" in hello.capabilities
            stream_id = str(getattr(hello, "observation_stream_id", "")) if reliable else ""
            resume_after = int(getattr(hello, "observation_resume_after", "0")) if reliable else 0
            if reliable and (not 1 <= len(stream_id) <= 128 or resume_after < 0):
                raise ValueError("invalid reliable observation handshake")
        except (asyncio.TimeoutError, asyncio.IncompleteReadError, OSError, ValueError, ValidationError):
            writer.close()
            with contextlib.suppress(Exception):
                await writer.wait_closed()
            return
        async with self._writer_lock:
            previous = self._writer
            self._writer = writer
            if self._capture_peer_id != (stream_id or hello.simulation_id) and self._capture_request:
                # A different simulator must never inherit a live episode.
                if self._capture_request["episode_id"] and self.on_transport_error:
                    self.on_transport_error("simulation peer changed during strict capture")
                self._capture_request = {"protocol": CONTROL_PROTOCOL, "type": "capture_control",
                                         "episode_id": "", "request_id": uuid.uuid4().hex}
            self._capture_peer_id = stream_id or hello.simulation_id
            self._simulation_id = hello.simulation_id
            self._capabilities = set(hello.capabilities)
            self._generation = hello.reset_generation
            self._latest_action = None
            self._latest_observation = None
        if previous and previous is not writer:
            previous.close()
        try:
            if reliable:
                async with self._observation_lock:
                    self._stream_acks.setdefault(stream_id, resume_after)
                    self._stream_acks.move_to_end(stream_id)
                    while len(self._stream_acks) > 8:
                        self._stream_acks.popitem(last=False)
                    await self._ack(writer, stream_id, self._stream_acks[stream_id], "observation_ready")
            async with self._writer_lock:
                if writer is self._writer and self.capture_on_demand_supported:
                    if self._capture_request is None:
                        self._capture_request = {"protocol": CONTROL_PROTOCOL, "type": "capture_control",
                                                 "episode_id": "", "request_id": uuid.uuid4().hex}
                    await write_async(writer, self._capture_request)
                if writer is self._writer and self.resetting and self._reset_sent:
                    await write_async(writer, self._reset_packet())
            while True:
                raw = await read_async(reader)
                if writer is not self._writer or raw.get("protocol") != CONTROL_PROTOCOL or raw.get("type") != "observation":
                    continue
                async with self._observation_lock:
                    if writer is not self._writer:
                        continue
                    if reliable:
                        if raw.get("observation_stream_id") != stream_id:
                            raise ValueError("observation stream changed inside connection")
                        sequence = int(raw.get("observation_sequence", "-1"))
                        last = self._stream_acks[stream_id]
                        if sequence <= last and sequence > 0:
                            self._duplicates += 1
                            await self._ack(writer, stream_id, last)
                            continue
                        if sequence != last + 1:
                            raise ValueError(f"authoritative state sequence gap: expected={last + 1}, received={sequence}")
                    observation = SimulationObservation.model_validate(raw)
                    await self._accept_observation(observation)
                    if reliable:
                        # Remember BEFORE ACK: reconnect after a lost ACK replays safely.
                        self._stream_acks[stream_id] = sequence
                        await self._ack(writer, stream_id, sequence)
                    self._transport_error = None
        except (asyncio.IncompleteReadError, ConnectionError, OSError, ValueError, ValidationError) as error:
            self._transport_error = f"{type(error).__name__}: {error}"
            logging.getLogger(__name__).warning("Simulation transport interrupted (replay enabled=%s): %s", reliable, error)
            if isinstance(error, ValueError) and self.on_transport_error:
                self.on_transport_error(self._transport_error)
        finally:
            async with self._writer_lock:
                if self._writer is writer:
                    self._writer = None
                    self._simulation_id = None
                    self._latest_action = None
            writer.close()
            with contextlib.suppress(Exception):
                await writer.wait_closed()

    def status(self) -> dict[str, Any]:
        observation = self.latest_observation
        return {
            "connected": self.connected,
            "capture_on_demand_supported": self.capture_on_demand_supported,
            "capture_episode_id": observation.capture_episode_id if observation else "",
            "capture_control_error": self._capture_error,
            "reliable_observations": "reliable_observations_v1" in self._capabilities,
            "observation_transport_error": self._transport_error,
            "observation_replays_deduplicated": self._duplicates,
            "resetting": self.resetting,
            "reset_supported": "scene_reset" in self._capabilities,
            "reset_error": self._reset_error,
            "simulation_id": self.simulation_id,
            "latest_observation": observation.model_dump(mode="json") if observation else None,
        }
