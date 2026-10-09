"""Length-prefixed JSON transport for BSK render protocol version 2."""

from __future__ import annotations

import json
import queue
import select
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Any


PROTOCOL_V2 = "bsk-render/2"
HEADER = struct.Struct("!I")
MAX_PACKET_BYTES = 32 * 1024 * 1024


def encode_packet(message: dict[str, Any]) -> bytes:
    """Encode one JSON object with a network-order length prefix."""

    body = json.dumps(message, separators=(",", ":"), allow_nan=False).encode("utf-8")
    if not body or len(body) > MAX_PACKET_BYTES:
        raise ValueError(f"invalid packet size: {len(body)} bytes")
    return HEADER.pack(len(body)) + body


def decode_packet(packet: bytes) -> dict[str, Any]:
    """Decode one complete length-prefixed packet."""

    if len(packet) < HEADER.size:
        raise ValueError("packet is shorter than its header")
    (length,) = HEADER.unpack(packet[: HEADER.size])
    if length != len(packet) - HEADER.size or length > MAX_PACKET_BYTES:
        raise ValueError("packet length does not match its header")
    result = json.loads(packet[HEADER.size :].decode("utf-8"))
    if not isinstance(result, dict):
        raise ValueError("packet body must be a JSON object")
    return result


@dataclass
class PublisherStats:
    """Observable non-blocking publisher counters."""

    frames_queued: int = 0
    frames_sent: int = 0
    frames_dropped: int = 0
    controls_sent: int = 0
    events_dropped: int = 0
    reconnects: int = 0
    commands_received: int = 0
    commands_rejected: int = 0
    last_error: str | None = None

    @property
    def queued(self) -> int:
        """Legacy alias for queued frame count."""

        return self.frames_queued

    @property
    def sent(self) -> int:
        """Legacy alias for sent frame count."""

        return self.frames_sent

    @property
    def dropped(self) -> int:
        """Legacy alias for dropped frame count."""

        return self.frames_dropped


class RecordingOnlyPublisher:
    """Discard live transport output while a bridge writes a ``.bskrec`` file.

    This transport-neutral sink lets any Basilisk scenario use the same bridge
    and recording code without opening a TCP connection or implementing a
    demo-specific dummy publisher.  Commands are intentionally unavailable in
    offline recording mode.
    """

    def retain_hello(self, _message: dict[str, Any]) -> None:
        pass

    def retain_manifest(self, _message: dict[str, Any]) -> None:
        pass

    def publish_frame(self, _message: dict[str, Any]) -> None:
        pass

    def has_frame_capacity(self, reserve: int = 2) -> bool:
        return True

    def publish_event(self, _message: dict[str, Any]) -> bool:
        return True

    def close(self) -> None:
        pass


class RenderPublisher:
    """Publish retained scene data; optionally preserve authoritative frames FIFO."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 5558,
        reconnect_period_s: float = 0.5,
        event_queue_size: int = 64,
        command_queue_size: int = 64,
        reliable_frames: bool = False,
    ) -> None:
        self.host = host
        self.port = int(port)
        self.reconnect_period_s = float(reconnect_period_s)
        self.stats = PublisherStats()
        self.reliable_frames = reliable_frames
        self._latest_frame: queue.Queue[bytes] = queue.Queue(maxsize=256 if reliable_frames else 1)
        # On-demand frames use an independent bounded FIFO. Preview replacement
        # must never evict a strict frame accepted before a STOP transition.
        self._strict_frames: queue.Queue[bytes] = queue.Queue(maxsize=256)
        self._preview_frames: queue.Queue[bytes] = queue.Queue(maxsize=1)
        self._events: queue.Queue[bytes] = queue.Queue(maxsize=event_queue_size)
        self._commands: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=command_queue_size)
        self._receive_buffer = bytearray()
        self._retained_lock = threading.Lock()
        self._hello: bytes | None = None
        self._manifest: bytes | None = None
        self._manifest_generation = 0
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        """Start the worker if it is not already running."""

        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._worker, name="bsk-render-publisher", daemon=True)
        self._thread.start()

    def retain_hello(self, message: dict[str, Any]) -> None:
        """Retain the hello packet for every connection."""

        with self._retained_lock:
            self._hello = encode_packet(message)
        self._wake.set()
        self.start()

    def retain_manifest(self, message: dict[str, Any]) -> None:
        """Retain the newest manifest and mark it for reliable resend."""

        with self._retained_lock:
            self._manifest = encode_packet(message)
            self._manifest_generation += 1
        self._wake.set()
        self.start()

    def publish_frame(self, message: dict[str, Any]) -> None:
        """Queue a frame; dataset mode applies bounded backpressure, preview replaces."""

        packet = encode_packet(message)
        on_demand = "capture_episode_id" in message
        strict = bool(message.get("capture_episode_id")) if on_demand else self.reliable_frames
        if strict:
            self.start()
            target = self._strict_frames if on_demand else self._latest_frame
            if on_demand:
                # An unsent pre-start preview is not an authoritative sample.
                try:
                    while True:
                        self._preview_frames.get_nowait()
                        self.stats.frames_dropped += 1
                except queue.Empty:
                    pass
            try:
                # Capacity is reserved BETWEEN physics advances, never by
                # sleeping inside a Basilisk/SWIG dynamics callback.
                target.put_nowait(packet)
            except queue.Full as error:
                raise RuntimeError("authoritative render queue full; refusing to drop a dataset frame") from error
            self.stats.frames_queued += 1
            self._wake.set()
            return
        target = self._preview_frames if on_demand else self._latest_frame
        try:
            target.put_nowait(packet)
        except queue.Full:
            try:
                target.get_nowait()
            except queue.Empty:
                pass
            else:
                self.stats.frames_dropped += 1
            target.put_nowait(packet)
        self.stats.frames_queued += 1
        self._wake.set()
        self.start()

    def has_frame_capacity(self, reserve: int = 2) -> bool:
        """Single-producer admission outside physics; the worker only frees slots.

        Reserve two slots for initial t=0 and the first render-grid boundary.
        No accepted strict frame is discarded, including across STOP.
        """
        return (not self._stop.is_set()
                and self._strict_frames.qsize() + reserve <= self._strict_frames.maxsize
                and (not self.reliable_frames
                     or self._latest_frame.qsize() + reserve <= self._latest_frame.maxsize))

    def publish_event(self, message: dict[str, Any]) -> bool:
        """Queue a bounded reliable event, requesting manifest recovery on overflow."""

        try:
            self._events.put_nowait(encode_packet(message))
        except queue.Full:
            self.stats.events_dropped += 1
            with self._retained_lock:
                self._manifest_generation += 1
            self._wake.set()
            return False
        self._wake.set()
        self.start()
        return True

    def close(self, timeout_s: float = 2.0) -> None:
        """Stop the worker and close its socket."""

        self._stop.set()
        self._wake.set()
        if self._thread:
            self._thread.join(timeout_s)

    def consume_command(self) -> dict[str, Any] | None:
        """Return one UE command for consumption on the simulation thread."""

        try:
            return self._commands.get_nowait()
        except queue.Empty:
            return None

    def _receive_commands(self, connection: socket.socket) -> None:
        while select.select([connection], [], [], 0.0)[0]:
            chunk = connection.recv(64 * 1024)
            if not chunk:
                raise ConnectionResetError("UE command channel closed")
            self._receive_buffer.extend(chunk)
        while len(self._receive_buffer) >= HEADER.size:
            (length,) = HEADER.unpack_from(self._receive_buffer)
            if length == 0 or length > MAX_PACKET_BYTES:
                raise ConnectionError(f"invalid UE command packet length: {length}")
            packet_end = HEADER.size + length
            if len(self._receive_buffer) < packet_end:
                break
            command = json.loads(bytes(self._receive_buffer[HEADER.size:packet_end]).decode("utf-8"))
            del self._receive_buffer[:packet_end]
            if not isinstance(command, dict) or command.get("protocol") != PROTOCOL_V2 or command.get("type") != "command":
                raise ConnectionError("UE returned a non-command packet on the command channel")
            try:
                self._commands.put_nowait(command)
                self.stats.commands_received += 1
            except queue.Full:
                self.stats.commands_rejected += 1
                rejection = {
                    "protocol": PROTOCOL_V2,
                    "type": "event",
                    "session_id": str(command.get("session_id", "")),
                    "sequence": "0",
                    "event_kind": "command_result",
                    "payload": {
                        "command_id": str(command.get("command_id", "")),
                        "command": str(command.get("command", "")),
                        "status": "rejected",
                        "severity": "error",
                        "message": "BSK command queue is full",
                    },
                }
                connection.sendall(encode_packet(rejection))

    def _retained_snapshot(self) -> tuple[bytes | None, bytes | None, int]:
        with self._retained_lock:
            return self._hello, self._manifest, self._manifest_generation

    def _worker(self) -> None:
        connection: socket.socket | None = None
        connection_generation = -1
        pending_event: bytes | None = None
        pending_frame: bytes | None = None
        pending_frame_reliable = False
        while not self._stop.is_set():
            if connection is None:
                try:
                    connection = socket.create_connection((self.host, self.port), timeout=1.0)
                    connection.settimeout(2.0)
                    self._receive_buffer.clear()
                    connection_generation = -1
                    self.stats.reconnects += 1
                except OSError as error:
                    self.stats.last_error = str(error)
                    self._stop.wait(self.reconnect_period_s)
                    continue
            try:
                self._receive_commands(connection)
                hello, manifest, generation = self._retained_snapshot()
                if connection_generation != generation:
                    if hello is not None:
                        connection.sendall(hello)
                        self.stats.controls_sent += 1
                    if manifest is not None:
                        connection.sendall(manifest)
                        self.stats.controls_sent += 1
                    connection_generation = generation

                if pending_event is None:
                    try:
                        pending_event = self._events.get_nowait()
                    except queue.Empty:
                        pass
                if pending_event is not None:
                    connection.sendall(pending_event)
                    pending_event = None
                    self.stats.controls_sent += 1
                    continue

                if pending_frame is None:
                    try:
                        try:
                            pending_frame = self._strict_frames.get_nowait()
                            pending_frame_reliable = True
                        except queue.Empty:
                            try:
                                pending_frame = self._preview_frames.get_nowait()
                                pending_frame_reliable = False
                            except queue.Empty:
                                pending_frame = self._latest_frame.get_nowait()
                                pending_frame_reliable = self.reliable_frames
                                if not pending_frame_reliable:
                                    while True:
                                        pending_frame = self._latest_frame.get_nowait()
                                        self.stats.frames_dropped += 1
                    except queue.Empty:
                        pass
                if pending_frame is not None:
                    connection.sendall(pending_frame)
                    pending_frame = None
                    self.stats.frames_sent += 1
                    self.stats.last_error = None
                    self._receive_commands(connection)
                    continue
                self._wake.wait(0.05)
                self._wake.clear()
            except (OSError, ConnectionError, ValueError, UnicodeError) as error:
                self.stats.last_error = str(error)
                if not pending_frame_reliable:
                    if pending_frame is not None:
                        self.stats.frames_dropped += 1
                    pending_frame = None
                try:
                    connection.close()
                finally:
                    connection = None
                time.sleep(0.0)
        if connection is not None:
            connection.close()
