"""Network-order uint32 length-prefixed JSON helpers."""

from __future__ import annotations

import asyncio
import json
import socket
import struct
from typing import Any


HEADER = struct.Struct("!I")
MAX_PACKET_BYTES = 8 * 1024 * 1024
CONTROL_PROTOCOL = "space-arm-control/1"
RUNTIME_CAPABILITIES = ("reliable_observations_v1", "capture_ack_v1", "capture_on_demand_v1")


def encode_packet(message: dict[str, Any]) -> bytes:
    body = json.dumps(message, separators=(",", ":"), allow_nan=False).encode("utf-8")
    if not body or len(body) > MAX_PACKET_BYTES:
        raise ValueError(f"invalid packet size: {len(body)}")
    return HEADER.pack(len(body)) + body


def decode_packet(packet: bytes) -> dict[str, Any]:
    if len(packet) < HEADER.size:
        raise ValueError("packet is shorter than its header")
    (length,) = HEADER.unpack_from(packet)
    if length == 0 or length > MAX_PACKET_BYTES or length != len(packet) - HEADER.size:
        raise ValueError("packet length does not match header")
    value = json.loads(packet[HEADER.size:].decode("utf-8"))
    if not isinstance(value, dict):
        raise ValueError("packet JSON must be an object")
    return value


async def read_async(reader: asyncio.StreamReader) -> dict[str, Any]:
    header = await reader.readexactly(HEADER.size)
    (length,) = HEADER.unpack(header)
    if length == 0 or length > MAX_PACKET_BYTES:
        raise ValueError(f"invalid packet size: {length}")
    body = await reader.readexactly(length)
    value = json.loads(body.decode("utf-8"))
    if not isinstance(value, dict):
        raise ValueError("packet JSON must be an object")
    return value


async def write_async(writer: asyncio.StreamWriter, message: dict[str, Any]) -> None:
    writer.write(encode_packet(message))
    await writer.drain()


def recv_socket(sock: socket.socket) -> dict[str, Any]:
    header = _recv_exact(sock, HEADER.size)
    (length,) = HEADER.unpack(header)
    if length == 0 or length > MAX_PACKET_BYTES:
        raise ValueError(f"invalid packet size: {length}")
    value = json.loads(_recv_exact(sock, length).decode("utf-8"))
    if not isinstance(value, dict):
        raise ValueError("packet JSON must be an object")
    return value


def _recv_exact(sock: socket.socket, count: int) -> bytes:
    chunks: list[bytes] = []
    remaining = count
    while remaining:
        chunk = sock.recv(remaining)
        if not chunk:
            raise EOFError("socket closed inside packet")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


class FramedSocketReader:
    """Keep partial headers/payloads across socket timeouts; one per connection."""
    def __init__(self, max_bytes: int = MAX_PACKET_BYTES):
        self.max_bytes = max_bytes
        self.buffer = bytearray()

    def receive_payload(self, connection: socket.socket) -> bytes:
        while True:
            if len(self.buffer) >= HEADER.size:
                length = HEADER.unpack_from(self.buffer)[0]
                if not 0 < length <= self.max_bytes:
                    raise ValueError(f"invalid packet size: {length}")
                end = HEADER.size + length
                if len(self.buffer) >= end:
                    payload = bytes(self.buffer[HEADER.size:end])
                    del self.buffer[:end]
                    return payload
            chunk = connection.recv(64 * 1024)
            if not chunk:
                raise EOFError("socket closed inside packet")
            self.buffer.extend(chunk)

    def receive(self, connection: socket.socket) -> dict[str, Any]:
        value = json.loads(self.receive_payload(connection).decode("utf-8"))
        if not isinstance(value, dict):
            raise ValueError("packet JSON must be an object")
        return value


def check_backend_capabilities(host: str, port: int, timeout: float = 3.0) -> dict[str, Any]:
    """Non-owning preflight: never registers a simulator or displaces a live one."""
    try:
        with socket.create_connection((host, port), timeout=timeout) as connection:
            connection.settimeout(timeout)
            connection.sendall(encode_packet({"protocol": CONTROL_PROTOCOL, "type": "capabilities_probe"}))
            reply = FramedSocketReader().receive(connection)
        if (reply.get("protocol") != CONTROL_PROTOCOL or reply.get("type") != "backend_capabilities"
                or not set(RUNTIME_CAPABILITIES).issubset(reply.get("capabilities", []))):
            raise ValueError("backend is missing the required state/RGB acknowledgement protocols")
        return reply
    except (OSError, EOFError, ValueError, TypeError) as error:
        raise RuntimeError(
            f"Backend compatibility check failed at {host}:{port}: {error}. "
            "Restart/update the whole platform backend, not only the UE scene; "
            "refusing to start physics with incompatible capture protocols."
        ) from error
