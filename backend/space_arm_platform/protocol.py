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
