"""Decode and persist camera data products emitted by the UE runtime.

Wire format ``bsk-capture/1``::

    uint32_be payload_length
    uint32_be metadata_json_length
    metadata_json_utf8
    concatenated product blobs

Blob offsets in metadata are relative to the start of the concatenated blob
area.  All 64-bit counters and timestamps remain decimal strings.
"""

from __future__ import annotations

import json
import socket
import struct
from pathlib import Path
from typing import Any, BinaryIO


CAPTURE_PROTOCOL = "bsk-capture/1"
HEADER = struct.Struct("!I")
DEFAULT_MAX_CAPTURE_BYTES = 512 * 1024 * 1024


def _read_exact(stream: BinaryIO | socket.socket, count: int) -> bytes:
    chunks: list[bytes] = []
    remaining = count
    while remaining:
        chunk = stream.recv(remaining) if isinstance(stream, socket.socket) else stream.read(remaining)
        if not chunk:
            raise EOFError("capture stream closed inside a packet")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def decode_capture_payload(payload: bytes) -> tuple[dict[str, Any], dict[str, bytes]]:
    """Decode one payload after its outer length prefix."""

    if len(payload) < HEADER.size:
        raise ValueError("capture payload is shorter than its JSON-length field")
    (metadata_length,) = HEADER.unpack_from(payload)
    if metadata_length == 0 or metadata_length > len(payload) - HEADER.size:
        raise ValueError("invalid capture metadata length")
    metadata_end = HEADER.size + metadata_length
    metadata = json.loads(payload[HEADER.size:metadata_end].decode("utf-8"))
    if not isinstance(metadata, dict) or metadata.get("protocol") != CAPTURE_PROTOCOL:
        raise ValueError("unsupported capture metadata protocol")
    products = metadata.get("products")
    if not isinstance(products, list):
        raise ValueError("capture metadata products must be an array")
    blob_area = payload[metadata_end:]
    decoded: dict[str, bytes] = {}
    occupied: list[tuple[int, int]] = []
    for product in products:
        if not isinstance(product, dict):
            raise ValueError("capture product metadata must be an object")
        name = product.get("name")
        if not isinstance(name, str) or not name or name in decoded:
            raise ValueError("capture product names must be unique non-empty strings")
        try:
            offset = int(product["blob_offset"])
            length = int(product["byte_length"])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"invalid offset/length for capture product {name!r}") from error
        end = offset + length
        if offset < 0 or length < 0 or end > len(blob_area):
            raise ValueError(f"capture product {name!r} lies outside the blob area")
        if any(offset < old_end and old_start < end for old_start, old_end in occupied):
            raise ValueError(f"capture product {name!r} overlaps another product")
        occupied.append((offset, end))
        decoded[name] = blob_area[offset:end]
    return metadata, decoded


def receive_capture_packet(
    stream: BinaryIO | socket.socket,
    *,
    max_packet_bytes: int = DEFAULT_MAX_CAPTURE_BYTES,
) -> tuple[dict[str, Any], dict[str, bytes]]:
    """Read and decode one complete length-prefixed capture packet."""

    (length,) = HEADER.unpack(_read_exact(stream, HEADER.size))
    if length < HEADER.size or length > max_packet_bytes:
        raise ValueError(f"invalid capture packet size: {length}")
    return decode_capture_payload(_read_exact(stream, length))


def save_capture_frame(
    output_directory: str | Path,
    metadata: dict[str, Any],
    products: dict[str, bytes],
) -> Path:
    """Persist one validated frame without trusting sender-supplied paths."""

    root = Path(output_directory)
    session = _safe_segment(str(metadata.get("session_id", "session")))
    camera = _safe_segment(str(metadata.get("camera_id", "camera")))
    sequence = _safe_segment(str(metadata.get("capture_sequence", "0")))
    directory = root / session / camera
    directory.mkdir(parents=True, exist_ok=True)
    for product in metadata["products"]:
        name = product["name"]
        if name not in products:
            raise ValueError(f"capture product {name!r} is missing")
        file_name = Path(str(product.get("file_name", f"{sequence}_{name}.bin"))).name
        (directory / file_name).write_bytes(products[name])
    metadata_path = directory / f"{sequence}.json"
    metadata_path.write_text(json.dumps(metadata, indent=2, ensure_ascii=False), encoding="utf-8")
    return metadata_path


def _safe_segment(value: str) -> str:
    safe = "".join("_" if char in '/\\:*?\"<>|' else char for char in value).strip(" .")
    return safe or "unnamed"
