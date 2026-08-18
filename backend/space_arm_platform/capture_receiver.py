"""Route smooth preview frames separately from authoritative dataset captures."""

from __future__ import annotations

import json
import socket
import struct
import threading
from dataclasses import dataclass
from typing import Any, Callable


HEADER = struct.Struct("!I")
MAX_CAPTURE_BYTES = 512 * 1024 * 1024


@dataclass(frozen=True)
class PreviewFrame:
    camera_id: str
    revision: int
    content_type: str
    data: bytes
    metadata: dict[str, Any]


class CaptureReceiver:
    def __init__(self, host: str, port: int, on_authoritative_capture: Callable[[dict[str, Any], dict[str, bytes]], None]) -> None:
        self.host = host
        self.port = int(port)
        self.on_authoritative_capture = on_authoritative_capture
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._listener: socket.socket | None = None
        self._condition = threading.Condition()
        self._frames: dict[str, PreviewFrame] = {}
        self._revision = 0
        self.last_error: str | None = None
        self.last_authoritative_error: str | None = None
        self.preview_count = 0
        self.authoritative_count = 0

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._worker, name="bsk-capture-receiver", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._listener:
            try:
                self._listener.close()
            except OSError:
                pass
        if self._thread:
            self._thread.join(timeout=2.0)

    def camera_ids(self) -> list[str]:
        with self._condition:
            return sorted(self._frames)

    def latest(self, camera_id: str | None = None) -> PreviewFrame | None:
        with self._condition:
            if camera_id and camera_id in self._frames:
                return self._frames[camera_id]
            if not self._frames:
                return None
            preferred = next((key for key in self._frames if "overview" in key.lower()), None)
            return self._frames[preferred or sorted(self._frames)[0]]

    def wait_for_frame(self, camera_id: str | None, revision: int, timeout_s: float = 2.0) -> PreviewFrame | None:
        with self._condition:
            self._condition.wait_for(
                lambda: self._stop.is_set()
                or ((frame := self.latest(camera_id)) is not None and frame.revision > revision),
                timeout=timeout_s,
            )
            frame = self.latest(camera_id)
            return frame if frame and frame.revision > revision else None

    def status(self) -> dict[str, Any]:
        with self._condition:
            return {
                "preview_cameras": sorted(self._frames),
                "preview_count": self.preview_count,
                "authoritative_count": self.authoritative_count,
                "last_error": self.last_error,
                "last_authoritative_error": self.last_authoritative_error,
            }

    def route_capture(self, metadata: dict[str, Any], products: dict[str, bytes]) -> None:
        """Route one decoded packet; preview bytes never enter the episode recorder."""
        stream_kind = str(metadata.get("stream_kind", ""))
        state_kind = str(metadata.get("state_kind", ""))
        if stream_kind == "preview":
            rgb = products.get("rgb")
            if not rgb:
                raise ValueError("preview packet must contain rgb")
            item = next(
                (entry for entry in metadata.get("products", []) if entry.get("name") == "rgb"),
                {},
            )
            file_name = str(item.get("file_name", "rgb.jpg")).lower()
            content_type = "image/jpeg" if file_name.endswith((".jpg", ".jpeg")) else "image/png"
            camera_id = str(metadata.get("camera_id", "camera"))
            with self._condition:
                self._revision += 1
                self.preview_count += 1
                self._frames[camera_id] = PreviewFrame(
                    camera_id, self._revision, content_type, rgb, metadata
                )
                self._condition.notify_all()
            return
        if stream_kind == "authoritative" and state_kind == "authoritative":
            try:
                self.on_authoritative_capture(metadata, products)
                self.last_authoritative_error = None
            except Exception as error:
                self.last_authoritative_error = str(error)
                raise
            with self._condition:
                self.authoritative_count += 1
            return
        raise ValueError(
            f"capture must declare preview or authoritative state, got stream={stream_kind!r}, state={state_kind!r}"
        )

    def _worker(self) -> None:
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener = listener
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((self.host, self.port))
        listener.listen(2)
        listener.settimeout(0.5)
        while not self._stop.is_set():
            try:
                connection, _ = listener.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with connection:
                connection.settimeout(2.0)
                while not self._stop.is_set():
                    try:
                        metadata, products = self._receive(connection)
                        self.route_capture(metadata, products)
                        self.last_error = None
                    except socket.timeout:
                        continue
                    except (EOFError, OSError, ValueError, UnicodeError) as error:
                        self.last_error = str(error)
                        break
        try:
            listener.close()
        except OSError:
            pass

    @staticmethod
    def _receive(connection: socket.socket) -> tuple[dict[str, Any], dict[str, bytes]]:
        (length,) = HEADER.unpack(_recv_exact(connection, HEADER.size))
        if length < HEADER.size or length > MAX_CAPTURE_BYTES:
            raise ValueError(f"invalid capture packet size: {length}")
        payload = _recv_exact(connection, length)
        (metadata_length,) = HEADER.unpack_from(payload)
        if metadata_length == 0 or metadata_length > len(payload) - HEADER.size:
            raise ValueError("invalid capture metadata length")
        metadata_end = HEADER.size + metadata_length
        metadata = json.loads(payload[HEADER.size:metadata_end].decode("utf-8"))
        if not isinstance(metadata, dict) or metadata.get("protocol") != "bsk-capture/1":
            raise ValueError("unsupported capture protocol")
        blob_area = payload[metadata_end:]
        products: dict[str, bytes] = {}
        for item in metadata.get("products", []):
            name = str(item["name"])
            offset = int(item["blob_offset"])
            byte_length = int(item["byte_length"])
            end = offset + byte_length
            if offset < 0 or byte_length < 0 or end > len(blob_area) or name in products:
                raise ValueError(f"invalid product range: {name}")
            products[name] = blob_area[offset:end]
        return metadata, products


def _recv_exact(connection: socket.socket, count: int) -> bytes:
    chunks: list[bytes] = []
    remaining = count
    while remaining:
        chunk = connection.recv(remaining)
        if not chunk:
            raise EOFError("capture connection closed")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)
