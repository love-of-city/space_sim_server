"""Route smooth preview frames separately from authoritative dataset captures."""

from __future__ import annotations

import json
import queue
import socket
import struct
import threading
import time
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
        self._authoritative_thread: threading.Thread | None = None
        # Keep the transport buffer bounded to the recorder's pairing window;
        # this prevents a slow disk/encoder from consuming unbounded RAM.
        self._authoritative_queue: queue.Queue[tuple[dict[str, Any], dict[str, bytes]] | None] = queue.Queue(maxsize=128)
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
        self._authoritative_thread = threading.Thread(
            target=self._authoritative_worker,
            name="bsk-authoritative-capture-writer",
            daemon=True,
        )
        self._authoritative_thread.start()
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
        writer = self._authoritative_thread
        if writer:
            deadline = time.monotonic() + 10.0
            while self._authoritative_queue.unfinished_tasks and time.monotonic() < deadline:
                time.sleep(0.01)
            # The sentinel is queued after packets already accepted by the
            # receiver, so the writer drains those packets before exiting.
            while True:
                try:
                    self._authoritative_queue.put(None, timeout=0.1)
                    break
                except queue.Full:
                    if time.monotonic() >= deadline:
                        break
            writer.join(timeout=max(0.0, deadline - time.monotonic()))
        self._authoritative_thread = None

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
            # Keep the socket reader cheap. Image decoding, synchronization,
            # disk writes, and LeRobot encoding run on a separate worker so a
            # slow writer cannot make UE's reliable capture queue overflow.
            if self._authoritative_thread and self._authoritative_thread.is_alive():
                with self._condition:
                    self.authoritative_count += 1
                self._authoritative_queue.put((metadata, products), timeout=10.0)
            else:
                with self._condition:
                    self.authoritative_count += 1
                self._process_authoritative_capture(metadata, products)
            return
        raise ValueError(
            f"capture must declare preview or authoritative state, got stream={stream_kind!r}, state={state_kind!r}"
        )

    def wait_for_authoritative_idle(self, timeout_s: float = 10.0) -> bool:
        """Wait until all accepted authoritative packets finish processing."""
        deadline = time.monotonic() + max(0.0, timeout_s)
        while self._authoritative_queue.unfinished_tasks:
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.01)
        return True

    def _process_authoritative_capture(self, metadata: dict[str, Any], products: dict[str, bytes]) -> None:
        try:
            self.on_authoritative_capture(metadata, products)
            with self._condition:
                self.last_authoritative_error = None
        except Exception as error:
            # A bad frame must be visible in status but must not tear down the
            # TCP stream and discard all subsequent camera frames.
            with self._condition:
                self.last_authoritative_error = str(error)

    def _authoritative_worker(self) -> None:
        while True:
            try:
                item = self._authoritative_queue.get()
            except Exception:
                continue
            if item is None:
                self._authoritative_queue.task_done()
                return
            try:
                self._process_authoritative_capture(*item)
            finally:
                self._authoritative_queue.task_done()

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
