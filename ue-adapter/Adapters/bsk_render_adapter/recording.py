"""Versioned recording and replay container for BSK render packets."""

from __future__ import annotations

from pathlib import Path
from typing import BinaryIO, Iterator

from .protocol import HEADER, MAX_PACKET_BYTES, decode_packet, encode_packet


MAGIC = b"BSKREC2\n"


class BskRecordingWriter:
    """Write protocol messages to a deterministic sequential recording."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self._stream: BinaryIO | None = None

    def open(self) -> None:
        """Create the recording and write its version marker."""

        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = self.path.open("wb")
        self._stream.write(MAGIC)

    def write(self, message: dict) -> None:
        """Append one protocol message."""

        if self._stream is None:
            self.open()
        assert self._stream is not None
        self._stream.write(encode_packet(message))

    def close(self) -> None:
        """Flush and close the recording."""

        if self._stream is not None:
            self._stream.close()
            self._stream = None

    def __enter__(self) -> "BskRecordingWriter":
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


class BskRecordingReader:
    """Iterate over validated messages from a ``.bskrec`` file."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)

    def messages(self) -> Iterator[dict]:
        """Yield messages in recorded order."""

        with self.path.open("rb") as stream:
            if stream.read(len(MAGIC)) != MAGIC:
                raise ValueError("not a BSKREC2 recording")
            while True:
                header = stream.read(HEADER.size)
                if not header:
                    return
                if len(header) != HEADER.size:
                    raise ValueError("truncated recording header")
                (length,) = HEADER.unpack(header)
                if length == 0 or length > MAX_PACKET_BYTES:
                    raise ValueError(f"invalid recorded packet length: {length}")
                body = stream.read(length)
                if len(body) != length:
                    raise ValueError("truncated recorded packet")
                yield decode_packet(header + body)
