"""Small, process-local protections for the single-worker operator service."""

from __future__ import annotations

import math
import time
from collections import OrderedDict
from urllib.parse import urlsplit


def validate_origins(origins: tuple[str, ...]) -> frozenset[str]:
    """Require exact serialized origins, never wildcards, paths or opaque origins."""
    for origin in origins:
        url = urlsplit(origin)
        if (
            url.scheme not in {"http", "https"} or not url.hostname
            or url.username is not None or url.password is not None
            or url.path or url.query or url.fragment
            or origin != f"{url.scheme}://{url.netloc}" or "*" in origin
            or any(char.isspace() for char in origin)
        ):
            raise ValueError(f"Invalid allowed origin: {origin!r}; use scheme://host[:port]")
        # Accessing port also rejects malformed ports.
        if url.port is not None and not 1 <= url.port <= 65535:
            raise ValueError("Origin port is out of range")
    return frozenset(origins)


class LoginRateLimiter:
    """Bounded per-client fixed window; no credentials or usernames are retained.

    This deliberately stays in one process, like the simulation/control state.
    Only trust proxy headers from the local reverse proxy when keying by client IP.
    """

    def __init__(self, limit: int = 10, window_s: float = 60.0, max_clients: int = 4096):
        if limit < 1 or window_s <= 0 or max_clients < 1:
            raise ValueError("Login rate limit settings must be positive")
        self.limit = limit
        self.window_s = window_s
        self.max_clients = max_clients
        self._clients: OrderedDict[str, tuple[float, int]] = OrderedDict()

    def retry_after(self, client: str, *, now: float | None = None) -> int:
        now = time.monotonic() if now is None else now
        start, count = self._clients.pop(client, (now, 0))
        if now - start >= self.window_s:
            start, count = now, 0
        self._clients[client] = (start, count + 1)
        if len(self._clients) > self.max_clients:
            self._clients.popitem(last=False)
        return max(1, math.ceil(self.window_s - (now - start))) if count >= self.limit else 0
