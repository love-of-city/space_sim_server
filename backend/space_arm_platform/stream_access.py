"""Small HS256 JWT implementation for scoped Pixel Streaming access."""

from __future__ import annotations

import base64
import hashlib
import hmac
import json
import time
import uuid
from typing import Any, Iterable


def _encode(value: bytes) -> str:
    return base64.urlsafe_b64encode(value).rstrip(b"=").decode("ascii")


def _decode(value: str) -> bytes:
    return base64.urlsafe_b64decode(value + "=" * (-len(value) % 4))


def issue_stream_access_token(
    secret: str,
    streamer_ids: Iterable[str],
    *,
    subject: str = "operator",
    ttl_seconds: int = 900,
) -> str:
    if not secret:
        raise ValueError("JWT secret must not be empty")
    now = int(time.time())
    header = {"alg": "HS256", "typ": "JWT"}
    payload = {
        "sub": subject,
        "role": "operator",
        "iat": now,
        "exp": now + max(30, int(ttl_seconds)),
        "jti": uuid.uuid4().hex,
        "streamer_ids": list(dict.fromkeys(streamer_ids)),
    }
    encoded_header = _encode(json.dumps(header, separators=(",", ":")).encode("utf-8"))
    encoded_payload = _encode(json.dumps(payload, separators=(",", ":")).encode("utf-8"))
    signed = f"{encoded_header}.{encoded_payload}"
    signature = hmac.new(secret.encode("utf-8"), signed.encode("ascii"), hashlib.sha256).digest()
    return f"{signed}.{_encode(signature)}"


def verify_stream_access_token(secret: str, token: str) -> dict[str, Any]:
    header_value, payload_value, signature_value = token.split(".")
    signed = f"{header_value}.{payload_value}"
    expected = hmac.new(secret.encode("utf-8"), signed.encode("ascii"), hashlib.sha256).digest()
    if not hmac.compare_digest(expected, _decode(signature_value)):
        raise ValueError("invalid token signature")
    header = json.loads(_decode(header_value))
    payload = json.loads(_decode(payload_value))
    if header != {"alg": "HS256", "typ": "JWT"}:
        raise ValueError("unsupported token header")
    if int(payload.get("exp", 0)) <= int(time.time()):
        raise ValueError("token expired")
    if not isinstance(payload.get("streamer_ids"), list):
        raise ValueError("token has no streamer permission list")
    return payload
