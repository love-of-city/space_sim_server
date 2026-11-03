"""Browser streaming configuration, live telemetry and camera preview routes."""
from __future__ import annotations

import asyncio
import hmac
from collections.abc import AsyncIterator
from typing import Any

from fastapi import HTTPException, Request
from fastapi.responses import Response, StreamingResponse

from .stream_access import issue_stream_access_token


def install_media_routes(app, *, config, safety, hub, recorder, captures, scenes):
    def authorize_access_key(candidate: str | None) -> None:
        if config.stream_access_key and not hmac.compare_digest(candidate or "", config.stream_access_key):
            raise HTTPException(status_code=401, detail="a valid stream access key is required")

    @app.get("/api/client-config")
    async def client_config(request: Request) -> dict[str, Any]:
        authorize_access_key(request.headers.get("x-space-arm-access-key") or request.query_params.get("access_key"))
        streamers = [
            {"id": config.pixel_streaming_streamer_id, "label": "主视口"},
            *(
                {"id": streamer_id, "label": label}
                for streamer_id, label in config.pixel_streaming_camera_streamers
            ),
        ]
        response = {
            "preview_transport": "pixel_streaming_2",
            "pixel_streaming_player_port": config.pixel_streaming_player_port,
            "pixel_streaming_streamer_id": config.pixel_streaming_streamer_id,
            "pixel_streaming_signalling_url": config.pixel_streaming_signalling_url,
            "pixel_streaming_streamers": streamers,
            "pixel_streaming_fps": round(config.runtime_preview_rate),
            "pixel_streaming_encoder_min_quality": config.runtime_encoder_min_quality,
        }
        if config.stream_access_jwt_secret:
            response["pixel_streaming_access_token"] = issue_stream_access_token(
                config.stream_access_jwt_secret,
                (item["id"] for item in streamers),
                ttl_seconds=config.stream_access_token_ttl_seconds,
            )
        return response

    @app.get("/api/state")
    async def state() -> dict[str, Any]:
        action = safety.last_action
        return {
            "simulation": hub.status(),
            "active_episode": recorder.episode_id,
            "episode_directory": str(recorder.episode_directory) if recorder.episode_directory else None,
            "latest_action": action.model_dump(mode="json") if action else None,
            "cameras": captures.camera_ids(),
            "capture_channels": captures.status(),
            "capture_sync": recorder.sync_status(),
            "scene_runtime": scenes.status(),
        }

    @app.get("/api/cameras")
    async def cameras() -> dict[str, Any]:
        return {"cameras": captures.camera_ids()}

    @app.get("/api/preview/{camera_id:path}")
    async def preview(camera_id: str) -> Response:
        frame = captures.latest(camera_id)
        if frame is None:
            raise HTTPException(status_code=404, detail="camera has not produced an RGB frame")
        return Response(frame.data, media_type=frame.content_type, headers={"Cache-Control": "no-store"})

    @app.get("/api/preview-stream")
    async def preview_stream(camera: str | None = None) -> StreamingResponse:
        async def frames() -> AsyncIterator[bytes]:
            revision = 0
            while True:
                frame = await asyncio.to_thread(captures.wait_for_frame, camera, revision, 2.0)
                if frame is None:
                    await asyncio.sleep(0.02)
                    continue
                revision = frame.revision
                yield (
                    b"--frame\r\n"
                    + f"Content-Type: {frame.content_type}\r\nContent-Length: {len(frame.data)}\r\n\r\n".encode("ascii")
                    + frame.data
                    + b"\r\n"
                )

        return StreamingResponse(
            frames(),
            media_type="multipart/x-mixed-replace; boundary=frame",
            headers={"Cache-Control": "no-store"},
        )
