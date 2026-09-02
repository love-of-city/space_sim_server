"""Command-line entry point for the platform backend."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import uvicorn

from .app import PlatformConfig, create_app


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--simulation-host", default="127.0.0.1")
    parser.add_argument("--simulation-port", type=int, default=8766)
    parser.add_argument("--capture-host", default="127.0.0.1")
    parser.add_argument("--capture-port", type=int, default=8767)
    parser.add_argument("--pixel-streaming-player-port", type=int, default=8080)
    parser.add_argument("--pixel-streaming-streamer-id", default="BskRenderer")
    parser.add_argument("--pixel-streaming-signalling-url", default="")
    parser.add_argument("--stream-access-jwt-secret", default="")
    parser.add_argument("--stream-access-key", default="")
    parser.add_argument("--stream-access-token-ttl-seconds", type=int, default=900)
    parser.add_argument(
        "--pixel-streaming-camera-streamer",
        action="append",
        default=[],
        metavar="ID=LABEL",
        help="Additional UE RenderTarget streamer exposed by the browser camera selector.",
    )
    parser.add_argument("--data-root", type=Path)
    parser.add_argument("--auth-database", type=Path)
    parser.add_argument(
        "--bootstrap-admin-username", default=os.environ.get("SPACE_SIM_ADMIN_USERNAME", "admin")
    )
    parser.add_argument(
        "--bootstrap-admin-password", default=os.environ.get("SPACE_SIM_ADMIN_PASSWORD", "ChangeMe123!")
    )
    parser.add_argument("--runtime-adapter-root", type=Path)
    parser.add_argument("--runtime-model-root", type=Path)
    parser.add_argument("--runtime-unreal-root", type=Path)
    parser.add_argument("--runtime-powershell-exe", type=Path)
    parser.add_argument("--runtime-render-port", type=int, default=5558)
    parser.add_argument("--runtime-pixel-streamer-port", type=int, default=8888)
    parser.add_argument("--runtime-pixel-streaming-camera-id", action="append", default=[])
    parser.add_argument("--runtime-pixel-streaming-camera-width", type=int, default=640)
    parser.add_argument("--runtime-pixel-streaming-camera-height", type=int, default=360)
    parser.add_argument("--runtime-preview-rate", type=float, default=60.0)
    parser.add_argument("--runtime-renderer-ready-timeout", type=int, default=240)
    parser.add_argument("--runtime-ik-rate", type=float, default=100.0)
    parser.add_argument("--runtime-simulation-rate", type=float, default=1.0)
    parser.add_argument("--runtime-capture-rate", type=float, default=10.0)
    parser.add_argument("--runtime-default-dataset-capture", action="store_true")
    parser.add_argument("--log-level", default="info")
    args = parser.parse_args()
    camera_streamers: list[tuple[str, str]] = []
    for value in args.pixel_streaming_camera_streamer:
        streamer_id, separator, label = value.partition("=")
        if not streamer_id.strip():
            parser.error("--pixel-streaming-camera-streamer requires a non-empty ID")
        camera_streamers.append((streamer_id.strip(), label.strip() if separator else streamer_id.strip()))
    project_root = Path(__file__).resolve().parents[2]
    app = create_app(
        PlatformConfig(
            project_root=project_root,
            data_root=(args.data_root or project_root / "data" / "episodes").resolve(),
            simulation_host=args.simulation_host,
            simulation_port=args.simulation_port,
            capture_host=args.capture_host,
            capture_port=args.capture_port,
            pixel_streaming_player_port=args.pixel_streaming_player_port,
            pixel_streaming_streamer_id=args.pixel_streaming_streamer_id,
            pixel_streaming_signalling_url=args.pixel_streaming_signalling_url,
            pixel_streaming_camera_streamers=tuple(camera_streamers),
            stream_access_jwt_secret=args.stream_access_jwt_secret,
            stream_access_key=args.stream_access_key,
            stream_access_token_ttl_seconds=args.stream_access_token_ttl_seconds,
            auth_database=args.auth_database.resolve() if args.auth_database else None,
            bootstrap_admin_username=args.bootstrap_admin_username,
            bootstrap_admin_password=args.bootstrap_admin_password,
            runtime_adapter_root=args.runtime_adapter_root.resolve() if args.runtime_adapter_root else None,
            runtime_model_root=args.runtime_model_root.resolve() if args.runtime_model_root else None,
            runtime_unreal_root=args.runtime_unreal_root.resolve() if args.runtime_unreal_root else None,
            runtime_powershell_exe=args.runtime_powershell_exe.resolve() if args.runtime_powershell_exe else None,
            runtime_render_port=args.runtime_render_port,
            runtime_pixel_streamer_port=args.runtime_pixel_streamer_port,
            runtime_pixel_streaming_camera_ids=tuple(args.runtime_pixel_streaming_camera_id),
            runtime_pixel_streaming_camera_width=args.runtime_pixel_streaming_camera_width,
            runtime_pixel_streaming_camera_height=args.runtime_pixel_streaming_camera_height,
            runtime_preview_rate=args.runtime_preview_rate,
            runtime_renderer_ready_timeout=args.runtime_renderer_ready_timeout,
            runtime_ik_rate=args.runtime_ik_rate,
            runtime_simulation_rate=args.runtime_simulation_rate,
            runtime_capture_rate=args.runtime_capture_rate,
            runtime_default_dataset_capture=args.runtime_default_dataset_capture,
        )
    )
    uvicorn.run(app, host=args.host, port=args.port, log_level=args.log_level)


if __name__ == "__main__":
    main()
