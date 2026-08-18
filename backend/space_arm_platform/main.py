"""Command-line entry point for the platform backend."""

from __future__ import annotations

import argparse
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
    parser.add_argument("--data-root", type=Path)
    parser.add_argument("--log-level", default="info")
    args = parser.parse_args()
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
        )
    )
    uvicorn.run(app, host=args.host, port=args.port, log_level=args.log_level)


if __name__ == "__main__":
    main()
