"""Listen for non-blocking UE camera products and optionally save them."""

from __future__ import annotations

import argparse
import socket
from pathlib import Path

from bsk_render_adapter.capture import receive_capture_packet, save_capture_frame


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5560)
    parser.add_argument("--output", type=Path, default=Path("Saved/BskCaptureNetwork"))
    args = parser.parse_args()
    with socket.create_server((args.host, args.port), reuse_port=False) as listener:
        print(f"BSK capture receiver listening on {args.host}:{args.port}")
        while True:
            connection, address = listener.accept()
            print(f"UE capture source connected: {address[0]}:{address[1]}")
            with connection:
                try:
                    while True:
                        metadata, products = receive_capture_packet(connection)
                        path = save_capture_frame(args.output, metadata, products)
                        print(
                            f"camera={metadata['camera_id']} sim_time_ns={metadata['sim_time_ns']} "
                            f"products={','.join(products)} metadata={path}"
                        )
                except EOFError:
                    print("UE capture source disconnected; waiting for reconnect")


if __name__ == "__main__":
    main()
