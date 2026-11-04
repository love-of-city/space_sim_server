"""Print the simulation-time span of a BSK renderer recording in seconds."""

from __future__ import annotations

import argparse

from bsk_render_adapter import BskRecordingReader


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("recording")
    args = parser.parse_args()

    first_ns: int | None = None
    last_ns: int | None = None
    for message in BskRecordingReader(args.recording).messages():
        if message.get("type") != "frame":
            continue
        timestamp_ns = int(message["sim_time_ns"])
        if first_ns is None:
            first_ns = timestamp_ns
        last_ns = timestamp_ns

    if first_ns is None or last_ns is None:
        raise RuntimeError(f"Recording contains no frames: {args.recording}")

    span_seconds = max((last_ns - first_ns) / 1e9, 1.0)
    print(span_seconds)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
