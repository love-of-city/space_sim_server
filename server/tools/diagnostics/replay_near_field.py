"""Render a recorded native near-field trajectory; never drives the physics.

Requires an already running local UE receiver. --at selects one recorded frame;
otherwise all recorded frames replay at --speed. Web task result remains saved.
"""
import argparse
import json
from pathlib import Path
import socket
import time
import uuid

from bsk_render_adapter.protocol import encode_packet
from bsk_render_adapter.recording import BskRecordingReader


def replay(folder, port, speed, at):
    session = "replay-"+uuid.uuid4().hex
    hello = manifest = frame = None
    events = []
    selected_events = []
    if at is not None:
        for item in BskRecordingReader(folder/"near_field.bskrec").messages():
            if item["type"] == "hello":
                hello = item
            elif item["type"] == "scene_manifest":
                manifest = item
            elif item["type"] == "event":
                events.append(item)
                events = events[-5:]
            elif item["type"] == "frame" and int(item["sim_time_ns"])*1e-9 <= at:
                frame, selected_events = item, events.copy()
        if frame is None:
            raise ValueError("No recorded frame at requested time")
        stream = [hello, manifest, frame, *selected_events]
    else:
        stream = BskRecordingReader(folder/"near_field.bskrec").messages()
    previous_stamp, wall = None, time.monotonic()
    sent = 0
    with socket.create_connection(("127.0.0.1", port), timeout=5) as channel:
        for item in stream:
            item = {**item, "session_id": session}
            if item["type"] == "scene_manifest":
                item["settings"] = {**item["settings"], "fill_light_intensity_lux": 100000.,
                                    "headlight_enabled": True, "headlight_diffuse_rgb": [10000., 10000., 10000.]}
            if item["type"] == "event":
                item["payload"] = {**item["payload"], "message": "REPLAY | "+item["payload"].get("message", "")}
            if item["type"] == "frame":
                stamp = int(item["sim_time_ns"])*1e-9
                if at is None and previous_stamp is not None:
                    time.sleep(max(0., (stamp-previous_stamp)/speed-(time.monotonic()-wall)))
                previous_stamp, wall = stamp, time.monotonic()
                sent += 1
            channel.sendall(encode_packet(item))
        time.sleep(2)  # allow the receiver to present its buffered final frame
    print(json.dumps({"mode": "REPLAY ONLY", "recording": str(folder), "frames": sent, "last_recorded_time_s": previous_stamp}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--port", type=int, default=5558)
    parser.add_argument("--speed", type=float, default=20.)
    parser.add_argument("--at", type=float)
    args = parser.parse_args()
    if not 0.1 <= args.speed <= 100 or not 1 <= args.port <= 65535:
        parser.error("speed must be in [0.1,100], port in [1,65535]")
    replay(args.run_dir, args.port, args.speed, args.at)
