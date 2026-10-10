"""Check team mesh bindings and optionally display recorded states in UE."""

import argparse
import copy
import json
import socket
import time
import uuid
from itertools import pairwise
from pathlib import Path

from bsk_render_adapter.protocol import encode_packet
from bsk_render_adapter.recording import BskRecordingReader


def display(path, focus, distance, port):
    messages = list(BskRecordingReader(path).messages())
    session = str(uuid.uuid4())
    header = next(m for m in messages if m["type"] == "hello")
    manifest = next(m for m in messages if m["type"] == "scene_manifest")
    frames = [m for m in messages if m["type"] == "frame"]
    manifest = copy.deepcopy(manifest)
    manifest["settings"]["default_camera_target"] = focus
    manifest["settings"]["default_camera_distance_m"] = distance
    with socket.create_connection(("127.0.0.1", port), timeout=5) as stream:
        for message in (header, manifest):
            message = {**message, "session_id": session}
            stream.sendall(encode_packet(message))
        # Keep supplying the final recorded state while the asset manifest is
        # compiled by UE. Only its presentation sequence is advanced.
        for i in range(40):
            frame = {**frames[-1], "session_id": session, "frame_id": str(i + 1)}
            stream.sendall(encode_packet(frame))
            time.sleep(0.1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--display", action="store_true")
    parser.add_argument("--previous", type=Path)
    parser.add_argument("--focus", default="team/cubesat_bus")
    parser.add_argument("--distance", type=float, default=2.5)
    parser.add_argument("--port", type=int, default=5558)
    args = parser.parse_args()
    messages = list(BskRecordingReader(args.recording).messages())
    manifest = next(m for m in messages if m["type"] == "scene_manifest")
    frames = [m for m in messages if m["type"] == "frame"]
    geometries = [g for o in manifest["objects"] for g in o.get("geometries", [])]
    meshes = [g for g in geometries if g["shape"] == "mesh"]
    times = [int(f["sim_time_ns"]) for f in frames]
    checks = {
        "team_objects": len(manifest["objects"]) == 15,
        "all_geometries": len(geometries) == 225,
        "all_mesh_instances": len(meshes) == 217,
        "all_team_assets_resolved": all(
            g.get("asset_path", "").startswith("/Game/BSK/Generated/SARM/")
            for g in meshes
        ),
        "strict_time_order": all(b > a for a, b in pairwise(times)),
        "frame_body_coverage": all(len(f["objects"]) == 15 for f in frames),
        "team_celestial_assets": {b["body_id"] for b in manifest["celestial_bodies"]} == {"earth", "sun"},
        "sun_drives_lighting": any(b["body_id"] == "sun" and b["drives_directional_light"]
                                   for b in manifest["celestial_bodies"]),
        "celestial_frame_coverage": all({b["body_id"] for b in f["celestial_bodies"]} == {"earth", "sun"}
                                        for f in frames),
    }
    result = {
        "passed": all(checks.values()),
        "checks": checks,
        "frames": len(frames),
        "first_time_s": times[0] * 1e-9,
        "last_time_s": times[-1] * 1e-9,
    }
    (args.recording.parent / "recording_validation.json").write_text(
        json.dumps(result, indent=2)
    )
    print(json.dumps(result))
    if args.display:
        if args.previous:
            display(args.previous, "rendezvous/target", 10.0, args.port)
        display(args.recording, args.focus, args.distance, args.port)
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
