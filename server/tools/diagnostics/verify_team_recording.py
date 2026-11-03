"""Check team mesh bindings and optionally display recorded states in UE."""

import argparse
import copy
import json
import socket
import time
import uuid
from itertools import pairwise
from pathlib import Path

import numpy as np

from bsk_render_adapter.protocol import encode_packet
from bsk_render_adapter.recording import BskRecordingReader


def verify_relative_positions(folder, manifest, frames):
    """Compare recorded body COMs against the independent raw dynamics states."""
    model = json.loads((folder / "model_manifest.json").read_text(encoding="utf-8"))
    names = ("capture_target", "cubesat_bus")
    centers = [np.asarray(model["bodies"][name]["center_body_m"]) for name in names]
    # Read compressed arrays once; indexing NpzFile in the frame loop repeatedly
    # decompresses a complete multi-hour flight and makes the audit quadratic.
    with np.load(folder / "team_telemetry.npz", allow_pickle=False) as data:
        times, truth = data["time_s"], data["truth"]
    errors = []
    for frame in frames:
        t = int(frame["sim_time_ns"]) * 1e-9
        index = int(np.searchsorted(times, t - 1e-7))
        if index >= len(times) or abs(times[index] - t) > 1e-7:
            raise ValueError(f"Rendered frame has no matching raw state: {t}")
        objects = {o["object_id"]: o for o in frame["objects"]}
        positions = []
        for name, center in zip(names, centers):
            obj = objects["team/" + name]
            w, x, y, z = obj["orientation_wxyz"]
            v = np.array([x, y, z])
            rotation = ((w*w - v@v) * np.eye(3) + 2*np.outer(v, v)
                        + 2*w*np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]]))
            positions.append(np.asarray(obj["position_m"]) + rotation @ center)
        rendered = positions[1] - positions[0]
        actual = truth[index, 1, :3] - truth[index, 0, :3]
        errors.append(float(np.linalg.norm(rendered - actual)))
    view = manifest.get("settings", {}).get("relative_view")
    view_matches = view is None or (
        view["primary_object_id"] == "team/cubesat_bus"
        and view["secondary_object_id"] == "team/capture_target"
        and np.allclose(view["primary_center_body_m"], centers[1], rtol=0, atol=1e-12)
        and np.allclose(view["secondary_center_body_m"], centers[0], rtol=0, atol=1e-12))
    maximum = max(errors, default=float("inf"))
    return {"passed": bool(len(errors) == len(times) and maximum < 1e-6 and view_matches),
            "frames": len(errors), "max_relative_position_error_m": maximum,
            "relative_view_matches_model": bool(view_matches), "relative_view_enabled": view is not None}


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
    visible = [g for g in geometries if g.get("color_rgba", [1, 1, 1, 1])[3] > 0]
    times = [int(f["sim_time_ns"]) for f in frames]
    relative = verify_relative_positions(args.recording.parent, manifest, frames)
    checks = {
        "team_objects": len(manifest["objects"]) == 15,
        # PR #17 omits five fully transparent collision boxes from rendering;
        # they remain in the unchanged physics MJCF. Legacy recordings include
        # them. Require the same 217 meshes and three visible wheel cylinders.
        "all_visible_geometries": len(visible) == 220,
        "only_transparent_collision_helpers_omitted": all(
            g["render_role"] == "collision" for g in geometries if g not in visible),
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
        "relative_positions_match_raw_dynamics": relative["passed"],
    }
    result = {
        "passed": all(checks.values()),
        "checks": checks,
        "frames": len(frames),
        "visible_geometries": len(visible),
        "transparent_helpers": len(geometries) - len(visible),
        "first_time_s": times[0] * 1e-9,
        "last_time_s": times[-1] * 1e-9,
        "relative_positions": relative,
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
