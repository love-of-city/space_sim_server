"""Send a smooth two-spacecraft rendezvous without requiring Basilisk."""

from __future__ import annotations

import argparse
import math
import time
import uuid

from bsk_render_adapter import PROTOCOL_V2, RenderPublisher


def hello_message(session_id: str) -> dict:
    return {
        "protocol": PROTOCOL_V2,
        "type": "hello",
        "session_id": session_id,
        "capabilities": ["scene_manifest", "bidirectional_commands"],
        "required_capabilities": ["scene_manifest"],
        "coordinates": {
            "length_unit": "m",
            "handedness": "right",
            "position_frame": "L",
            "quaternion_order": "wxyz",
            "rotation_semantics": "active_parent_from_child",
        },
    }


def manifest_message(session_id: str) -> dict:
    def object_definition(object_id: str, color: list[float]) -> dict:
        return {
            "object_id": object_id,
            "display_name": object_id,
            "parent_id": "",
            "transform_space": "world",
            "asset_path": "",
            "semantic_label": object_id,
            "geometries": [
                {
                    "geometry_id": f"{object_id}/placeholder",
                    "shape": "box",
                    "dimensions_m": [1.5, 1.0, 0.8],
                    "position_body_m": [0.0, 0.0, 0.0],
                    "orientation_body_from_geometry_wxyz": [1.0, 0.0, 0.0, 0.0],
                    "color_rgba": color,
                    "scale": [1.0, 1.0, 1.0],
                    "render_role": "visual",
                }
            ],
        }

    return {
        "protocol": PROTOCOL_V2,
        "type": "scene_manifest",
        "session_id": session_id,
        "revision": "1",
        "objects": [
            object_definition("chaser", [0.15, 0.55, 1.0, 1.0]),
            object_definition("target", [1.0, 0.35, 0.1, 1.0]),
        ],
        "celestial_bodies": [],
        "visuals": [],
        "cameras": [
            {
                "camera_id": "chaser/opnav_camera",
                "display_name": "Chaser OpNav Camera",
                "parent_id": "chaser",
                "position_body_m": [0.0, -4.0, 2.0],
                "orientation_body_from_camera_wxyz": [0.7071067811865476, 0.0, 0.0, 0.7071067811865475],
                "field_of_view_rad": 1.0471975511965976,
                "resolution": [320, 180],
                "semantic_label": "opnav_camera",
                "picture_in_picture": False,
                "capture_rate_hz": 2.0,
                "capture_products": [],
            }
        ],
        "settings": {
            "origin_object_id": "chaser",
            "skybox": "black",
            "default_camera_target": "chaser",
            "default_camera_distance_m": 25.0,
            "interpolation_delay_ms": 100.0,
            "max_extrapolation_ms": 100.0,
            "ui": {
                "commands": [
                    {"command": "renderer.ping", "label": "Ping mock sender", "payload": {}, "requires_confirmation": False},
                    {"command": "demo.freeze", "label": "Freeze target motion", "payload": {}, "requires_confirmation": False},
                    {"command": "demo.resume", "label": "Resume target motion", "payload": {}, "requires_confirmation": False},
                ]
            },
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5558)
    parser.add_argument("--rate", type=float, default=30.0)
    parser.add_argument("--duration", type=float, default=60.0)
    args = parser.parse_args()
    if args.rate <= 0.0:
        parser.error("--rate must be positive")

    session_id = str(uuid.uuid4())
    publisher = RenderPublisher(args.host, args.port)
    publisher.retain_hello(hello_message(session_id))
    publisher.retain_manifest(manifest_message(session_id))
    publisher.start()
    started = time.monotonic()
    deadline = started
    frame_id = 0
    event_sequence = 0
    motion_frozen = False
    motion_time = 0.0
    previous_wall_time = started
    try:
        while time.monotonic() - started < args.duration:
            now = time.monotonic()
            sim_time = now - started
            if not motion_frozen:
                motion_time += now - previous_wall_time
            previous_wall_time = now
            while True:
                command = publisher.consume_command()
                if command is None:
                    break
                name = str(command.get("command", ""))
                status = "accepted"
                message = f"{name} completed"
                if command.get("session_id") != session_id:
                    status, message = "rejected", "command session mismatch"
                elif name == "demo.freeze":
                    motion_frozen = True
                elif name == "demo.resume":
                    motion_frozen = False
                elif name != "renderer.ping":
                    status, message = "rejected", f"unknown command: {name}"
                event_sequence += 1
                publisher.publish_event(
                    {
                        "protocol": PROTOCOL_V2,
                        "type": "event",
                        "session_id": session_id,
                        "sequence": str(event_sequence),
                        "event_kind": "command_result",
                        "payload": {
                            "command_id": str(command.get("command_id", "")),
                            "command": name,
                            "status": status,
                            "severity": "info" if status == "accepted" else "error",
                            "message": message,
                            "sim_time_ns": str(int(sim_time * 1e9)),
                            "result": {"motion_frozen": motion_frozen},
                        },
                    }
                )
            angle = 0.35 * motion_time
            range_m = max(2.5, 12.0 - 0.12 * motion_time)
            objects = [
                {
                    "object_id": "chaser",
                    "position_m": [0.0, 0.0, 0.0],
                    "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
                    "velocity_mps": [0.0, 0.0, 0.0],
                    "angular_velocity_B_radps": [0.0, 0.0, 0.0],
                },
                {
                    "object_id": "target",
                    "position_m": [0.8 * math.sin(0.2 * motion_time), range_m, 0.5 * math.cos(0.2 * motion_time)],
                    "orientation_wxyz": [math.cos(angle / 2.0), 0.0, 0.0, math.sin(angle / 2.0)],
                    "velocity_mps": [0.0, 0.0, 0.0] if motion_frozen else [0.16 * math.cos(0.2 * motion_time), -0.12, -0.1 * math.sin(0.2 * motion_time)],
                    "angular_velocity_B_radps": [0.0, 0.0, 0.0 if motion_frozen else 0.35],
                },
            ]
            publisher.publish_frame(
                {
                    "protocol": PROTOCOL_V2,
                    "type": "frame",
                    "session_id": session_id,
                    "manifest_revision": "1",
                    "frame_id": str(frame_id),
                    "sim_time_ns": str(int(sim_time * 1e9)),
                    "wall_time_ns": str(time.time_ns()),
                    "origin_N_m": [7_000_000.0, 0.0, 0.0],
                    "c_LN": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                    "objects": objects,
                    "celestial_bodies": [],
                    "visual_states": [],
                }
            )
            frame_id += 1
            deadline += 1.0 / args.rate
            time.sleep(max(0.0, deadline - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        publisher.close()
    print(f"queued={publisher.stats.queued} sent={publisher.stats.sent} dropped={publisher.stats.dropped}")


if __name__ == "__main__":
    main()
