"""Validate structural and numeric contracts of a Demo 8 recording."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from bsk_render_adapter import BskRecordingReader


def validate(path: Path) -> None:
    messages = list(BskRecordingReader(path).messages())
    manifests = [message for message in messages if message.get("type") == "scene_manifest"]
    frames = [message for message in messages if message.get("type") == "frame"]
    assert len(manifests) == 1, f"expected one manifest, got {len(manifests)}"
    assert len(frames) >= 1000, f"expected a full 0.75-orbit recording, got {len(frames)} frames"
    manifest = manifests[0]
    assert len(manifest["objects"]) == 12
    assert sum(len(item["geometries"]) for item in manifest["objects"]) == 11
    assert len(manifest["visuals"]) == 6
    assert len(manifest["celestial_bodies"]) == 3
    assert len(manifest["cameras"]) == 1
    css_visuals = [item for item in manifest["visuals"] if item["kind"] == "css"]
    assert len(css_visuals) == 4
    for visual in css_visuals:
        assert visual["channel_schema"]["signal"]["type"] == "number"
        assert visual["channel_schema"]["valid"]["type"] == "boolean"

    first = {item["object_id"]: item for item in frames[0]["objects"]}
    last = {item["object_id"]: item for item in frames[-1]["objects"]}
    assert math.isclose(
        math.dist(first["primary/hub"]["position_m"], first["companion/companion_hub"]["position_m"]),
        15.0,
        abs_tol=1.0e-6,
    )
    assert math.isclose(
        math.dist(first["primary/hub"]["position_m"], first["trailer"]["position_m"]),
        15.0,
        abs_tol=1.0e-6,
    )
    panel_ids = [object_id for object_id in first if "panel" in object_id]
    assert len(panel_ids) == 8
    for object_id in panel_ids:
        dot = abs(sum(a * b for a, b in zip(first[object_id]["orientation_wxyz"], last[object_id]["orientation_wxyz"])))
        assert dot < 0.999, f"panel did not deploy: {object_id}"
    assert int(frames[1]["sim_time_ns"]) - int(frames[0]["sim_time_ns"]) == 4_000_000_000
    first_visuals = {item["visual_id"]: item for item in frames[0]["visual_states"]}
    last_visuals = {item["visual_id"]: item for item in frames[-1]["visual_states"]}
    assert len(first_visuals) == 6
    assert any(
        last_visuals[item["visual_id"]]["channels"]["signal"]
        != first_visuals[item["visual_id"]]["channels"]["signal"]
        for item in css_visuals
    ), "CSS dynamic channels did not change"
    print(
        f"Demo 8 validation passed: {len(frames)} frames, 12 bodies, "
        "11 geoms, 8 deploying panels, 6 typed device visuals, 3 celestial bodies"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    args = parser.parse_args()
    validate(args.recording)


if __name__ == "__main__":
    main()
