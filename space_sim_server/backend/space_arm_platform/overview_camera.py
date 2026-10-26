"""Versioned, body-fixed overview camera presets for the 20261012 workbench.

Wire camera axes are +X forward, +Y left, +Z up, unlike MJCF camera axes.
Selection happens at scene/attempt creation; loading and resetting only replay.
"""
from __future__ import annotations

import math
import random
from copy import deepcopy

SCHEMA = "space-arm-overview-camera/4"
STREAM = "space-arm-overview-camera-presets-v4"
TEMPLATES = ("sarm-task-box-20261012", "sarm-task-box-20261012-sdf")
CAMERA_ID = "teleop/camera/spacecraft_overview"
DECK_CENTER_BODY_M = (.35, .34, 0.)
PRESETS = {
    "P0": ("原安装点低机位 · 30 cm", (-.53, -.07, .30)),
    "P1": ("−Y 外侧安装点 · 30 cm", (-.53, -.28, .30)),
    "P2": ("+Y 外侧安装点 · 30 cm", (-.53, .22, .30)),
    "P3": ("侧边中段安装点 · 30 cm", (-.53, .08, .30)),
    "P4": ("+Y 内侧安装点 · 30 cm", (-.45, .24, .30)),
}
# Calibrated flat z=0 patch on base_link_mesh, inset from its edges.
# Every preset's vertical projection and a 15 mm mounting footprint fit here.
# The actual STL surface is independently checked in the geometry regression.
MOUNTING_PATCH_BODY_XY_M = ((-.20, -.08), (-.08, .60))
MOUNTING_FOOTPRINT_RADIUS_M = .015
# A static base exclusion, not a claim about the arm's full swept volume.
MIN_ARM_BASE_MOUNT_AXIS_DISTANCE_M = .18
# Saved scenes and recordings retain their original calibration and selection.
LEGACY_SCHEMA = "space-arm-overview-camera/1"
LEGACY_STREAM = "space-arm-overview-camera-presets-v1"
LEGACY_PRESETS = {
    "P0": ("当前机位", (-.53, -.07, .50)),
    "P1": ("−Y 侧斜视", (-.50, -.30, .55)),
    "P2": ("+Y 侧斜视", (-.50, .30, .55)),
    "P3": ("−Y 侧高俯视", (-.38, -.20, .72)),
    "P4": ("+Y 侧高俯视", (-.38, .20, .72)),
}
V2_SCHEMA = "space-arm-overview-camera/2"
V2_PRESETS = {
    "P0": ("正面斜视 · 30 cm", (-.62, .00, .30)),
    "P1": ("−Y 侧斜视 · 30 cm", (-.52, -.30, .30)),
    "P2": ("+Y 侧斜视 · 30 cm", (-.52, .30, .30)),
    "P3": ("−Y 侧向斜视 · 30 cm", (-.30, -.52, .30)),
    "P4": ("+Y 侧向斜视 · 30 cm", (-.30, .52, .30)),
}
V3_SCHEMA = "space-arm-overview-camera/3"
V3_PRESETS = {
    "P0": ("原安装点低机位 · 30 cm", (-.53, -.07, .30)),
    "P1": ("−Y 外侧安装点 · 30 cm", (-.53, -.28, .30)),
    "P2": ("+Y 外侧安装点 · 30 cm", (-.53, .22, .30)),
    "P3": ("−Y 内侧安装点 · 30 cm", (-.45, -.39, .30)),
    "P4": ("+Y 内侧安装点 · 30 cm", (-.45, .24, .30)),
}


def _version(schema):
    if schema == SCHEMA:
        return PRESETS, STREAM
    if schema == V2_SCHEMA:
        return V2_PRESETS, "space-arm-overview-camera-presets-v2"
    if schema == V3_SCHEMA:
        return V3_PRESETS, "space-arm-overview-camera-presets-v3"
    if schema == LEGACY_SCHEMA:
        return LEGACY_PRESETS, LEGACY_STREAM
    raise ValueError("unsupported overview camera preset version")


def _multiply(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return [w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v]


def camera_pose(preset_id, *, schema=SCHEMA):
    presets, _ = _version(schema)
    label, offset = presets[preset_id]
    position = [a+b for a, b in zip(DECK_CENTER_BODY_M, offset, strict=True)]
    target = [.35, .34, .06]
    if schema == LEGACY_SCHEMA and preset_id == "P0":
        # Preserve the delivered MJCF camera exactly, including its slight
        # off-centre aim. Convert once to the existing adapter's wire basis.
        position = [-.18, .27, .5]
        orientation = _multiply(
            [.6851394915897261, .30992428380402737,
             -.27168234994635526, -.600599297452435],
            [.5, -.5, .5, .5],
        )
        target = None
    else:
        dx, dy, dz = [a-b for a, b in zip(target, position, strict=True)]
        yaw = math.atan2(dy, dx)
        pitch = math.atan2(-dz, math.hypot(dx, dy))
        cy, sy = math.cos(yaw/2), math.sin(yaw/2)
        cp, sp = math.cos(pitch/2), math.sin(pitch/2)
        orientation = [cy*cp, -sy*sp, cy*sp, sy*cp]
    mounting = {}
    if schema in (V3_SCHEMA, SCHEMA):
        (xmin, ymin), (xmax, ymax) = MOUNTING_PATCH_BODY_XY_M
        x, y, z = position
        r = MOUNTING_FOOTPRINT_RADIUS_M
        if not (xmin+r <= x <= xmax-r and ymin+r <= y <= ymax-r and z == .30):
            raise ValueError("overview camera must project onto the calibrated satellite mounting surface")
        if schema == SCHEMA and math.hypot(x, y) < MIN_ARM_BASE_MOUNT_AXIS_DISTANCE_M:
            raise ValueError("overview camera mounting point is too close to the arm base")
        mounting = dict(mount_position_body_m=[x, y, 0.],
                        height_above_mount_m=z, mounting_footprint_radius_m=r)
    return dict(preset_id=preset_id, label=label, parent_id="teleop/cubesat_bus",
                position_body_m=position,
                orientation_body_from_camera_wxyz=orientation,
                camera_axes="+X forward,+Y left,+Z up",
                deck_center_body_m=list(DECK_CENTER_BODY_M),
                position_relative_to_deck_center_m=list(offset),
                look_at_body_m=target,
                field_of_view_rad=math.radians(90.0 if schema in (V3_SCHEMA, SCHEMA) else 80.5929896518),
                **mounting)


def sample_overview_camera(seed, *, schema=SCHEMA):
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError("overview camera seed must be a nonnegative integer")
    presets, stream = _version(schema)
    preset = random.Random(f"{stream}:{seed}").choice(tuple(presets))
    return dict(schema=schema, enabled=True, stream=stream, selection_seed=seed,
                distribution="uniform_discrete", camera_id=CAMERA_ID,
                **camera_pose(preset, schema=schema))


def _same_saved_calibration(actual, expected):
    # Saved trigonometric results can differ by a few ULPs across platforms.
    # Keep the saved values; allow only numerical roundoff when validating them.
    if isinstance(expected, float):
        return (isinstance(actual, (int, float)) and not isinstance(actual, bool)
                and math.isclose(actual, expected, rel_tol=0., abs_tol=1e-14))
    if isinstance(expected, dict):
        return (isinstance(actual, dict) and actual.keys() == expected.keys()
                and all(_same_saved_calibration(actual[key], value) for key, value in expected.items()))
    if isinstance(expected, list):
        return (isinstance(actual, list) and len(actual) == len(expected)
                and all(_same_saved_calibration(a, b) for a, b in zip(actual, expected, strict=True)))
    return actual == expected


def validate_overview_camera(scene):
    enabled = scene.get("randomize_overview_camera", False)
    if not isinstance(enabled, bool):
        raise ValueError("randomize_overview_camera must be a boolean")
    record = scene.get("overview_camera")
    if not enabled:
        if record is not None:
            raise ValueError("overview_camera requires randomize_overview_camera=true")
        return None
    if scene.get("template_id") not in TEMPLATES:
        raise ValueError("overview camera presets require the full 20261012 SARM workbench template")
    if not isinstance(record, dict) or not _same_saved_calibration(record, sample_overview_camera(
            scene.get("seed"), schema=record.get("schema"))):
        raise ValueError("overview_camera does not match the saved seed and versioned presets")
    return deepcopy(record)


def initialize_overview_camera(scene):
    """Sample explicitly at a new episode boundary, never during playback."""
    enabled = scene.get("randomize_overview_camera", False)
    if not isinstance(enabled, bool):
        raise ValueError("randomize_overview_camera must be a boolean")
    if enabled:
        scene["overview_camera"] = sample_overview_camera(scene["seed"])
    else:
        scene.pop("overview_camera", None)
    return validate_overview_camera(scene)


def overview_camera_catalog():
    return dict(schema=SCHEMA, templates=list(TEMPLATES), presets=[camera_pose(key) for key in PRESETS],
                description="每条任务从 5 个卫星实体台面上方 30 cm 的安装机位中等概率选择，视野 90°，整段保持固定；相同种子可复现。")


def validate_episode_overview_camera(episode, expected):
    import json
    scene = json.loads((episode / "metadata.json").read_text(encoding="utf-8"))["scene_instance"]
    if validate_overview_camera(scene) != expected:
        raise ValueError("Recorded overview camera differs from the selected attempt")
    return dict(passed=True, preset_id=expected["preset_id"])


def validate_overview_capture(metadata, observation, expected):
    """Check UE's actual view and extrinsics against the same-frame body pose."""
    if metadata["camera_id"] != CAMERA_ID:
        return
    import numpy as np
    from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix

    body_to_inertial = quaternion_wxyz_to_matrix(
        observation["attitude_control"]["orientation_inertial_wxyz"])
    camera_to_body = quaternion_wxyz_to_matrix(expected["orientation_body_from_camera_wxyz"])
    actual = metadata["extrinsics"]
    np.testing.assert_allclose(
        [metadata["intrinsics"][key] for key in ("fov_x_rad", "fov_y_rad")],
        expected["field_of_view_rad"], atol=2e-6, rtol=0,
        err_msg="UE overview field of view differs from selected preset")
    inertial_to_local = np.asarray(metadata["floating_origin"]["c_LN"]).reshape(3, 3)
    np.testing.assert_allclose(
        np.asarray(actual["c_NC"]).reshape(3, 3), body_to_inertial @ camera_to_body,
        atol=2e-5, rtol=0, err_msg="UE overview orientation differs from selected preset")
    np.testing.assert_allclose(
        actual["position_L_m"], inertial_to_local @ body_to_inertial @ expected["position_body_m"],
        atol=2e-4, rtol=0, err_msg="UE overview position differs from selected preset")
