"""Versioned, body-fixed overview camera sampling for the 20261012 workbench.

Wire camera axes are +X forward, +Y left, +Z up, unlike MJCF camera axes.
Selection happens at scene/attempt creation; loading and resetting only replay.
"""
from __future__ import annotations

import math
import random
from copy import deepcopy

SCHEMA = "space-arm-overview-camera/7"
STREAM = "space-arm-overview-camera-presets-v7"
V6_SCHEMA = "space-arm-overview-camera/6"
V6_STREAM = "space-arm-overview-camera-presets-v6"
# Frozen v5 sampling is retained only to validate and replay saved recordings.
# New scenes and attempts always use the five discrete presets.
_V5_SCHEMA = "space-arm-overview-camera/5"
_V5_STREAM = "space-arm-overview-camera-continuous-v5"
TEMPLATES = ("sarm-task-box-20261012", "sarm-task-box-20261012-sdf")
CAMERA_ID = "teleop/camera/spacecraft_overview"
DECK_CENTER_BODY_M = (.35, .34, 0.)
V4_SCHEMA = "space-arm-overview-camera/4"
V4_STREAM = "space-arm-overview-camera-presets-v4"
V4_PRESETS = {
    "P0": ("原安装点低机位 · 30 cm", (-.53, -.07, .30)),
    "P1": ("−Y 外侧安装点 · 30 cm", (-.53, -.28, .30)),
    "P2": ("+Y 外侧安装点 · 30 cm", (-.53, .22, .30)),
    "P3": ("侧边中段安装点 · 30 cm", (-.53, .08, .30)),
    "P4": ("+Y 内侧安装点 · 30 cm", (-.45, .24, .30)),
}
# Historical v3/v4 and continuous v5 share this frozen conservative patch.
MOUNTING_PATCH_BODY_XY_M = ((-.20, -.08), (-.08, .60))
MOUNTING_FOOTPRINT_RADIUS_M = .015
# A static base exclusion, not a claim about the arm's full swept volume.
MIN_ARM_BASE_MOUNT_AXIS_DISTANCE_M = .18
# Uniform proposals within the footprint-inset mounting surface. Rejection
# excludes the base and positions that cannot see the full working envelope.
CONTINUOUS_CENTER_BOUNDS_BODY_XY_M = tuple(
    tuple(value + sign*MOUNTING_FOOTPRINT_RADIUS_M for value in bound)
    for bound, sign in zip(MOUNTING_PATCH_BODY_XY_M, (1, -1), strict=True)
)
WORKSPACE_VIEW_BOUNDS_BODY_M = ((.125, .065, 0.), (.575, .615, .18))
WORKSPACE_FRAME_MARGIN_FRACTION = .15
# The same real z=0 satellite surface also supports a left strip and a front
# strip beside the task deck. The 15 mm disk must fit one complete rectangle.
# Independent regression intersects these mounts with the delivered STL.
PRESET_MOUNTING_REGIONS_BODY_XY_M = (
    ((-.20, -.11), (.125, .64)),
    ((.125, -.11), (.60, .065)),
)
PRESET_MIN_ARM_BASE_MOUNT_AXIS_DISTANCE_M = .30
PRESET_HEIGHT_RANGE_M = (.30, .45)
V6_PRESETS = {
    "P0": ("左侧中部近机位 · 38 cm", (-.41, -.04, .38)),
    "P1": ("前沿中央近机位 · 42 cm", (0., -.30, .42)),
    "P2": ("前沿右侧近机位 · 42 cm", (.15, -.30, .42)),
    "P3": ("左侧后段近机位 · 42 cm", (-.28, .20, .42)),
    "P4": ("左后角近机位 · 36 cm", (-.29, .24, .36)),
}
# Aim height and rectilinear FOV are calibrated together for the full 18 cm
# working volume. A wider view alone would reduce useful workbench pixels.
V6_PRESET_VIEWS = {
    "P0": (.06, 101.),
    "P1": (.02, 105.),
    "P2": (.06, 104.),
    "P3": (.04, 95.),
    "P4": (0., 95.),
}
# One regular mounting rail on the real left satellite surface. Front-edge
# mounts put the arm between the camera and the workbench during manipulation.
# Five positions are 7 cm apart; a shared height/aim/lens keeps images comparable.
PRESETS = {
    "P0": ("左侧规则机位 1 · 45 cm", (-.47, -.02, .45)),
    "P1": ("左侧规则机位 2 · 45 cm", (-.47, .05, .45)),
    "P2": ("左侧规则机位 3 · 45 cm", (-.47, .12, .45)),
    "P3": ("左侧规则机位 4 · 45 cm", (-.47, .19, .45)),
    "P4": ("左侧规则机位 5 · 45 cm", (-.47, .26, .45)),
}
PRESET_VIEWS = {key: (.06, 85.) for key in PRESETS}
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
    if schema == V6_SCHEMA:
        return V6_PRESETS, V6_STREAM
    if schema == V4_SCHEMA:
        return V4_PRESETS, V4_STREAM
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
    calibrated = schema in (SCHEMA, V6_SCHEMA)
    views = PRESET_VIEWS if schema == SCHEMA else V6_PRESET_VIEWS
    target = [.35, .34, views[preset_id][0] if calibrated else .06]
    field_of_view_deg = (views[preset_id][1] if calibrated else
                         90.0 if schema in (V3_SCHEMA, V4_SCHEMA) else 80.5929896518)
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
    if calibrated:
        x, y, z = position
        r = MOUNTING_FOOTPRINT_RADIUS_M
        supported = any(
            low[0]+r <= x <= high[0]-r and low[1]+r <= y <= high[1]-r
            for low, high in PRESET_MOUNTING_REGIONS_BODY_XY_M)
        if not supported or not PRESET_HEIGHT_RANGE_M[0] <= z <= PRESET_HEIGHT_RANGE_M[1]:
            raise ValueError("overview camera must project onto the calibrated satellite mounting surface")
        if math.hypot(x, y) < PRESET_MIN_ARM_BASE_MOUNT_AXIS_DISTANCE_M:
            raise ValueError("overview camera mounting point is too close to the arm base")
        if not _preset_workspace_visible(position, target, math.radians(field_of_view_deg)):
            raise ValueError("overview camera must contain the full workspace with its frame margin")
        mounting = dict(
            mount_position_body_m=[x, y, 0.], height_above_mount_m=z,
            mounting_footprint_radius_m=r,
            mounting_regions_body_xy_m=[[list(bound) for bound in region]
                                        for region in PRESET_MOUNTING_REGIONS_BODY_XY_M],
            minimum_arm_base_mount_axis_distance_m=PRESET_MIN_ARM_BASE_MOUNT_AXIS_DISTANCE_M,
            workspace_view_bounds_body_m=[list(bound) for bound in WORKSPACE_VIEW_BOUNDS_BODY_M],
            workspace_frame_margin_fraction=WORKSPACE_FRAME_MARGIN_FRACTION)
    elif schema in (V3_SCHEMA, V4_SCHEMA):
        (xmin, ymin), (xmax, ymax) = MOUNTING_PATCH_BODY_XY_M
        x, y, z = position
        r = MOUNTING_FOOTPRINT_RADIUS_M
        if not (xmin+r <= x <= xmax-r and ymin+r <= y <= ymax-r and z == .30):
            raise ValueError("overview camera must project onto the calibrated satellite mounting surface")
        if schema == V4_SCHEMA and math.hypot(x, y) < MIN_ARM_BASE_MOUNT_AXIS_DISTANCE_M:
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
                field_of_view_rad=math.radians(field_of_view_deg),
                **mounting)


def _preset_workspace_visible(position, target, field_of_view_rad):
    dx, dy, dz = [a-b for a, b in zip(target, position, strict=True)]
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    axes = ((cy*cp, sy*cp, -sp), (-sy, cy, 0.), (cy*sp, sy*sp, cp))
    limit = (1-WORKSPACE_FRAME_MARGIN_FRACTION)*math.tan(field_of_view_rad/2)
    low, high = WORKSPACE_VIEW_BOUNDS_BODY_M
    for x in (low[0], high[0]):
        for y in (low[1], high[1]):
            for z in (low[2], high[2]):
                delta = (x-position[0], y-position[1], z-position[2])
                forward, left, up = [sum(a*b for a, b in zip(axis, delta, strict=True))
                                     for axis in axes]
                if forward <= 0 or max(abs(left), abs(up)) >= limit*forward:
                    return False
    return True


def _continuous_pose(position):
    target = [.35, .34, .06]
    dx, dy, dz = [a-b for a, b in zip(target, position, strict=True)]
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    return dict(label="连续随机安装点 · 30 cm", parent_id="teleop/cubesat_bus",
                position_body_m=position,
                orientation_body_from_camera_wxyz=[cy*cp, -sy*sp, cy*sp, sy*cp],
                camera_axes="+X forward,+Y left,+Z up",
                deck_center_body_m=list(DECK_CENTER_BODY_M),
                position_relative_to_deck_center_m=[
                    a-b for a, b in zip(position, DECK_CENTER_BODY_M, strict=True)],
                look_at_body_m=target, field_of_view_rad=math.pi/2,
                mount_position_body_m=[*position[:2], 0.],
                height_above_mount_m=.30,
                mounting_footprint_radius_m=MOUNTING_FOOTPRINT_RADIUS_M)


def _workspace_visible(position):
    dx, dy, dz = [.35-position[0], .34-position[1], .06-position[2]]
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    forward, left, up = (cy*cp, sy*cp, -sp), (-sy, cy, 0.), (cy*sp, sy*sp, cp)
    low, high = WORKSPACE_VIEW_BOUNDS_BODY_M
    for x in (low[0], high[0]):
        for y in (low[1], high[1]):
            for z in (low[2], high[2]):
                delta = (x-position[0], y-position[1], z-position[2])
                f, l, u = [sum(a*b for a, b in zip(axis, delta, strict=True))
                           for axis in (forward, left, up)]
                if f <= 0 or max(abs(l), abs(u)) >= (1-WORKSPACE_FRAME_MARGIN_FRACTION)*f:
                    return False
    return True


def _sample_continuous_camera(seed):
    rng = random.Random(f"{_V5_STREAM}:{seed}")
    low, high = CONTINUOUS_CENTER_BOUNDS_BODY_XY_M
    for attempt in range(1, 10001):
        position = [rng.uniform(low[0], high[0]), rng.uniform(low[1], high[1]), .30]
        if (math.hypot(*position[:2]) >= MIN_ARM_BASE_MOUNT_AXIS_DISTANCE_M
                and _workspace_visible(position)):
            return dict(
                schema=_V5_SCHEMA, enabled=True, stream=_V5_STREAM,
                selection_seed=seed, distribution="uniform_continuous_rejection",
                camera_id=CAMERA_ID, sampling_attempts=attempt,
                sampling_bounds_body_xy_m=[list(bound) for bound in CONTINUOUS_CENTER_BOUNDS_BODY_XY_M],
                mounting_surface_bounds_body_xy_m=[list(bound) for bound in MOUNTING_PATCH_BODY_XY_M],
                minimum_arm_base_mount_axis_distance_m=MIN_ARM_BASE_MOUNT_AXIS_DISTANCE_M,
                workspace_view_bounds_body_m=[list(bound) for bound in WORKSPACE_VIEW_BOUNDS_BODY_M],
                workspace_frame_margin_fraction=WORKSPACE_FRAME_MARGIN_FRACTION,
                **_continuous_pose(position))
    raise ValueError("no overview camera position satisfies the calibrated mounting and view constraints")


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


def _sampling_mode(scene):
    mode = scene.get("overview_camera_sampling", "presets")
    if mode not in ("presets", "continuous"):
        raise ValueError("overview_camera_sampling must be presets or continuous")
    return mode


def validate_overview_camera(scene):
    _sampling_mode(scene)
    enabled = scene.get("randomize_overview_camera", False)
    if not isinstance(enabled, bool):
        raise ValueError("randomize_overview_camera must be a boolean")
    record = scene.get("overview_camera")
    if not enabled:
        if record is not None:
            raise ValueError("overview_camera requires randomize_overview_camera=true")
        return None
    if scene.get("template_id") not in TEMPLATES:
        raise ValueError("overview camera randomization requires the full 20261012 SARM workbench template")
    seed = scene.get("overview_camera_seed", scene.get("seed"))
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError("overview camera seed must be a nonnegative integer")
    expected = None
    if isinstance(record, dict):
        expected = (_sample_continuous_camera(seed) if record.get("schema") == _V5_SCHEMA
                    else sample_overview_camera(seed, schema=record.get("schema")))
    if not isinstance(record, dict) or not _same_saved_calibration(record, expected):
        raise ValueError("overview_camera does not match the saved seed and versioned sampling")
    return deepcopy(record)


def initialize_overview_camera(scene):
    """Sample explicitly at a new episode boundary, never during playback."""
    _sampling_mode(scene)
    enabled = scene.get("randomize_overview_camera", False)
    if not isinstance(enabled, bool):
        raise ValueError("randomize_overview_camera must be a boolean")
    # A new episode supersedes a saved source scene's old sampling preference.
    # Historical records are still replayed unchanged by validate_overview_camera.
    scene["overview_camera_sampling"] = "presets"
    if enabled:
        seed = scene.get("overview_camera_seed", scene["seed"])
        scene["overview_camera"] = sample_overview_camera(seed)
    else:
        scene.pop("overview_camera", None)
    return validate_overview_camera(scene)


def overview_camera_catalog():
    return dict(schema=SCHEMA, templates=list(TEMPLATES), presets=[camera_pose(key) for key in PRESETS],
                description="每条任务从卫星实体底板左侧等间距排列的 5 个固定机位中等概率选择，高度 45 cm、视野 85°，覆盖任务台及上方 18 cm 操作空间；避开原前沿遮挡机位，整段保持固定，相同版本及种子可复现。")


def validate_episode_overview_camera(episode, expected):
    import json
    scene = json.loads((episode / "metadata.json").read_text(encoding="utf-8"))["scene_instance"]
    if validate_overview_camera(scene) != expected:
        raise ValueError("Recorded overview camera differs from the selected attempt")
    if "preset_id" in expected:
        return dict(passed=True, preset_id=expected["preset_id"])
    return dict(passed=True, schema=expected["schema"],
                position_body_m=deepcopy(expected["position_body_m"]))


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
        err_msg="UE overview field of view differs from selected camera")
    inertial_to_local = np.asarray(metadata["floating_origin"]["c_LN"]).reshape(3, 3)
    np.testing.assert_allclose(
        np.asarray(actual["c_NC"]).reshape(3, 3), body_to_inertial @ camera_to_body,
        atol=2e-5, rtol=0, err_msg="UE overview orientation differs from selected camera")
    np.testing.assert_allclose(
        actual["position_L_m"], inertial_to_local @ body_to_inertial @ expected["position_body_m"],
        atol=2e-4, rtol=0, err_msg="UE overview position differs from selected camera")
