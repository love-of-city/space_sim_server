"""Validate and restore saved overview-camera records without resampling scenes."""
from __future__ import annotations

import math
import random
from copy import deepcopy

from .overview_camera_calibration import (
    CAMERA_ID, TEMPLATES, SCHEMA, LEGACY_SCHEMA, V2_SCHEMA, V3_SCHEMA, V4_SCHEMA, V6_SCHEMA,
    DECK_CENTER_BODY_M, CONTINUOUS_CENTER_BOUNDS_BODY_XY_M, MIN_ARM_BASE_MOUNT_AXIS_DISTANCE_M,
    MOUNTING_PATCH_BODY_XY_M, MOUNTING_FOOTPRINT_RADIUS_M, WORKSPACE_VIEW_BOUNDS_BODY_M,
    WORKSPACE_FRAME_MARGIN_FRACTION, _V5_SCHEMA, _V5_STREAM, _version,
    calibrated_camera_pose as record_camera_pose,
)


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


def restore_overview_camera(seed, *, schema=SCHEMA):
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError("overview camera seed must be a nonnegative integer")
    if schema == _V5_SCHEMA:
        return _sample_continuous_camera(seed)
    presets, stream = _version(schema)
    preset = random.Random(f"{stream}:{seed}").choice(tuple(presets))
    return dict(schema=schema, enabled=True, stream=stream, selection_seed=seed,
                distribution="uniform_discrete", camera_id=CAMERA_ID,
                **record_camera_pose(preset, schema=schema))


def _sampling_mode(scene):
    mode = scene.get("overview_camera_sampling", "presets")
    if mode not in ("presets", "continuous"):
        raise ValueError("overview_camera_sampling must be presets or continuous")
    return mode


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
                    else restore_overview_camera(seed, schema=record.get("schema")))
    if not isinstance(record, dict) or not _same_saved_calibration(record, expected):
        raise ValueError("overview_camera does not match the saved seed and versioned sampling")
    return deepcopy(record)


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
