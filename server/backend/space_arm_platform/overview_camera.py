"""Create current overview-camera presets at new scene/attempt boundaries only."""
from __future__ import annotations

import random
from copy import deepcopy

from .overview_camera_calibration import (
    CAMERA_ID, TEMPLATES, SCHEMA, STREAM, PRESETS, WORKSPACE_VIEW_BOUNDS_BODY_M,
    calibrated_camera_pose,
)


def camera_pose(preset_id):
    return calibrated_camera_pose(preset_id, schema=SCHEMA)


def sample_overview_camera(seed):
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError("overview camera seed must be a nonnegative integer")
    preset = random.Random(f"{STREAM}:{seed}").choice(tuple(PRESETS))
    return dict(schema=SCHEMA, enabled=True, stream=STREAM, selection_seed=seed,
                distribution="uniform_discrete", camera_id=CAMERA_ID, **camera_pose(preset))


def initialize_overview_camera(scene):
    """Replace saved sampling preferences only when explicitly creating an episode."""
    mode = scene.get("overview_camera_sampling", "presets")
    if mode not in ("presets", "continuous"):
        raise ValueError("overview_camera_sampling must be presets or continuous")
    enabled = scene.get("randomize_overview_camera", False)
    if not isinstance(enabled, bool):
        raise ValueError("randomize_overview_camera must be a boolean")
    if enabled and scene.get("template_id") not in TEMPLATES:
        raise ValueError("overview camera randomization requires the full 20261012 SARM workbench template")
    scene["overview_camera_sampling"] = "presets"
    if enabled:
        seed = scene.get("overview_camera_seed", scene["seed"])
        scene["overview_camera"] = sample_overview_camera(seed)
    else:
        scene.pop("overview_camera", None)
    return deepcopy(scene.get("overview_camera"))


def overview_camera_catalog():
    return dict(schema=SCHEMA, templates=list(TEMPLATES), presets=[camera_pose(key) for key in PRESETS],
                description="每条任务从卫星实体底板左侧等间距排列的 5 个固定机位中等概率选择，高度 45 cm、视野 85°，覆盖任务台及上方 18 cm 操作空间；避开原前沿遮挡机位，整段保持固定，相同版本及种子可复现。")
