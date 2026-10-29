"""Real light protocol and full-workspace coverage for fixed overview cameras."""
import itertools
import math

import numpy as np
import pytest

from bsk_render_adapter import VisualElement
from simulation.camera_lighting import overview_camera_lamp
from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix
from space_arm_platform.overview_camera import (
    CAMERA_ID, PRESETS, WORKSPACE_VIEW_BOUNDS_BODY_M, camera_pose,
)


def test_overview_lamp_uses_the_existing_camera_mounted_light_protocol():
    lamp = overview_camera_lamp()
    assert isinstance(lamp, VisualElement)
    payload = lamp.to_payload()
    assert payload["visual_id"] == "teleop/light/overview_lamp"
    assert payload["kind"] == "light" and payload["parent_id"] == CAMERA_ID
    assert payload["position_body_m"] == [0., 0., 0.]
    assert payload["normal_body"] == [1., 0., 0.]
    assert payload["range_m"] == 2.
    assert payload["properties"] == {
        "light_type": "spot", "diffuse_rgb": (1., 1., 1.),
        "specular_rgb": (.15, .15, .15), "intensity": 8000.,
        "cutoff_deg": 89., "cast_shadows": False,
    }


@pytest.mark.parametrize("key", PRESETS)
def test_all_workspace_corners_are_inside_the_overview_lamp_range_and_inner_cone(key):
    pose = camera_pose(key)
    lamp = overview_camera_lamp()
    rotation = quaternion_wxyz_to_matrix(pose["orientation_body_from_camera_wxyz"])
    low, high = WORKSPACE_VIEW_BOUNDS_BODY_M
    # Match the UE light's inner-cone factor; being inside the outer cone only
    # would permit the far edge of the workbench to fade almost to black.
    inner_half_angle = math.radians(lamp.properties["cutoff_deg"] * .65)
    for corner in itertools.product(*zip(low, high, strict=True)):
        ray = np.array(corner) - pose["position_body_m"]
        distance = np.linalg.norm(ray)
        assert distance < lamp.range_m
        assert np.dot(rotation[:, 0], ray) / distance > math.cos(inner_half_angle)
