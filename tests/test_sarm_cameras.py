from __future__ import annotations

from pathlib import Path
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from simulation.serial_chain_kinematics import SerialChainKinematics, quaternion_wxyz_to_matrix
from space_arm_platform.scene_runtime import SCENE_TEMPLATES

MODEL_ROOT = Path(__file__).resolve().parents[1] / "model" / "SARM"
CAMERAS = {"spacecraft_overview", "sarm_wrist_cam"}


@pytest.mark.parametrize("relative,base", [("mjcf/SARM.xml", "sarm_base"), ("platform/sarm_platform.xml", "cubesat_bus")])
def test_sarm_camera_mounts_and_framing(relative, base):
    root = ET.parse(MODEL_ROOT / relative).getroot()
    cameras = {camera.get("name"): (body.get("name"), camera) for body in root.iter("body") for camera in body.findall("camera")}
    assert set(cameras) == CAMERAS
    assert cameras["spacecraft_overview"][0] == base
    assert cameras["sarm_wrist_cam"][0] == "link6"
    targets = {"spacecraft_overview": (0.20, 0.10, 0.10), "sarm_wrist_cam": (0.08, 0, 0.29)}
    for name, (_, camera) in cameras.items():
        assert camera.get("mode") == "fixed"
        assert camera.get("resolution") == "640 360"
        quat = np.fromstring(camera.get("quat"), sep=" ")
        assert np.linalg.norm(quat) == pytest.approx(1.0, abs=1e-9)
        rotation = quaternion_wxyz_to_matrix(quat)
        position = np.fromstring(camera.get("pos"), sep=" ")
        forward = -rotation[:, 2]  # MJCF camera looks along its local -Z.
        sight = np.array(targets[name]) - position
        assert np.dot(forward, sight / np.linalg.norm(sight)) > 0.999999
        assert 0 < float(camera.get("fovy")) < 180


def test_source_and_runtime_xml_use_identical_camera_calibration():
    def cameras(relative):
        return {c.get("name"): c.attrib for c in ET.parse(MODEL_ROOT / relative).iter("camera")}
    assert cameras("mjcf/SARM.xml") == cameras("platform/sarm_platform.xml")


def test_stream_ids_match_model_and_no_python_camera_override():
    project = Path(__file__).resolve().parents[1]
    script = (project / "scripts/run_platform.ps1").read_text(encoding="utf-8-sig")
    runtime = (project / "simulation/teleop_grasp_unreal.py").read_text(encoding="utf-8")
    expected = {f"teleop/camera/{name}" for name in CAMERAS}
    assert all(f"'{name}'" in script for name in expected)
    assert "teleop/camera/so101_wrist_cam" not in script
    assert 'default_camera_target="teleop/cubesat_bus"' in runtime
    assert "bridge.add_camera(" not in runtime
    assert "camera_picture_in_picture_start_slot=1" in runtime
    assert any(set(template["camera_ids"]) == expected for template in SCENE_TEMPLATES)


def test_wrist_view_contains_gripper_and_nominal_capture_target():
    path = MODEL_ROOT / "platform/sarm_platform.xml"
    root = ET.parse(path).getroot()
    camera = root.find('.//camera[@name="sarm_wrist_cam"]')
    tool = root.find('.//site[@name="sarm_ee"]')
    kinematics = SerialChainKinematics.from_mjcf(
        path, base_body="cubesat_bus", joint_names=tuple(f"joint{i}" for i in range(1, 7)), tool_site="sarm_ee"
    )
    # Standard PREGRASP, before any live dynamics or teleoperation is applied.
    tool_position, body_rotation = kinematics.forward(np.array([0, -0.1790243, 0.2159404, -0.0368382, 0, 0]))
    mount = np.fromstring(camera.get("pos"), sep=" ")
    tool_local = np.fromstring(tool.get("pos"), sep=" ")
    camera_position = tool_position + body_rotation @ (mount - tool_local)
    camera_rotation = body_rotation @ quaternion_wxyz_to_matrix(np.fromstring(camera.get("quat"), sep=" "))
    half_height = math.tan(math.radians(float(camera.get("fovy"))) / 2)
    half_width = half_height * 640 / 360
    points = [
        tool_position,
        tool_position + body_rotation @ np.array([0, -0.0375, -0.02]),
        tool_position + body_rotation @ np.array([0, 0.0375, -0.02]),
        np.array([0.38754456, -0.00109359, 0.42397138]),
    ]
    for point in points:
        right, up, back = camera_rotation.T @ (point - camera_position)
        depth = -back
        assert depth > 0.01
        assert abs(right / depth) < half_width
        assert abs(up / depth) < half_height
