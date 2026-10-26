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
    expected = cameras("mjcf/SARM.xml")
    for relative in (
        "platform/sarm_platform.xml",
        "platform/sarm_ground_target.xml",
        "platform/sarm_ground_target_all1_trial.xml",
        "platform/sarm_ground_target_self_collision.xml",
        "platform/sarm_task_box_module.xml",
        "platform/sarm_task_box_plugs.xml",
        "../ground_validation_satellite/mesh_collision_trial/sarm_mesh_collision.xml",
    ):
        assert cameras(relative) == expected, relative


def test_stream_ids_match_model_and_preserve_stream_slots():
    project = Path(__file__).resolve().parents[1]
    script = (project / "scripts/run_platform.ps1").read_text(encoding="utf-8-sig")
    # The camera wiring and scene settings moved into the render component during
    # the component refactor; assert the same facts at their new home.
    render = (project / "simulation/components/render.py").read_text(encoding="utf-8")
    expected = {f"teleop/camera/{name}" for name in CAMERAS}
    assert all(f"'{name}'" in script for name in expected)
    assert "teleop/camera/so101_wrist_cam" not in script
    assert "bridge.add_camera(" not in render
    assert "camera_picture_in_picture_start_slot=1" in render
    assert any(set(template["camera_ids"]) == expected for template in SCENE_TEMPLATES)


def test_wrist_view_contains_horizontal_fingers_entering_from_below():
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
    ]
    projected = []
    for point in points:
        right, up, back = camera_rotation.T @ (point - camera_position)
        depth = -back
        assert depth > 0.01
        assert abs(right / depth) < half_width
        assert abs(up / depth) < half_height
        projected.append(np.array([right, up]) / depth)

    center, finger1, finger2 = projected
    assert finger1[0] < center[0] < finger2[0]
    assert finger1[1] == pytest.approx(finger2[1], abs=1e-9)
    # The wrist lies behind the fingertips along the tool's local Z axis.
    wrist = tool_position + body_rotation @ np.array([0., 0., -0.10])
    right, up, back = camera_rotation.T @ (wrist - camera_position)
    assert -back > 0.01
    assert up / -back < min(finger1[1], finger2[1], center[1])


@pytest.mark.parametrize("template", ["sarm-task-box-20261012", "sarm-task-box-20261012-sdf", "sarm-task-box-20261012-demo"])
@pytest.mark.parametrize("mode", ["rgb", "fisheye", "monochrome", "monochrome_fisheye"])
def test_camera_mode_preserves_pose_and_only_demo_overrides_overview(monkeypatch, template, mode):
    import sys
    from types import SimpleNamespace
    import importlib.util
    # Basic CI has no Basilisk. Isolate only the scheduling boundary, and load
    # this component separately so no fake assembly is cached for native tests.
    monkeypatch.setitem(sys.modules, "simulation.assembly", SimpleNamespace(
        AssemblyContext=object, Slot=SimpleNamespace(OUTPUT=0)))
    spec = importlib.util.spec_from_file_location(
        "camera_render_probe", Path(__file__).resolve().parents[1] / "simulation/components/render.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    RenderComponent = module.RenderComponent

    overview = SimpleNamespace(camera_id="teleop/camera/spacecraft_overview", parent_id="teleop/cubesat_bus",
                               position_body_m=(1, 2, 3), field_of_view_rad=0.9)
    wrist = SimpleNamespace(camera_id="teleop/camera/sarm_wrist_cam", parent_id="teleop/ee_base",
                            position_body_m=(4, 5, 6), field_of_view_rad=1.2, display_name="Wrist")
    bridge = SimpleNamespace(_cameras=[overview, wrist], add_mj_scene=lambda *a, **kw: None)
    bridge.set_scene_settings = lambda settings: setattr(bridge, "settings", settings)
    monkeypatch.setitem(sys.modules, "bsk_render_adapter", SimpleNamespace(
        BasiliskRenderBridge=lambda **kw: bridge, SceneSettings=SimpleNamespace))
    monkeypatch.setitem(sys.modules, "simulation.teleop_grasp_unreal", SimpleNamespace(
        _register_celestial_bodies=lambda *a: None))
    ctx = SimpleNamespace(scene=object(), extra={"template_id": template},
                          add=lambda *a, **kw: None, keep_alive=lambda *a: None)
    RenderComponent(model_path="unused.xml", catalog={}, host="localhost", port=5558,
                    render_rate_hz=60, capture_rate_hz=30, capture_state_provider=None,
                    wrist_camera_mode=mode).install(ctx)
    assert wrist.parent_id == "teleop/ee_base"
    assert wrist.position_body_m == (4, 5, 6)
    assert wrist.camera_mode == mode
    assert wrist.field_of_view_rad == pytest.approx(
        math.radians(160) if mode in ("fisheye", "monochrome_fisheye") else 1.2)
    if template.endswith("-demo"):
        assert overview.parent_id == "teleop/locking_rod"
        assert bridge.settings.default_camera_target == "teleop/fastener"
        assert bridge.settings.default_camera_distance_m == .30
    else:
        assert overview.parent_id == "teleop/cubesat_bus"
        assert overview.position_body_m == (1, 2, 3)
        assert overview.field_of_view_rad == .9
        assert bridge.settings.default_camera_target == "teleop/cubesat_bus"
        assert bridge.settings.default_camera_distance_m == 2.8
