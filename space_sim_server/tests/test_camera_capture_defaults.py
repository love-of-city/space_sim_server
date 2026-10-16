"""Camera manifest and launch defaults agree with square dataset capture."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
from space_arm_platform.camera_defaults import CAPTURE_CAMERA_RESOLUTION
from space_arm_platform.app import PlatformConfig
from bsk_render_adapter.mjcf_assets import parse_mjcf_scene_metadata

ROOT = Path(__file__).resolve().parents[1]

def test_active_cameras_square_and_preserve_horizontal_fov():
    model = ROOT / "model/SARM/platform/sarm_20261012.xml"
    cameras = ET.parse(model).findall(".//camera")
    metadata = parse_mjcf_scene_metadata(model, "teleop")
    assert len(cameras) == len(metadata.cameras) == 2
    legacy_fovy = {"spacecraft_overview": 51, "sarm_wrist_cam": 70}
    by_name = {camera["display_name"]: camera for camera in metadata.cameras}
    for camera in cameras:
        parsed = by_name[camera.get("name")]
        assert tuple(map(int, camera.get("resolution").split())) == CAPTURE_CAMERA_RESOLUTION == (1024, 1024)
        expected = 2 * math.atan(math.tan(math.radians(legacy_fovy[camera.get("name")]) / 2) * 16 / 9)
        assert math.isclose(parsed["field_of_view_rad"], expected, abs_tol=1e-10)
        assert parsed["resolution"] == CAPTURE_CAMERA_RESOLUTION

def test_launch_defaults_match_capture_canvas():
    config = PlatformConfig(project_root=ROOT, data_root=ROOT / "data")
    assert (config.runtime_pixel_streaming_camera_width, config.runtime_pixel_streaming_camera_height) == CAPTURE_CAMERA_RESOLUTION
    scripts = [ROOT / "scripts" / name for name in ("run_platform.ps1", "run_backend.ps1", "start_scene_instance.ps1")]
    scripts.append(ROOT.parent / "space_sim_UE_Adapter/Unreal/BskUnrealRenderer/scripts/start_renderer.ps1")
    for script in scripts:
        content = script.read_text(encoding="utf-8-sig")
        assert "$PixelStreamingCameraWidth = 1024" in content
        assert "$PixelStreamingCameraHeight = 1024" in content
