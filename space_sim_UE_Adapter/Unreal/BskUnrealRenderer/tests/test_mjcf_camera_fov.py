import math
from pathlib import Path
import tempfile
import unittest

from bsk_render_adapter.mjcf_assets import parse_mjcf_scene_metadata


class MjcfCameraFovTests(unittest.TestCase):
    def parse(self, camera):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scene.xml"
            path.write_text(f'<mujoco><compiler angle="radian"/><worldbody><body name="wrist">{camera}</body></worldbody></mujoco>', encoding="utf-8")
            return parse_mjcf_scene_metadata(path, namespace="teleop").cameras[0]

    def test_vertical_fov_is_converted_to_horizontal(self):
        for width, height in [(640, 360), (360, 640), (400, 400)]:
            with self.subTest(resolution=(width, height)):
                camera = self.parse(f'<camera name="sarm_wrist_cam" fovy="60" resolution="{width} {height}"/>')
                expected = 2 * math.atan(math.tan(math.radians(30)) * width / height)
                self.assertAlmostEqual(camera["field_of_view_rad"], expected)
                self.assertEqual(camera["camera_id"], "teleop/camera/sarm_wrist_cam")
                self.assertEqual(camera["parent_id"], "teleop/wrist")

    def test_sensor_fov_remains_horizontal(self):
        camera = self.parse('<camera sensorsize="0.00576 0.00324" focal="0.0036 0.0036" resolution="640 360"/>')
        self.assertAlmostEqual(camera["field_of_view_rad"], 2 * math.atan2(0.00576, 0.0072))

    def test_invalid_resolution_falls_back_without_division_by_zero(self):
        camera = self.parse('<camera fovy="60" resolution="640 0"/>')
        self.assertEqual(camera["resolution"], (1920, 1080))
        self.assertTrue(math.isfinite(camera["field_of_view_rad"]))


if __name__ == "__main__":
    unittest.main()
