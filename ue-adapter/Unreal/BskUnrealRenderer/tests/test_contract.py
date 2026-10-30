import json
import math
import struct
import tomllib
import unittest
from pathlib import Path

from bsk_render_adapter.frames import FrameConverter
from bsk_render_adapter.protocol import PROTOCOL_V2, encode_packet
from bsk_unreal_adapter.coordinates import (
    wire_position_to_unreal_cm,
    wire_quat_wxyz_to_ue_xyzw,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PROJECT_ROOT.parents[1]


class WireContractTests(unittest.TestCase):
    def test_repository_versions_are_synchronized(self):
        version = (REPOSITORY_ROOT / "VERSION").read_text(encoding="utf-8").strip()
        project = tomllib.loads((REPOSITORY_ROOT / "pyproject.toml").read_text(encoding="utf-8"))
        plugin = json.loads(
            (PROJECT_ROOT / "Plugins" / "BskUnrealRuntime" / "BskUnrealRuntime.uplugin").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(version, project["project"]["version"])
        self.assertEqual(version, plugin["VersionName"])

    def test_reuses_network_order_length_prefixed_json(self):
        message = {
            "protocol": PROTOCOL_V2,
            "type": "frame",
            "session_id": "contract-test",
            "manifest_revision": "1",
            "frame_id": "4",
            "sim_time_ns": "12",
            "origin_N_m": [0.0, 0.0, 0.0],
            "objects": [],
        }
        packet = encode_packet(message)
        (length,) = struct.unpack("!I", packet[:4])
        self.assertEqual(length, len(packet) - 4)
        self.assertEqual(json.loads(packet[4:].decode("utf-8")), message)

    def test_floating_origin_then_unreal_units_and_axes(self):
        converter = FrameConverter([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        local = converter.position([1002.0, 2003.0, 3004.0], [1000.0, 2000.0, 3000.0])
        self.assertEqual(wire_position_to_unreal_cm(local), [300.0, 200.0, 400.0])

    def test_bsk_passive_mrp_to_active_ue_quaternion(self):
        sigma = [0.0, 0.0, math.tan(math.pi / 8.0)]
        wire_wxyz = FrameConverter().body_orientation(sigma)
        ue_xyzw = wire_quat_wxyz_to_ue_xyzw(wire_wxyz)
        expected = math.sqrt(0.5)
        for actual, wanted in zip(ue_xyzw, [0.0, 0.0, -expected, expected]):
            self.assertAlmostEqual(actual, wanted, places=12)

    def test_zero_quaternion_is_rejected(self):
        with self.assertRaises(ValueError):
            wire_quat_wxyz_to_ue_xyzw([0.0, 0.0, 0.0, 0.0])

    def test_sensor_helpers_are_hidden_by_default_and_have_runtime_keys(self):
        config = json.loads((PROJECT_ROOT / "Config" / "bsk_unreal_scene.json").read_text(encoding="utf-8"))
        visibility = config["visuals"]["visibility_by_kind"]
        self.assertFalse(visibility["css"])
        self.assertFalse(visibility["generic_sensor"])
        self.assertFalse(visibility["transceiver"])

        input_config = (PROJECT_ROOT / "Config" / "DefaultInput.ini").read_text(encoding="utf-8")
        self.assertIn('ActionName="ToggleCssVisuals"', input_config)
        self.assertIn('ActionName="ToggleGenericSensorVisuals"', input_config)
        self.assertIn('ActionName="ToggleTransceiverVisuals"', input_config)
        self.assertIn('ActionName="TogglePictureInPictureOne"', input_config)
        self.assertIn('ActionName="TogglePictureInPictureTwo"', input_config)


if __name__ == "__main__":
    unittest.main()
