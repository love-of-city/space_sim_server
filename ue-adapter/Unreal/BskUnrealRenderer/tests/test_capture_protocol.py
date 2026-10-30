import io
import json
import struct
import tempfile
import unittest
from pathlib import Path

from bsk_render_adapter.capture import decode_capture_payload, receive_capture_packet, save_capture_frame


class CaptureProtocolTests(unittest.TestCase):
    @staticmethod
    def packet() -> bytes:
        rgb = b"rgb-data"
        depth = b"depth-data"
        metadata = {
            "protocol": "bsk-capture/1",
            "type": "camera_frame",
            "session_id": "session-a",
            "camera_id": "sat/camera",
            "capture_sequence": "7",
            "sim_time_ns": "9007199254740993",
            "products": [
                {"name": "rgb", "encoding": "png/bgra8_srgb", "file_name": "rgb.png", "blob_offset": "0", "byte_length": str(len(rgb))},
                {"name": "depth", "encoding": "pfm/float32/metres/camera_z", "file_name": "depth.pfm", "blob_offset": str(len(rgb)), "byte_length": str(len(depth))},
            ],
        }
        encoded = json.dumps(metadata, separators=(",", ":")).encode()
        payload = struct.pack("!I", len(encoded)) + encoded + rgb + depth
        return struct.pack("!I", len(payload)) + payload

    def test_decode_preserves_timestamps_and_product_boundaries(self):
        metadata, products = receive_capture_packet(io.BytesIO(self.packet()))
        self.assertEqual(metadata["sim_time_ns"], "9007199254740993")
        self.assertEqual(products, {"rgb": b"rgb-data", "depth": b"depth-data"})

    def test_rejects_overlapping_products(self):
        packet = self.packet()
        payload = packet[4:]
        metadata_length = struct.unpack("!I", payload[:4])[0]
        metadata = json.loads(payload[4 : 4 + metadata_length])
        metadata["products"][1]["blob_offset"] = "1"
        encoded = json.dumps(metadata).encode()
        malformed = struct.pack("!I", len(encoded)) + encoded + payload[4 + metadata_length :]
        with self.assertRaisesRegex(ValueError, "overlaps"):
            decode_capture_payload(malformed)

    def test_safe_disk_output(self):
        metadata, products = receive_capture_packet(io.BytesIO(self.packet()))
        metadata["products"][0]["file_name"] = "../rgb.png"
        with tempfile.TemporaryDirectory() as directory:
            path = save_capture_frame(directory, metadata, products)
            self.assertTrue(path.is_file())
            self.assertTrue((path.parent / "rgb.png").is_file())
            self.assertFalse((Path(directory) / "rgb.png").exists())


if __name__ == "__main__":
    unittest.main()
