"""Independent vision requests preserve exact source frames without an episode."""
import json
from unittest.mock import patch

import pytest

from bsk_render_adapter import BasiliskRenderBridge, CameraVisual
from bsk_render_adapter.protocol import RenderPublisher


class Publisher:
    def __init__(self): self.frames = []
    def retain_hello(self, message): pass
    def retain_manifest(self, message): pass
    def publish_frame(self, message): self.frames.append(message)
    def publish_event(self, message): return True
    def close(self): pass


def test_default_bridge_does_not_request_vision():
    publisher = Publisher()
    bridge = BasiliskRenderBridge(publisher=publisher)
    bridge.Reset(0)
    bridge.UpdateState(0)
    assert not publisher.frames[0].get("vision_capture_requested", False)
    assert "vision_capture_camera_ids" not in publisher.frames[0]


def test_vision_trigger_uses_published_simulation_grid_and_preserves_dataset_idle():
    publisher = Publisher()
    bridge = BasiliskRenderBridge(
        publisher=publisher, frame_rate_hz=50,
        capture_state_provider=lambda: {"capture_episode_id": "", "capture_request_id": ""},
        vision_capture_camera_ids=("teleop/camera/left", "teleop/camera/right"),
        vision_capture_rate_hz=10,
    )
    bridge.Reset(0)
    for stamp in range(0, 220_000_001, 10_000_000):
        bridge.UpdateState(stamp)
    requested = [f for f in publisher.frames if f.get("vision_capture_requested")]
    assert [f["sim_time_ns"] for f in requested] == ["0", "100000000", "200000000"]
    assert [f["frame_id"] for f in requested] == ["0", "5", "10"]
    assert all(f["capture_episode_id"] == "" for f in publisher.frames)
    assert all(f["vision_capture_camera_ids"] == ["teleop/camera/left", "teleop/camera/right"] for f in requested)
    assert all("vision_capture_camera_ids" not in f for f in publisher.frames if not f.get("vision_capture_requested"))


@pytest.mark.parametrize("rate", [0, -1, 7, 9.5, float("nan"), True])
def test_vision_rejects_invalid_sampling_rate(rate):
    with pytest.raises(ValueError, match="vision_capture_rate_hz"):
        BasiliskRenderBridge(publisher=Publisher(), vision_capture_camera_ids=("left",), vision_capture_rate_hz=rate)


def test_vision_rejects_rate_not_contained_in_published_frame_grid():
    with pytest.raises(ValueError, match="frame_rate_hz"):
        BasiliskRenderBridge(publisher=Publisher(), frame_rate_hz=30,
                            vision_capture_camera_ids=("left",), vision_capture_rate_hz=25)


@pytest.mark.parametrize("legacy_reliable", [False, True])
def test_vision_source_frames_survive_idle_preview_replacement(legacy_reliable):
    publisher = RenderPublisher(reliable_frames=legacy_reliable)
    with patch.object(publisher, "start"):
        for i in range(20):
            publisher.publish_frame(dict(frame_id=str(i), capture_episode_id="",
                                         vision_capture_requested=i in (0, 5, 10)))
    frames = [json.loads(publisher._strict_frames.get_nowait()[4:])["frame_id"] for _ in range(3)]
    assert frames == ["0", "5", "10"]
    assert json.loads(publisher._preview_frames.get_nowait()[4:])["frame_id"] == "19"
    assert publisher.stats.vision_frames_queued == 3


def test_vision_and_legacy_reliable_frames_share_ordered_fifo():
    publisher = RenderPublisher(reliable_frames=True)
    with patch.object(publisher, "start"):
        for i in range(4):
            publisher.publish_frame(dict(frame_id=str(i), vision_capture_requested=i == 1))
    assert [json.loads(publisher._strict_frames.get_nowait()[4:])["frame_id"] for _ in range(4)] == ["0", "1", "2", "3"]


def test_vision_camera_descriptor_defaults_and_calibration_fields():
    legacy = CameraVisual("legacy").to_payload()
    assert legacy["capture_consumer"] == "dataset"
    assert legacy["exposure_time_s"] == legacy["aperture_fnumber"] == 0.0
    vision = CameraVisual("left", capture_consumer="vision", exposure_time_s=.002,
                          aperture_fnumber=4, resolution=(1920, 1200), capture_rate_hz=10).to_payload()
    assert vision["capture_consumer"] == "vision"
    assert vision["exposure_time_s"] == .002
    assert vision["aperture_fnumber"] == 4


@pytest.mark.parametrize("fields", [{"capture_consumer": "bogus"}, {"exposure_time_s": -1},
                                  {"aperture_fnumber": float("nan")}])
def test_camera_rejects_invalid_capture_contract(fields):
    with pytest.raises(ValueError):
        CameraVisual("camera", **fields).to_payload()
