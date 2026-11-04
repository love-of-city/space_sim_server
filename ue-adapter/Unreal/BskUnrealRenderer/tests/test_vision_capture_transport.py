"""Vision protocol tests exercise Python scheduling without a native simulator.

Only the missing native base constructor is stubbed for the protocol harness;
bridge initialization, validation, Reset, and UpdateState remain real. When
Basilisk is installed, these same cases use its real SysModel base instead.
"""
import json
import math
from unittest.mock import patch

import pytest

from bsk_render_adapter import BasiliskRenderBridge, CameraVisual
from bsk_render_adapter import bridge as bridge_module
from bsk_render_adapter.protocol import RenderPublisher


class Publisher:
    def __init__(self): self.frames = []
    def retain_hello(self, message): pass
    def retain_manifest(self, message): pass
    def publish_frame(self, message): self.frames.append(message)
    def publish_event(self, message): return True
    def close(self): pass


@pytest.fixture
def protocol_bridge(monkeypatch):
    if bridge_module.sysModel is None:
        monkeypatch.setattr(bridge_module._MissingBasiliskBase, "__init__", lambda self: None)
    return BasiliskRenderBridge


def test_default_bridge_does_not_request_vision(protocol_bridge):
    publisher = Publisher()
    bridge = protocol_bridge(publisher=publisher)
    bridge.Reset(0)
    bridge.UpdateState(0)
    assert not publisher.frames[0].get("vision_capture_requested", False)
    assert "vision_capture_camera_ids" not in publisher.frames[0]


def test_vision_trigger_uses_published_simulation_grid_and_preserves_dataset_idle(protocol_bridge):
    publisher = Publisher()
    bridge = protocol_bridge(
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
def test_vision_rejects_invalid_sampling_rate(rate, protocol_bridge):
    with pytest.raises(ValueError, match="vision_capture_rate_hz"):
        protocol_bridge(publisher=Publisher(), vision_capture_camera_ids=("left",), vision_capture_rate_hz=rate)


def test_vision_rejects_rate_not_contained_in_published_frame_grid(protocol_bridge):
    with pytest.raises(ValueError, match="frame_rate_hz"):
        protocol_bridge(publisher=Publisher(), frame_rate_hz=30,
                        vision_capture_camera_ids=("left",), vision_capture_rate_hz=25)


def test_protocol_harness_preserves_missing_basilisk_guard():
    # The fixture must not make the production bridge silently usable without
    # its native simulator, or alter subsequent tests in the same process.
    guarded_type = BasiliskRenderBridge if bridge_module.sysModel is None else bridge_module._MissingBasiliskBase
    with pytest.raises(RuntimeError, match="Basilisk is unavailable"):
        guarded_type()


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


@pytest.mark.parametrize("mode", ["rgb", "fisheye", "monochrome", "monochrome_fisheye"])
def test_vision_camera_descriptor_defaults_and_calibration_fields(mode):
    legacy = CameraVisual("legacy").to_payload()
    assert legacy["capture_consumer"] == "dataset"
    assert legacy["exposure_time_s"] == legacy["aperture_fnumber"] == 0.0
    vision = CameraVisual("left", capture_consumer="vision", exposure_time_s=.002,
                          aperture_fnumber=4, resolution=(1920, 1200), capture_rate_hz=10,
                          camera_mode=mode, field_of_view_rad=math.radians(160)).to_payload()
    assert vision["capture_consumer"] == "vision"
    assert vision["camera_mode"] == mode
    assert vision["exposure_time_s"] == .002
    assert vision["aperture_fnumber"] == 4


@pytest.mark.parametrize("fields", [{"capture_consumer": "bogus"}, {"exposure_time_s": -1},
                                  {"aperture_fnumber": float("nan")}])
def test_camera_rejects_invalid_capture_contract(fields):
    with pytest.raises(ValueError):
        CameraVisual("camera", **fields).to_payload()
