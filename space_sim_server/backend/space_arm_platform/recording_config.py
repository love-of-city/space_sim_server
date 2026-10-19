"""Validated capture, recording and video defaults loaded at backend startup."""
from pathlib import Path
import re
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from .sampling import DEFAULT_CAPTURE_HZ


DEFAULT_CAMERA_IDS = ("teleop/camera/spacecraft_overview", "teleop/camera/sarm_wrist_cam")


class VideoEncodingConfig(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)

    codec: Literal["h264", "h264_nvenc"] = "h264"
    streaming_encoding: bool = False
    encoder_threads: int = Field(default=2, ge=1, le=64)
    image_writer_threads: int = Field(default=0, ge=0, le=64)
    encoder_queue_maxsize: int = Field(default=30, ge=1, le=1024)
    quality: int = Field(default=30, ge=0, le=51)
    keyframe_interval: int = Field(default=2, ge=1, le=250)

    @model_validator(mode="after")
    def validate_encoding(self):
        if self.codec == "h264_nvenc" and self.keyframe_interval < 2:
            raise ValueError("NVENC requires keyframe_interval >= 2")
        if not self.streaming_encoding and (self.quality != 30 or self.keyframe_interval != 2):
            raise ValueError("Custom quality/keyframe interval requires streaming_encoding")
        return self


class RecordingConfig(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)

    save_raw_camera_images: bool = False
    storage_profile: Literal["compact", "diagnostic"] = "compact"
    save_dynamics_trace: bool = False
    scripted_native_geometry: bool = False
    capture_backpressure_poll_s: float = Field(default=0.002, ge=0.001, le=0.1)
    dataset_capture: bool = True
    capture_rate_hz: Literal[1, 2, 5, 10, 25, 50] = DEFAULT_CAPTURE_HZ
    camera_ids: list[str] = Field(default_factory=lambda: list(DEFAULT_CAMERA_IDS))
    capture_products: list[Literal["rgb"]] = Field(default_factory=lambda: ["rgb"])
    max_frames: int = Field(default=18000, ge=1, le=108000)
    video: VideoEncodingConfig = Field(default_factory=VideoEncodingConfig)

    @model_validator(mode="after")
    def validate_capture(self):
        keys = [re.sub(r"[^a-zA-Z0-9_]", "_", camera) for camera in self.camera_ids]
        if any(not camera.strip() for camera in self.camera_ids) or len(set(keys)) != len(keys):
            raise ValueError("camera IDs must be nonempty and have unique dataset keys")
        if len(set(self.capture_products)) != len(self.capture_products):
            raise ValueError("duplicate capture products")
        if self.camera_ids and "rgb" not in self.capture_products:
            raise ValueError("RGB is required for camera datasets")
        return self

    def episode_defaults(self) -> dict:
        return {"fps": self.capture_rate_hz, "camera_ids": list(self.camera_ids),
                "capture_products": list(self.capture_products), "max_frames": self.max_frames}


def load_recording_config(path: Path) -> RecordingConfig:
    try:
        document = path.read_text(encoding="utf-8-sig")
    except FileNotFoundError:
        return RecordingConfig()
    try:
        return RecordingConfig.model_validate_json(document)
    except ValueError as error:
        raise ValueError(f"Invalid recording configuration {path}: {error}") from error
