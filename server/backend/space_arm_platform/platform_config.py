"""Validated backend configuration shared by CLI and application assembly."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from .camera_defaults import CAPTURE_CAMERA_WIDTH, CAPTURE_CAMERA_HEIGHT
from .sampling import DEFAULT_CAPTURE_HZ, DEFAULT_IK_HZ, LIVE_CAPTURE_FPS, ik_step_stride


@dataclass(frozen=True)
class PlatformConfig:
    project_root: Path
    data_root: Path
    simulation_host: str = "127.0.0.1"
    simulation_port: int = 8766
    capture_host: str = "127.0.0.1"
    capture_port: int = 8767
    deadman_timeout_s: float = 0.25
    pixel_streaming_player_port: int = 8080
    pixel_streaming_streamer_id: str = "BskRenderer"
    pixel_streaming_signalling_url: str = ""
    pixel_streaming_camera_streamers: tuple[tuple[str, str], ...] = ()
    stream_access_jwt_secret: str = ""
    stream_access_key: str = ""
    stream_access_token_ttl_seconds: int = 900
    runtime_adapter_root: Path | None = None
    runtime_model_root: Path | None = None
    runtime_unreal_root: Path | None = None
    runtime_powershell_exe: Path | None = None
    runtime_render_port: int = 5558
    runtime_pixel_streamer_port: int = 8888
    runtime_pixel_streaming_camera_ids: tuple[str, ...] = ()
    runtime_pixel_streaming_camera_width: int = CAPTURE_CAMERA_WIDTH
    runtime_pixel_streaming_camera_height: int = CAPTURE_CAMERA_HEIGHT
    runtime_preview_rate: float = 90.0
    runtime_encoder_min_quality: int = 60
    runtime_renderer_ready_timeout: int = 240
    runtime_ik_rate: float = DEFAULT_IK_HZ
    runtime_simulation_rate: float = 1.0
    runtime_capture_rate: float = DEFAULT_CAPTURE_HZ
    runtime_default_dataset_capture: bool = True
    auth_database: Path | None = None
    bootstrap_admin_username: str = "admin"
    bootstrap_admin_password: str = "ChangeMe123!"
    secure_cookies: bool = False
    allowed_origins: tuple[str, ...] = ()
    login_attempts_per_minute: int = 10

    def __post_init__(self) -> None:
        ik_step_stride(self.runtime_ik_rate)
        if self.runtime_default_dataset_capture and self.runtime_capture_rate not in LIVE_CAPTURE_FPS:
            raise ValueError(f"dataset capture rate must be one of {LIVE_CAPTURE_FPS}")
        if type(self.runtime_encoder_min_quality) is not int or not 0 <= self.runtime_encoder_min_quality <= 100:
            raise ValueError("runtime_encoder_min_quality must be an integer in [0, 100]")
