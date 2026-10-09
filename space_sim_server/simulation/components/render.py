"""Render bridge: publish body/geometry state to the UE renderer at the render rate."""
from __future__ import annotations

from typing import Any

from simulation.assembly import AssemblyContext, Slot


class RenderComponent:
    """Own the `BasiliskRenderBridge`: scene registration, settings, and schedule."""

    name = "render"

    def __init__(self, *, model_path: Any, catalog: Any, host: str, port: int,
                 render_rate_hz: int, capture_rate_hz: float,
                 capture_state_provider: Any,
                 sunlight_intensity_scale: float = 1.0,
                 camera_display_names: dict[str, str] | None = None) -> None:
        self.model_path = model_path
        self.catalog = catalog
        self.host = host
        self.port = port
        self.render_rate_hz = int(render_rate_hz)
        self.capture_rate_hz = float(capture_rate_hz)
        self.capture_state_provider = capture_state_provider
        self.sunlight_intensity_scale = float(sunlight_intensity_scale)
        self.camera_display_names = camera_display_names or {}
        self.bridge: Any = None

    def install(self, ctx: AssemblyContext) -> None:
        from bsk_render_adapter import BasiliskRenderBridge, SceneSettings

        from simulation.teleop_grasp_unreal import _register_celestial_bodies

        bridge = BasiliskRenderBridge(
            reliable_frames=False,
            capture_state_provider=self.capture_state_provider,
            host=self.host,
            port=self.port,
            origin_object="teleop/cubesat_bus",
            frame_rate_hz=self.render_rate_hz,
        )
        bridge.add_mj_scene(
            ctx.scene,
            namespace="teleop",
            source_path=self.model_path,
            mesh_asset_catalog=self.catalog,
            semantic_label="spacecraft_robot_link",
            camera_picture_in_picture=True,
            camera_capture_rate_hz=self.capture_rate_hz,
            camera_capture_products=("rgb",),
            camera_pip_resolution=(640, 360),
            camera_picture_in_picture_start_slot=1,
            camera_display_names=self.camera_display_names,
        )
        celestial = ctx.extra.get("celestial")
        if celestial is not None:
            _register_celestial_bodies(bridge, *celestial)
        bridge.set_scene_settings(
            SceneSettings(
                sunlight_intensity_scale=self.sunlight_intensity_scale,
                origin_object_id="teleop/cubesat_bus",
                # Focus targets must be registered body IDs, not MJCF sites.
                default_camera_target="teleop/cubesat_bus",
                default_camera_distance_m=2.8,
                orbit_lines=False,
                trajectory_history=False,
                interpolation_delay_ms=15.0,
                max_extrapolation_ms=50.0,
            )
        )
        self.bridge = bridge
        # OUTPUT runs after the physics core, so the frame carries this step's state.
        ctx.add(bridge, Slot.OUTPUT, every=1)
        ctx.keep_alive(bridge)
