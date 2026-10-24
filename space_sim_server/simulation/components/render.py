"""Render bridge: publish body/geometry state to the UE renderer at the render rate."""
from __future__ import annotations

from typing import Any

from simulation.assembly import AssemblyContext, Slot
from space_arm_platform.camera_defaults import CAPTURE_CAMERA_RESOLUTION


class RenderComponent:
    """Own the `BasiliskRenderBridge`: scene registration, settings, and schedule."""

    name = "render"

    def __init__(self, *, model_path: Any, catalog: Any, host: str, port: int,
                 render_rate_hz: int, capture_rate_hz: float,
                 capture_state_provider: Any,
                 sunlight_intensity_scale: float = 1.0,
                 camera_display_names: dict[str, str] | None = None,
                 wrist_camera_mode: str = "rgb",
                 overview_camera: dict | None = None) -> None:
        if wrist_camera_mode not in ("rgb", "fisheye", "monochrome"):
            raise ValueError("unsupported wrist_camera_mode")
        self.wrist_camera_mode = wrist_camera_mode
        self.overview_camera = overview_camera
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
            camera_pip_resolution=CAPTURE_CAMERA_RESOLUTION,
            camera_picture_in_picture_start_slot=1,
            camera_display_names=self.camera_display_names,
        )
        for camera in bridge._cameras:
            if camera.camera_id == "teleop/camera/sarm_wrist_cam":
                camera.camera_mode = self.wrist_camera_mode
                if self.wrist_camera_mode == "fisheye":
                    import math
                    camera.field_of_view_rad = math.radians(160.0)
                camera.display_name += {"rgb": "", "fisheye": " (Fisheye 160°)",
                                        "monochrome": " (Monochrome)"}[self.wrist_camera_mode]
        if self.overview_camera is not None:
            from space_arm_platform.overview_camera import CAMERA_ID
            matches = [camera for camera in bridge._cameras if camera.camera_id == CAMERA_ID]
            if len(matches) != 1:
                raise ValueError("Selected overview camera is missing from the render scene")
            camera = matches[0]
            for key in ("parent_id", "position_body_m", "orientation_body_from_camera_wxyz", "field_of_view_rad"):
                setattr(camera, key, self.overview_camera[key])
            camera.display_name += f" ({self.overview_camera['preset_id']})"
        if ctx.extra.get('template_id') == 'sarm-task-box-20261012-demo':
            # Reuse the existing overview stream slot for a body-fixed close-up.
            # Pose derived from the validated MuJoCo side-view camera, in rod axes.
            for camera in bridge._cameras:
                if camera.camera_id == 'teleop/camera/spacecraft_overview':
                    camera.parent_id = 'teleop/locking_rod'
                    camera.position_body_m = (-.03594668, .11668338, -.25216004)
                    camera.orientation_body_from_camera_wxyz = (.53344552, -.53344625, -.36045813, -.54858083)
                    camera.field_of_view_rad = 1.0471975511965976
                    camera.display_name = 'Fastener close-up (demo)'
        celestial = ctx.extra.get("celestial")
        if celestial is not None:
            _register_celestial_bodies(bridge, *celestial)
        bridge.set_scene_settings(
            SceneSettings(
                sunlight_intensity_scale=self.sunlight_intensity_scale,
                origin_object_id="teleop/cubesat_bus",
                # Focus targets must be registered body IDs, not MJCF sites.
                default_camera_target=("teleop/fastener" if ctx.extra.get('template_id') == 'sarm-task-box-20261012-demo' else "teleop/cubesat_bus"),
                default_camera_distance_m=(.30 if ctx.extra.get('template_id') == 'sarm-task-box-20261012-demo' else 2.8),
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
