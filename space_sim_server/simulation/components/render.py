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
                 visual_attachments: list[dict[str, str]] | None = None,
                 vision_capture: dict | None = None) -> None:
        self.model_path = model_path
        self.catalog = catalog
        self.host = host
        self.port = port
        self.render_rate_hz = int(render_rate_hz)
        self.capture_rate_hz = float(capture_rate_hz)
        self.capture_state_provider = capture_state_provider
        self.sunlight_intensity_scale = float(sunlight_intensity_scale)
        self.camera_display_names = camera_display_names or {}
        self.visual_attachments = visual_attachments or []
        self.vision_capture = vision_capture
        self.bridge: Any = None

    def install(self, ctx: AssemblyContext) -> None:
        from bsk_render_adapter import BasiliskRenderBridge, SceneSettings

        from simulation.teleop_grasp_unreal import _register_celestial_bodies
        from pathlib import Path
        from simulation.stereo_cameras import load_stereo_setup, register_stereo_cameras, configure_wrist_capture

        stereo = load_stereo_setup(Path(__file__).resolve().parents[2], self.vision_capture)
        wrist_enabled=bool(self.vision_capture and self.vision_capture.get('wrist_alignment_enabled') and ctx.extra.get('template_id')=='sarm-dual-capture26')
        vision_ids=list(stereo['camera_ids'].values()) if stereo else []
        if wrist_enabled:vision_ids.append('teleop/camera/capture_wrist_cam_alignment')

        bridge = BasiliskRenderBridge(
            reliable_frames=False,
            capture_state_provider=self.capture_state_provider,
            host=self.host,
            port=self.port,
            origin_object="teleop/cubesat_bus",
            frame_rate_hz=self.render_rate_hz,
            vision_capture_camera_ids=tuple(vision_ids),
            vision_capture_rate_hz=float(stereo.get('trigger_rate_hz', 10)) if stereo else 10,
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
        # Added after MJCF registration to preserve full stereo resolution and all
        # existing preview-camera poses, dimensions and product settings.
        register_stereo_cameras(bridge, stereo)
        configure_wrist_capture(bridge, wrist_enabled)
        if self.visual_attachments:
            import json
            from pathlib import Path
            from simulation.visual_attachments import attach_visual_geometries

            count = attach_visual_geometries(bridge._objects, self.visual_attachments,
                                            Path(__file__).resolve().parents[2])
            print(json.dumps({'type': 'visual_attachments_configuration',
                              'geometry_count': count, 'attachments': self.visual_attachments,
                              'scope': 'rendering_only'}), flush=True)
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
        if ctx.extra.get('template_id') == 'sarm-dual-capture26':
            # Internal reaction wheels remain in dynamics; omit only their display geometry.
            for body in bridge._objects:
                if body.object_id in {'teleop/rw_x', 'teleop/rw_y', 'teleop/rw_z'}:
                    body.geometries = []
            # Preview work lights are attached to optical components in UE.
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
