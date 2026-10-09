"""Renderer-neutral static scene descriptors.

All lengths use metres, angles use radians, and quaternions are active
parent-from-child rotations in ``(w, x, y, z)`` order.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import math
from typing import Any, Callable, Mapping, Sequence


def _list(values: Sequence[float]) -> list[float]:
    return [float(value) for value in values]


@dataclass
class GeometryVisual:
    """Describe one geometry attached to a rigid body."""

    geometry_id: str
    shape: str
    dimensions_m: Sequence[float]
    position_body_m: Sequence[float] = (0.0, 0.0, 0.0)
    orientation_body_from_geometry_wxyz: Sequence[float] = (1.0, 0.0, 0.0, 0.0)
    color_rgba: Sequence[float] = (0.7, 0.7, 0.7, 1.0)
    asset_type: str = ""
    asset_path: str = ""
    scale: Sequence[float] = (1.0, 1.0, 1.0)
    render_role: str = "visual"
    asset_key: str = ""
    material_name: str = ""
    material_specular: float = 0.5
    material_shininess: float = 0.25
    material_reflectance: float = 0.0
    material_emission: float = 0.0
    material_texture: str = ""
    material_texture_asset_path: str = ""
    material_texture_repeat: Sequence[float] = (1.0, 1.0)
    use_asset_materials: bool = False

    def to_payload(self) -> dict[str, Any]:
        payload = asdict(self)
        for key in (
            "dimensions_m",
            "position_body_m",
            "orientation_body_from_geometry_wxyz",
            "color_rgba",
            "scale",
            "material_texture_repeat",
        ):
            payload[key] = _list(payload[key])
        return payload


@dataclass
class VisualElement:
    """Describe a sensor, actuator, annotation, or other body-local visual."""

    visual_id: str
    kind: str
    parent_id: str
    position_body_m: Sequence[float] = (0.0, 0.0, 0.0)
    orientation_body_from_visual_wxyz: Sequence[float] = (1.0, 0.0, 0.0, 0.0)
    normal_body: Sequence[float] = (1.0, 0.0, 0.0)
    field_of_view_rad: Sequence[float] = ()
    size_m: float = 1.0
    range_m: float = 0.0
    color_rgba: Sequence[float] = (1.0, 1.0, 1.0, 1.0)
    label: str = ""
    properties: Mapping[str, Any] = field(default_factory=dict)
    channel_schema: Mapping[str, Mapping[str, Any]] = field(default_factory=dict)
    state_provider: Callable[..., Mapping[str, Any]] | None = field(default=None, repr=False)

    def to_payload(self) -> dict[str, Any]:
        return {
            "visual_id": self.visual_id,
            "kind": self.kind,
            "parent_id": self.parent_id,
            "position_body_m": _list(self.position_body_m),
            "orientation_body_from_visual_wxyz": _list(
                self.orientation_body_from_visual_wxyz
            ),
            "normal_body": _list(self.normal_body),
            "field_of_view_rad": _list(self.field_of_view_rad),
            "size_m": float(self.size_m),
            "range_m": float(self.range_m),
            "color_rgba": _list(self.color_rgba),
            "label": self.label,
            "properties": dict(self.properties),
            "channel_schema": {
                str(name): dict(definition)
                for name, definition in self.channel_schema.items()
            },
        }


@dataclass
class CameraVisual:
    """Describe a render camera attached to an optional body."""

    camera_id: str
    parent_id: str = ""
    position_body_m: Sequence[float] = (0.0, 0.0, 0.0)
    orientation_body_from_camera_wxyz: Sequence[float] = (1.0, 0.0, 0.0, 0.0)
    field_of_view_rad: float = 1.0471975511965976  # [rad]
    resolution: Sequence[int] = (1920, 1080)
    semantic_label: str = "camera"
    display_name: str = ""
    picture_in_picture: bool = False
    capture_rate_hz: float = 15.0
    picture_in_picture_slot: int = 0
    capture_products: Sequence[str] = ()

    def to_payload(self) -> dict[str, Any]:
        products = [str(value).strip().lower() for value in self.capture_products]
        unknown = sorted(set(products) - {"rgb"})
        if unknown:
            raise ValueError(f"unsupported camera capture products: {', '.join(unknown)}")
        return {
            "camera_id": self.camera_id,
            "parent_id": self.parent_id,
            "position_body_m": _list(self.position_body_m),
            "orientation_body_from_camera_wxyz": _list(
                self.orientation_body_from_camera_wxyz
            ),
            "field_of_view_rad": float(self.field_of_view_rad),
            "resolution": [int(value) for value in self.resolution],
            "semantic_label": self.semantic_label,
            "display_name": self.display_name or self.camera_id,
            "picture_in_picture": bool(self.picture_in_picture),
            "capture_rate_hz": float(self.capture_rate_hz),
            "picture_in_picture_slot": int(self.picture_in_picture_slot),
            "capture_products": list(dict.fromkeys(products)),
        }


@dataclass
class CelestialBodyVisual:
    """Static visualization properties for a celestial body."""

    body_id: str
    display_name: str
    mu_m3_s2: float
    equatorial_radius_m: float
    polar_radius_ratio: float = 1.0
    asset_path: str = ""
    visual_role: str = "body"
    luminous: bool = False
    drives_directional_light: bool = False
    light_color_rgb: Sequence[float] = (1.0, 0.98, 0.92)
    light_illuminance_lux_at_reference_distance: float = 0.0
    light_reference_distance_m: float = 149_597_870_693.0

    def to_payload(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["light_color_rgb"] = _list(self.light_color_rgb)
        return payload


@dataclass
class SceneSettings:
    """Renderer hints that never alter authoritative simulation state."""

    origin_object_id: str = ""
    skybox: str = "black"
    orbit_lines: bool = True
    trajectory_history: bool = True
    default_camera_target: str = ""
    default_camera_distance_m: float = 25.0
    spacecraft_scale_far_view: float = 1.0
    interpolation_delay_ms: float = 100.0  # [ms]
    max_extrapolation_ms: float = 100.0  # [ms]
    use_scene_lighting: bool = False
    headlight_enabled: bool = False
    headlight_diffuse_rgb: Sequence[float] = (0.6, 0.6, 0.6)
    headlight_ambient_rgb: Sequence[float] = (0.1, 0.1, 0.1)
    headlight_specular_rgb: Sequence[float] = (0.0, 0.0, 0.0)
    # Negative keeps the renderer configuration value; zero disables the
    # non-physical readability fill for ephemeris-lit scenes.
    fill_light_intensity_lux: float = -1.0
    sunlight_intensity_scale: float = 1.0  # [-], independent of renderer calibration

    def to_payload(self) -> dict[str, Any]:
        scale = self.sunlight_intensity_scale
        if (isinstance(scale, bool) or not isinstance(scale, (int, float))
                or not math.isfinite(scale) or not 0.0 <= scale <= 20_000.0):
            raise ValueError("sunlight_intensity_scale must be a finite number in [0, 20000]")
        payload = asdict(self)
        payload["sunlight_intensity_scale"] = float(scale)
        for key in ("headlight_diffuse_rgb", "headlight_ambient_rgb", "headlight_specular_rgb"):
            payload[key] = _list(payload[key])
        return payload
