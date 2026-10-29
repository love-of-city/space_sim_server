"""Camera-mounted work lights shared by live preview and dataset rendering."""
from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from bsk_render_adapter import VisualElement


def overview_camera_lamp(
    camera_id: str = "teleop/camera/spacecraft_overview",
) -> VisualElement:
    """Illuminate the metre-scale workbench along the overview camera's +X axis.

    This uses the same shadowless spot-light path as the MJCF wrist lamp. The
    camera mount is independent of sunlight and follows camera pose changes.
    Intensity uses the adapter's existing light scale (650 UE units per unit).
    """
    from bsk_render_adapter import VisualElement

    return VisualElement(
        visual_id="teleop/light/overview_lamp",
        kind="light",
        parent_id=camera_id,
        position_body_m=(0.0, 0.0, 0.0),
        normal_body=(1.0, 0.0, 0.0),
        range_m=2.0,
        color_rgba=(1.0, 1.0, 1.0, 1.0),
        properties={
            "light_type": "spot",
            "diffuse_rgb": (1.0, 1.0, 1.0),
            "specular_rgb": (0.15, 0.15, 0.15),
            "intensity": 8000.0,
            # A square 90-degree camera has a 54.7-degree half diagonal.
            # The wider cone keeps its corners lit rather than fading to black.
            "cutoff_deg": 89.0,
            "cast_shadows": False,
        },
    )
