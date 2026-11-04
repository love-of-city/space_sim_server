"""Registration of reaction wheels, thrusters and other bridge device visuals."""
from __future__ import annotations

from typing import Any, Iterable, Mapping

from .descriptors import VisualElement
from .scene_description import normalize_identifier


class DeviceRegistration:
    """Expose device discovery through the existing bridge registration API."""

    def add_reaction_wheels(
        self,
        visuals_or_effector: Any,
        *,
        parent_id: str | None = None,
        prefix: str | None = None,
    ) -> None:
        """Register explicit visuals or auto-discover a Basilisk RW effector."""

        if isinstance(visuals_or_effector, VisualElement) or (
            isinstance(visuals_or_effector, (list, tuple))
            and all(isinstance(item, VisualElement) for item in visuals_or_effector)
        ):
            self._add_visual_kind("reaction_wheel", visuals_or_effector)
            return
        if not parent_id:
            raise ValueError("parent_id is required when auto-discovering reaction wheels")
        from .device_adapters import reaction_wheel_visuals

        self._add_visual_kind(
            "reaction_wheel",
            reaction_wheel_visuals(visuals_or_effector, normalize_identifier(parent_id), prefix),
        )

    def add_mj_reaction_wheels(
        self,
        scene: Any,
        wheel_definitions: Iterable[Mapping[str, Any]],
        *,
        parent_id: str,
        prefix: str | None = None,
    ) -> None:
        """Register reaction wheels whose authoritative states are MJ joints."""

        from .device_adapters import mjscene_reaction_wheel_visuals

        self._add_visual_kind(
            "reaction_wheel",
            mjscene_reaction_wheel_visuals(
                scene, wheel_definitions, normalize_identifier(parent_id), prefix
            ),
        )

    def add_thrusters(
        self,
        visuals_or_effectors: Any,
        *,
        parent_id: str | None = None,
        colors: Any = None,
        prefix: str | None = None,
    ) -> None:
        """Register explicit visuals or auto-discover thruster effectors."""

        if isinstance(visuals_or_effectors, VisualElement) or (
            isinstance(visuals_or_effectors, (list, tuple))
            and all(isinstance(item, VisualElement) for item in visuals_or_effectors)
        ):
            self._add_visual_kind("thruster", visuals_or_effectors)
            return
        if not parent_id:
            raise ValueError("parent_id is required when auto-discovering thrusters")
        from .device_adapters import thruster_visuals

        self._add_visual_kind(
            "thruster",
            thruster_visuals(
                visuals_or_effectors,
                normalize_identifier(parent_id),
                colors=colors,
                prefix=prefix,
            ),
        )

    def add_css(
        self,
        visuals_or_devices: Any,
        *,
        parent_id: str | None = None,
        prefix: str | None = None,
    ) -> None:
        """Register explicit visuals or auto-discover coarse sun sensors."""

        if isinstance(visuals_or_devices, VisualElement) or (
            isinstance(visuals_or_devices, (list, tuple))
            and all(isinstance(item, VisualElement) for item in visuals_or_devices)
        ):
            self._add_visual_kind("css", visuals_or_devices)
            return
        if not parent_id:
            raise ValueError("parent_id is required when auto-discovering CSS devices")
        from .device_adapters import css_visuals

        self._add_visual_kind(
            "css",
            css_visuals(visuals_or_devices, normalize_identifier(parent_id), prefix),
        )

    def add_generic_sensors(self, visuals: VisualElement | Iterable[VisualElement]) -> None:
        self._add_visual_kind("generic_sensor", visuals)

    def add_transceivers(self, visuals: VisualElement | Iterable[VisualElement]) -> None:
        self._add_visual_kind("transceiver", visuals)

    def add_lights(self, visuals: VisualElement | Iterable[VisualElement]) -> None:
        self._add_visual_kind("light", visuals)

    def add_storage(self, visuals: VisualElement | Iterable[VisualElement]) -> None:
        self._add_visual_kind("storage", visuals)
