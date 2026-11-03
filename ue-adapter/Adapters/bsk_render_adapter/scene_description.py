"""Scene descriptors, MJCF visual conversion and bridge object registration."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np

from .descriptors import CameraVisual, CelestialBodyVisual, GeometryVisual, SceneSettings, VisualElement
from .frames import vector
from .mjcf_assets import (
    load_asset_catalog, load_texture_catalog, parse_mjcf_geometry_metadata,
    parse_mjcf_body_parents, parse_mjcf_scene_metadata, resolve_asset, resolve_texture,
)
from .protocol import PROTOCOL_V2

try:
    from Basilisk.architecture import messaging
except ImportError:
    messaging = None


@dataclass
class ObjectBinding:
    object_id: str
    display_name: str
    reader: Any
    parent_id: str = ""
    asset_path: str = ""
    semantic_label: str = "spacecraft"
    geometries: list[GeometryVisual] = field(default_factory=list)


@dataclass
class CelestialBinding:
    visual: CelestialBodyVisual
    reader: Any


_MJGEOM_SPHERE = 2
_MJGEOM_CAPSULE = 3
_MJGEOM_ELLIPSOID = 4
_MJGEOM_CYLINDER = 5
_MJGEOM_BOX = 6


def normalize_identifier(value: str) -> str:
    result = value.strip().replace("\\", "/").strip("/")
    if not result or ".." in result.split("/"):
        raise ValueError(f"invalid stable identifier: {value!r}")
    return result


def rgba(values: Iterable[float]) -> list[float]:
    result = vector(values, 4, "rgba")
    return np.clip(result, 0.0, 1.0).tolist()


def primitive_geometry(geom: Any, geometry_id: str) -> GeometryVisual | None:
    size = vector(geom.size, 3, "geom.size")
    geom_type = int(geom.type)
    if geom_type == _MJGEOM_BOX:
        shape = "box"
        dimensions = 2.0 * size
    elif geom_type == _MJGEOM_SPHERE:
        shape = "sphere"
        dimensions = np.repeat(2.0 * size[0], 3)
    elif geom_type == _MJGEOM_CYLINDER:
        shape = "cylinder"
        dimensions = np.array([2.0 * size[0], 2.0 * size[0], 2.0 * size[1]])
    elif geom_type == _MJGEOM_CAPSULE:
        shape = "capsule"
        dimensions = np.array([2.0 * size[0], 2.0 * size[0], 2.0 * (size[1] + size[0])])
    elif geom_type == _MJGEOM_ELLIPSOID:
        shape = "ellipsoid"
        dimensions = 2.0 * size
    else:
        return None
    return GeometryVisual(
        geometry_id=geometry_id,
        shape=shape,
        dimensions_m=dimensions.tolist(),
        position_body_m=vector(geom.pos, 3, "geom.pos").tolist(),
        orientation_body_from_geometry_wxyz=vector(geom.quat, 4, "geom.quat").tolist(),
        color_rgba=rgba(geom.rgba),
    )


def mesh_geometry(
    geom: Any,
    geometry_id: str,
    metadata: Any,
    asset_catalog: Mapping[str, Any],
    texture_catalog: Mapping[str, str],
) -> GeometryVisual:
    asset = resolve_asset(asset_catalog, metadata.source_path)
    catalog_scale = asset.component_scale if asset else (1.0, 1.0, 1.0)
    scale = [float(metadata.mesh_scale[index]) * float(catalog_scale[index]) for index in range(3)]
    return GeometryVisual(
        geometry_id=geometry_id,
        shape="mesh",
        dimensions_m=(1.0, 1.0, 1.0),
        position_body_m=list(metadata.position_body_m),
        orientation_body_from_geometry_wxyz=list(metadata.orientation_body_from_geometry_wxyz),
        color_rgba=list(metadata.material_rgba) if metadata.material_rgba else rgba(geom.rgba),
        asset_type=asset.asset_type if asset else "static_mesh",
        asset_path=asset.asset_path if asset else "",
        scale=scale,
        render_role=metadata.role,
        asset_key=metadata.mesh_name,
        material_name=metadata.material_name,
        material_specular=metadata.material_specular,
        material_shininess=metadata.material_shininess,
        material_reflectance=metadata.material_reflectance,
        material_emission=metadata.material_emission,
        material_texture=metadata.material_texture,
        material_texture_asset_path=resolve_texture(texture_catalog, metadata.material_texture_source),
        material_texture_repeat=metadata.material_texture_repeat,
        use_asset_materials=metadata.use_asset_materials,
    )


class SceneRegistration:
    """Descriptor registration shared by the public render bridge."""

    def add_object(
        self,
        object_id: str,
        state_message: Any,
        *,
        display_name: str | None = None,
        parent_id: str = "",
        asset_path: str = "",
        semantic_label: str = "spacecraft",
        geometries: Sequence[GeometryVisual] = (),
    ) -> str:
        """Register one ``SCStates`` compatible output message."""

        if messaging is None:
            raise RuntimeError("Basilisk messaging is unavailable")
        stable_id = normalize_identifier(object_id)
        if any(binding.object_id == stable_id for binding in self._objects):
            raise ValueError(f"duplicate object id: {stable_id}")
        reader = messaging.SCStatesMsgReader()
        reader.subscribeTo(state_message)
        self._objects.append(
            ObjectBinding(
                stable_id,
                display_name or stable_id.rsplit("/", 1)[-1],
                reader,
                normalize_identifier(parent_id) if parent_id else "",
                asset_path,
                semantic_label,
                list(geometries),
            )
        )
        self._manifest_revision += 1
        return stable_id

    def add_spacecraft(
        self,
        spacecraft: Any,
        *,
        object_id: str | None = None,
        name: str | None = None,
        asset_path: str = "",
        semantic_label: str = "spacecraft",
    ) -> str:
        """Register a regular Basilisk ``Spacecraft``."""

        stable_id = object_id or name or spacecraft.ModelTag
        return self.add_object(
            stable_id,
            spacecraft.scStateOutMsg,
            display_name=name or spacecraft.ModelTag,
            asset_path=asset_path,
            semantic_label=semantic_label,
        )

    def add_mj_scene(
        self,
        scene: Any,
        *,
        namespace: str | None = None,
        asset_map: Mapping[str, str] | None = None,
        source_path: str | Path | None = None,
        mesh_asset_catalog: str | Path | Mapping[str, Any] | None = None,
        semantic_label: str = "spacecraft_part",
        camera_picture_in_picture: bool = False,
        camera_capture_rate_hz: float = 15.0,
        camera_pip_resolution: Sequence[int] = (480, 270),
        camera_picture_in_picture_start_slot: int = 1,
        camera_display_names: Mapping[str, str] | None = None,
        camera_capture_products: Sequence[str] = (),
    ) -> dict[str, str]:
        """Auto-register MJScene bodies, hierarchy, geometry, and state messages."""

        prefix = normalize_identifier(namespace or getattr(scene, "ModelTag", "mjscene"))
        body_names = list(scene.getBodyNames())
        ids = {name: f"{prefix}/{name}" for name in body_names}
        geometry_by_body: dict[str, list[GeometryVisual]] = {name: [] for name in body_names}
        geom_infos = scene.getGeomInfos()
        source_metadata = parse_mjcf_geometry_metadata(source_path) if source_path else []
        source_parents = parse_mjcf_body_parents(source_path) if source_path else {}
        catalog = load_asset_catalog(mesh_asset_catalog)
        texture_catalog = load_texture_catalog(mesh_asset_catalog)
        for index in range(len(geom_infos)):
            geom = geom_infos[index]
            body_name = str(geom.bodyName)
            if body_name not in geometry_by_body:
                continue
            metadata = source_metadata[index] if index < len(source_metadata) else None
            if rgba(geom.rgba)[3] <= 0:
                continue
            if metadata is not None and metadata.body_name == body_name and metadata.mesh_name:
                geometry = mesh_geometry(
                    geom,
                    f"{ids[body_name]}/geom/{index}",
                    metadata,
                    catalog,
                    texture_catalog,
                )
            else:
                geometry = primitive_geometry(geom, f"{ids[body_name]}/geom/{index}")
                if geometry is not None and metadata is not None:
                    geometry.render_role = metadata.role
                    geometry.material_name = metadata.material_name
                    geometry.material_specular = metadata.material_specular
                    geometry.material_shininess = metadata.material_shininess
                    geometry.material_reflectance = metadata.material_reflectance
                    geometry.material_emission = metadata.material_emission
                    geometry.material_texture = metadata.material_texture
                    geometry.material_texture_repeat = metadata.material_texture_repeat
                    if metadata.material_rgba:
                        geometry.color_rgba = list(metadata.material_rgba)
            if geometry is not None:
                geometry_by_body[body_name].append(geometry)
        assets = dict(asset_map or {})
        for body_name in body_names:
            # Basilisk's SWIG getBodyParentName can access-violate for bodies
            # attached through an MjSpec <frame>.  Prefer the renderer metadata
            # source when available and keep the API fallback for source-less
            # scenes and existing integrations.
            parent_name = source_parents.get(body_name)
            if parent_name is None:
                parent_name = str(scene.getBodyParentName(body_name))
            self.add_object(
                ids[body_name],
                scene.getBody(body_name).getOrigin().stateOutMsg,
                display_name=body_name,
                parent_id="" if parent_name == "world" else ids[parent_name],
                asset_path=assets.get(body_name, ""),
                semantic_label=semantic_label,
                geometries=geometry_by_body[body_name],
            )
        if source_path:
            scene_metadata = parse_mjcf_scene_metadata(source_path, prefix)
            if scene_metadata.headlight_enabled or scene_metadata.lights:
                self._settings.use_scene_lighting = True
            if scene_metadata.headlight_enabled:
                self._settings.headlight_enabled = True
                self._settings.headlight_diffuse_rgb = scene_metadata.headlight_diffuse_rgb
                self._settings.headlight_ambient_rgb = scene_metadata.headlight_ambient_rgb
                self._settings.headlight_specular_rgb = scene_metadata.headlight_specular_rgb
            for light in scene_metadata.lights:
                self.add_visual(
                    VisualElement(
                        visual_id=light["visual_id"],
                        kind="light",
                        parent_id=light["parent_id"],
                        position_body_m=light["position_body_m"],
                        normal_body=light["normal_body"],
                        color_rgba=light["color_rgba"],
                        properties=light["properties"],
                    )
                )
            for slot, camera in enumerate(
                scene_metadata.cameras, start=max(1, int(camera_picture_in_picture_start_slot))
            ):
                resolution = camera_pip_resolution if camera_picture_in_picture else camera["resolution"]
                camera_payload = {key: value for key, value in camera.items() if key != "resolution"}
                camera_payload["display_name"] = (camera_display_names or {}).get(
                    camera["display_name"], camera["display_name"]
                )
                self.add_camera(
                    CameraVisual(
                        **camera_payload,
                        resolution=resolution,
                        semantic_label="mjcf_camera",
                        picture_in_picture=camera_picture_in_picture,
                        capture_rate_hz=camera_capture_rate_hz,
                        picture_in_picture_slot=slot if camera_picture_in_picture else 0,
                        capture_products=camera_capture_products,
                    )
                )
        return ids

    def add_celestial_bodies(
        self,
        bodies: Iterable[Any],
        *,
        visual_overrides: Mapping[str, Mapping[str, Any]] | None = None,
    ) -> None:
        """Register gravity bodies and renderer-neutral ephemeris visuals.

        ``visual_overrides`` is keyed by stable body ID or display name.  It
        makes star/light semantics explicit without requiring a renderer to
        infer physics from a display string.  The legacy ``sun`` name default
        remains for existing scenarios, but new scenarios should declare it.
        """

        normalized_overrides = {
            str(key).casefold(): dict(value)
            for key, value in (visual_overrides or {}).items()
        }
        allowed_overrides = {
            "asset_path",
            "visual_role",
            "luminous",
            "drives_directional_light",
            "light_color_rgb",
            "light_illuminance_lux_at_reference_distance",
            "light_reference_distance_m",
        }

        for body in bodies:
            display_name = str(getattr(body, "displayName", "") or getattr(body, "planetName", "body"))
            body_id = normalize_identifier(display_name.casefold())
            if any(binding.visual.body_id == body_id for binding in self._celestial):
                continue
            legacy_star = display_name.casefold() == "sun"
            override = normalized_overrides.get(body_id.casefold(), normalized_overrides.get(display_name.casefold(), {}))
            unknown = sorted(set(override) - allowed_overrides)
            if unknown:
                raise ValueError(f"unsupported celestial visual overrides for {body_id}: {', '.join(unknown)}")
            visual_values: dict[str, Any] = {
                "asset_path": str(getattr(body, "modelDictionaryKey", "")),
                "visual_role": "star" if legacy_star else "body",
                "luminous": legacy_star,
                "drives_directional_light": legacy_star,
                **override,
            }
            visual = CelestialBodyVisual(
                body_id=body_id,
                display_name=display_name,
                mu_m3_s2=float(body.mu),
                equatorial_radius_m=float(body.radEquator),
                polar_radius_ratio=float(getattr(body, "radiusRatio", 1.0)),
                **visual_values,
            )
            self._celestial.append(CelestialBinding(visual, body.planetBodyInMsg))
        self._manifest_revision += 1

    def add_visual(self, visual: VisualElement) -> str:
        """Register a renderer-neutral visual element."""

        visual.visual_id = normalize_identifier(visual.visual_id)
        visual.parent_id = normalize_identifier(visual.parent_id) if visual.parent_id else ""
        if any(item.visual_id == visual.visual_id for item in self._visuals):
            raise ValueError(f"duplicate visual id: {visual.visual_id}")
        self._visuals.append(visual)
        self._manifest_revision += 1
        return visual.visual_id

    def _add_visual_kind(self, kind: str, visuals: VisualElement | Iterable[VisualElement]) -> None:
        items = [visuals] if isinstance(visuals, VisualElement) else list(visuals)
        for item in items:
            item.kind = kind
            self.add_visual(item)

    def add_camera(self, camera: CameraVisual) -> str:
        """Register one camera descriptor."""

        camera.camera_id = normalize_identifier(camera.camera_id)
        if camera.parent_id:
            camera.parent_id = normalize_identifier(camera.parent_id)
        self._cameras.append(camera)
        self._manifest_revision += 1
        return camera.camera_id

    def set_scene_settings(self, settings: SceneSettings | Mapping[str, Any]) -> None:
        """Replace renderer hints without changing dynamics."""

        self._settings = settings if isinstance(settings, SceneSettings) else SceneSettings(**settings)
        self._manifest_revision += 1

    def add_ui_setting(self, category: str, setting: Mapping[str, Any]) -> None:
        """Store renderer UI hints; unknown consumers may safely ignore them."""

        if category not in self._ui_settings:
            raise ValueError(f"unknown UI setting category: {category}")
        self._ui_settings[category].append(dict(setting))
        self._manifest_revision += 1

    def _hello_message(self) -> dict[str, Any]:
        return {
            "protocol": PROTOCOL_V2,
            "type": "hello",
            "session_id": self.session_id,
            "capabilities": [
                "scene_manifest",
                "events",
                "recording",
                "multi_camera",
                "typed_visual_channels",
                "camera_data_products",
                "bidirectional_commands",
            ],
            "required_capabilities": ["scene_manifest"],
            "coordinates": {
                "length_unit": "m",
                "handedness": "right",
                "position_frame": "L",
                "quaternion_order": "wxyz",
                "rotation_semantics": "active_parent_from_child",
            },
        }

    def _manifest_message(self) -> dict[str, Any]:
        return {
            "protocol": PROTOCOL_V2,
            "type": "scene_manifest",
            "session_id": self.session_id,
            "revision": str(self._manifest_revision),
            "objects": [
                {
                    "object_id": item.object_id,
                    "display_name": item.display_name,
                    "parent_id": item.parent_id,
                    "transform_space": "world",
                    "asset_path": item.asset_path,
                    "semantic_label": item.semantic_label,
                    "geometries": [geometry.to_payload() for geometry in item.geometries],
                }
                for item in self._objects
            ],
            "celestial_bodies": [item.visual.to_payload() for item in self._celestial],
            "visuals": [item.to_payload() for item in self._visuals],
            "cameras": [item.to_payload() for item in self._cameras],
            "settings": {
                **self._settings.to_payload(),
                "ui": {key: list(value) for key, value in self._ui_settings.items()},
            },
        }
