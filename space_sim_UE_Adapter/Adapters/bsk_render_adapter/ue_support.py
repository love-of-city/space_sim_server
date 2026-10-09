"""Vizard-shaped convenience API for the BSK Unreal renderer.

Only visualization registration is replaced.  Basilisk and MJScene remain the
sole dynamics authority.  Parameter names intentionally mirror
``vizSupport.enableUnityVisualization`` to keep scenario migrations small.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np

from .bridge import BasiliskRenderBridge
from .descriptors import SceneSettings, VisualElement


def _as_list(value: Any) -> list[Any]:
    if value is None:
        return []
    return list(value) if isinstance(value, (list, tuple)) else [value]


def _per_spacecraft(value: Any, count: int, name: str) -> list[Any]:
    if value is None:
        return [None] * count
    if count == 1:
        if isinstance(value, (list, tuple)) and len(value) == 1 and (
            value[0] is None or isinstance(value[0], (list, tuple, dict))
        ):
            return [value[0]]
        return [value]
    if not isinstance(value, (list, tuple)) or len(value) != count:
        raise ValueError(f"{name} must contain one entry per spacecraft ({count})")
    return list(value)


def _per_scene_option(value: Any, count: int, name: str) -> list[Any]:
    if value is None:
        return [None] * count
    if not isinstance(value, (list, tuple)) or len(value) != count:
        raise ValueError(f"{name} must contain one entry per scene ({count})")
    return list(value)


def _color_rgba255(values: Any, default: Sequence[float]) -> list[float]:
    raw = list(values or [])
    if len(raw) < 3:
        return [float(item) for item in default]
    if len(raw) < 4:
        raw.append(255)
    return [max(0, min(255, int(item))) / 255.0 for item in raw[:4]]


def _body_targets(value: Any, body_ids: Mapping[str, str]) -> Iterable[tuple[str, list[Any]]]:
    if value is None:
        return []
    if isinstance(value, Mapping):
        result: list[tuple[str, list[Any]]] = []
        for body_name, entries in value.items():
            if body_name not in body_ids:
                raise ValueError(f"unknown body {body_name!r}; available bodies: {sorted(body_ids)}")
            result.append((body_ids[body_name], _as_list(entries)))
        return result
    return [(next(iter(body_ids.values())), _as_list(value))]


def _visual_from_vizard(kind: str, source: Any, parent_id: str, index: int) -> VisualElement:
    if kind == "generic_sensor":
        position = getattr(source, "r_SB_B", [0.0, 0.0, 0.0])
        field_of_view = list(getattr(source, "fieldOfView", []))
        size = float(getattr(source, "size", 0.0) or 0.15)
        state_attribute = "genericSensorCmd"
        default_color = (1.0, 1.0, 0.0, 0.45)
        range_m = max(2.5, size)
    elif kind == "transceiver":
        position = getattr(source, "r_SB_B", [0.0, 0.0, 0.0])
        field_of_view = [float(getattr(source, "fieldOfView", np.pi / 6.0))]
        size = 0.15
        state_attribute = "transceiverState"
        default_color = (0.0, 1.0, 1.0, 0.45)
        range_m = 4.0
    elif kind == "light":
        position = getattr(source, "position", [0.0, 0.0, 0.0])
        field_of_view = [float(getattr(source, "fieldOfView", np.pi / 4.0))]
        size = float(getattr(source, "markerDiameter", 0.05) or 0.05)
        state_attribute = "lightOn"
        default_color = (1.0, 1.0, 0.85, 0.7)
        range_m = float(getattr(source, "range", 2.0) or 2.0)
    else:
        position = [0.0, 0.0, 0.0]
        field_of_view = []
        size = 0.15
        state_attribute = "currentValue"
        default_color = (0.3, 0.9, 0.3, 1.0)
        range_m = 0.15

    def state_provider(device=source, attribute=state_attribute, visual_kind=kind):
        value = float(getattr(device, attribute, 0.0))
        enabled = value > 0.0 if visual_kind != "storage" else True
        return {
            "visible": enabled if visual_kind in {"light", "generic_sensor"} else True,
            "value": value,
            "channels": {"value": value, "enabled": enabled},
        }

    return VisualElement(
        visual_id=f"{parent_id}/{kind}/{index}",
        kind=kind,
        parent_id=parent_id,
        position_body_m=np.asarray(position, dtype=float).reshape(3).tolist(),
        normal_body=np.asarray(getattr(source, "normalVector", [1.0, 0.0, 0.0]), dtype=float).reshape(3).tolist(),
        field_of_view_rad=field_of_view,
        size_m=size,
        range_m=range_m,
        color_rgba=_color_rgba255(getattr(source, "color", []), default_color),
        label=str(getattr(source, "label", "") or f"{kind}_{index}"),
        properties={"source": "vizSupport-compatible", "device_index": index},
        channel_schema={
            "value": {"type": "number"},
            "enabled": {"type": "boolean"},
        },
        state_provider=state_provider,
    )


class _UnrealVizSettings:
    """Small compatibility facade for common ``viz.settings`` assignments."""

    def __init__(self, bridge: BasiliskRenderBridge) -> None:
        self._bridge = bridge
        self._orbit_lines_on = 1

    @property
    def orbitLinesOn(self) -> int:
        return self._orbit_lines_on

    @orbitLinesOn.setter
    def orbitLinesOn(self, value: int) -> None:
        self._orbit_lines_on = int(value)
        self._bridge._settings.orbit_lines = self._orbit_lines_on > 0
        self._bridge._manifest_revision += 1


def enableUnrealVisualization(
    scSim: Any,
    simTaskName: str,
    scList: Any,
    saveFile: str | None = None,
    rwEffectorList: Any = None,
    thrEffectorList: Any = None,
    thrColors: Any = None,
    cssList: Any = None,
    genericSensorList: Any = None,
    ellipsoidList: Any = None,
    lightList: Any = None,
    genericStorageList: Any = None,
    transceiverList: Any = None,
    spriteList: Any = None,
    modelDictionaryKeyList: Any = None,
    logoTextureList: Any = None,
    oscOrbitColorList: Any = None,
    trueOrbitColorList: Any = None,
    groundTrackColorList: Any = None,
    msmInfoList: Any = None,
    trueOrbitColorInMsgList: Any = None,
    groundTrackBodyNameList: Any = None,
    liveStream: bool = True,
    broadcastStream: bool = False,
    noDisplay: bool = False,
    *,
    host: str = "127.0.0.1",
    port: int = 5558,
    publisher: Any = None,
    recordingPath: str | Path | None = None,
    originObject: str | None = None,
    namespaceList: Sequence[str] | None = None,
    mjcfPathList: Sequence[str | Path | None] | None = None,
    meshAssetCatalogList: Sequence[str | Path | Mapping[str, Any] | None] | None = None,
) -> BasiliskRenderBridge:
    """Create and schedule the renderer bridge using Vizard-style arguments."""

    del ellipsoidList, spriteList, logoTextureList, oscOrbitColorList
    del trueOrbitColorList, groundTrackColorList, msmInfoList
    del trueOrbitColorInMsgList, groundTrackBodyNameList, broadcastStream, noDisplay

    spacecraft = _as_list(scList)
    if not spacecraft:
        raise ValueError("scList must contain at least one spacecraft or MJScene")
    if namespaceList is not None and len(namespaceList) != len(spacecraft):
        raise ValueError("namespaceList must match scList length")
    if recordingPath is None and saveFile:
        recordingPath = Path(saveFile).with_suffix(".bskrec")

    bridge = BasiliskRenderBridge(
        host=host,
        port=port,
        origin_object=originObject,
        publisher=publisher,
        recording_path=recordingPath,
    )
    body_maps: list[dict[str, str]] = []
    mjcf_paths = _per_scene_option(mjcfPathList, len(spacecraft), "mjcfPathList")
    mesh_catalogs = _per_scene_option(meshAssetCatalogList, len(spacecraft), "meshAssetCatalogList")
    previous_root = ""
    for index, item in enumerate(spacecraft):
        namespace = namespaceList[index] if namespaceList is not None else str(getattr(item, "ModelTag", f"spacecraft_{index}"))
        if hasattr(item, "getBodyNames") and hasattr(item, "getGeomInfos"):
            body_map = bridge.add_mj_scene(
                item,
                namespace=namespace,
                source_path=mjcf_paths[index],
                mesh_asset_catalog=mesh_catalogs[index],
            )
        elif isinstance(item, tuple) and len(item) == 2:
            object_id = bridge.add_object(
                str(item[0]),
                item[1],
                display_name=str(item[0]),
                parent_id=previous_root,
            )
            body_map = {str(item[0]): object_id}
        else:
            asset = ""
            if modelDictionaryKeyList is not None:
                models = _per_spacecraft(modelDictionaryKeyList, len(spacecraft), "modelDictionaryKeyList")
                asset = str(models[index] or "")
            object_id = bridge.add_spacecraft(item, object_id=namespace, asset_path=asset)
            body_map = {str(getattr(item, "ModelTag", namespace)): object_id}
        body_maps.append(body_map)
        previous_root = next(iter(body_map.values()))

    if bridge.origin_object is None:
        bridge.origin_object = next(iter(body_maps[0].values()))
        bridge._settings.origin_object_id = bridge.origin_object
        bridge._settings.default_camera_target = bridge.origin_object

    rw_by_spacecraft = _per_spacecraft(rwEffectorList, len(spacecraft), "rwEffectorList")
    thr_by_spacecraft = _per_spacecraft(thrEffectorList, len(spacecraft), "thrEffectorList")
    colors_by_spacecraft = _per_spacecraft(thrColors, len(spacecraft), "thrColors")
    css_by_spacecraft = _per_spacecraft(cssList, len(spacecraft), "cssList")
    generic_by_spacecraft = _per_spacecraft(genericSensorList, len(spacecraft), "genericSensorList")
    transceiver_by_spacecraft = _per_spacecraft(transceiverList, len(spacecraft), "transceiverList")
    lights_by_spacecraft = _per_spacecraft(lightList, len(spacecraft), "lightList")
    storage_by_spacecraft = _per_spacecraft(genericStorageList, len(spacecraft), "genericStorageList")

    generic_index = 0
    for index, body_map in enumerate(body_maps):
        root_id = next(iter(body_map.values()))
        if rw_by_spacecraft[index] is not None:
            bridge.add_reaction_wheels(rw_by_spacecraft[index], parent_id=root_id)
        if thr_by_spacecraft[index] is not None:
            bridge.add_thrusters(
                thr_by_spacecraft[index],
                parent_id=root_id,
                colors=colors_by_spacecraft[index],
            )
        for parent_id, devices in _body_targets(css_by_spacecraft[index], body_map):
            bridge.add_css(devices, parent_id=parent_id)
        for kind, per_spacecraft in (
            ("generic_sensor", generic_by_spacecraft),
            ("transceiver", transceiver_by_spacecraft),
            ("light", lights_by_spacecraft),
            ("storage", storage_by_spacecraft),
        ):
            for parent_id, devices in _body_targets(per_spacecraft[index], body_map):
                for device in devices:
                    bridge.add_visual(_visual_from_vizard(kind, device, parent_id, generic_index))
                    generic_index += 1

    gravity_bodies: list[Any] = []
    seen_gravity: set[int] = set()
    for item in spacecraft:
        candidates = getattr(item, "_vizGravBodies", None)
        if candidates is None:
            candidates = getattr(getattr(item, "gravField", None), "gravBodies", [])
        if isinstance(candidates, Mapping):
            candidates = candidates.values()
        for body in list(candidates or []):
            if id(body) not in seen_gravity:
                seen_gravity.add(id(body))
                gravity_bodies.append(body)
    if gravity_bodies:
        bridge.add_celestial_bodies(gravity_bodies)

    bridge.settings = _UnrealVizSettings(bridge)
    bridge.reqComAddress = host
    bridge.liveStream = bool(liveStream)
    scSim.AddModelToTask(simTaskName, bridge, -100)
    return bridge


def enable_unreal_visualization(*args: Any, **kwargs: Any) -> BasiliskRenderBridge:
    """PEP-8 alias for :func:`enableUnrealVisualization`."""

    return enableUnrealVisualization(*args, **kwargs)


def setInstrumentGuiSetting(viz: BasiliskRenderBridge, spacecraftName: str | None = None, **kwargs: Any) -> None:
    viz.add_ui_setting("instruments", {"spacecraft_name": spacecraftName or "", **kwargs})


def setActuatorGuiSetting(viz: BasiliskRenderBridge, spacecraftName: str | None = None, **kwargs: Any) -> None:
    viz.add_ui_setting("actuators", {"spacecraft_name": spacecraftName or "", **kwargs})
