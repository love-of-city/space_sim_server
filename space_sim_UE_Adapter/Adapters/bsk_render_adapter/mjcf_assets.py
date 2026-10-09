"""Read renderer metadata that MJScene does not expose at runtime.

The Basilisk ``MJGeomInfo`` API intentionally exposes compiled transforms and
primitive sizes, but not the source mesh file.  This module reads only the
visual metadata from the original MJCF and never participates in dynamics.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class MjcfGeometryMetadata:
    body_name: str
    mesh_name: str = ""
    source_path: str = ""
    mesh_scale: tuple[float, float, float] = (1.0, 1.0, 1.0)
    position_body_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation_body_from_geometry_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    material_name: str = ""
    material_rgba: tuple[float, float, float, float] | None = None
    material_specular: float = 0.5
    material_shininess: float = 0.25
    material_reflectance: float = 0.0
    material_emission: float = 0.0
    material_texture: str = ""
    material_texture_source: str = ""
    material_texture_repeat: tuple[float, float] = (1.0, 1.0)
    use_asset_materials: bool = False
    role: str = "visual"


@dataclass(frozen=True)
class RenderAsset:
    asset_type: str
    asset_path: str
    component_scale: tuple[float, float, float] = (1.0, 1.0, 1.0)


@dataclass(frozen=True)
class MjcfSceneMetadata:
    headlight_enabled: bool = False
    headlight_diffuse_rgb: tuple[float, float, float] = (0.6, 0.6, 0.6)
    headlight_ambient_rgb: tuple[float, float, float] = (0.1, 0.1, 0.1)
    headlight_specular_rgb: tuple[float, float, float] = (0.0, 0.0, 0.0)
    lights: tuple[dict[str, Any], ...] = ()
    cameras: tuple[dict[str, Any], ...] = ()


def _vec3(value: str | None, default: Sequence[float] = (1.0, 1.0, 1.0)) -> tuple[float, float, float]:
    if not value:
        return (float(default[0]), float(default[1]), float(default[2]))
    parts = [float(item) for item in value.split()]
    if len(parts) == 1:
        return (parts[0], parts[0], parts[0])
    if len(parts) != 3:
        raise ValueError(f"expected one or three values, got {value!r}")
    return (parts[0], parts[1], parts[2])


def _quat(value: str | None) -> tuple[float, float, float, float]:
    if not value:
        return (1.0, 0.0, 0.0, 0.0)
    parts = tuple(float(item) for item in value.split())
    if len(parts) != 4:
        raise ValueError(f"expected four quaternion values, got {value!r}")
    return parts


def _vec2(value: str | None, default: Sequence[float] = (1.0, 1.0)) -> tuple[float, float]:
    if not value:
        return (float(default[0]), float(default[1]))
    parts = tuple(float(item) for item in value.split())
    if len(parts) != 2:
        raise ValueError(f"expected two values, got {value!r}")
    return parts


def _material_class_defaults(root: ET.Element) -> dict[str, dict[str, str]]:
    defaults: dict[str, dict[str, str]] = {}

    def walk(node: ET.Element, inherited: Mapping[str, str]) -> None:
        values = dict(inherited)
        material = node.find("material")
        if material is not None:
            values.update({key: value for key, value in material.attrib.items() if key != "class"})
        class_name = node.get("class", "")
        if class_name:
            defaults[class_name] = values
        for child in node.findall("default"):
            walk(child, values)

    for default in root.findall("./default"):
        walk(default, {})
    return defaults


def _quat_multiply(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float, float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


# MuJoCo cameras look down -Z with +Y up.  The renderer-neutral camera frame
# looks down +X with +Z up and +Y left.  This proper rotation maps the latter
# basis into the former without leaking a renderer-specific convention onto
# the wire.
_MUJOCO_FROM_RENDER_CAMERA = (0.5, -0.5, 0.5, 0.5)


def _rotate(q: Sequence[float], vector: Sequence[float]) -> tuple[float, float, float]:
    rotated = _quat_multiply(_quat_multiply(q, (0.0, *vector)), (q[0], -q[1], -q[2], -q[3]))
    return (rotated[1], rotated[2], rotated[3])


def _compiled_source_mesh_poses(mjcf_path: Path) -> list[tuple[tuple[float, float, float], tuple[float, float, float, float]]]:
    """Remove MuJoCo's automatic mesh recentering from compiled geom poses."""

    try:
        import mujoco  # type: ignore
        model_type = mujoco.MjModel
    except (ImportError, AttributeError):
        # A Windows Basilisk process may deliberately defer the Python MuJoCo
        # extension to avoid loading two incompatible MuJoCo DLL builds.  XML
        # metadata remains usable; only compiler recenter compensation is
        # unavailable in that process.
        return []
    model = model_type.from_xml_path(str(mjcf_path.resolve()))
    poses = []
    for geom_index in range(model.ngeom):
        if int(model.geom_bodyid[geom_index]) == 0:
            continue
        geom_position = tuple(float(value) for value in model.geom_pos[geom_index])
        geom_quaternion = tuple(float(value) for value in model.geom_quat[geom_index])
        mesh_index = int(model.geom_dataid[geom_index]) if int(model.geom_type[geom_index]) == 7 else -1
        if mesh_index < 0:
            poses.append((geom_position, geom_quaternion))
            continue
        mesh_position = tuple(float(value) for value in model.mesh_pos[mesh_index])
        mesh_quaternion = tuple(float(value) for value in model.mesh_quat[mesh_index])
        source_quaternion = _quat_multiply(
            geom_quaternion,
            (mesh_quaternion[0], -mesh_quaternion[1], -mesh_quaternion[2], -mesh_quaternion[3]),
        )
        rotated_mesh_position = _rotate(source_quaternion, mesh_position)
        source_position = tuple(geom_position[index] - rotated_mesh_position[index] for index in range(3))
        poses.append((source_position, source_quaternion))
    return poses


def _normalize_source(path: str | Path) -> str:
    return Path(path).resolve().as_posix().casefold()


def _annotate_mesh_sources(root: ET.Element, document: Path) -> None:
    compiler = next((item for item in root.findall("compiler")), None)
    mesh_dir = Path(compiler.get("meshdir", "")) if compiler is not None else Path()
    for mesh in root.findall("./asset/mesh"):
        source = mesh.get("file", "")
        if source:
            mesh.set("_bsk_source_path", str((document.parent / mesh_dir / source).resolve()))
    texture_dir = Path(compiler.get("texturedir", "")) if compiler is not None else Path()
    for texture in root.findall("./asset/texture"):
        source = texture.get("file", "")
        if source:
            texture.set("_bsk_source_path", str((document.parent / texture_dir / source).resolve()))


def _obj_has_resolvable_mtl(source: str) -> bool:
    path = Path(source)
    if path.suffix.casefold() != ".obj" or not path.is_file():
        return False
    try:
        with path.open("r", encoding="utf-8", errors="ignore") as stream:
            for line in stream:
                if line.casefold().startswith("mtllib "):
                    return all((path.parent / name).is_file() for name in line.split()[1:])
                if line.startswith(("v ", "vn ", "vt ", "f ")):
                    continue
    except OSError:
        return False
    return False


def _load_expanded(document: Path, stack: tuple[Path, ...] = ()) -> ET.Element:
    document = document.resolve()
    if document in stack:
        raise ValueError(f"recursive MJCF include: {document}")
    root = ET.parse(document).getroot()
    _annotate_mesh_sources(root, document)
    for parent in list(root.iter()):
        children = list(parent)
        for index, child in enumerate(children):
            if child.tag != "include" or not child.get("file"):
                continue
            included = _load_expanded(document.parent / child.get("file", ""), stack + (document,))
            replacement = list(included) if included.tag == "mujoco" else [included]
            parent.remove(child)
            for offset, node in enumerate(replacement):
                parent.insert(index + offset, node)
    return root


def parse_mjcf_geometry_metadata(mjcf_path: str | Path) -> list[MjcfGeometryMetadata]:
    """Return body geometry metadata in MuJoCo's body/geom traversal order."""

    root = _load_expanded(Path(mjcf_path))
    meshes: dict[str, tuple[str, tuple[float, float, float]]] = {}
    material_defaults = _material_class_defaults(root)
    textures: dict[str, dict[str, Any]] = {}
    materials: dict[str, dict[str, Any]] = {}
    for asset in root.findall("./asset"):
        for texture in asset.findall("texture"):
            name = texture.get("name", "")
            if name:
                textures[name] = {
                    "source": texture.get("_bsk_source_path", ""),
                    "builtin": texture.get("builtin", ""),
                }
        for material in asset.findall("material"):
            name = material.get("name", "")
            if name:
                values = dict(material_defaults.get(material.get("class", ""), {}))
                values.update({key: value for key, value in material.attrib.items() if key not in {"name", "class"}})
                rgba = tuple(float(value) for value in values.get("rgba", "0.5 0.5 0.5 1").split())
                if len(rgba) != 4:
                    raise ValueError(f"material {name!r} must contain four rgba values")
                materials[name] = {
                    "rgba": rgba,
                    "specular": float(values.get("specular", 0.5)),
                    "shininess": float(values.get("shininess", 0.25)),
                    "reflectance": float(values.get("reflectance", 0.0)),
                    "emission": float(values.get("emission", 0.0)),
                    "texture": str(values.get("texture", "")),
                    "texture_source": textures.get(str(values.get("texture", "")), {}).get("source", ""),
                    "texrepeat": _vec2(values.get("texrepeat")),
                }
        for mesh in asset.findall("mesh"):
            source = mesh.get("_bsk_source_path", "")
            name = mesh.get("name") or (Path(source).stem if source else "")
            if name:
                meshes[name] = (source, _vec3(mesh.get("scale")))

    output: list[MjcfGeometryMetadata] = []

    def walk_container(container: ET.Element, body_name: str) -> None:
        for child in list(container):
            if child.tag == "geom":
                mesh_name = child.get("mesh", "")
                source, mesh_scale = meshes.get(mesh_name, ("", (1.0, 1.0, 1.0)))
                geom_class = child.get("class", "").casefold()
                role = "collision" if "collision" in geom_class or child.get("group") == "3" else "visual"
                if mesh_name:
                    role = "visual"
                material_name = child.get("material", "")
                material = materials.get(material_name, {})
                output.append(
                    MjcfGeometryMetadata(
                        body_name=body_name,
                        mesh_name=mesh_name,
                        source_path=source,
                        mesh_scale=mesh_scale,
                        position_body_m=_vec3(child.get("pos"), (0.0, 0.0, 0.0)),
                        orientation_body_from_geometry_wxyz=_quat(child.get("quat")),
                        material_name=material_name,
                        material_rgba=material.get("rgba"),
                        material_specular=float(material.get("specular", 0.5)),
                        material_shininess=float(material.get("shininess", 0.25)),
                        material_reflectance=float(material.get("reflectance", 0.0)),
                        material_emission=float(material.get("emission", 0.0)),
                        material_texture=str(material.get("texture", "")),
                        material_texture_source=str(material.get("texture_source", "")),
                        material_texture_repeat=material.get("texrepeat", (1.0, 1.0)),
                        use_asset_materials=not material_name and _obj_has_resolvable_mtl(source),
                        role=role,
                    )
                )
            elif child.tag == "body":
                walk_container(child, child.get("name", ""))
            elif child.tag == "frame":
                # MjSpec attachment commonly inserts a named frame between a
                # parent body and a vendored articulated model.  A frame is not
                # a dynamic body: direct geoms still belong to body_name, while
                # nested bodies establish their own names.
                walk_container(child, body_name)

    for worldbody in root.findall("./worldbody"):
        for body in worldbody.findall("body"):
            walk_container(body, body.get("name", ""))
    compiled_poses = _compiled_source_mesh_poses(Path(mjcf_path))
    if len(compiled_poses) == len(output):
        output = [
            replace(item, position_body_m=compiled_poses[index][0], orientation_body_from_geometry_wxyz=compiled_poses[index][1])
            if item.mesh_name else item
            for index, item in enumerate(output)
        ]
    return output


def parse_mjcf_body_parents(mjcf_path: str | Path) -> dict[str, str]:
    """Return named body parents while treating MJCF frames as transparent."""

    root = _load_expanded(Path(mjcf_path))
    parents: dict[str, str] = {}

    def walk(container: ET.Element, parent_body: str) -> None:
        for child in list(container):
            if child.tag == "body":
                body_name = child.get("name", "")
                if body_name:
                    parents[body_name] = parent_body or "world"
                    walk(child, body_name)
                else:
                    walk(child, parent_body)
            elif child.tag == "frame":
                walk(child, parent_body)

    for worldbody in root.findall("./worldbody"):
        walk(worldbody, "world")
    return parents


def parse_mjcf_scene_metadata(mjcf_path: str | Path, namespace: str = "") -> MjcfSceneMetadata:
    """Read renderer-only MJCF headlight, light, and camera declarations."""

    root = _load_expanded(Path(mjcf_path))
    visual = root.find("./visual")
    headlight = visual.find("headlight") if visual is not None else None
    headlight_enabled = headlight is not None and headlight.get("active", "1").casefold() not in {"0", "false"}
    diffuse = _vec3(headlight.get("diffuse") if headlight is not None else None, (0.6, 0.6, 0.6))
    ambient = _vec3(headlight.get("ambient") if headlight is not None else None, (0.1, 0.1, 0.1))
    specular = _vec3(headlight.get("specular") if headlight is not None else None, (0.0, 0.0, 0.0))
    prefix = namespace.strip().replace("\\", "/").strip("/")
    lights: list[dict[str, Any]] = []
    cameras: list[dict[str, Any]] = []

    def add_light(node: ET.Element, parent_name: str = "") -> None:
        index = len(lights)
        name = node.get("name", f"light_{index}")
        target = node.get("target", "")
        directional = node.get("directional", "false").casefold() in {"1", "true"}
        diffuse_rgb = _vec3(node.get("diffuse"), (0.7, 0.7, 0.7))
        lights.append({
            "visual_id": f"{prefix}/light/{name}" if prefix else f"light/{name}",
            "parent_id": f"{prefix}/{parent_name}" if prefix and parent_name else parent_name,
            "position_body_m": _vec3(node.get("pos"), (0.0, 0.0, 0.0)),
            "normal_body": _vec3(node.get("dir"), (0.0, 0.0, -1.0)),
            "color_rgba": (*diffuse_rgb, 1.0),
            "properties": {
                "light_type": "directional" if directional else "spot",
                "diffuse_rgb": diffuse_rgb,
                "specular_rgb": _vec3(node.get("specular"), (0.3, 0.3, 0.3)),
                "intensity": max(diffuse_rgb),
                "cast_shadows": node.get("castshadow", "true").casefold() not in {"0", "false"},
                "cutoff_deg": float(node.get("cutoff", 45.0)),
                "target_id": f"{prefix}/{target}" if prefix and target else target,
            },
        })

    def add_camera(node: ET.Element, parent_name: str) -> None:
        name = node.get("name", f"camera_{len(cameras)}")
        resolution = tuple(int(value) for value in node.get("resolution", "1920 1080").split())
        if len(resolution) != 2 or any(value <= 0 for value in resolution):
            resolution = (1920, 1080)
        field_of_view = node.get("fovy")
        if field_of_view is not None:
            # MJCF fovy is vertical (degrees), even with compiler angle=radian.
            # CameraVisual and UE use horizontal FOV. Preserve the XML framing.
            vertical_fov_rad = math.radians(float(field_of_view))
            aspect_ratio = resolution[0] / resolution[1]
            field_of_view_rad = 2.0 * math.atan(math.tan(vertical_fov_rad / 2.0) * aspect_ratio)
        else:
            sensor = _vec2(node.get("sensorsize"), (0.00576, 0.00324))
            focal = _vec2(node.get("focal"), (0.0036, 0.0036))
            # UE and the wire protocol use horizontal FOV.
            field_of_view_rad = 2.0 * math.atan2(sensor[0], 2.0 * focal[0])
        source_quat = _quat(node.get("quat"))
        cameras.append({
            "camera_id": f"{prefix}/camera/{name}" if prefix else f"camera/{name}",
            "display_name": name,
            "parent_id": f"{prefix}/{parent_name}" if prefix and parent_name else parent_name,
            "position_body_m": _vec3(node.get("pos"), (0.0, 0.0, 0.0)),
            "orientation_body_from_camera_wxyz": _quat_multiply(
                source_quat, _MUJOCO_FROM_RENDER_CAMERA
            ),
            "field_of_view_rad": field_of_view_rad,
            "resolution": resolution,
        })

    def walk_container(container: ET.Element, parent_name: str) -> None:
        for light in container.findall("light"):
            add_light(light, parent_name)
        for camera in container.findall("camera"):
            add_camera(camera, parent_name)
        for child in list(container):
            if child.tag == "body":
                walk_container(child, child.get("name", "") or parent_name)
            elif child.tag == "frame":
                # Frames are renderer-transparent just as they are in the
                # MJScene body hierarchy adapter.
                walk_container(child, parent_name)

    for worldbody in root.findall("./worldbody"):
        walk_container(worldbody, "")
    return MjcfSceneMetadata(
        headlight_enabled, diffuse, ambient, specular, tuple(lights), tuple(cameras)
    )


def load_asset_catalog(catalog: str | Path | Mapping[str, Any] | None) -> dict[str, RenderAsset]:
    """Load a renderer-specific source-file to packaged-asset mapping."""

    if catalog is None:
        return {}
    payload: Mapping[str, Any]
    if isinstance(catalog, Mapping):
        payload = catalog
    else:
        payload = json.loads(Path(catalog).read_text(encoding="utf-8"))
    entries = payload.get("assets", payload)
    result: dict[str, RenderAsset] = {}
    for source, raw in entries.items():
        if isinstance(raw, str):
            raw = {"asset_path": raw}
        scale = _vec3(" ".join(str(value) for value in raw.get("component_scale", (1.0, 1.0, 1.0))))
        result[_normalize_source(source)] = RenderAsset(
            asset_type=str(raw.get("asset_type", "static_mesh")),
            asset_path=str(raw.get("asset_path", "")),
            component_scale=scale,
        )
    return result


def load_texture_catalog(catalog: str | Path | Mapping[str, Any] | None) -> dict[str, str]:
    if catalog is None:
        return {}
    payload = catalog if isinstance(catalog, Mapping) else json.loads(Path(catalog).read_text(encoding="utf-8"))
    return {_normalize_source(source): str(asset_path) for source, asset_path in payload.get("textures", {}).items()}


def resolve_asset(catalog: Mapping[str, RenderAsset], source_path: str) -> RenderAsset | None:
    if not source_path:
        return None
    return catalog.get(_normalize_source(source_path))


def resolve_texture(catalog: Mapping[str, str], source_path: str) -> str:
    if not source_path:
        return ""
    return catalog.get(_normalize_source(source_path), "")


__all__ = [
    "MjcfGeometryMetadata",
    "MjcfSceneMetadata",
    "RenderAsset",
    "load_asset_catalog",
    "load_texture_catalog",
    "parse_mjcf_geometry_metadata",
    "parse_mjcf_scene_metadata",
    "resolve_asset",
    "resolve_texture",
]
