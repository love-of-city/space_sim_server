"""Deterministic ASCII/binary STL ingestion for Unreal asset preparation.

STL remains a source format.  Packaged Unreal runtimes load cooked StaticMesh
assets, so the editor-side preparation pipeline converts STL triangles to an
OBJ staging file before invoking Unreal's normal asset importer.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import struct
from typing import Iterable


_CONVERTER_VERSION = 4
# CAD meshes expressed in metres can legitimately contain sub-micrometre
# triangles.  Use a near-zero squared-area threshold rather than classifying
# small but valid detail as degenerate.
_VECTOR_EPSILON = 1.0e-40

Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class StlTriangle:
    normal: Vector3
    vertices: tuple[Vector3, Vector3, Vector3]


@dataclass(frozen=True)
class StlConversionResult:
    source_path: str
    output_path: str
    source_format: str
    source_sha256: str
    triangle_count: int
    unique_vertex_count: int
    degenerate_triangle_count: int
    normal_mode: str
    smoothing_angle_degrees: float
    cached: bool = False


def _is_finite_vector(value: Iterable[float]) -> bool:
    return all(math.isfinite(component) for component in value)


def _subtract(left: Vector3, right: Vector3) -> Vector3:
    return tuple(left[index] - right[index] for index in range(3))  # type: ignore[return-value]


def _cross(left: Vector3, right: Vector3) -> Vector3:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _dot(left: Vector3, right: Vector3) -> float:
    return sum(left[index] * right[index] for index in range(3))


def _length_squared(value: Vector3) -> float:
    return _dot(value, value)


def _normalize(value: Vector3, fallback: Vector3 = (0.0, 0.0, 1.0)) -> Vector3:
    length_squared = _length_squared(value)
    if length_squared <= _VECTOR_EPSILON:
        return fallback
    inverse_length = 1.0 / math.sqrt(length_squared)
    return tuple(component * inverse_length for component in value)  # type: ignore[return-value]


def _geometric_normal(vertices: tuple[Vector3, Vector3, Vector3]) -> tuple[Vector3, Vector3, bool]:
    area_normal = _cross(_subtract(vertices[1], vertices[0]), _subtract(vertices[2], vertices[0]))
    degenerate = _length_squared(area_normal) <= _VECTOR_EPSILON
    return _normalize(area_normal), area_normal, degenerate


def _validate_triangle(triangle: StlTriangle, index: int) -> None:
    if not _is_finite_vector(triangle.normal):
        raise ValueError(f"STL triangle {index} contains a non-finite normal")
    for vertex_index, vertex in enumerate(triangle.vertices):
        if not _is_finite_vector(vertex):
            raise ValueError(f"STL triangle {index} vertex {vertex_index} contains a non-finite value")


def _binary_layout(data: bytes) -> tuple[bool, int]:
    if len(data) < 84:
        return False, 0
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    expected_size = 84 + triangle_count * 50
    return expected_size == len(data), triangle_count


def _parse_binary(data: bytes, triangle_count: int) -> list[StlTriangle]:
    triangles: list[StlTriangle] = []
    offset = 84
    for index in range(triangle_count):
        values = struct.unpack_from("<12fH", data, offset)
        triangle = StlTriangle(
            normal=(float(values[0]), float(values[1]), float(values[2])),
            vertices=(
                (float(values[3]), float(values[4]), float(values[5])),
                (float(values[6]), float(values[7]), float(values[8])),
                (float(values[9]), float(values[10]), float(values[11])),
            ),
        )
        _validate_triangle(triangle, index)
        triangles.append(triangle)
        offset += 50
    return triangles


def _parse_ascii(data: bytes) -> list[StlTriangle]:
    try:
        text = data.decode("ascii")
    except UnicodeDecodeError as error:
        raise ValueError("STL is neither a valid binary STL nor ASCII text") from error

    triangles: list[StlTriangle] = []
    current_normal: Vector3 | None = None
    current_vertices: list[Vector3] = []
    for line_number, raw_line in enumerate(text.splitlines(), start=1):
        fields = raw_line.strip().split()
        if not fields:
            continue
        keyword = fields[0].casefold()
        if keyword == "facet":
            if len(fields) != 5 or fields[1].casefold() != "normal":
                raise ValueError(f"invalid ASCII STL facet at line {line_number}")
            if current_normal is not None:
                raise ValueError(f"nested ASCII STL facet at line {line_number}")
            current_normal = tuple(float(value) for value in fields[2:5])  # type: ignore[assignment]
            current_vertices = []
        elif keyword == "vertex":
            if current_normal is None or len(fields) != 4:
                raise ValueError(f"invalid ASCII STL vertex at line {line_number}")
            current_vertices.append(tuple(float(value) for value in fields[1:4]))  # type: ignore[arg-type]
        elif keyword == "endfacet":
            if current_normal is None or len(current_vertices) != 3:
                raise ValueError(f"ASCII STL facet at line {line_number} does not have exactly three vertices")
            triangle = StlTriangle(current_normal, tuple(current_vertices))  # type: ignore[arg-type]
            _validate_triangle(triangle, len(triangles))
            triangles.append(triangle)
            current_normal = None
            current_vertices = []
    if current_normal is not None:
        raise ValueError("unterminated ASCII STL facet")
    if not triangles:
        raise ValueError("ASCII STL contains no triangles")
    return triangles


def read_stl(path: str | Path) -> tuple[str, list[StlTriangle]]:
    """Read a strict binary or ASCII STL without silently dropping triangles."""

    source = Path(path)
    data = source.read_bytes()
    is_binary, triangle_count = _binary_layout(data)
    if is_binary:
        return "binary", _parse_binary(data, triangle_count)
    return "ascii", _parse_ascii(data)


def _vertex_normals(
    triangles: list[StlTriangle],
    normal_mode: str,
    smoothing_angle_degrees: float,
) -> tuple[list[Vector3], list[Vector3], list[tuple[int, int, int]], int]:
    vertices: list[Vector3] = []
    vertex_indices: dict[Vector3, int] = {}
    faces: list[tuple[int, int, int]] = []
    face_normals: list[Vector3] = []
    area_normals: list[Vector3] = []
    incident_faces: dict[int, list[int]] = {}
    degenerate_count = 0

    for face_index, triangle in enumerate(triangles):
        indices: list[int] = []
        for vertex in triangle.vertices:
            index = vertex_indices.get(vertex)
            if index is None:
                index = len(vertices)
                vertex_indices[vertex] = index
                vertices.append(vertex)
            indices.append(index)
            incident_faces.setdefault(index, []).append(face_index)
        faces.append(tuple(indices))  # type: ignore[arg-type]
        geometric_normal, area_normal, degenerate = _geometric_normal(triangle.vertices)
        if degenerate:
            degenerate_count += 1
            geometric_normal = _normalize(triangle.normal)
            area_normal = geometric_normal
        source_normal = _normalize(triangle.normal, geometric_normal)
        face_normals.append(source_normal if normal_mode == "preserve" else geometric_normal)
        area_normals.append(area_normal)

    if normal_mode in {"preserve", "recompute"}:
        corner_normals = [normal for normal in face_normals for _ in range(3)]
        return vertices, corner_normals, faces, degenerate_count

    threshold = math.cos(math.radians(smoothing_angle_degrees))
    corner_normals: list[Vector3] = []
    for face_index, face in enumerate(faces):
        reference = face_normals[face_index]
        for vertex_index in face:
            candidates = [
                area_normals[candidate]
                for candidate in incident_faces[vertex_index]
                if _dot(reference, face_normals[candidate]) >= threshold
            ]
            accumulated: Vector3 = (
                sum(value[0] for value in candidates),
                sum(value[1] for value in candidates),
                sum(value[2] for value in candidates),
            )
            corner_normals.append(_normalize(accumulated, reference))
    return vertices, corner_normals, faces, degenerate_count


def convert_stl_to_obj(
    source_path: str | Path,
    output_path: str | Path,
    *,
    normal_mode: str = "auto",
    smoothing_angle_degrees: float = 60.0,
) -> StlConversionResult:
    """Convert all STL triangles to an Unreal-importable OBJ staging mesh."""

    mode = normal_mode.strip().casefold()
    if mode not in {"auto", "preserve", "recompute"}:
        raise ValueError(f"invalid STL normal mode: {normal_mode}")
    if not 0.0 <= smoothing_angle_degrees <= 180.0:
        raise ValueError("smoothing_angle_degrees must be between 0 and 180")

    source = Path(source_path).resolve()
    output = Path(output_path).resolve()
    source_format, triangles = read_stl(source)
    vertices, corner_normals, faces, degenerate_count = _vertex_normals(
        triangles, mode, smoothing_angle_degrees
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    with temporary.open("w", encoding="ascii", newline="\n") as stream:
        stream.write(f"# Converted from {source.name}; triangles={len(faces)}\n")
        stream.write("o bsk_stl_mesh\n")
        for vertex in vertices:
            stream.write(f"v {vertex[0]:.17g} {vertex[1]:.17g} {vertex[2]:.17g}\n")
        for normal in corner_normals:
            stream.write(f"vn {normal[0]:.17g} {normal[1]:.17g} {normal[2]:.17g}\n")
        # STL has no UV channel, while UE's MikkTSpace tangent builder expects
        # one.  A deterministic per-triangle basis prevents zero tangents; the
        # runtime MJCF material supplies colour and does not sample this UV.
        for _ in faces:
            stream.write("vt 0 0\nvt 1 0\nvt 0 1\n")
        for face_index, face in enumerate(faces):
            normal_base = face_index * 3 + 1
            stream.write(
                "f "
                + " ".join(
                    f"{vertex_index + 1}/{normal_base + corner_index}/{normal_base + corner_index}"
                    for corner_index, vertex_index in enumerate(face)
                )
                + "\n"
            )
    temporary.replace(output)

    return StlConversionResult(
        source_path=str(source),
        output_path=str(output),
        source_format=source_format,
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        triangle_count=len(faces),
        unique_vertex_count=len(vertices),
        degenerate_triangle_count=degenerate_count,
        normal_mode=mode,
        smoothing_angle_degrees=float(smoothing_angle_degrees),
    )


def prepare_stl_for_import(
    source_path: str | Path,
    cache_directory: str | Path,
    *,
    normal_mode: str = "auto",
    smoothing_angle_degrees: float = 60.0,
) -> StlConversionResult:
    """Return a cached OBJ staging file for an STL source."""

    source = Path(source_path).resolve()
    cache = Path(cache_directory).resolve()
    output = cache / f"{source.stem}.obj"
    metadata_path = output.with_suffix(".stl-cache.json")
    source_sha256 = hashlib.sha256(source.read_bytes()).hexdigest()
    signature = {
        "converter_version": _CONVERTER_VERSION,
        "source_path": str(source),
        "source_sha256": source_sha256,
        "normal_mode": normal_mode.strip().casefold(),
        "smoothing_angle_degrees": float(smoothing_angle_degrees),
    }
    if output.is_file() and metadata_path.is_file():
        try:
            cached = json.loads(metadata_path.read_text(encoding="utf-8"))
            if cached.get("signature") == signature:
                cached_result = dict(cached["result"])
                cached_result["cached"] = True
                return StlConversionResult(**cached_result)
        except (OSError, ValueError, TypeError, KeyError):
            pass

    result = convert_stl_to_obj(
        source,
        output,
        normal_mode=normal_mode,
        smoothing_angle_degrees=smoothing_angle_degrees,
    )
    metadata_path.write_text(
        json.dumps({"signature": signature, "result": asdict(result)}, indent=2),
        encoding="utf-8",
    )
    return result


__all__ = [
    "StlConversionResult",
    "StlTriangle",
    "convert_stl_to_obj",
    "prepare_stl_for_import",
    "read_stl",
]
