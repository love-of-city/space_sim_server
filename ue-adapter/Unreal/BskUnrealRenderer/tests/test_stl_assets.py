import json
import struct
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from bsk_render_adapter.stl_conversion import convert_stl_to_obj, prepare_stl_for_import, read_stl
from bsk_render_adapter.mjcf_assets import parse_mjcf_body_parents


TRIANGLES = [
    ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    ((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
    ((1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, 1.0, 0.0)),
]


def write_ascii_stl(path: Path) -> None:
    lines = ["solid tetrahedron"]
    for vertices in TRIANGLES:
        lines.extend(["  facet normal 0 0 1", "    outer loop"])
        lines.extend(f"      vertex {x} {y} {z}" for x, y, z in vertices)
        lines.extend(["    endloop", "  endfacet"])
    lines.append("endsolid tetrahedron")
    path.write_text("\n".join(lines), encoding="ascii")


def write_binary_stl(path: Path) -> None:
    # Binary STL headers are arbitrary and may legally begin with "solid".
    output = bytearray(b"solid BSK binary STL".ljust(80, b"\0"))
    output.extend(struct.pack("<I", len(TRIANGLES)))
    for vertices in TRIANGLES:
        values = (0.0, 0.0, 1.0, *(value for vertex in vertices for value in vertex), 0)
        output.extend(struct.pack("<12fH", *values))
    path.write_bytes(output)


class StlAssetTests(unittest.TestCase):
    def test_reads_ascii_and_binary_without_losing_triangles(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            ascii_path = root / "ascii.stl"
            binary_path = root / "binary.stl"
            write_ascii_stl(ascii_path)
            write_binary_stl(binary_path)

            ascii_format, ascii_triangles = read_stl(ascii_path)
            binary_format, binary_triangles = read_stl(binary_path)
            self.assertEqual(ascii_format, "ascii")
            self.assertEqual(binary_format, "binary")
            self.assertEqual(len(ascii_triangles), len(TRIANGLES))
            self.assertEqual(len(binary_triangles), len(TRIANGLES))
            self.assertEqual(ascii_triangles[3].vertices, binary_triangles[3].vertices)

    def test_conversion_preserves_faces_bounds_and_writes_corner_normals(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "part.stl"
            output = root / "part.obj"
            write_binary_stl(source)

            result = convert_stl_to_obj(source, output, normal_mode="auto", smoothing_angle_degrees=60.0)
            lines = output.read_text(encoding="ascii").splitlines()
            vertices = [tuple(float(value) for value in line.split()[1:]) for line in lines if line.startswith("v ")]
            self.assertEqual(result.triangle_count, 4)
            self.assertEqual(result.unique_vertex_count, 4)
            self.assertEqual(sum(line.startswith("f ") for line in lines), 4)
            self.assertEqual(sum(line.startswith("vn ") for line in lines), 12)
            self.assertEqual(sum(line.startswith("vt ") for line in lines), 12)
            self.assertEqual(tuple(min(vertex[index] for vertex in vertices) for index in range(3)), (0.0, 0.0, 0.0))
            self.assertEqual(tuple(max(vertex[index] for vertex in vertices) for index in range(3)), (1.0, 1.0, 1.0))

    def test_cache_is_content_and_normal_mode_sensitive(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "part.stl"
            write_ascii_stl(source)
            first = prepare_stl_for_import(source, root / "cache", normal_mode="preserve")
            second = prepare_stl_for_import(source, root / "cache", normal_mode="preserve")
            third = prepare_stl_for_import(source, root / "cache", normal_mode="auto")
            self.assertFalse(first.cached)
            self.assertTrue(second.cached)
            self.assertFalse(third.cached)

    def test_catalog_maps_original_stl_to_converted_import_source(self):
        project = Path(__file__).resolve().parents[1]
        generator = project / "scripts" / "generate_mjcf_asset_catalog.py"
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "part.stl"
            mjcf = root / "model.xml"
            catalog_path = root / "catalog.json"
            settings_path = root / "import.json"
            write_binary_stl(source)
            mjcf.write_text(
                '<mujoco><asset><mesh name="part" file="part.stl"/></asset>'
                '<worldbody><body name="parent"><frame name="mount">'
                '<body name="body"><geom type="mesh" mesh="part"/></body>'
                '</frame></body></worldbody></mujoco>',
                encoding="utf-8",
            )
            subprocess.run(
                [
                    sys.executable,
                    str(generator),
                    "--mjcf",
                    str(mjcf),
                    "--destination",
                    "/Game/BSK/Test/Stl",
                    "--catalog",
                    str(catalog_path),
                    "--import-settings",
                    str(settings_path),
                    "--normal-mode",
                    "auto",
                ],
                check=True,
            )
            catalog = json.loads(catalog_path.read_text(encoding="utf-8"))
            self.assertEqual(parse_mjcf_body_parents(mjcf), {"parent": "world", "body": "parent"})
            entry = catalog["assets"][str(source.resolve())]
            import_settings = json.loads(settings_path.read_text(encoding="utf-8"))
            self.assertEqual(entry["source_format"], "stl")
            self.assertEqual(entry["stl"]["triangle_count"], 4)
            self.assertEqual(entry["build_scale"], [100.0, 100.0, 100.0])
            self.assertTrue(Path(entry["import_source"]).is_file())
            self.assertTrue(entry["import_source"].endswith("part.obj"))
            self.assertEqual(import_settings["ImportGroups"][0]["Filenames"][0], entry["import_source"])


if __name__ == "__main__":
    unittest.main()
