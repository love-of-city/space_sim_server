"""Geometry helpers must not hide clearance errors by shrinking contact parts."""
import unittest
import json
from unittest import mock
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

from tools.task_box_proxy_experiment import circle_groups, mesh_read, partition_surface, proxy, simplify_plugs


class ProxyGeometryTests(unittest.TestCase):
    def test_deployed_plug_proxies_match_original_cad_recipe(self):
        repo = Path(__file__).resolve().parents[1]
        path = repo / 'model/SARM/platform/sarm_task_box_plugs.xml'
        root = ET.parse(path).getroot()
        metadata = json.loads(path.with_suffix('.manifest.json').read_text(encoding='utf-8'))['plugs']
        folder = repo / 'model/task_box/free_plugs'
        expected_heads = []
        for item in metadata:
            body = root.find(f'./worldbody/body[@name="{item["name"]}"]')
            expected_heads.append(mesh_read(path.parent / body.find('flexcomp').get('file')))
            for geom in list(body.findall('geom')):
                if '_proxy_' in geom.get('name', ''):
                    body.remove(geom)
        with tempfile.TemporaryDirectory() as temp:
            details = simplify_plugs(root, Path(temp), metadata,
                                    mesh_source=lambda name: folder / (name + '.obj'))
            self.assertEqual([row['retained_head_and_key_faces'] for row in details], [144, 132])
            self.assertEqual([row['primitive_count'] for row in details], [1, 5])
            deployed = ET.parse(path).getroot()
            for item, expected in zip(metadata, expected_heads):
                name = item['name']
                actual = mesh_read(Path(temp) / (name + '_head.obj'))
                for left, right in zip(actual, expected):
                    np.testing.assert_array_equal(left, right)
                query = f'./worldbody/body[@name="{name}"]/geom'
                generated = [g.attrib for g in root.findall(query) if '_proxy_' in g.get('name', '')]
                installed = [g.attrib for g in deployed.findall(query) if '_proxy_' in g.get('name', '')]
                self.assertEqual(generated, installed)

    @staticmethod
    def pins(radius=.0025):
        angles = np.linspace(0, 2*np.pi, 36, endpoint=False)
        circle = radius*np.column_stack((np.cos(angles), np.sin(angles)))
        centers = np.array([[-.0075, -.0075], [-.0075, .0075],
                            [.0075, -.0075], [.0075, .0075]])
        return centers, np.concatenate([circle+c for c in centers]+[centers])

    def test_pin_fit_preserves_cad_radius_and_ignores_cap_centers(self):
        centers, points = self.pins()
        fitted = circle_groups(points)
        self.assertEqual(len(fitted), 4)
        for center, radius in fitted:
            self.assertLess(np.min(np.linalg.norm(centers-center, axis=1)), 1e-10)
            self.assertAlmostEqual(radius, .0025, places=10)

    def test_wrong_pin_radius_fails_instead_of_resizing(self):
        _, points = self.pins(.002)
        with self.assertRaisesRegex(ValueError, "tolerance"):
            circle_groups(points)

    def test_missing_ring_is_rejected(self):
        _, points = self.pins()
        with self.assertRaisesRegex(ValueError, "four perimeter rings"):
            circle_groups(points[:108])

    def test_primitive_has_no_added_mass_and_preserves_contact_parameters(self):
        body = ET.Element("body")
        contact = ET.Element("contact", contype="4", conaffinity="7", condim="3",
                             friction=".7 .01 .001", solref=".02 1", internal="false")
        geom = proxy(body, "shaft", "cylinder", [0, 0, 0], [.02, .035], contact)
        self.assertEqual(geom.get("mass"), "0")
        self.assertEqual(geom.get("rgba"), "0 .7 .3 0")
        for key in ("contype", "conaffinity", "condim", "friction", "solref"):
            self.assertEqual(geom.get(key), contact.get(key))
        self.assertNotIn("internal", geom.attrib)

    def test_partition_assigns_every_whole_triangle_once(self):
        centers = [[.28, .275], [.37, .22], [.535, .275], [.535, .195], [.1, .1]]
        vertices = np.array([[x+dx, y+dy, -.01] for x, y in centers
                             for dx, dy in [(0, 0), (.001, 0), (0, .001)]])
        faces = np.arange(15).reshape(-1, 3)
        original_vertices, original_faces = vertices.copy(), faces.copy()
        regions = partition_surface(vertices, faces)
        np.testing.assert_array_equal(regions, np.arange(5))
        np.testing.assert_array_equal(vertices, original_vertices)
        np.testing.assert_array_equal(faces, original_faces)
        self.assertEqual(sum(len(faces[regions == i]) for i in range(5)), len(faces))

    def test_large_triangle_is_not_cut_or_assigned_by_centroid(self):
        vertices = np.array([[.28, .275, 0], [.28, .276, 0], [.5, .275, 0]])
        regions = partition_surface(vertices, np.array([[0, 1, 2]]))
        np.testing.assert_array_equal(regions, [4])

    def test_preserved_pins_do_not_create_analytic_pin_cylinders(self):
        root = ET.fromstring('<mujoco><worldbody><body name="aviation_plug">'
                             '<flexcomp file="unused.obj"><contact contype="4" conaffinity="7"/>'
                             '</flexcomp></body></worldbody></mujoco>')
        # Head, bottom plate, pin side, shaft side: retain the first three.
        vertices = np.array([[0, 0, 0], [.01, 0, 0], [0, .01, .025],
                             [0, 0, -.04], [.01, 0, -.04], [0, .01, -.04],
                             [0, 0, -.055], [.001, 0, -.04], [.001, 0, -.055]])
        faces = np.array([[0, 1, 2], [3, 4, 5], [6, 7, 8], [0, 3, 4]])
        metadata = [dict(name="aviation_plug", bus_center_m=[0, 0, 0])]
        with tempfile.TemporaryDirectory() as folder:
            with mock.patch('tools.task_box_proxy_experiment.mesh_read', return_value=(vertices, faces)):
                details = simplify_plugs(root, Path(folder), metadata, preserve_pins=True)
            saved_vertices, saved_faces = mesh_read(Path(folder)/'aviation_plug_head.obj')
        np.testing.assert_array_equal(saved_vertices[saved_faces], vertices[faces[:3]])
        self.assertEqual(details[0]['primitive_count'], 1)
        self.assertTrue(details[0]['original_pins_retained'])
        self.assertEqual(len(root.findall('.//geom')), 1)


if __name__ == "__main__":
    unittest.main()
