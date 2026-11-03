"""Renderer metadata must match native MuJoCo IDs, not XML interleaving."""
from pathlib import Path
import tempfile
import unittest

from bsk_render_adapter.mjcf_assets import parse_mjcf_geometry_metadata

try:
    import mujoco
except ImportError:
    mujoco = None


@unittest.skipIf(mujoco is None, 'MuJoCo is required for native ordering comparison')
class GeometryOrderTest(unittest.TestCase):
    def test_parent_geoms_after_children_and_frames_match_native_ids(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'interleaved.xml'
            (Path(directory)/'tetra.obj').write_text(
                'v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\n'
                'f 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n', encoding='ascii')
            path.write_text('''<mujoco>
              <compiler fusestatic="false"/>
              <asset><mesh name="parent_mesh" file="tetra.obj"/>
                <mesh name="child_mesh" file="tetra.obj" scale=".5 .5 .5"/></asset>
              <worldbody><body name="parent">
                <body name="child"><geom name="child_first" type="mesh" mesh="child_mesh"/></body>
                <geom name="parent_late" type="mesh" mesh="parent_mesh" pos="2 0 0"/>
                <frame><body name="second_child"><geom name="nested" type="mesh" mesh="child_mesh"/></body>
                  <geom name="parent_in_frame" type="mesh" mesh="parent_mesh" pos="0 2 0"/>
                </frame>
              </body></worldbody></mujoco>''', encoding='utf-8')
            native = mujoco.MjModel.from_xml_path(str(path))
            expected = [(native.body(int(native.geom_bodyid[i])).name,
                         native.mesh(int(native.geom_dataid[i])).name) for i in range(native.ngeom)]
            for compile_poses in (False, True):
                metadata = parse_mjcf_geometry_metadata(path, compile_mesh_poses=compile_poses)
                self.assertEqual([(x.body_name,x.mesh_name) for x in metadata],expected)


if __name__ == '__main__':
    unittest.main()
